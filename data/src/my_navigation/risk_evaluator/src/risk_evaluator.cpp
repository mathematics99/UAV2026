#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float64.hpp"

// =====================================================================
// risk_evaluator V3
//
// 输入:
//   /uav/odom       (nav_msgs/Odometry)       世界系, 位姿与速度
//   /uav/pointcloud (sensor_msgs/PointCloud2) 世界系, 前向障碍点云
//
// 几何运动风险模型 (R_g ∈ [0, 1]):
//
//   1) 体素占用率  rho_occ = N_occupied / N_total        ∈ [0, 1]
//      前向 ROI 划分 0.2m 立方体素, 体素内 >=1 点即为 occupied。
//
//   2) 动态制动裕度  d_req = d0 + v*tau + v^2/(2*a_b)
//                     R_b  = clip(d_req / (d_min + eps), 0, 1)
//      同一几何环境下, 速度越大 -> 所需制动距离越大 -> 风险越高。
//
//   3) 前向通行空间收缩 (V3):
//        - 沿 X 切成 n_slices 片, 每片截面开阔度 F_i = 1 - O_i
//        - 整体收缩  C_global = clip((F_near-F_far)/(F_near+eps), 0, 1)
//          (近端/远端各 3 片平均)
//        - 局部突变  G_c = max_i clip((F_i-F_{i+1})/(F_i+eps), 0, 1)
//        - 组合      R_c = eta*C_global + (1-eta)*G_c
//
//   4) 组合          R_g = w_rho*rho_occ + w_b*R_b + w_c*R_c
//
//   5) 安全距离映射  d_safe = safe_min + R_g * (safe_max - safe_min)
//
// 输出 (/risk/*, std_msgs/Float64):
//   /risk/obstacle_density     rho_occ  体素占用率
//   /risk/min_distance         d_min    最近障碍距离
//   /risk/current_velocity     v        当前速度
//   /risk/braking_distance     d_req    所需制动距离 (debug)
//   /risk/braking_risk         R_b      制动裕度风险 (debug)
//   /risk/contraction_global   C_global 整体通行空间收缩 (debug)
//   /risk/contraction_gradient G_c      局部瓶颈突变梯度 (debug)
//   /risk/contraction_risk     R_c      收缩风险 (debug)
//   /risk/risk_score           R_g      综合几何风险
//   /risk/safe_distance        d_safe   安全距离
// =====================================================================

class RiskEvaluator : public rclcpp::Node
{
public:
    RiskEvaluator()
    : Node("risk_evaluator")
    {
        // =========================
        // 1. 声明参数
        // =========================

        this->declare_parameter<std::string>("input_odom_topic", "/uav/odom");
        this->declare_parameter<std::string>("input_cloud_topic", "/uav/pointcloud");

        // 前向 ROI 长方体 (机体坐标系)
        this->declare_parameter<double>("roi/x_min", 0.0);
        this->declare_parameter<double>("roi/x_max", 5.0);
        this->declare_parameter<double>("roi/y_half", 2.0);
        this->declare_parameter<double>("roi/z_half", 1.5);

        // 体素大小
        this->declare_parameter<double>("voxel_size", 0.2);

        // 制动距离模型: d_req = d0 + v*tau + v^2/(2*a_b)
        this->declare_parameter<double>("braking/d0", 0.3);
        this->declare_parameter<double>("braking/tau", 0.3);
        this->declare_parameter<double>("braking/a_b", 2.0);
        this->declare_parameter<double>("braking/epsilon", 0.1);
        this->declare_parameter<bool>("braking/use_d0", false);

        // 前向通行空间收缩 (V3)
        this->declare_parameter<int>("contraction/slices", 10);
        this->declare_parameter<double>("contraction/eta", 0.6);
        this->declare_parameter<double>("contraction/epsilon", 0.1);

        // 风险权重 (w_rho + w_b + w_c = 1)
        this->declare_parameter<double>("weight/density", 0.20);
        this->declare_parameter<double>("weight/braking", 0.55);
        this->declare_parameter<double>("weight/contraction", 0.25);

        // 安全距离映射上下界
        this->declare_parameter<double>("safe/min", 0.5);
        this->declare_parameter<double>("safe/max", 1.5);

        // =========================
        // 2. 获取参数
        // =========================

        input_odom_topic_ =
            this->get_parameter("input_odom_topic").as_string();
        input_cloud_topic_ =
            this->get_parameter("input_cloud_topic").as_string();

        roi_x_min_ = this->get_parameter("roi/x_min").as_double();
        roi_x_max_ = this->get_parameter("roi/x_max").as_double();
        roi_y_half_ = this->get_parameter("roi/y_half").as_double();
        roi_z_half_ = this->get_parameter("roi/z_half").as_double();

        voxel_size_ = this->get_parameter("voxel_size").as_double();

        d0_  = this->get_parameter("braking/d0").as_double();
        tau_ = this->get_parameter("braking/tau").as_double();
        a_b_ = this->get_parameter("braking/a_b").as_double();
        eps_ = this->get_parameter("braking/epsilon").as_double();
        use_d0_ = this->get_parameter("braking/use_d0").as_bool();

        n_slices_ = this->get_parameter("contraction/slices").as_int();
        eta_ = this->get_parameter("contraction/eta").as_double();
        eps_c_ = this->get_parameter("contraction/epsilon").as_double();

        w_rho_ = this->get_parameter("weight/density").as_double();
        w_b_   = this->get_parameter("weight/braking").as_double();
        w_c_   = this->get_parameter("weight/contraction").as_double();

        safe_min_ = this->get_parameter("safe/min").as_double();
        safe_max_ = this->get_parameter("safe/max").as_double();

        // =========================
        // 3. 预计算体素网格尺寸
        // =========================

        nx_ = static_cast<int>(std::ceil((roi_x_max_ - roi_x_min_) / voxel_size_));
        ny_ = static_cast<int>(std::ceil((2.0 * roi_y_half_) / voxel_size_));
        nz_ = static_cast<int>(std::ceil((2.0 * roi_z_half_) / voxel_size_));
        n_total_ = nx_ * ny_ * nz_;

        occupied_.resize(static_cast<size_t>(n_total_), 0);

        // 前向 X 向切片 (V3): 每片 slice_width 米, 片内独立体素网格
        slice_width_ = (roi_x_max_ - roi_x_min_) / static_cast<double>(n_slices_);
        nx_slice_ = static_cast<int>(std::ceil(slice_width_ / voxel_size_));
        n_total_slice_ = nx_slice_ * ny_ * nz_;

        slice_occupied_.resize(static_cast<size_t>(n_slices_));
        for (auto & s : slice_occupied_)
            s.resize(static_cast<size_t>(n_total_slice_), 0);

        // =========================
        // 4. 创建发布器
        // =========================

        pub_rho_   = this->create_publisher<std_msgs::msg::Float64>(
            "/risk/obstacle_density", 10);
        pub_dmin_  = this->create_publisher<std_msgs::msg::Float64>(
            "/risk/min_distance", 10);
        pub_v_     = this->create_publisher<std_msgs::msg::Float64>(
            "/risk/current_velocity", 10);
        pub_dreq_  = this->create_publisher<std_msgs::msg::Float64>(
            "/risk/braking_distance", 10);
        pub_rb_    = this->create_publisher<std_msgs::msg::Float64>(
            "/risk/braking_risk", 10);
        pub_risk_  = this->create_publisher<std_msgs::msg::Float64>(
            "/risk/risk_score", 10);
        pub_dsafe_ = this->create_publisher<std_msgs::msg::Float64>(
            "/risk/safe_distance", 10);
        // V3 收缩风险 debug 输出
        pub_cg_ = this->create_publisher<std_msgs::msg::Float64>(
            "/risk/contraction_global", 10);
        pub_gc_ = this->create_publisher<std_msgs::msg::Float64>(
            "/risk/contraction_gradient", 10);
        pub_rc_ = this->create_publisher<std_msgs::msg::Float64>(
            "/risk/contraction_risk", 10);
        pub_timing_ = this->create_publisher<std_msgs::msg::Float64>(
            "/risk/timing_ms", 10);

        // =========================
        // 5. 创建订阅器
        // =========================

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic_,
            10,
            std::bind(
                &RiskEvaluator::odomCallback,
                this,
                std::placeholders::_1));

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(
                &RiskEvaluator::cloudCallback,
                this,
                std::placeholders::_1));

        // =========================
        // 6. 输出启动信息
        // =========================

        RCLCPP_INFO(this->get_logger(), "Risk Evaluator V3 started.");
        RCLCPP_INFO(
            this->get_logger(),
            "Input: odom=%s, cloud=%s",
            input_odom_topic_.c_str(),
            input_cloud_topic_.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "ROI: x=[%.1f, %.1f], y=+-%.1f, z=+-%.1f, voxel=%.2f (grid %dx%dx%d)",
            roi_x_min_, roi_x_max_, roi_y_half_, roi_z_half_,
            voxel_size_, nx_, ny_, nz_);
        RCLCPP_INFO(
            this->get_logger(),
            "Model: R_g = %.2f*rho + %.2f*R_b + %.2f*R_c, d_safe in [%.1f, %.1f]",
            w_rho_, w_b_, w_c_, safe_min_, safe_max_);
        RCLCPP_INFO(
            this->get_logger(),
            "Contraction: %d X-slices (%.2fm), F_near/F_far=3-slice mean, "
            "R_c = %.2f*C_global + %.2f*G_c",
            n_slices_, slice_width_, eta_, 1.0 - eta_);
    }

private:
    // =========================
    // Odometry 回调
    // =========================

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        has_odom_ = true;

        uav_x_ = msg->pose.pose.position.x;
        uav_y_ = msg->pose.pose.position.y;
        uav_z_ = msg->pose.pose.position.z;

        qx_ = msg->pose.pose.orientation.x;
        qy_ = msg->pose.pose.orientation.y;
        qz_ = msg->pose.pose.orientation.z;
        qw_ = msg->pose.pose.orientation.w;

        const double vx = msg->twist.twist.linear.x;
        const double vy = msg->twist.twist.linear.y;
        const double vz = msg->twist.twist.linear.z;
        velocity_ = std::sqrt(vx * vx + vy * vy + vz * vz);
    }

    // =========================
    // PointCloud 回调
    // =========================

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!has_odom_)
            return;

        const auto t_start = std::chrono::steady_clock::now();

        // 默认: 前方 ROI 内无障碍, 最近距离取 ROI 最远边界
        double d_min = roi_x_max_;

        // 清零体素占用标记
        std::fill(occupied_.begin(), occupied_.end(), 0);
        for (auto & s : slice_occupied_)
            std::fill(s.begin(), s.end(), 0);

        // body -> world 旋转矩阵 R (由四元数计算)
        double R[3][3];
        {
            const double x = qx_;
            const double y = qy_;
            const double z = qz_;
            const double w = qw_;

            R[0][0] = 1.0 - 2.0 * (y * y + z * z);
            R[0][1] = 2.0 * (x * y - w * z);
            R[0][2] = 2.0 * (x * z + w * y);
            R[1][0] = 2.0 * (x * y + w * z);
            R[1][1] = 1.0 - 2.0 * (x * x + z * z);
            R[1][2] = 2.0 * (y * z - w * x);
            R[2][0] = 2.0 * (x * z - w * y);
            R[2][1] = 2.0 * (y * z + w * x);
            R[2][2] = 1.0 - 2.0 * (x * x + y * y);
        }

        // 遍历点云
        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
        {
            // 世界系 -> 机体系: 先平移, 再旋转 (R^T * d)
            const double dx = *it_x - uav_x_;
            const double dy = *it_y - uav_y_;
            const double dz = *it_z - uav_z_;

            const double bx = R[0][0] * dx + R[1][0] * dy + R[2][0] * dz;
            const double by = R[0][1] * dx + R[1][1] * dy + R[2][1] * dz;
            const double bz = R[0][2] * dx + R[1][2] * dy + R[2][2] * dz;

            // 前向 ROI 筛选
            if (bx < roi_x_min_ || bx > roi_x_max_)
                continue;
            if (std::fabs(by) > roi_y_half_)
                continue;
            if (std::fabs(bz) > roi_z_half_)
                continue;

            // 最近障碍距离
            const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist < d_min)
                d_min = dist;

            // 体素索引 (边界点归入边界体素)
            int ix = static_cast<int>(std::floor((bx - roi_x_min_) / voxel_size_));
            int iy = static_cast<int>(std::floor((by + roi_y_half_) / voxel_size_));
            int iz = static_cast<int>(std::floor((bz + roi_z_half_) / voxel_size_));

            ix = std::min(std::max(ix, 0), nx_ - 1);
            iy = std::min(std::max(iy, 0), ny_ - 1);
            iz = std::min(std::max(iz, 0), nz_ - 1);

            const int idx = ix + nx_ * (iy + ny_ * iz);
            occupied_[static_cast<size_t>(idx)] = 1;

            // 前向切片体素标记 (V3)
            const int si = std::min(
                std::max(static_cast<int>(std::floor(bx / slice_width_)), 0),
                n_slices_ - 1);
            const int sx = std::min(
                std::max(static_cast<int>(std::floor(
                    (bx - si * slice_width_) / voxel_size_)), 0),
                nx_slice_ - 1);
            const int sidx = sx + nx_slice_ * (iy + ny_ * iz);
            slice_occupied_[static_cast<size_t>(si)][static_cast<size_t>(sidx)] = 1;
        }

        // 体素占用率 rho_occ ∈ [0, 1]
        int n_occupied = 0;
        for (const auto v : occupied_)
        {
            if (v)
                ++n_occupied;
        }
        const double rho_occ =
            static_cast<double>(n_occupied) / static_cast<double>(n_total_);

        // 动态制动裕度 (V4.1): 基础安全距离 d0 已解耦, 不计入制动距离
        // d_brake = v*tau + v^2/(2*a_b)
        // 消融实验 (任务2 D2): braking/use_d0:=true 恢复旧公式 d_req = d0 + v*tau + v^2/(2*a_b)
        const double d_brake =
            (use_d0_ ? d0_ : 0.0) +
            velocity_ * tau_ +
            velocity_ * velocity_ / (2.0 * a_b_);

        // R_b = clip(d_brake / (d_min + eps), 0, 1)  => v=0 时 R_b=0
        const double r_b = clip(d_brake / (d_min + eps_), 0.0, 1.0);

        // 前向通行空间收缩风险 (V3)
        double c_global = 0.0;
        double g_c = 0.0;
        double r_c = 0.0;
        calculateContractionRisk(c_global, g_c, r_c);

        // 综合几何风险 R_g ∈ [0, 1]
        const double r_g = w_rho_ * rho_occ + w_b_ * r_b + w_c_ * r_c;

        // 安全距离映射 d_safe ∈ [safe_min, safe_max]
        const double d_safe = safe_min_ + r_g * (safe_max_ - safe_min_);

        // 发布
        publish(pub_rho_, rho_occ);
        publish(pub_dmin_, d_min);
        publish(pub_v_, velocity_);
        publish(pub_dreq_, d_brake);
        publish(pub_rb_, r_b);
        publish(pub_risk_, r_g);
        publish(pub_dsafe_, d_safe);
        publish(pub_cg_, c_global);
        publish(pub_gc_, g_c);
        publish(pub_rc_, r_c);

        const auto t_end = std::chrono::steady_clock::now();
        const double timing_ms =
            std::chrono::duration<double, std::milli>(t_end - t_start).count();
        publish(pub_timing_, timing_ms);
    }

    // =========================
    // 工具
    // =========================

    static double clip(double value, double lo, double hi)
    {
        return std::min(std::max(value, lo), hi);
    }

    // =========================
    // 前向通行空间收缩风险 (V3)
    //   F_i       = 1 - O_i  (截面开阔度 Cross-sectional Openness Index)
    //   C_global  = clip((F_near - F_far) / (F_near + eps), 0, 1)
    //   G_c       = max_i clip((F_i - F_{i+1}) / (F_i + eps), 0, 1)
    //   R_c       = eta * C_global + (1 - eta) * G_c
    // =========================

    void calculateContractionRisk(
        double & c_global,
        double & g_c,
        double & r_c)
    {
        // 每个切片的截面开阔度 F_i = 1 - O_i
        std::vector<double> F(static_cast<size_t>(n_slices_), 0.0);
        for (int i = 0; i < n_slices_; ++i)
        {
            int n_occ = 0;
            for (const auto v : slice_occupied_[static_cast<size_t>(i)])
            {
                if (v)
                    ++n_occ;
            }
            const double O =
                static_cast<double>(n_occ) / static_cast<double>(n_total_slice_);
            F[static_cast<size_t>(i)] = 1.0 - O;
        }

        // 近端/远端各 3 片平均
        const int n_near = std::min(3, n_slices_);
        const int n_far = std::min(3, n_slices_);

        double f_near = 0.0;
        for (int i = 0; i < n_near; ++i)
            f_near += F[static_cast<size_t>(i)];
        f_near /= static_cast<double>(n_near);

        double f_far = 0.0;
        for (int i = n_slices_ - n_far; i < n_slices_; ++i)
            f_far += F[static_cast<size_t>(i)];
        f_far /= static_cast<double>(n_far);

        // 整体收缩: 近端开阔而远端狭窄 -> 风险
        c_global = clip((f_near - f_far) / (f_near + eps_c_), 0.0, 1.0);

        // 局部突变: 相邻切片最大正向收缩梯度
        g_c = 0.0;
        for (int i = 0; i < n_slices_ - 1; ++i)
        {
            const double grad = clip(
                (F[static_cast<size_t>(i)] - F[static_cast<size_t>(i + 1)]) /
                    (F[static_cast<size_t>(i)] + eps_c_),
                0.0, 1.0);
            if (grad > g_c)
                g_c = grad;
        }

        // 组合
        r_c = eta_ * c_global + (1.0 - eta_) * g_c;
    }

    void publish(
        const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & pub,
        double value)
    {
        auto m = std_msgs::msg::Float64();
        m.data = value;
        pub->publish(m);
    }

    // =========================
    // 参数
    // =========================

    std::string input_odom_topic_;
    std::string input_cloud_topic_;

    double roi_x_min_;
    double roi_x_max_;
    double roi_y_half_;
    double roi_z_half_;

    double voxel_size_;

    double d0_;
    bool use_d0_ = false;
    double tau_;
    double a_b_;
    double eps_;

    double w_rho_;
    double w_b_;
    double w_c_;

    double safe_min_;
    double safe_max_;

    // 体素网格
    int nx_;
    int ny_;
    int nz_;
    int n_total_;
    std::vector<uint8_t> occupied_;

    // 前向切片 (V3)
    int n_slices_;
    double slice_width_;
    int nx_slice_;
    int n_total_slice_;
    double eta_;
    double eps_c_;
    std::vector<std::vector<uint8_t>> slice_occupied_;

    // =========================
    // 状态
    // =========================

    bool has_odom_ = false;

    double uav_x_ = 0.0;
    double uav_y_ = 0.0;
    double uav_z_ = 0.0;

    double qx_ = 0.0;
    double qy_ = 0.0;
    double qz_ = 0.0;
    double qw_ = 1.0;

    double velocity_ = 0.0;

    // =========================
    // ROS2 通信对象
    // =========================

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        cloud_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rho_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dmin_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_v_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dreq_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rb_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_risk_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dsafe_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cg_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_gc_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rc_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_timing_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RiskEvaluator>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
