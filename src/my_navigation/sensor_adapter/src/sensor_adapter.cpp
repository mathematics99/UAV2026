#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class SensorAdapter : public rclcpp::Node
{
public:
    SensorAdapter()
    : Node("sensor_adapter")
    {
        // =========================
        // 1. 声明参数
        // =========================

        this->declare_parameter<std::string>(
            "input_odom_topic",
            "/visual/odom");

        this->declare_parameter<std::string>(
            "input_cloud_topic",
            "/lidar/points");

        this->declare_parameter<std::string>(
            "output_odom_topic",
            "/uav/odom");

        this->declare_parameter<std::string>(
            "output_cloud_topic",
            "/uav/pointcloud");


        // =========================
        // 2. 获取参数
        // =========================

        input_odom_topic_ =
            this->get_parameter("input_odom_topic").as_string();

        input_cloud_topic_ =
            this->get_parameter("input_cloud_topic").as_string();

        output_odom_topic_ =
            this->get_parameter("output_odom_topic").as_string();

        output_cloud_topic_ =
            this->get_parameter("output_cloud_topic").as_string();


        // =========================
        // 3. 创建发布器
        // =========================

        odom_pub_ =
            this->create_publisher<nav_msgs::msg::Odometry>(
                output_odom_topic_, 10);

        cloud_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                output_cloud_topic_,
                rclcpp::SensorDataQoS());


        // =========================
        // 4. 创建订阅器
        // =========================

        odom_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                input_odom_topic_,
                10,
                std::bind(
                    &SensorAdapter::odomCallback,
                    this,
                    std::placeholders::_1));

        cloud_sub_ =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                input_cloud_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(
                    &SensorAdapter::cloudCallback,
                    this,
                    std::placeholders::_1));


        // =========================
        // 5. 输出启动信息
        // =========================

        RCLCPP_INFO(
            this->get_logger(),
            "Sensor Adapter started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Odometry: %s -> %s",
            input_odom_topic_.c_str(),
            output_odom_topic_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "PointCloud: %s -> %s",
            input_cloud_topic_.c_str(),
            output_cloud_topic_.c_str());
    }

private:

    // =========================
    // Odometry回调
    // =========================

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto output_msg = *msg;

        // 第一版暂时不修改坐标系
        // 后面可以在这里统一 map / odom / base_link

        odom_pub_->publish(output_msg);
    }


    // =========================
    // PointCloud回调
    // =========================

    void cloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto output_msg = *msg;

        // 第一版直接转发点云
        // 后面可以在这里：
        // 1. 坐标变换
        // 2. ROI裁剪
        // 3. 降采样
        // 4. 去除机体点云

        cloud_pub_->publish(output_msg);
    }


    // =========================
    // Topic名称
    // =========================

    std::string input_odom_topic_;
    std::string input_cloud_topic_;

    std::string output_odom_topic_;
    std::string output_cloud_topic_;


    // =========================
    // ROS2通信对象
    // =========================

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        cloud_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        odom_pub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        cloud_pub_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<SensorAdapter>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}