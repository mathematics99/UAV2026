#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

// V4.1: 风险解耦自适应规划 (Risk-Decoupled Adaptive Planning)
// 核心思想: 同一个风险 R_g 不应统一驱动所有规划参数向"更保守"变化,
// 而是按各参数物理角色分别调节:
//   - v_max  : 直接由 R_bar 决定 (高风险 -> 降速, 始终成立)
//   - d_safe : 受收缩风险 R_c 门控 (开阔区 R_c 低 -> 可增距; 窄道 R_c 高 -> 不盲目增距)
//   - lambda_obs : 压缩范围 + 收缩门控 (窄道里仍增权重但增长被限制)
class AdaptivePlanner : public rclcpp::Node
{
public:
  AdaptivePlanner() : Node("adaptive_planner")
  {
    this->declare_parameter("smoothing/lambda", 0.9);      // EMA 平滑系数
    this->declare_parameter("smoothing/max_delta", 0.05);  // R_bar 每周期最大变化率限制
    this->declare_parameter("safe/d_base", 0.5);           // 基础安全距离 (= EGO 基线 clearance)
    this->declare_parameter("safe/d_max", 1.0);
    this->declare_parameter("speed/v_low", 0.8);
    this->declare_parameter("speed/v_high", 2.0);          // = 原始 EGO max_vel
    this->declare_parameter("weight/lambda_min", 0.5);     // = 原始 EGO lambda_collision
    this->declare_parameter("weight/lambda_max", 2.0);
    this->declare_parameter("weight/gamma", 2.0);
    this->declare_parameter("weight/contraction_gain", 0.5); // λ_obs 收缩门控系数
    this->declare_parameter("gate/enabled", true);           // 收缩门控开关 (消融 C2: false)

    this->get_parameter("smoothing/lambda", lambda_);
    this->get_parameter("smoothing/max_delta", max_delta_);
    this->get_parameter("safe/d_base", d_base_);
    this->get_parameter("safe/d_max", d_max_);
    this->get_parameter("speed/v_low", v_low_);
    this->get_parameter("speed/v_high", v_high_);
    this->get_parameter("weight/lambda_min", w_min_);
    this->get_parameter("weight/lambda_max", w_max_);
    this->get_parameter("weight/gamma", gamma_);
    this->get_parameter("weight/contraction_gain", c_gain_);
    this->get_parameter("gate/enabled", gate_enabled_);

    risk_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/risk/risk_score", 10,
        [this](const std::shared_ptr<const std_msgs::msg::Float64> &msg)
        {
          this->riskCallback(msg);
        });
    contraction_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/risk/contraction_risk", 10,
        [this](const std::shared_ptr<const std_msgs::msg::Float64> &msg)
        {
          this->contractionCallback(msg);
        });

    safe_pub_ = this->create_publisher<std_msgs::msg::Float64>("/adaptive/safe_distance", 10);
    vel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/adaptive/max_velocity", 10);
    weight_pub_ = this->create_publisher<std_msgs::msg::Float64>("/adaptive/collision_weight", 10);
    smooth_pub_ = this->create_publisher<std_msgs::msg::Float64>("/adaptive/smoothed_risk", 10);
    smooth_contraction_pub_ = this->create_publisher<std_msgs::msg::Float64>("/adaptive/smoothed_contraction", 10);
    gate_pub_ = this->create_publisher<std_msgs::msg::Float64>("/adaptive/contraction_gate", 10);
    timing_pub_ = this->create_publisher<std_msgs::msg::Float64>("/adaptive/timing_ms", 10);

    RCLCPP_INFO(this->get_logger(),
                "Adaptive Planner V4.1 (decoupled) started: v_max[%.2f,%.2f], d_safe=%.2f+G_c*R*(%.2f-%.2f), lambda=%.2f+(1-%.2f*Rc)*R^%.1f*(%.2f-%.2f)",
                v_low_, v_high_, d_base_, d_base_, d_max_, w_min_, c_gain_, gamma_, w_min_, w_max_);
  }

private:
  void riskCallback(const std::shared_ptr<const std_msgs::msg::Float64> &msg)
  {
    const double r = std::clamp(msg->data, 0.0, 1.0);

    // 1) 指数滑动平均 (EMA)
    if (!init_risk_)
    {
      r_bar_ = r;
      init_risk_ = true;
    }
    else
    {
      r_bar_ = lambda_ * r_bar_ + (1.0 - lambda_) * r;
    }

    // 2) 变化率限制
    const double delta = r_bar_ - r_bar_last_;
    if (std::fabs(delta) > max_delta_)
    {
      r_bar_ = r_bar_last_ + (delta > 0.0 ? max_delta_ : -max_delta_);
    }
    r_bar_last_ = r_bar_;

    publishOutputs();
  }

  void contractionCallback(const std::shared_ptr<const std_msgs::msg::Float64> &msg)
  {
    const double rc = std::clamp(msg->data, 0.0, 1.0);

    // 收缩风险也做 EMA 平滑 (作为门控量, 需稳定)
    if (!init_rc_)
    {
      rc_bar_ = rc;
      init_rc_ = true;
    }
    else
    {
      rc_bar_ = lambda_ * rc_bar_ + (1.0 - lambda_) * rc;
    }

    publishOutputs();
  }

  void publishOutputs()
  {
    if (!init_risk_)
      return; // 还没收到 R_g, 无法计算

    const auto t_start = std::chrono::steady_clock::now();

    // 收缩门控量 G_c = 1 - R_c ∈ [0, 1] (开阔=1, 窄道=0)
    // 消融实验 C2: gate/enabled=false 时 G_c 恒为 1, 收缩增益也置 0
    const double g_c = gate_enabled_ ? std::clamp(1.0 - rc_bar_, 0.0, 1.0) : 1.0;
    const double c_gain_eff = gate_enabled_ ? c_gain_ : 0.0;

    // 3) 风险解耦映射
    const double v_max = v_high_ - r_bar_ * (v_high_ - v_low_);
    const double d_safe = d_base_ + g_c * r_bar_ * (d_max_ - d_base_);
    const double lambda_obs = w_min_ +
                              (1.0 - c_gain_eff * rc_bar_) * std::pow(r_bar_, gamma_) * (w_max_ - w_min_);

    publish(safe_pub_, d_safe);
    publish(vel_pub_, v_max);
    publish(weight_pub_, lambda_obs);
    publish(smooth_pub_, r_bar_);
    publish(smooth_contraction_pub_, rc_bar_);
    publish(gate_pub_, g_c);

    const auto t_end = std::chrono::steady_clock::now();
    const double timing_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();
    publish(timing_pub_, timing_ms);
  }

  void publish(const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &pub, double v)
  {
    std_msgs::msg::Float64 m;
    m.data = v;
    pub->publish(m);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr risk_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr contraction_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr safe_pub_, vel_pub_, weight_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr smooth_pub_, smooth_contraction_pub_, gate_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timing_pub_;

  double lambda_, max_delta_;
  double d_base_, d_max_, v_low_, v_high_, w_min_, w_max_, gamma_, c_gain_;
  bool gate_enabled_ = true;
  bool init_risk_ = false, init_rc_ = false;
  double r_bar_ = 0.0, r_bar_last_ = 0.0, rc_bar_ = 0.0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdaptivePlanner>());
  rclcpp::shutdown();
  return 0;
}
