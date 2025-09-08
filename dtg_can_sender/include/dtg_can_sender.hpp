#ifndef DTG_CAN_SENDER__HPP__
#define DTG_CAN_SENDER__HPP__

#include "rclcpp/rclcpp.hpp"

#include <can_msgs/msg/frame.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>

// 신규 상태 토픽 (레거시 타입으로 통일)
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
// Actuation status (가속/브레이크 실제 상태)
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>

#include <mutex>
#include <array>
#include <string>

namespace dtg_can_sender {

class DtgCanSender : public rclcpp::Node {
public:
  explicit DtgCanSender(const rclcpp::NodeOptions & options);

private:
  // utils
  static inline double clamp(double v, double lo, double hi);
  static std::string dump_bytes(const std::array<uint8_t, 8> &data, uint8_t dlc);
  static double yaw_from_quat(const geometry_msgs::msg::Quaternion &q) {
    // normalize to avoid drift when |q| != 1
    const double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    const double w = q.w / n, x = q.x / n, y = q.y / n, z = q.z / n;
    const double s = 2.0 * (w * z + x * y);
    const double c = 1.0 - 2.0 * (x * x + y * y);
    return std::atan2(s, c);  // Z-yaw (ENU)
  }
  static uint8_t map_gear_to_dbc(uint8_t report) {
  using autoware_vehicle_msgs::msg::GearReport;
    switch (report) {
      case GearReport::PARK: return 0x00;
      case GearReport::REVERSE: return 0x01;
      case GearReport::NEUTRAL: return 0x02;
      case GearReport::DRIVE: return 0x03;
      default: return 0x00;
    }
  }

  // callbacks (commands)
  void on_control_cmd(const autoware_control_msgs::msg::Control::ConstSharedPtr msg);
  void on_gear_cmd(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
  void on_turn_indicators_cmd(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
  void on_operation_mode(const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg);

  // callbacks (status/localization)
  void on_velocity_status(const autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg);
  void on_steering_status(const autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg);
  void on_gear_status(const autoware_vehicle_msgs::msg::GearReport::ConstSharedPtr msg);
  void on_engine_rpm(const std_msgs::msg::Float32::ConstSharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void on_actuation_status(const tier4_vehicle_msgs::msg::ActuationStatusStamped::ConstSharedPtr msg);

  // timer
  void on_timer();

  // encoders
  void encode_and_publish_0x100();
  void encode_and_publish_0x200();
  void encode_and_publish_0x201();
  void encode_and_publish_0x400();

  // ROS I/F
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_indicators_cmd_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr operation_mode_sub_;

  // 신규 구독
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_status_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rpm_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr actuation_status_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // caches (commands)
  autoware_control_msgs::msg::Control::ConstSharedPtr latest_control_cmd_;
  autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr latest_gear_cmd_;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr latest_turn_indicators_cmd_;
  autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr latest_operation_mode_;

  // caches (status/localization)
  autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr latest_velocity_status_;
  autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr latest_steering_status_;
  autoware_vehicle_msgs::msg::GearReport::ConstSharedPtr latest_gear_status_;
  std_msgs::msg::Float32::ConstSharedPtr latest_rpm_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
  tier4_vehicle_msgs::msg::ActuationStatusStamped::ConstSharedPtr latest_actuation_status_;

  // odom-derived accel
  double last_ax_{0.0};
  double last_ay_{0.0};
  bool has_prev_odom_{false};
  rclcpp::Time prev_odom_stamp_{};
  double prev_vx_{0.0};
  double prev_vy_{0.0};

  double latest_yaw_deg_{0.0};  // <- 새로 추가: 0x400 Yaw 인코딩용

  // Mode state
  std::mutex data_mutex_;
  bool is_autonomous_ = false;  // 하위 nibble(Mode Change Flag)는 사용하지 않음

  double altitude_scale_{0.0};
  double fixed_altitude_m_{0.0};
};

} // namespace dtg_can_sender
#endif
