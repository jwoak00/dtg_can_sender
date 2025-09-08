#include <dtg_can_sender.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace dtg_can_sender {

DtgCanSender::DtgCanSender(const rclcpp::NodeOptions & options)
: Node("dtg_can_sender", options)
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  // 새 파라미터: 0x400 Altitude 인코딩용
  altitude_scale_ = declare_parameter<double>("altitude_scale", 1.0);        // decoder와 동일 스케일
  fixed_altitude_m_ = declare_parameter<double>("fixed_altitude_m", 0.0);    // 임시 고정 값

  // 퍼블리시 토픽 파라미터화 (can_driver 호환 기본값: /transmit/frame)
  const std::string can_tx_topic =
      this->declare_parameter<std::string>("can_tx_topic", "/transmit/frame");
  can_pub_ = this->create_publisher<can_msgs::msg::Frame>(can_tx_topic, rclcpp::QoS(10));

  // 오퍼레이션 모드 토픽 (DBC 표에 맞춰 기본: /system/operation_mode/state)
  const std::string op_mode_topic =
      this->declare_parameter<std::string>("operation_mode_topic", "/system/operation_mode/state");
  operation_mode_sub_ =
      this->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
          op_mode_topic, rclcpp::QoS(1).transient_local(),
          std::bind(&DtgCanSender::on_operation_mode, this, _1));

  control_cmd_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd", 10, std::bind(&DtgCanSender::on_control_cmd, this, _1));
  gear_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1, std::bind(&DtgCanSender::on_gear_cmd, this, _1));
  turn_indicators_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
    "/control/command/turn_indicators_cmd", 1, std::bind(&DtgCanSender::on_turn_indicators_cmd, this, _1));

  // 신규 상태/로컬라이제이션 구독
  velocity_status_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", 10, std::bind(&DtgCanSender::on_velocity_status, this, _1));
  steering_status_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", 10, std::bind(&DtgCanSender::on_steering_status, this, _1));
  gear_status_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", 10, std::bind(&DtgCanSender::on_gear_status, this, _1));
  rpm_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/vehicle/status/engine_rpm", 10, std::bind(&DtgCanSender::on_engine_rpm, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", 10, std::bind(&DtgCanSender::on_odom, this, _1));
  actuation_status_sub_ = this->create_subscription<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
    "/vehicle/status/actuation_status", 10, std::bind(&DtgCanSender::on_actuation_status, this, _1));

  using namespace std::chrono_literals;
  // 기존 하드코딩(100ms) 대신 파라미터화
  const int timer_period_ms = this->declare_parameter<int>("timer_period_ms", 100);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms),
                                   std::bind(&DtgCanSender::on_timer, this));

  RCLCPP_INFO(this->get_logger(),
              "DTG CAN Sender started. tx_topic=%s, op_mode_topic=%s, period=%dms",
              can_tx_topic.c_str(), op_mode_topic.c_str(), timer_period_ms);
}

// static
inline double DtgCanSender::clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

// static
std::string DtgCanSender::dump_bytes(const std::array<uint8_t, 8> &data, uint8_t dlc) {
  std::ostringstream oss;
  oss << std::uppercase << std::hex << std::setfill('0');
  const size_t n = std::min<size_t>(dlc, data.size());
  for (size_t i = 0; i < n; ++i) {
    if (i) oss << ' ';
    oss << std::setw(2) << static_cast<unsigned>(data[i]);
  }
  return oss.str();
}

void DtgCanSender::on_control_cmd(const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_control_cmd_ = msg;
  RCLCPP_DEBUG(this->get_logger(), "RX control: steer=%.3f rad, speed=%.3f m/s, accel=%.3f m/s^2",
               msg->lateral.steering_tire_angle, msg->longitudinal.velocity, msg->longitudinal.acceleration);
}

void DtgCanSender::on_gear_cmd(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_gear_cmd_ = msg;
  RCLCPP_DEBUG(this->get_logger(), "RX gear: %u", msg->command);
}

void DtgCanSender::on_turn_indicators_cmd(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_turn_indicators_cmd_ = msg;
  RCLCPP_DEBUG(this->get_logger(), "RX turn: %u", msg->command);
}

void DtgCanSender::on_operation_mode(const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  is_autonomous_ =
    (msg->mode == autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS);
  latest_operation_mode_ = msg;
  RCLCPP_DEBUG(this->get_logger(), "RX op_mode: %u (auto=%d)",
              msg->mode, is_autonomous_);
}

void DtgCanSender::on_velocity_status(
  const autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  latest_velocity_status_ = msg;
}

void DtgCanSender::on_steering_status(const autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_steering_status_ = msg;
}
void DtgCanSender::on_gear_status(const autoware_vehicle_msgs::msg::GearReport::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_gear_status_ = msg;
}
void DtgCanSender::on_engine_rpm(const std_msgs::msg::Float32::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_rpm_ = msg;
}
void DtgCanSender::on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  const rclcpp::Time t{msg->header.stamp};
  const double vx = msg->twist.twist.linear.x;
  const double vy = msg->twist.twist.linear.y;
  const double wz = msg->twist.twist.angular.z;

  // Yaw 계산 (쿼터니언 일반식)
  const auto & q = msg->pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double yaw_rad = std::atan2(siny_cosp, cosy_cosp);
  double yaw_deg = yaw_rad * 180.0 / M_PI;
  if (yaw_deg < 0.0) yaw_deg += 360.0;
  latest_yaw_deg_ = yaw_deg;

  if (has_prev_odom_) {
    const double dt = (t - prev_odom_stamp_).seconds();
    if (dt > 1e-3) {
      last_ax_ = (vx - prev_vx_) / dt;
      last_ay_ = vx * wz;  // vx*wz 로 측방 가속 근사
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Skip accel update: dt=%.6f", dt);
    }
  }
  prev_vx_ = vx;
  prev_vy_ = vy;
  prev_odom_stamp_ = t;
  has_prev_odom_ = true;
  latest_odom_ = msg;
}

void DtgCanSender::on_actuation_status(const tier4_vehicle_msgs::msg::ActuationStatusStamped::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_actuation_status_ = msg;
}

void DtgCanSender::on_timer()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!latest_operation_mode_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "No operation_mode received");
  }
  if (!latest_control_cmd_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "No control_cmd received");
  }
  // 0x201 재도입: accel/brake normalized status
  encode_and_publish_0x200();
  encode_and_publish_0x201();
  encode_and_publish_0x100();
  encode_and_publish_0x400();
}

void DtgCanSender::encode_and_publish_0x200()
{
  // 입력 없으면 기본값 사용
  const bool auto_on = latest_operation_mode_
    ? (latest_operation_mode_->mode == autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS)
    : false;
  const double steer_rad = latest_control_cmd_ ? latest_control_cmd_->lateral.steering_tire_angle : 0.0;
  const double speed_ms  = latest_control_cmd_ ? latest_control_cmd_->longitudinal.velocity : 0.0;
  const double accel     = latest_control_cmd_ ? latest_control_cmd_->longitudinal.acceleration : 0.0;

  can_msgs::msg::Frame frame;
  frame.header.stamp = this->now();
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;
  frame.dlc = 8;
  frame.id = 0x200;
  std::fill(frame.data.begin(), frame.data.end(), 0);

  // byte0: 상위 nibble 자율 상태(0x10 or 0x00). 하위 nibble 미사용.
  frame.data[0] = static_cast<uint8_t>((auto_on ? 0x01 : 0x00) << 4);

  constexpr double kPi = 3.14159265358979323846;
  const double steering_deg = steer_rad * 180.0 / kPi;
  const double steering_deg_clamped = clamp(steering_deg, -2048.0, 2047.9375);
  const uint16_t steering_raw =
       static_cast<uint16_t>(std::lround((steering_deg_clamped - (-2048.0)) / 0.0625));
  frame.data[1] = static_cast<uint8_t>(steering_raw & 0xFF);
  frame.data[2] = static_cast<uint8_t>((steering_raw >> 8) & 0xFF);

  const double speed_kmh = speed_ms * 3.6;
  const double speed_kmh_clamped = clamp(speed_kmh, 0.0, 250.996);
  const uint16_t speed_raw = static_cast<uint16_t>(std::lround(speed_kmh_clamped / 0.00390625));
  frame.data[3] = static_cast<uint8_t>(speed_raw & 0xFF);
  frame.data[4] = static_cast<uint8_t>((speed_raw >> 8) & 0xFF);

  const double accel_clamped = clamp(accel, -5.0, 3.0);
  const uint16_t accel_raw =
       static_cast<uint16_t>(std::lround((accel_clamped - (-5.0)) / 0.01));
  frame.data[5] = static_cast<uint8_t>(accel_raw & 0xFF);
  frame.data[6] = static_cast<uint8_t>((accel_raw >> 8) & 0xFF);

  // byte7: 가속 페달(0~255) 유지
  uint8_t accel_pedal = 0;
  if (accel > 0.0) {
    accel_pedal = static_cast<uint8_t>(clamp(std::lround((accel / 3.0) * 255.0), 0.0, 255.0));
  }
  frame.data[7] = accel_pedal;

  can_pub_->publish(frame);
  RCLCPP_DEBUG(this->get_logger(), "TX 0x%03X [%u] %s", static_cast<unsigned>(frame.id), frame.dlc,
               dump_bytes(frame.data, frame.dlc).c_str());
}

void DtgCanSender::encode_and_publish_0x201() {
  // ActuationStatus: accel_status (0~127), brake_status (0~4000)
  uint8_t accel_norm = 0;
  uint8_t brake_norm = 0;
  if (latest_actuation_status_) {
    const double a = latest_actuation_status_->status.accel_status; // 기대 범위 0~127
    const double b = latest_actuation_status_->status.brake_status; // 기대 범위 0~4000
    if (a > 0.0) {
      accel_norm = static_cast<uint8_t>(clamp(std::lround(a * (255.0 / 127.0)), 0.0, 255.0));
    }
    if (b > 0.0) {
      brake_norm = static_cast<uint8_t>(clamp(std::lround(b * (255.0 / 4000.0)), 0.0, 255.0));
    }
  }

  can_msgs::msg::Frame frame;
  frame.header.stamp = this->now();
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;
  frame.dlc = 8;
  frame.id = 0x201;
  std::fill(frame.data.begin(), frame.data.end(), 0);

  // Layout 제안: byte0=accel_norm, byte1=brake_norm, 나머지 0 (확장 여지)
  frame.data[0] = accel_norm;
  frame.data[1] = brake_norm;

  can_pub_->publish(frame);
  RCLCPP_DEBUG(this->get_logger(), "TX 0x201 accel_norm=%u brake_norm=%u bytes=%s",
               accel_norm, brake_norm, dump_bytes(frame.data, frame.dlc).c_str());
}

void DtgCanSender::encode_and_publish_0x100()
{
  // speed [km/h], prefer status else fallback to control
  double speed_kmh = 0.0;
  if (latest_velocity_status_) {
    speed_kmh = latest_velocity_status_->longitudinal_velocity * 3.6;
  } else if (latest_control_cmd_) {
    speed_kmh = latest_control_cmd_->longitudinal.velocity * 3.6;
  }
  const double speed_kmh_clamped = clamp(speed_kmh, 0.0, 250.996);
  const uint16_t speed_raw = static_cast<uint16_t>(std::lround(speed_kmh_clamped / 0.00390625));

  // steering [deg], prefer status else fallback to control
  constexpr double kPi = 3.14159265358979323846;
  double steering_deg = 0.0;
  if (latest_steering_status_) {
    steering_deg = latest_steering_status_->steering_tire_angle * 180.0 / kPi;
  } else if (latest_control_cmd_) {
    steering_deg = latest_control_cmd_->lateral.steering_tire_angle * 180.0 / kPi;
  }
  const double steering_deg_clamped = clamp(steering_deg, -2048.0, 2047.9375);
  const uint16_t steering_raw =
      static_cast<uint16_t>(std::lround((steering_deg_clamped - (-2048.0)) / 0.0625));

  // engine rpm, default 0 if missing
  double rpm = 0.0;
  if (latest_rpm_) rpm = latest_rpm_->data;
  const double rpm_clamped = clamp(rpm, 0.0, 8031.875);
  const uint16_t rpm_raw = static_cast<uint16_t>(std::lround(rpm_clamped / 0.125));

  // gear byte from status
  uint8_t gear_byte = 0x00;
  if (latest_gear_status_) gear_byte = map_gear_to_dbc(latest_gear_status_->report);

  can_msgs::msg::Frame frame;
  frame.header.stamp = this->now();
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;
  frame.dlc = 8;
  frame.id = 0x100;
  std::fill(frame.data.begin(), frame.data.end(), 0);

  // LE packing
  frame.data[0] = static_cast<uint8_t>(speed_raw & 0xFF);
  frame.data[1] = static_cast<uint8_t>((speed_raw >> 8) & 0xFF);
  frame.data[2] = static_cast<uint8_t>(rpm_raw & 0xFF);
  frame.data[3] = static_cast<uint8_t>((rpm_raw >> 8) & 0xFF);
  frame.data[4] = static_cast<uint8_t>(steering_raw & 0xFF);
  frame.data[5] = static_cast<uint8_t>((steering_raw >> 8) & 0xFF);
  frame.data[6] = gear_byte;  // Gear Pos
  frame.data[7] = 0x00;       // reserved

  can_pub_->publish(frame);
  RCLCPP_DEBUG(this->get_logger(), "TX 0x%03X [%u] %s", static_cast<unsigned>(frame.id), frame.dlc,
               dump_bytes(frame.data, frame.dlc).c_str());
}

void DtgCanSender::encode_and_publish_0x400()
{
  // yaw / accel 계산은 기존 값 사용, altitude 는 항상 0
  uint16_t altitude_raw = 0; // 고정 0

  // Yaw (0~359.99) → 0.01 deg LSB, U16
  double yaw_deg = latest_yaw_deg_;
  if (yaw_deg < 0.0) yaw_deg += 360.0;
  if (yaw_deg >= 360.0) yaw_deg -= 360.0;
  uint16_t yaw_raw = static_cast<uint16_t>(std::lround(clamp(yaw_deg, 0.0, 359.99) / 0.01));

  int16_t ax_raw = static_cast<int16_t>(std::lround(clamp(last_ax_, -20.0, 20.0) / 0.01));
  int16_t ay_raw = static_cast<int16_t>(std::lround(clamp(last_ay_, -20.0, 20.0) / 0.01));

  can_msgs::msg::Frame frame;
  frame.is_extended = false;
  frame.id = 0x400;
  frame.dlc = 8;
  std::fill(frame.data.begin(), frame.data.end(), 0);

  frame.data[0] = static_cast<uint8_t>(altitude_raw & 0xFF);
  frame.data[1] = static_cast<uint8_t>((altitude_raw >> 8) & 0xFF);
  frame.data[2] = static_cast<uint8_t>(yaw_raw & 0xFF);
  frame.data[3] = static_cast<uint8_t>((yaw_raw >> 8) & 0xFF);
  frame.data[4] = static_cast<uint8_t>(ax_raw & 0xFF);
  frame.data[5] = static_cast<uint8_t>((ax_raw >> 8) & 0xFF);
  frame.data[6] = static_cast<uint8_t>(ay_raw & 0xFF);
  frame.data[7] = static_cast<uint8_t>((ay_raw >> 8) & 0xFF);

  can_pub_->publish(frame);
  RCLCPP_DEBUG(this->get_logger(), "TX 0x400 Alt=0 yaw=%.2f ax=%.2f ay=%.2f bytes=%s",
               yaw_deg, last_ax_, last_ay_, dump_bytes(frame.data, frame.dlc).c_str());
}

} // namespace dtg_can_sender

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dtg_can_sender::DtgCanSender)