#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <algorithm>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class RoverSimNode : public rclcpp::Node
{
public:
  RoverSimNode()
  : Node("rover_sim_node")
  {
    publish_rate_hz_     = this->declare_parameter<double>("publish_rate", 30.0);
    rover_d              = this->declare_parameter<double>("rover_d",   0.125);  // 半トレッド（y+左）
    rover_hb             = this->declare_parameter<double>("rover_hb",  0.125);  // ホイールベース
    wheel_radius         = this->declare_parameter<double>("wheel_radius", 0.070);
    steer_time_constant  = this->declare_parameter<double>("steer_tau", 0.08);
    wheel_time_constant  = this->declare_parameter<double>("wheel_tau", 0.05);

    deadband_linear_     = this->declare_parameter<double>("deadband_linear", 0.01);
    deadband_angular_    = this->declare_parameter<double>("deadband_angular", 0.01);
    precision_digits_    = this->declare_parameter<int>("precision_digits", 2);

    use_cmd_angular_for_odo = this->declare_parameter<bool>("use_cmd_angular_for_odo", true);
    use_cmd_linear_for_odo  = this->declare_parameter<bool>("use_cmd_linear_for_odo", true);
    use_cmd_always          = this->declare_parameter<bool>("use_cmd_always", true);
    max_allowed_angular     = this->declare_parameter<double>("max_allowed_angular", 10.0);
    cmd_freshness_s         = this->declare_parameter<double>("cmd_freshness_s", 0.5);

    // モード: normal / odo_driven
    std::string operation_mode = this->declare_parameter<std::string>("operation_mode", "normal");
    std::string mode_alias     = this->declare_parameter<std::string>("mode", operation_mode);
    if (mode_alias == "odo" || mode_alias == "odo_driven" || mode_alias == "real_sim" || mode_alias == "real+sim")
      mode_ = Mode::ODO_DRIVEN;
    else
      mode_ = Mode::NORMAL;

    // 出力可否（既定はモードに応じて）
    publish_odo_enabled_    = this->declare_parameter<bool>("publish_odo",    mode_ == Mode::NORMAL);
    publish_sensor_enabled_ = this->declare_parameter<bool>("publish_sensor", mode_ == Mode::NORMAL);

    steer_limit_deg_ = this->declare_parameter<double>("steer_limit_deg", 95.0);
    steer_limit_rad_ = steer_limit_deg_ * M_PI / 180.0;

    steer_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "steer_joint_names",
      std::vector<std::string>{"front_left_steering_joint","front_right_steering_joint","back_right_steering_joint","back_left_steering_joint"});
    sus_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "sus_joint_names",
      std::vector<std::string>{"front_left_sus_joint","front_right_sus_joint","back_right_sus_joint","back_left_sus_joint"});
    wheel_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "wheel_joint_names",
      std::vector<std::string>{"front_left_wheel_joint","front_right_wheel_joint","back_right_wheel_joint","back_left_wheel_joint"});

    joint_states_topic_ = this->declare_parameter<std::string>("joint_states_topic", "/rover_sim/joint_states");
    cmd_topic           = this->declare_parameter<std::string>("cmd_topic", "rover_twist");
    odo_topic           = this->declare_parameter<std::string>("odo_topic", "rover_odo");
    sensor_topic        = this->declare_parameter<std::string>("sensor_topic", "rover_sensor");
    steer_cmd_topic_    = this->declare_parameter<std::string>("steer_cmd_topic", "/steer_position_controller/commands");
    wheel_cmd_topic_    = this->declare_parameter<std::string>("wheel_cmd_topic", "/wheel_velocity_controller/commands");

    timer_period_ = std::chrono::duration<double>(1.0 / publish_rate_hz_);

    F_R = 0; F_L = 1; R_L = 2; R_R = 3;

    for (int i=0;i<4;++i) {
      avr_v[i] = 0.0;
      steer_pos_now[i] = 0.0;
      steer_center[i]  = 0.0;
      wheel_angle[i]   = 0.0;
    }

    auto qos_cmd = rclcpp::QoS(10).best_effort();
    auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort();

    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic, qos_cmd, std::bind(&RoverSimNode::twistCallback, this, std::placeholders::_1));

    if (mode_ == Mode::ODO_DRIVEN) {
      sub_odo_ = this->create_subscription<geometry_msgs::msg::Twist>(
        odo_topic, qos_cmd, std::bind(&RoverSimNode::odoCallback, this, std::placeholders::_1));
    }

    if (publish_odo_enabled_) {
      pub_odo_ = this->create_publisher<geometry_msgs::msg::Twist>(odo_topic, rclcpp::QoS(10).best_effort());
    }
    if (publish_sensor_enabled_) {
      pub_sensor_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(sensor_topic, rclcpp::QoS(10).best_effort());
    }
    pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, qos_pub);
    pub_steer_cmd_    = this->create_publisher<std_msgs::msg::Float64MultiArray>(steer_cmd_topic_, rclcpp::QoS(10).best_effort());
    pub_wheel_cmd_    = this->create_publisher<std_msgs::msg::Float64MultiArray>(wheel_cmd_topic_, rclcpp::QoS(10).best_effort());

    last_cmd_time_ = this->now();

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(timer_period_),
      std::bind(&RoverSimNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(),
      "mode=%s (publish_odo=%d, publish_sensor=%d)",
      (mode_==Mode::NORMAL?"normal":"odo_driven"),
      publish_odo_enabled_, publish_sensor_enabled_);

    RCLCPP_INFO(this->get_logger(),
      "rover_sim_node started. joint_states='%s' steer_limit=±%.1f deg",
      joint_states_topic_.c_str(), steer_limit_deg_);
  }

private:
  enum class Mode { NORMAL=0, ODO_DRIVEN=1 };
  Mode mode_{Mode::NORMAL};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_odo_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_odo_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_sensor_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_steer_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_wheel_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  double publish_rate_hz_{200.0};
  std::chrono::duration<double> timer_period_{std::chrono::duration<double>(0.005)};
  double rover_d{0.125}, rover_hb{0.125}, wheel_radius{0.085};
  double steer_time_constant{0.08}, wheel_time_constant{0.05};
  double deadband_linear_{0.01}, deadband_angular_{0.01};
  int    precision_digits_{2};
  bool   use_cmd_angular_for_odo{true}, use_cmd_linear_for_odo{true}, use_cmd_always{true};
  double max_allowed_angular{10.0};
  double cmd_freshness_s{0.5};
  bool   publish_odo_enabled_{true}, publish_sensor_enabled_{true};

  double steer_limit_deg_{95.0};
  double steer_limit_rad_{M_PI * (95.0/180.0)};

  int F_R{0}, F_L{1}, R_L{2}, R_R{3};

  double avr_v[4]{};
  double steer_pos_now[4]{};
  double steer_center[4]{};
  double wheel_angle[4]{};

  std::vector<std::string> steer_joint_names_, sus_joint_names_, wheel_joint_names_;
  std::string cmd_topic, odo_topic, sensor_topic;
  std::string joint_states_topic_;
  std::string steer_cmd_topic_, wheel_cmd_topic_;

  geometry_msgs::msg::Twist::SharedPtr last_cmd_;
  geometry_msgs::msg::Twist::SharedPtr last_odo_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_odo_time_;

  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_cmd_ = msg;
    last_cmd_time_ = this->now();
  }
  void odoCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_odo_ = msg;
    last_odo_time_ = this->now();
  }

  void onTimer() {
    rclcpp::Time now = this->now();
    static rclcpp::Time prev = now;
    double dt = (now - prev).seconds();
    if (dt <= 0.0) dt = 1.0 / publish_rate_hz_;
    prev = now;

    double cmd_vx=0, cmd_vy=0, cmd_w=0;
    bool have_cmd=false;

    if (mode_ == Mode::ODO_DRIVEN) {
      if (last_odo_) {
        cmd_vx = last_odo_->linear.x;
        cmd_vy = last_odo_->linear.y;
        cmd_w  = last_odo_->angular.z;
        have_cmd = true;
      }
    } else { 
      if (last_cmd_) {
        cmd_vx = last_cmd_->linear.x;
        cmd_vy = last_cmd_->linear.y;
        cmd_w  = last_cmd_->angular.z;
        if (use_cmd_always || (now - last_cmd_time_).seconds() <= cmd_freshness_s) have_cmd=true;
      }
    }

    double pos_x[4], pos_y[4];
    pos_x[F_R] = +rover_hb/2.0; pos_y[F_R] = -rover_d;
    pos_x[F_L] = +rover_hb/2.0; pos_y[F_L] = +rover_d;
    pos_x[R_L] = -rover_hb/2.0; pos_y[R_L] = +rover_d;
    pos_x[R_R] = -rover_hb/2.0; pos_y[R_R] = -rover_d;

    double target_avr_v[4]{}, target_steer[4]{};
    for (int i=0;i<4;++i){
      const double rot_gain = ( (rover_hb*0.5) + rover_d ) / std::sqrt( (rover_hb*0.5)*(rover_hb*0.5) + rover_d*rover_d );
      const double vwx = cmd_vx - (rot_gain * 1.5 * cmd_w) * pos_y[i];
      const double vwy = cmd_vy + (rot_gain * 1.5 * cmd_w) * pos_x[i];
      const double speed = std::hypot(vwx, vwy);
      double dir = (speed > 1e-6) ? std::atan2(vwy, vwx)
                                  : std::atan2(-pos_x[i], pos_y[i]); 
      target_avr_v[i] = speed;
      target_steer[i] = dir;
    }

    for (int i=0;i<4;++i){
      double s = target_steer[i];
      double v = target_avr_v[i];
      normalize_with_limit_and_speed_(steer_pos_now[i], steer_center[i], s, steer_limit_rad_, &s, &v);
      const double alpha_s = 1.0 - std::exp(-dt / std::max(1e-6, steer_time_constant));
      const double alpha_w = 1.0 - std::exp(-dt / std::max(1e-6, wheel_time_constant));
      steer_pos_now[i] = stepTowardAngle(steer_pos_now[i], s, alpha_s);
      avr_v[i]         = avr_v[i] + alpha_w * (v - avr_v[i]);
    }


    for (int i=0;i<4;++i) {
      wheel_angle[i] += (avr_v[i] / std::max(1e-9, wheel_radius)) * dt;
    }

    if (pub_odo_ || pub_sensor_) {
      double msg_vx=0, msg_vy=0, msg_az=0;
      setRoverOdo(&msg_vx, &msg_vy, &msg_az);

      double chosen_vx = msg_vx;
      double chosen_az = msg_az;
      if (have_cmd && use_cmd_linear_for_odo)  chosen_vx = cmd_vx;
      if (have_cmd && use_cmd_angular_for_odo) chosen_az = cmd_w;

      double vx_out = applyDeadbandAndRound(chosen_vx, deadband_linear_,  precision_digits_);
      double vy_out = applyDeadbandAndRound(msg_vy,     deadband_linear_,  precision_digits_);
      double az_out = applyDeadbandAndRound(chosen_az,  deadband_angular_, precision_digits_);
      if (!std::isfinite(az_out) || std::fabs(az_out) > max_allowed_angular) {
        az_out = std::copysign(std::min(std::fabs(az_out), max_allowed_angular), az_out);
      }

      if (pub_odo_) {
        geometry_msgs::msg::Twist odo_msg;
        odo_msg.linear.x = vx_out; odo_msg.linear.y = vy_out; odo_msg.angular.z = az_out;
        pub_odo_->publish(odo_msg);
      }

      if (pub_sensor_) {
        std_msgs::msg::Int16MultiArray sensor_msg;
        sensor_msg.data.resize(8);
        for (int i=0;i<4;++i) sensor_msg.data[i] = clamp_to_int16(static_cast<int32_t>(std::round(avr_v[i]*100.0)));
        for (int i=0;i<4;++i) {
          double deg = steer_pos_now[i] * 180.0 / M_PI;
          sensor_msg.data[4+i] = clamp_to_int16(static_cast<int32_t>(std::round(deg)));
        }
        pub_sensor_->publish(sensor_msg);
      }
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    const int urdf_corner_idx[4] = { /*FL=*/1, /*FR=*/0, /*BR=*/3, /*BL=*/2 };
    for (int c=0;c<4;++c) {
      int idx = urdf_corner_idx[c];
      // steer
      const std::string &steer_nm = (c < (int)steer_joint_names_.size()) ? steer_joint_names_[c] : ("steer_" + std::to_string(c));
      js.name.push_back(steer_nm);
      js.position.push_back(steer_pos_now[idx]);
      js.velocity.push_back(0.0);
      // sus
      const std::string &sus_nm = (c < (int)sus_joint_names_.size()) ? sus_joint_names_[c] : ("sus_" + std::to_string(c));
      js.name.push_back(sus_nm);
      js.position.push_back(0.0);
      js.velocity.push_back(0.0);
      // wheel
      const std::string &wheel_nm = (c < (int)wheel_joint_names_.size()) ? wheel_joint_names_[c] : ("wheel_" + std::to_string(c));
      js.name.push_back(wheel_nm);
      js.position.push_back(wheel_angle[idx]);
      double wheel_ang_vel = avr_v[idx] / std::max(1e-9, wheel_radius);
      if (std::fabs(wheel_ang_vel) < 1e-8) wheel_ang_vel = 0.0;
      js.velocity.push_back(wheel_ang_vel);
    }
    pub_joint_states_->publish(js);

    {
      std_msgs::msg::Float64MultiArray steer_cmd;
      steer_cmd.data.resize(4);
      int map_idx[4] = { /*FL=*/F_L, /*FR=*/F_R, /*BR=*/R_R, /*BL=*/R_L };
      for (int i=0;i<4;++i) steer_cmd.data[i] = steer_pos_now[ map_idx[i] ];
      pub_steer_cmd_->publish(steer_cmd);
    }

    {
      std_msgs::msg::Float64MultiArray wheel_cmd;
      wheel_cmd.data.resize(4);
      int map_idx[4] = { /*FL=*/F_L, /*FR=*/F_R, /*BR=*/R_R, /*BL=*/R_L };
      for (int i=0;i<4;++i) {
        double w = avr_v[ map_idx[i] ] / std::max(1e-9, wheel_radius);
        wheel_cmd.data[i] = (std::fabs(w) < 1e-8) ? 0.0 : w;
      }
      pub_wheel_cmd_->publish(wheel_cmd);
    }
  }

  static inline int16_t clamp_to_int16(int32_t v) {
    if (v >  32767) return 32767;
    if (v < -32768) return -32768;
    return static_cast<int16_t>(v);
  }

  static void normalize_with_limit_and_speed_(
      double current_angle, double center_angle,
      double target_angle, double limit_rad,
      double *out_angle, double *inout_speed)
  {
    auto wrap_to_near = [](double target, double current){
      while (target - current >  M_PI) target -= 2.0*M_PI;
      while (target - current < -M_PI) target += 2.0*M_PI;
      return target;
    };
    auto signed_diff = [&](double ang){
      double d = ang - center_angle;
      while (d >  M_PI) d -= 2.0*M_PI;
      while (d < -M_PI) d += 2.0*M_PI;
      return d;
    };
    auto absdiff = [&](double ang){ return std::fabs(signed_diff(ang)); };

    double a = wrap_to_near(target_angle, current_angle);
    double b = wrap_to_near(target_angle + M_PI, current_angle);

    double use_ang = a;
    double use_v   = *inout_speed;
    if (absdiff(b) < absdiff(a)) { use_ang = b; use_v = -*inout_speed; }

    double d = signed_diff(use_ang);
    if (d >  limit_rad)  use_ang = center_angle + limit_rad;
    if (d < -limit_rad)  use_ang = center_angle - limit_rad;

    *out_angle   = use_ang;
    *inout_speed = use_v;
  }

  static double normalizeAngle(double a) {
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a <= -M_PI) a += 2.0*M_PI;
    return a;
  }

  static double stepTowardAngle(double cur, double target, double alpha) {
    cur = normalizeAngle(cur); target = normalizeAngle(target);
    double diff = target - cur;
    if (diff >  M_PI) diff -= 2.0*M_PI;
    if (diff < -M_PI) diff += 2.0*M_PI;
    return normalizeAngle(cur + alpha * diff);
  }

  static double applyDeadbandAndRound(double v, double db, int digits){
    if (std::fabs(v) < db) return 0.0;
    double s = std::pow(10.0, digits);
    return std::round(v*s)/s;
  }

  void setRoverOdo(double *msg_vx, double *msg_vy, double *msg_az) {
    double steer_rad_w[4];
    steer_rad_w[F_R] = -(steer_pos_now[F_R] - steer_center[F_R] - (M_PI/4.0));
    steer_rad_w[F_L] = -(steer_pos_now[F_L] - steer_center[F_L] + (M_PI/4.0));
    steer_rad_w[R_L] = -(steer_pos_now[R_L] - steer_center[R_L] - (M_PI/4.0));
    steer_rad_w[R_R] = -(steer_pos_now[R_R] - steer_center[R_R] + (M_PI/4.0));
    for (int i=0;i<4;++i) while (steer_rad_w[i] < 0.0) steer_rad_w[i] += M_PI*2.0;

    double wheel_vx[4]{}, wheel_vy[4]{};
    for (int i=0;i<4;++i) {
      wheel_vx[i] = avr_v[i] * std::cos(steer_rad_w[i]);
      wheel_vy[i] = avr_v[i] * std::sin(steer_rad_w[i]);
    }
    
    for (int i=1;i<3;++i) { wheel_vx[i] = -wheel_vx[i]; wheel_vy[i] = -wheel_vy[i]; }

    double rover_vx=0, rover_vy=0;
    for (int i=0;i<4;++i) { rover_vx += wheel_vx[i]; rover_vy += wheel_vy[i]; }
    rover_vx /= 4.0; rover_vy /= 4.0;
    *msg_vx = rover_vx; *msg_vy = rover_vy;

    auto angDiffOK = [&](double a, double b)->bool{
      double d = std::fabs(a-b);
      if (std::fabs(d - M_PI*2.0) < 1e-6) d = 0.0;
      return (std::fabs(a-b) < 0.03 || std::fabs(std::fabs(a-b) - M_PI*2.0) < 0.03);
    };

    if ( (angDiffOK(steer_rad_w[F_R], steer_rad_w[R_L])) &&
         (angDiffOK(steer_rad_w[F_L], steer_rad_w[R_R])) ) {
      int run_mode_odo = 0;
      if (std::fabs(steer_pos_now[F_R]-steer_center[F_R]) < 0.01 &&
          std::fabs(steer_pos_now[F_L]-steer_center[F_L]) < 0.01 &&
          std::fabs(steer_pos_now[R_L]-steer_center[R_L]) < 0.01 &&
          std::fabs(steer_pos_now[R_R]-steer_center[R_R]) < 0.01) run_mode_odo = 0;
      else if (std::fabs(rover_vx) >= std::fabs(rover_vy)) run_mode_odo = 1;
      else run_mode_odo = 2;

      double v_l=0, v_r=0;
      if (run_mode_odo == 0) {
        *msg_az = ((avr_v[F_R] + avr_v[F_L] + avr_v[R_L] + avr_v[R_R]) / 4.0) /
                  (std::sqrt(rover_d*rover_d + rover_hb*rover_hb) + 1e-9);
      } else if (run_mode_odo == 1) {
        v_l = (avr_v[F_L] + avr_v[R_L]) / 2.0;
        v_r = (avr_v[F_R] + avr_v[R_R]) / 2.0;
        *msg_az = ((v_r - (-1.0 * v_l)) / (2.0 * rover_d + 1e-9));
      } else {
        v_l = (avr_v[R_L] + avr_v[R_R]) / 2.0;
        v_r = (avr_v[F_L] + avr_v[F_R]) / 2.0;
        *msg_az = ((v_r - (-1.0 * v_l)) / (2.0 * rover_hb + 1e-9));
      }
    } else {
      double steer_rad_normal_w[4];
      steer_rad_normal_w[F_R] = steer_rad_w[F_R] - M_PI/2.0;
      steer_rad_normal_w[F_L] = steer_rad_w[F_L] + M_PI/2.0;
      steer_rad_normal_w[R_L] = steer_rad_w[R_L] + M_PI/2.0;
      steer_rad_normal_w[R_R] = steer_rad_w[R_R] - M_PI/2.0;
      for (int i=0;i<4;++i) {
        while (steer_rad_normal_w[i] < 0.0) steer_rad_normal_w[i] += M_PI;
        while (steer_rad_normal_w[i] > M_PI) steer_rad_normal_w[i] -= M_PI;
      }

      bool is_l_can_cal=false, is_r_can_cal=false;
      double rad_rc=0.0;
      double rad_nsx[4]{}, d_str2center[4]{}, cal_omega[4]{};

      if (std::fabs(steer_rad_normal_w[F_L] - steer_rad_normal_w[R_L]) >= 0.03) {
        is_l_can_cal = true;
        rad_rc = std::fabs(steer_rad_normal_w[F_L] - steer_rad_normal_w[R_L]);
        double sin_rc = std::sin(rad_rc);
        if (std::fabs(sin_rc) < 1e-6) { *msg_az = 0.0; goto finalize; }
        if (steer_rad_normal_w[F_L] > steer_rad_normal_w[R_L]) {
          rad_nsx[F_L] = M_PI - steer_rad_normal_w[F_L];
          rad_nsx[R_L] = steer_rad_normal_w[R_L];
        } else {
          rad_nsx[F_L] = steer_rad_normal_w[F_L];
          rad_nsx[R_L] = M_PI - steer_rad_normal_w[R_L];
        }
        d_str2center[F_L] = ((2.0*rover_d)/sin_rc)*std::sin(rad_nsx[R_L]);
        d_str2center[R_L] = ((2.0*rover_d)/sin_rc)*std::sin(rad_nsx[F_L]);
        cal_omega[F_L] = std::fabs(avr_v[F_L]/(d_str2center[F_L] + 1e-9));
        cal_omega[R_L] = std::fabs(avr_v[R_L]/(d_str2center[R_L] + 1e-9));
      }

      if (std::fabs(steer_rad_normal_w[F_R] - steer_rad_normal_w[R_R]) >= 0.03) {
        is_r_can_cal = true;
        rad_rc = std::fabs(steer_rad_normal_w[F_R] - steer_rad_normal_w[R_R]);
        double sin_rc = std::sin(rad_rc);
        if (std::fabs(sin_rc) < 1e-6) { *msg_az = 0.0; goto finalize; }
        if (steer_rad_normal_w[F_R] > steer_rad_normal_w[R_R]) {
          rad_nsx[F_R] = M_PI - steer_rad_normal_w[F_R];
          rad_nsx[R_R] = steer_rad_normal_w[R_R];
        } else {
          rad_nsx[F_R] = steer_rad_normal_w[F_R];
          rad_nsx[R_R] = M_PI - steer_rad_normal_w[R_R];
        }
        d_str2center[F_R] = ((2.0*rover_d)/sin_rc)*std::sin(rad_nsx[R_R]);
        d_str2center[R_R] = ((2.0*rover_d)/sin_rc)*std::sin(rad_nsx[F_R]);
        cal_omega[F_R] = std::fabs(avr_v[F_R]/(d_str2center[F_R] + 1e-9));
        cal_omega[R_R] = std::fabs(avr_v[R_R]/(d_str2center[R_R] + 1e-9));
      }

      {
        double ave_cal_omega = 0.0;
        if (is_l_can_cal && is_r_can_cal) {
          for (int i=0;i<4;++i) ave_cal_omega += cal_omega[i];
          ave_cal_omega /= 4.0;
        } else if (is_l_can_cal) {
          ave_cal_omega = (cal_omega[F_L] + cal_omega[R_L]) / 2.0;
        } else if (is_r_can_cal) {
          ave_cal_omega = (cal_omega[F_R] + cal_omega[R_R]) / 2.0;
        }

        double rad_v = 0.0;
        if (std::abs(rover_vx) > 1e-9 || std::abs(rover_vy) > 1e-9) rad_v = std::atan2(rover_vy, rover_vx);

        if (rad_v >= 0 && rad_v < M_PI/2.0) {
          if (avr_v[F_L] >= 0.0) steer_rad_w[F_L] -= M_PI;
          while (steer_rad_w[F_L] < rad_v - M_PI) steer_rad_w[F_L] += M_PI*2.0;
          while (steer_rad_w[F_L] > rad_v + M_PI) steer_rad_w[F_L] -= M_PI*2.0;
          if (rad_v - steer_rad_w[F_L] >= 0.0) ave_cal_omega *= -1.0;
        } else if (rad_v >= M_PI/2.0) {
          if (avr_v[R_L] >= 0.0) steer_rad_w[R_L] -= M_PI;
          while (steer_rad_w[R_L] < rad_v - M_PI) steer_rad_w[R_L] += M_PI*2.0;
          while (steer_rad_w[R_L] > rad_v + M_PI) steer_rad_w[R_L] -= M_PI*2.0;
          if (rad_v - steer_rad_w[R_L] >= 0.0) ave_cal_omega *= -1.0;
        } else if (rad_v < -M_PI/2.0) {
          if (avr_v[R_R] < 0.0) steer_rad_w[R_R] -= M_PI;
          while (steer_rad_w[R_R] < rad_v - M_PI) steer_rad_w[R_R] += M_PI*2.0;
          while (steer_rad_w[R_R] > rad_v + M_PI) steer_rad_w[R_R] -= M_PI*2.0;
          if (rad_v - steer_rad_w[R_R] >= 0.0) ave_cal_omega *= -1.0;
        } else {
          if (avr_v[F_R] < 0.0) steer_rad_w[F_R] -= M_PI;
          while (steer_rad_w[F_R] < rad_v - M_PI) steer_rad_w[F_R] += M_PI*2.0;
          while (steer_rad_w[F_R] > rad_v + M_PI) steer_rad_w[F_R] -= M_PI*2.0;
          if (rad_v - steer_rad_w[F_R] >= 0.0) ave_cal_omega *= -1.0;
        }

        *msg_az = ave_cal_omega;
      }
    }

  finalize:
    if (!std::isfinite(*msg_az)) *msg_az = 0.0;
    if (std::fabs(*msg_az) > max_allowed_angular) *msg_az = std::copysign(max_allowed_angular, *msg_az);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoverSimNode>());
  rclcpp::shutdown();
  return 0;
}

