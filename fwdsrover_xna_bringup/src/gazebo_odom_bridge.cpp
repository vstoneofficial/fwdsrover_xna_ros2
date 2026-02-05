#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <gazebo_msgs/msg/model_states.hpp>
#include <cmath>

class GazeboOdomBridge : public rclcpp::Node {
public:
  GazeboOdomBridge() : Node("gazebo_odom_bridge") {
    model_name_ = declare_parameter<std::string>("model_name", "x40a");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    flatten_2d_ = declare_parameter<bool>("flatten_to_2d", true);
    cov_lin_    = declare_parameter<double>("cov_linear", 1e-3);
    cov_ang_    = declare_parameter<double>("cov_angular", 1e-3);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      "/model_states", rclcpp::QoS(10),
      std::bind(&GazeboOdomBridge::cb, this, std::placeholders::_1));

  }

private:
  void cb(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
    int idx = -1;
    for (size_t i = 0; i < msg->name.size(); ++i) if (msg->name[i] == model_name_) { idx = (int)i; break; }
    if (idx < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Model '%s' not found", model_name_.c_str());
      return;
    }

    const auto &pose  = msg->pose[idx];
    const auto &twist = msg->twist[idx];

    nav_msgs::msg::Odometry od;
    od.header.stamp = now();
    od.header.frame_id = odom_frame_;
    od.child_frame_id  = base_frame_;
    od.pose.pose = pose;
    od.twist.twist = twist;

    if (flatten_2d_) {
      od.pose.pose.position.z = 0.0;
      od.twist.twist.linear.z = 0.0;
      od.twist.twist.angular.x = 0.0;
      od.twist.twist.angular.y = 0.0;
      // roll/pitchを0にしてyawだけ残す
      double siny_cosp = 2.0*(pose.orientation.w*pose.orientation.z + pose.orientation.x*pose.orientation.y);
      double cosy_cosp = 1.0 - 2.0*(pose.orientation.y*pose.orientation.y + pose.orientation.z*pose.orientation.z);
      double yaw = std::atan2(siny_cosp, cosy_cosp);
      geometry_msgs::msg::Quaternion q;
      q.x = 0.0; q.y = 0.0; q.z = std::sin(yaw*0.5); q.w = std::cos(yaw*0.5);
      od.pose.pose.orientation = q;
    }

    for (int i=0;i<36;i++){ od.pose.covariance[i]=0.0; od.twist.covariance[i]=0.0; }
    od.pose.covariance[0] = od.pose.covariance[7] = cov_lin_;
    od.pose.covariance[35]= cov_ang_;
    od.twist.covariance[0]= od.twist.covariance[7]= cov_lin_;
    od.twist.covariance[35]= cov_ang_;

    odom_pub_->publish(od);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = od.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = od.pose.pose.position.x;
      tf.transform.translation.y = od.pose.pose.position.y;
      tf.transform.translation.z = od.pose.pose.position.z;
      tf.transform.rotation      = od.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  std::string model_name_, odom_frame_, base_frame_;
  bool publish_tf_, flatten_2d_;
  double cov_lin_, cov_ang_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboOdomBridge>());
  rclcpp::shutdown();
  return 0;
}

