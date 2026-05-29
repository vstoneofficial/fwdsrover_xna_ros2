#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class PubOdomPoseNode : public rclcpp::Node
{
public:
  PubOdomPoseNode()
  : Node("odometry_pose_publisher")
  {
    pose_topic_ = declare_parameter<std::string>("pose_topic", "rover_odom_pose");
    twist_topic_ = declare_parameter<std::string>("twist_topic", "rover_odom_twist");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::SensorDataQoS(), std::bind(&PubOdomPoseNode::pose_callback, this, _1));
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_topic_, rclcpp::SensorDataQoS(), std::bind(&PubOdomPoseNode::twist_callback, this, _1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto stamp = get_clock()->now();

    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose = msg->pose;
    if (has_twist_) {
      odom.twist.twist = latest_twist_;
    }

    odom_pub_->publish(odom);

    if (!publish_tf_) {
      return;
    }

    auto transform = geometry_msgs::msg::TransformStamped();
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = msg->pose.position.x;
    transform.transform.translation.y = msg->pose.position.y;
    transform.transform.translation.z = msg->pose.position.z;
    transform.transform.rotation = msg->pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    latest_twist_ = msg->twist;
    has_twist_ = true;
  }

  std::string pose_topic_;
  std::string twist_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_ = true;
  bool has_twist_ = false;

  geometry_msgs::msg::Twist latest_twist_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubOdomPoseNode>());
  rclcpp::shutdown();
  return 0;
}
