#ifndef MAVROS_BRIDGE_COMPONENT_HPP_
#define MAVROS_BRIDGE_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/msg/companion_process_status.hpp"
#include "mavros_msgs/msg/state.hpp"
#include <deque>
#include <mutex>

namespace uosm
{
  namespace mavros
  {

    class MavrosBridgeComponent : public rclcpp::Node
    {
    public:
      explicit MavrosBridgeComponent(const rclcpp::NodeOptions &options);

    private:
      void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
      void publish_status();
      void publish_buffered_odom();

      // Subscription and publishers
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
      rclcpp::Publisher<mavros_msgs::msg::CompanionProcessStatus>::SharedPtr mavros_system_status_pub_;

      // Timers
      rclcpp::TimerBase::SharedPtr status_timer_;
      rclcpp::TimerBase::SharedPtr odom_publish_timer_;

      // Frames
      std::string odom_frame_;
      std::string base_frame_;
      std::string odom_topic_;

      // Tracking state
      std::deque<double> covariance_window_;
      int window_size_;
      uint8_t system_status_; // Use uint8_t for MAV_STATE enums
      rclcpp::Time last_callback_time;

      // Buffering for consistent publishing rate
      std::deque<std::unique_ptr<nav_msgs::msg::Odometry>> odom_buffer_;
      std::mutex buffer_mutex_;
      double publish_rate_hz_;
      int buffer_max_size_;
    };

  } // namespace mavros
} // namespace uosm

#endif // MAVROS_BRIDGE_COMPONENT_HPP_