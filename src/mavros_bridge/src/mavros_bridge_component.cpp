#include "mavros_bridge_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <deque>
#include <numeric>
#include <mutex>

namespace uosm
{
  namespace mavros
  {

    constexpr float EMERGENCY_THRESHOLD = 1.0;
    constexpr float CRITICAL_THRESHOLD = 0.5;

    // Define MAV_STATE enum values (from mavlink common message definitions)
    enum MAV_STATE
    {
      MAV_STATE_UNINIT = 0,
      MAV_STATE_BOOT,
      MAV_STATE_CALIBRATING,
      MAV_STATE_STANDBY,
      MAV_STATE_ACTIVE,
      MAV_STATE_CRITICAL,
      MAV_STATE_EMERGENCY,
      MAV_STATE_POWEROFF,
      MAV_STATE_FLIGHT_TERMINATION
    };

    MavrosBridgeComponent::MavrosBridgeComponent(const rclcpp::NodeOptions &options)
        : Node("mavros_bridge", options),
          system_status_(MAV_STATE_UNINIT),
          window_size_(10),   // Default window size of 10 messages
          buffer_max_size_(5) // Default buffer size of 5 messages
    {
      // Declare parameters
      this->declare_parameter("odom_frame", "odom");
      this->declare_parameter("base_frame", "base_link");
      this->declare_parameter("window_size", 10);
      this->declare_parameter("publish_rate_hz", 40.0);
      this->declare_parameter("buffer_size", 5);
      this->declare_parameter("odom_topic", "/vins/odometry");

      odom_frame_ = this->get_parameter("odom_frame").as_string();
      base_frame_ = this->get_parameter("base_frame").as_string();
      odom_topic_ = this->get_parameter("odom_topic").as_string();
      window_size_ = this->get_parameter("window_size").as_int();
      publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
      buffer_max_size_ = this->get_parameter("buffer_size").as_int();

      // Initialize the window for covariance values
      covariance_window_.resize(window_size_, 0.0);

      // Create the odometry subscription and publisher
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          odom_topic_, 100, // Increased queue size to handle higher rate
          std::bind(&MavrosBridgeComponent::odom_callback, this, std::placeholders::_1));

      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
          "/mavros/odometry/out", 30);

      // Create the status publisher
      mavros_system_status_pub_ = this->create_publisher<mavros_msgs::msg::CompanionProcessStatus>(
          "/mavros/companion_process/status", 10);

      system_status_ = MAV_STATE_ACTIVE;

      // Create a timer for status publication at 1 Hz
      status_timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&MavrosBridgeComponent::publish_status, this));

      // Create a timer for consistent odometry publishing at the specified rate
      const auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
      odom_publish_timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(publish_period),
          std::bind(&MavrosBridgeComponent::publish_buffered_odom, this));

      RCLCPP_INFO(this->get_logger(), "MavrosBridgeComponent initialized with map frame: %s, base frame: %s, window size: %d",
                  odom_frame_.c_str(), base_frame_.c_str(), window_size_);
      RCLCPP_INFO(this->get_logger(), "Publishing odometry at %f Hz with buffer size %d",
                  publish_rate_hz_, buffer_max_size_);
    }

    void MavrosBridgeComponent::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      // Create new odometry message
      auto transformed_msg = std::make_unique<nav_msgs::msg::Odometry>();

      // Copy header timestamp
      transformed_msg->header.stamp = msg->header.stamp;

      // Set the correct frames that MAVROS expects
      transformed_msg->header.frame_id = odom_frame_;
      transformed_msg->child_frame_id = base_frame_;

      transformed_msg->pose = msg->pose;
      transformed_msg->twist = msg->twist;

      // Check covariance across position (indices 0, 7, 14 for x, y, z)
      double current_max_cov = std::max({msg->pose.covariance[0],
                                         msg->pose.covariance[7],
                                         msg->pose.covariance[14]});

      // Check for NaN or extremely high values immediately
      if (std::isnan(current_max_cov) || current_max_cov > 10.0)
      {
        system_status_ = MAV_STATE_FLIGHT_TERMINATION;
        RCLCPP_ERROR(this->get_logger(), "Tracking completely lost! Covariance: %f", current_max_cov);
      }
      else
      {
        // Add the current covariance to the window
        covariance_window_.pop_front();
        covariance_window_.push_back(current_max_cov);

        // Calculate average covariance in the window
        double avg_cov = std::accumulate(covariance_window_.begin(), covariance_window_.end(), 0.0) /
                         covariance_window_.size();

        // Calculate how many values in the window exceed our thresholds
        int critical_count = 0;
        int emergency_count = 0;

        for (const auto &cov : covariance_window_)
        {
          if (cov >= EMERGENCY_THRESHOLD)
            emergency_count++;
          else if (cov >= CRITICAL_THRESHOLD)
            critical_count++;
        }

        // Determine status based on window analysis
        if (emergency_count >= window_size_ * 0.3)
        { // 30% of window in emergency
          system_status_ = MAV_STATE_EMERGENCY;
          RCLCPP_ERROR(this->get_logger(), "Emergency: High covariance detected in window, avg: %f", avg_cov);
        }
        else if (critical_count >= window_size_ * 0.5)
        { // 50% of window in critical
          system_status_ = MAV_STATE_CRITICAL;
          RCLCPP_WARN(this->get_logger(), "Critical: Elevated covariance detected in window, avg: %f", avg_cov);
        }
        else
        {
          system_status_ = MAV_STATE_ACTIVE;
        }
      }

      // Store the last message timestamp
      last_callback_time = msg->header.stamp;

      // Add to buffer instead of publishing directly
      {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        odom_buffer_.push_back(std::move(transformed_msg));

        // If buffer exceeds maximum size, remove oldest messages
        while (odom_buffer_.size() > buffer_max_size_)
        {
          odom_buffer_.pop_front();
        }
      }

      RCLCPP_DEBUG(this->get_logger(), "Added odometry to buffer, current size: %ld", odom_buffer_.size());
    }

    void MavrosBridgeComponent::publish_buffered_odom()
    {
      std::unique_ptr<nav_msgs::msg::Odometry> msg_to_publish = nullptr;

      {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (!odom_buffer_.empty())
        {
          // Get the most recent message from the buffer
          msg_to_publish = std::move(odom_buffer_.back());
          odom_buffer_.clear(); // Clear the buffer after taking the most recent message
        }
      }

      if (msg_to_publish)
      {
        // Publish the most recent odometry message
        odom_pub_->publish(std::move(msg_to_publish));
        RCLCPP_DEBUG(this->get_logger(), "Published odometry at fixed rate with frame_id: %s, child_frame_id: %s",
                     odom_frame_.c_str(), base_frame_.c_str());
      }
      else
      {
        RCLCPP_DEBUG(this->get_logger(), "No odometry message available to publish");
      }
    }

    void MavrosBridgeComponent::publish_status()
    {
      auto mavros_status = std::make_unique<mavros_msgs::msg::CompanionProcessStatus>();
      mavros_status->header.stamp = last_callback_time;
      mavros_status->set__component(197U); // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
      mavros_status->set__state(static_cast<int>(system_status_));
      mavros_system_status_pub_->publish(std::move(mavros_status));
    }

  } // namespace mavros
} // namespace uosm

RCLCPP_COMPONENTS_REGISTER_NODE(uosm::mavros::MavrosBridgeComponent)