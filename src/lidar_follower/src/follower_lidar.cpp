/*
 * ENGR 857 – HRI | Spring 2026
 * Name: Anthony Delacruz
 * Filename: follower_lidar.cpp
 *
 * Description:
 * This ROS2 node implements a LIDAR-based follower behavior for the QBot.
 * The node subscribes to /scan (sensor_msgs::msg::LaserScan) and publishes
 * velocity commands to /cmd_vel (geometry_msgs::msg::Twist).
 *
 * Assignment Requirements Implemented:
 *  (a) Maintain approximately 1.0 meter from an object in front
 *  (b) Turn to keep the detected object centered
 *  (c) Stop if no valid object is detected within 2.0 meters in front
 */

#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class LidarFollower : public rclcpp::Node
{
public:
  /**
   * @brief Constructor: declares parameters and initializes publisher/subscriber.
   */
  LidarFollower()
  : Node("lidar_follower")
  {
    // -----------------------
    // Tunable Parameters
    // -----------------------

    // Desired following distance (meters)
    target_distance_m_ =
      this->declare_parameter<double>("target_distance_m", 1.0);

    // Maximum distance considered a valid object (meters)
    max_detect_distance_m_ =
      this->declare_parameter<double>("max_detect_distance_m", 2.0);

    // Half-width of front detection window (degrees)
    front_window_half_angle_deg_ =
      this->declare_parameter<double>("front_window_half_angle_deg", 20.0);

    // Proportional gains
    k_linear_  = this->declare_parameter<double>("k_linear", 0.8);
    k_angular_ = this->declare_parameter<double>("k_angular", 1.8);

    // Speed limits
    max_linear_speed_  =
      this->declare_parameter<double>("max_linear_speed", 0.35);
    max_angular_speed_ =
      this->declare_parameter<double>("max_angular_speed", 1.2);

    // Minimum number of valid scan points required
    min_valid_points_ =
      this->declare_parameter<int>("min_valid_points", 8);

    // Optional debug flag
    debug_ =
      this->declare_parameter<bool>("debug", false);

    // -----------------------
    // ROS Interfaces
    // -----------------------

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      10,
      std::bind(&LidarFollower::scan_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(),
                "lidar_follower started. Subscribing to /scan, publishing to /cmd_vel");
  }

private:

  /**
   * @brief Clamp a value between specified bounds.
   */
  static double clamp(double value, double low, double high)
  {
    return std::max(low, std::min(value, high));
  }

  /**
   * @brief Publish zero velocities to stop the robot.
   */
  void publish_stop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  /**
   * @brief LaserScan callback implementing follower logic.
   *
   * Steps:
   *  1. Define front detection window
   *  2. Accumulate valid scan points within window and distance threshold
   *  3. If insufficient points -> stop
   *  4. Compute average distance and angle
   *  5. Apply proportional control
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (msg->ranges.empty())
    {
      publish_stop();
      return;
    }

    const double half_window_rad =
      front_window_half_angle_deg_ * M_PI / 180.0;

    const double angle_min = msg->angle_min;
    const double angle_inc = msg->angle_increment;

    if (angle_inc <= 0.0)
    {
      publish_stop();
      return;
    }

    // Determine index range corresponding to front window
    int start_i = static_cast<int>(
      std::floor(((-half_window_rad) - angle_min) / angle_inc));

    int end_i = static_cast<int>(
      std::ceil(((+half_window_rad) - angle_min) / angle_inc));

    start_i = std::max(0, start_i);
    end_i   = std::min(static_cast<int>(msg->ranges.size()) - 1, end_i);

    double sum_angle = 0.0;
    double sum_dist  = 0.0;
    int valid_count  = 0;

    // -----------------------
    // Accumulate valid points
    // -----------------------
    for (int i = start_i; i <= end_i; i++)
    {
      const float r = msg->ranges[i];

      if (!std::isfinite(r)) continue;
      if (r < msg->range_min || r > msg->range_max) continue;
      if (r > max_detect_distance_m_) continue;

      const double ang = angle_min + i * angle_inc;

      sum_angle += ang;
      sum_dist  += r;
      valid_count++;
    }

    // Stop if insufficient valid detections
    if (valid_count < min_valid_points_)
    {
      publish_stop();
      return;
    }

    const double avg_angle = sum_angle / valid_count;
    const double avg_dist  = sum_dist  / valid_count;

    // -----------------------
    // Proportional Control
    // -----------------------
    const double dist_error = avg_dist - target_distance_m_;

    double linear  = k_linear_  * dist_error;
    double angular = k_angular_ * avg_angle;

    linear  = clamp(linear,  -max_linear_speed_,  max_linear_speed_);
    angular = clamp(angular, -max_angular_speed_, max_angular_speed_);

    // Small deadband to reduce jitter
    if (std::fabs(dist_error) < 0.03) linear = 0.0;
    if (std::fabs(avg_angle)  < 0.03) angular = 0.0;

    if (debug_)
    {
      RCLCPP_INFO(this->get_logger(),
                  "avg_dist: %.2f | avg_angle: %.2f | linear: %.2f | angular: %.2f",
                  avg_dist, avg_angle, linear, angular);
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = linear;
    cmd.angular.z = angular;
    cmd_pub_->publish(cmd);
  }

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  // Parameters
  double target_distance_m_;
  double max_detect_distance_m_;
  double front_window_half_angle_deg_;
  double k_linear_;
  double k_angular_;
  double max_linear_speed_;
  double max_angular_speed_;
  int min_valid_points_;
  bool debug_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFollower>());
  rclcpp::shutdown();
  return 0;
}   