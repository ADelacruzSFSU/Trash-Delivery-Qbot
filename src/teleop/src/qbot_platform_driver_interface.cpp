/**
 * @file qbot_platform_driver_interface.cpp
 * @brief Minimal QBot Platform driver interface for teleop.
 *
 * This node connects to the QUARC QBot driver model over a persistent stream,
 * sends motion commands (cmd_vel), and forwards LED strip commands.
 *
 * Subscribes:
 *   - geometry_msgs/Twist on "cmd_vel"
 *   - std_msgs/ColorRGBA on "qbot_led_strip"
 *   - std_msgs/Bool on "qbot_kill"   (kill switch)
 *
 * Parameters:
 *   - driver_uri (string): default "tcpip://localhost:18888"
 *   - arm_robot (bool): default true
 *   - speed_limit_mode (int): 0 education, 1 research
 *   - hold (bool): default false
 *
 * Kill behavior:
 *   - On kill=true, disarms motors and zeros speed commands, sends once,
 *     closes the stream, and shuts down.
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "quanser/quanser_persistent_stream.h"
#include "quanser/quanser_thread.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

class QBotPlatformDriverInterface : public rclcpp::Node
{
public:
  QBotPlatformDriverInterface()
  : Node("qbot_platform_driver_interface")
  {
    other_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = other_cb_group_;

    // ---- Parameters ----
    auto desc = rcl_interfaces::msg::ParameterDescriptor{};
    desc.description = "URI to interface with the QBot driver model (default tcpip://localhost:18888).";
    this->declare_parameter("driver_uri", "tcpip://localhost:18888", desc);
    driver_uri_ = this->get_parameter("driver_uri").as_string();

    desc.description = "Arm/disarm motors.";
    this->declare_parameter("arm_robot", true, desc);
    arm_robot_ = this->get_parameter("arm_robot").as_bool();

    desc.description = "Speed limit mode: 0 education, 1 research.";
    this->declare_parameter("speed_limit_mode", 0, desc);
    speed_limit_mode_ = this->get_parameter("speed_limit_mode").as_int();

    desc.description = "Hold robot position (true) or allow motion (false).";
    this->declare_parameter("hold", false, desc);
    hold_ = this->get_parameter("hold").as_bool();

    // ---- QUARC Persistent Stream setup ----
    setup_pstream_options(options_);

    int rc = pstream_connect(driver_uri_.c_str(), &options_, &client_);
    if (rc != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error connecting to driver stream: %d", rc);
      connected_ = false;
      return;
    }

    connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Connected to driver stream: %s", driver_uri_.c_str());

    // ---- Subscribers ----
    led_sub_ = this->create_subscription<std_msgs::msg::ColorRGBA>(
      "qbot_led_strip", 10,
      std::bind(&QBotPlatformDriverInterface::led_callback, this, _1),
      sub_options
    );

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&QBotPlatformDriverInterface::cmd_callback, this, _1),
      sub_options
    );

    kill_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "qbot_kill", 10,
      std::bind(&QBotPlatformDriverInterface::kill_callback, this, _1),
      sub_options
    );

    // ---- Timer loop for stream send/receive ----
    timer_ = this->create_wall_timer(16ms, std::bind(&QBotPlatformDriverInterface::timer_callback, this), timer_cb_group_);
  }

  ~QBotPlatformDriverInterface() override
  {
    if (connected_)
    {
      pstream_close(client_);
      connected_ = false;
    }
  }

private:
  void setup_pstream_options(t_pstream_options & options)
  {
    options.receive_thread_attributes = NULL;
    options.send_thread_attributes = NULL;

    options.num_receive_units = 17;     // keep same receive format (driver model expects it)
    options.num_send_units = 10;        // keep same send format

    options.receive_unit_size = sizeof(double);
    options.send_unit_size = sizeof(double);

    options.num_send_dimensions = 0;
    options.num_receive_dimensions = 0;

    options.receive_buffer_size = 1000;
    options.send_buffer_size = 1000;

    options.send_fifo_size = 1000;
    options.receive_fifo_size = 1000;

    options.flags = 0;
    options.flags |= PSTREAM_FLAG_MULTITHREADED;
    options.flags |= PSTREAM_FLAG_MINIMIZE_LATENCY | PSTREAM_FLAG_SEND_MOST_RECENT | PSTREAM_FLAG_RECEIVE_MOST_RECENT;

    options.size = sizeof(options);
  }

  void led_callback(const std_msgs::msg::ColorRGBA & led)
  {
    // a used as "enable user LED"
    send_buffer_[1] = led.a;
    send_buffer_[2] = led.r;
    send_buffer_[3] = led.g;
    send_buffer_[4] = led.b;
  }

  void cmd_callback(const geometry_msgs::msg::Twist & twist)
  {
    // Body control (linear + angular)
    // send_buffer_[7] linear velocity, send_buffer_[8] angular velocity
    send_buffer_[7] = twist.linear.x;
    send_buffer_[8] = twist.angular.z;

    // body mode
    wheel_control_ = false;
  }

  void kill_callback(const std_msgs::msg::Bool & msg)
  {
    if (msg.data)
    {
      killed_ = true;
      RCLCPP_WARN(this->get_logger(), "Kill signal received. Disarming and shutting down...");
    }
  }

  void timer_callback()
  {
    if (!connected_) return;

    // Receive once (not strictly required for teleop, but keeps stream healthy)
    double receive_buffer[17];
    int rc = pstream_receive(client_, receive_buffer);
    (void)rc; // ignore errors except for debugging (could log if desired)

    // Refresh params
    speed_limit_mode_ = this->get_parameter("speed_limit_mode").as_int();
    hold_ = this->get_parameter("hold").as_bool();
    arm_robot_ = this->get_parameter("arm_robot").as_bool();

    // Mode selection
    // 0: wheeled edu, 1: body edu, 2: wheeled research, 3: body research
    if (wheel_control_)
      send_buffer_[0] = (speed_limit_mode_ == 0) ? 0.0 : 2.0;
    else
      send_buffer_[0] = (speed_limit_mode_ == 0) ? 1.0 : 3.0;

    // Hold
    send_buffer_[6] = hold_ ? 1.0 : 0.0;

    // Kill overrides everything
    if (killed_)
    {
      // Disarm + zero commands
      send_buffer_[5] = 0.0;  // motor enable
      send_buffer_[7] = 0.0;  // linear
      send_buffer_[8] = 0.0;  // angular
    }
    else
    {
      // Arm logic
      send_buffer_[5] = arm_robot_ ? 1.0 : 0.0;
      if (!arm_robot_)
      {
        send_buffer_[7] = 0.0;
        send_buffer_[8] = 0.0;
      }
    }

    // Timestamp
    send_buffer_[9] = this->get_clock()->now().seconds();

    // Send to driver
    rc = pstream_send(client_, send_buffer_);
    if (rc < 0)
    {
      // Optional: log once, but avoid spamming
      // RCLCPP_WARN(this->get_logger(), "pstream_send error: %d", rc);
    }

    // If killed, send once then shutdown cleanly
    if (killed_)
    {
      pstream_close(client_);
      connected_ = false;
      rclcpp::shutdown();
    }
  }

  // ROS callback groups + timer
  rclcpp::CallbackGroup::SharedPtr other_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr led_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub_;

  // QUARC stream
  t_pstream client_;
  t_pstream_options options_{};
  bool connected_{false};

  // Send buffer format expected by driver model (10 doubles)
  // 0 mode, 1 user led enable, 2-4 rgb, 5 motor enable, 6 hold, 7 linear, 8 angular, 9 timestamp
  double send_buffer_[10] = {0.0};

  // State
  bool killed_{false};
  bool wheel_control_{true};

  // Parameters
  std::string driver_uri_;
  int speed_limit_mode_{0};
  bool arm_robot_{true};
  bool hold_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QBotPlatformDriverInterface>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}