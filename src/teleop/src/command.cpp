/**
 * @file command.cpp
 * @brief Joystick teleoperation node for QBot Platform.
 *
 * Publishes:
 *   - geometry_msgs/Twist on "cmd_vel"
 *   - std_msgs/ColorRGBA on "qbot_led_strip"
 *   - std_msgs/Bool on "qbot_kill"
 *
 * Controls (more intuitive mapping):
 *   - Hold LB: enable motion (deadman/safety)
 *   - Left stick Y: forward/backward speed (proportional)
 *   - Right stick X: turning rate (proportional)
 *   - Hold RB: half-speed mode
 *   - Press X: kill (stop robot + request other node shutdown)
 *
 * LED behavior:
 *   - Forward: green
 *   - Backward: blue
 *   - Stationary: yellow
 */

#include "rclcpp/rclcpp.hpp"
#include <iostream>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "quanser/quanser_messages.h"
#include "quanser/quanser_memory.h"
#include "quanser/quanser_hid.h"

#include <algorithm>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

static inline double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

static inline double apply_deadzone(double v, double dz)
{
  return (std::fabs(v) < dz) ? 0.0 : v;
}

class CommandPublisher : public rclcpp::Node
{
public:
  CommandPublisher()
  : Node("joystick_publisher")
  {
    // Publishers
    cmd_pub_  = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    led_pub_  = this->create_publisher<std_msgs::msg::ColorRGBA>("qbot_led_strip", 10);
    kill_pub_ = this->create_publisher<std_msgs::msg::Bool>("qbot_kill", 10);

    // Attempt to connect to joystick/game controller
    controller_number_ = 1;
    buffer_size_ = 12;

    for (int i = 0; i < 6; i++) { deadzone_[i] = 0.0; saturation_[i] = 0.0; }

    result_ = game_controller_open(
      controller_number_,
      buffer_size_,
      deadzone_,
      saturation_,
      auto_center_,
      max_force_feedback_effects_,
      force_feedback_gain_,
      &gamepad_
    );

    if (result_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open game controller (result=%d).", result_);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Game controller opened successfully.");
    }

    // 50 Hz update
    timer_ = this->create_wall_timer(20ms, std::bind(&CommandPublisher::timer_callback, this));
  }

  ~CommandPublisher() override
  {
    // Close controller cleanly
    if (result_ >= 0)
    {
      game_controller_close(gamepad_);
    }
  }

private:
  void publish_led(double r, double g, double b)
  {
    std_msgs::msg::ColorRGBA led;
    led.r = (float)r;
    led.g = (float)g;
    led.b = (float)b;
    led.a = 1.0f; // Using alpha as "enable user LED color" per driver interface
    led_pub_->publish(led);
  }

  void timer_callback()
  {
    if (result_ < 0) return; // controller not opened

    // Poll once per timer tick (do NOT block in a while loop)
    t_boolean is_new = false;
    t_game_controller_states data{};
    t_error poll_result = game_controller_poll(gamepad_, &data, &is_new);

    std::cout << "Axes: "
          << "x=" << data.x
          << " y=" << data.y
          << " rx=" << data.rx
          << " ry=" << data.ry
          << " rz=" << data.rz
          << std::endl;

    if (poll_result < 0)
    {
      // If polling fails intermittently, just stop publishing motion
      geometry_msgs::msg::Twist zero;
      cmd_pub_->publish(zero);
      publish_led(1.0, 1.0, 0.0); // yellow
      return;
    }

    // Button bit mapping (common Xbox-style)
    // A: bit0, B: bit1, X: bit2, Y: bit3, LB: bit4, RB: bit5
    const bool btn_x  = (data.buttons & (1 << 2)) != 0;
    const bool btn_lb = (data.buttons & (1 << 4)) != 0; // deadman enable
    const bool btn_rb = (data.buttons & (1 << 5)) != 0; // slow mode

    // ---- KILL behavior ----
    if (btn_x)
    {
      // Stop robot immediately
      geometry_msgs::msg::Twist zero;
      cmd_pub_->publish(zero);

      // Yellow LED to indicate stopped
      publish_led(1.0, 1.0, 0.0);

      // Publish kill signal so the driver interface can also shutdown
      std_msgs::msg::Bool kill;
      kill.data = true;
      kill_pub_->publish(kill);

      RCLCPP_WARN(this->get_logger(), "KILL pressed (X). Shutting down teleop node.");
      rclcpp::shutdown();
      return;
    }

    // ---- Intuitive control mapping + proportional velocity ----
    // Left stick Y controls linear speed (forward/back), Right stick X controls turn.
    // Quanser controller axes commonly:
    //   data.y  = left stick Y
    //   data.rx = right stick X
    //
    // If your hardware swaps axes, change these two lines only.
    double left_y  = -static_cast<double>(data.y);   // forward positive
    double right_x = -static_cast<double>(data.x);  // right positive

    // Deadzone
    left_y  = apply_deadzone(left_y,  0.08);
    right_x = apply_deadzone(right_x, 0.08);

    // Clamp to [-1, 1]
    left_y  = clamp(left_y,  -1.0, 1.0);
    right_x = clamp(right_x, -1.0, 1.0);

    // Speed limits (tune as desired)
    const double max_linear  = 0.30; // m/s (similar to your previous 0.3 scaling)
    const double max_angular = 0.50; // rad/s (similar to your previous 0.5 scaling)

    double speed_scale = 1.0;
    if (btn_rb)
    {
      // (a) Half-speed mode
      speed_scale = 0.5;
    }

    geometry_msgs::msg::Twist twist;

    // Deadman safety: only move when LB is held
    if (btn_lb)
    {
      twist.linear.x  = speed_scale * max_linear  * left_y;
      twist.angular.z = speed_scale * max_angular * right_x;
    }
    else
    {
      twist.linear.x  = 0.0;
      twist.angular.z = 0.0;
    }

    cmd_pub_->publish(twist);

    // ---- LED color based on direction ----
    const double eps = 1e-3;
    if (twist.linear.x > eps)
    {
      // forward -> green
      publish_led(0.0, 1.0, 0.0);
    }
    else if (twist.linear.x < -eps)
    {
      // backward -> blue
      publish_led(0.0, 0.0, 1.0);
    }
    else
    {
      // stationary -> yellow
      publish_led(1.0, 1.0, 0.0);
    }
  }

  // ROS publishers/timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr led_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kill_pub_;

  // Quanser HID
  t_game_controller gamepad_{};
  t_error result_{-1};

  t_uint8 controller_number_{1};
  t_uint16 buffer_size_{12};
  t_double deadzone_[6]{0.0};
  t_double saturation_[6]{0.0};
  t_boolean auto_center_{false};
  t_uint16 max_force_feedback_effects_{0};
  t_double force_feedback_gain_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandPublisher>());
  rclcpp::shutdown();
  return 0;
}