// Copyright 2026 stregatto
// SPDX-License-Identifier: Apache-2.0
//
// Plugin ros2_control di Sesame (fase 5): ponte verso l'ESP32 tramite i topic micro-ROS.
//
//   write(): comandi dei controllori (rad, URDF) -> /sesame/joint_commands (gradi del servo,
//            8 valori in ordine firmware)
//   read():  /sesame/joint_echo (gradi del servo scritti dal firmware) -> stati (rad, URDF)
//
// Conversione e subtrim solo qui; limiti, passo massimo e watchdog anche nel firmware.

#ifndef SESAME_HARDWARE__SESAME_SYSTEM_HARDWARE_HPP_
#define SESAME_HARDWARE__SESAME_SYSTEM_HARDWARE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "sesame_hardware/conversion.hpp"

namespace sesame_hardware
{

constexpr std::size_t kJointCount = 8;

// API "classica" di Jazzy (on_init con HardwareInfo, export_*_interfaces con puntatori):
// presente in tutte le versioni 4.x, nelle ultime solo segnata come deprecata.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

class SesameSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SesameSystemHardware)

  ~SesameSystemHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  void echo_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void stop_node();
  /// Converte l'ultima eco in stati (rad). Ritorna false se non c'e' ancora un'eco.
  bool update_states_from_echo(bool reset_velocity);

  // Parametri (dallo xacro)
  std::string command_topic_{"/sesame/joint_commands"};
  std::string echo_topic_{"/sesame/joint_echo"};
  int activate_timeout_ms_{2000};
  int echo_timeout_ms_{250};
  double maximum_subtrim_deg_{8.0};
  double servo_min_deg_{0.0};
  double servo_max_deg_{180.0};
  std::vector<JointCalibration> calibration_;  // stesso ordine di info_.joints

  // Valori delle interfacce (stesso ordine di info_.joints)
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> position_commands_;

  // Nodo interno con executor in un thread separato
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::atomic<bool> stop_spin_{false};
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr echo_sub_;
  std_msgs::msg::Float32MultiArray command_msg_;

  // Ultima eco (scritta dal thread dell'executor)
  std::mutex echo_mutex_;
  std::array<float, kJointCount> echo_servo_deg_{};
  std::uint64_t echo_count_{0};
  SteadyTime echo_time_{};
  std::uint64_t echo_bad_{0};

  // Stato del ciclo read/write
  std::uint64_t last_echo_seen_{0};
  SteadyTime last_echo_seen_time_{};
  bool echo_lost_{false};
  std::atomic<bool> active_{false};
  rclcpp::Clock::SharedPtr steady_clock_{std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)};
};

#pragma GCC diagnostic pop

}  // namespace sesame_hardware

#endif  // SESAME_HARDWARE__SESAME_SYSTEM_HARDWARE_HPP_
