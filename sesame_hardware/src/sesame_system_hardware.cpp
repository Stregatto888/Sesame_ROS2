// Copyright 2026 stregatto
// SPDX-License-Identifier: Apache-2.0

#include "sesame_hardware/sesame_system_hardware.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

// Vedi l'header: API classica di Jazzy, deprecata solo nelle ultime 4.x.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace sesame_hardware
{

namespace
{

using CallbackReturn = hardware_interface::CallbackReturn;
using hardware_interface::return_type;

constexpr int kWarnThrottleMs = 2000;

rclcpp::Logger logger() { return rclcpp::get_logger("SesameSystemHardware"); }

/// Parametro testuale: valore dello xacro o predefinito se assente.
std::string param_string(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  const std::string & fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}

/// Parametro numerico obbligatorio (eccezione con messaggio chiaro se manca o non e' un numero).
double param_double(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  const std::string & owner)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    throw std::runtime_error("parametro '" + key + "' mancante in " + owner);
  }
  try {
    std::size_t used = 0;
    const double value = std::stod(it->second, &used);
    if (used != it->second.size() || !std::isfinite(value)) {
      throw std::invalid_argument(it->second);
    }
    return value;
  } catch (const std::exception &) {
    throw std::runtime_error(
      "parametro '" + key + "' di " + owner + " non valido: '" + it->second + "'");
  }
}

}  // namespace

SesameSystemHardware::~SesameSystemHardware() { stop_node(); }

// ---------------------------------------------------------------------------------------------
// on_init: lettura e controllo dei parametri dello xacro
// ---------------------------------------------------------------------------------------------
CallbackReturn SesameSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    const auto & hw = info_.hardware_parameters;
    command_topic_ = param_string(hw, "command_topic", command_topic_);
    echo_topic_ = param_string(hw, "echo_topic", echo_topic_);
    activate_timeout_ms_ = static_cast<int>(param_double(hw, "activate_timeout_ms", "hardware"));
    echo_timeout_ms_ = static_cast<int>(param_double(hw, "echo_timeout_ms", "hardware"));
    maximum_subtrim_deg_ = param_double(hw, "maximum_subtrim_degrees", "hardware");
    servo_min_deg_ = param_double(hw, "servo_min_degrees", "hardware");
    servo_max_deg_ = param_double(hw, "servo_max_degrees", "hardware");

    if (activate_timeout_ms_ <= 0 || echo_timeout_ms_ <= 0 || servo_min_deg_ >= servo_max_deg_) {
      throw std::runtime_error("parametri di <hardware> non coerenti (tempi o campo del servo)");
    }
    if (info_.joints.size() != kJointCount) {
      throw std::runtime_error(
        "servono " + std::to_string(kJointCount) + " giunti, nello xacro ce ne sono " +
        std::to_string(info_.joints.size()));
    }

    calibration_.clear();
    std::set<int> indices;
    for (const auto & joint : info_.joints) {
      const std::string owner = "giunto " + joint.name;

      if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        throw std::runtime_error(owner + ": serve solo il comando 'position'");
      }
      if (joint.state_interfaces.size() != 2 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        throw std::runtime_error(owner + ": servono gli stati 'position' e 'velocity' (in ordine)");
      }

      JointCalibration cal;
      const double index = param_double(joint.parameters, "firmware_index", owner);
      cal.firmware_index = static_cast<int>(index);
      cal.direction = param_double(joint.parameters, "direction", owner);
      cal.subtrim_deg = param_double(joint.parameters, "subtrim_degrees", owner);
      cal.min_deg = param_double(joint.parameters, "min_degrees", owner);
      cal.max_deg = param_double(joint.parameters, "max_degrees", owner);

      if (index != cal.firmware_index || cal.firmware_index < 0 ||
        cal.firmware_index >= static_cast<int>(kJointCount) ||
        !indices.insert(cal.firmware_index).second)
      {
        throw std::runtime_error(owner + ": firmware_index non valido o ripetuto");
      }
      if (cal.direction != 1.0 && cal.direction != -1.0) {
        throw std::runtime_error(owner + ": direction deve essere 1 o -1");
      }
      if (std::abs(cal.subtrim_deg) > maximum_subtrim_deg_) {
        throw std::runtime_error(
          owner + ": |subtrim| oltre maximum_subtrim_degrees (rimontare la squadretta)");
      }
      if (cal.min_deg >= cal.max_deg) {
        throw std::runtime_error(owner + ": min_degrees deve essere minore di max_degrees");
      }

      // Limiti URDF che il servo non puo' raggiungere con questa calibrazione: solo avviso.
      const double servo_a = urdf_to_servo_deg(cal.min_deg, cal);
      const double servo_b = urdf_to_servo_deg(cal.max_deg, cal);
      if (std::min(servo_a, servo_b) < servo_min_deg_ || std::max(servo_a, servo_b) > servo_max_deg_) {
        RCLCPP_WARN(
          logger(), "%s: i limiti %.1f-%.1f° diventano %.1f-%.1f° sul servo, fuori dal campo "
          "%.0f-%.0f°: vicino al limite il giunto si fermera' prima", joint.name.c_str(),
          cal.min_deg, cal.max_deg, servo_a, servo_b, servo_min_deg_, servo_max_deg_);
      }
      calibration_.push_back(cal);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "configurazione non valida: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Valori iniziali: initial_value dello xacro (stand) finche' non arriva l'eco.
  position_states_.assign(kJointCount, 0.0);
  velocity_states_.assign(kJointCount, 0.0);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto & initial = info_.joints[i].state_interfaces[0].initial_value;
    if (!initial.empty()) {
      position_states_[i] = std::stod(initial);
    }
  }
  position_commands_ = position_states_;
  command_msg_.data.assign(kJointCount, 0.0F);

  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto & cal = calibration_[i];
    RCLCPP_INFO(
      logger(), "%s: indice firmware %d, direzione %+.0f, subtrim %+.1f°, limiti %.0f-%.0f°",
      info_.joints[i].name.c_str(), cal.firmware_index, cal.direction, cal.subtrim_deg,
      cal.min_deg, cal.max_deg);
  }
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// Interfacce
// ---------------------------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> SesameSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    // Costruzione qui (non con emplace_back): l'avviso di deprecazione resta sotto il pragma.
    interfaces.push_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    interfaces.push_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> SesameSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    interfaces.push_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }
  return interfaces;
}

// ---------------------------------------------------------------------------------------------
// Ciclo di vita
// ---------------------------------------------------------------------------------------------
CallbackReturn SesameSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  stop_node();

  // Nodo proprio: niente argomenti globali (remapping e parametri del controller_manager).
  node_ = std::make_shared<rclcpp::Node>(
    "sesame_hardware", rclcpp::NodeOptions().use_global_arguments(false));

  // Come il firmware: best-effort, solo l'ultimo messaggio, niente storico.
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.durability_volatile();

  command_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(command_topic_, qos);
  echo_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    echo_topic_, qos,
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) { echo_callback(msg); });

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  // spin_once a tempo invece di spin(): il thread si ferma sempre con stop_spin_.
  stop_spin_ = false;
  spin_thread_ = std::thread([this, executor = executor_]() {
      while (!stop_spin_ && rclcpp::ok()) {
        executor->spin_once(std::chrono::milliseconds(50));
      }
    });

  {
    std::lock_guard<std::mutex> lock(echo_mutex_);
    echo_count_ = 0;
    echo_bad_ = 0;
  }
  last_echo_seen_ = 0;
  echo_lost_ = false;

  RCLCPP_INFO(
    logger(), "configurato: comandi su %s, eco da %s (best-effort)", command_topic_.c_str(),
    echo_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SesameSystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  active_ = false;
  stop_node();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SesameSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (!node_) {
    RCLCPP_ERROR(logger(), "attivazione senza configurazione");
    return CallbackReturn::ERROR;
  }

  // Serve un'eco nuova, arrivata dopo l'inizio dell'attivazione.
  std::uint64_t start_count = 0;
  {
    std::lock_guard<std::mutex> lock(echo_mutex_);
    start_count = echo_count_;
  }
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(activate_timeout_ms_);
  bool received = false;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(echo_mutex_);
      received = echo_count_ > start_count;
    }
    if (received) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!received) {
    RCLCPP_ERROR(
      logger(), "nessuna eco da %s in %d ms: attivazione rifiutata, nessun comando inviato. "
      "Controllare scheda collegata, agente micro-ROS acceso (T2) e ROS_DOMAIN_ID=0",
      echo_topic_.c_str(), activate_timeout_ms_);
    return CallbackReturn::ERROR;
  }

  // Si parte dalla posa reale: stati e comandi = eco, quindi nessun movimento all'attivazione.
  update_states_from_echo(true);
  position_commands_ = position_states_;
  echo_lost_ = false;
  active_ = true;

  std::string pose;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    char item[32];
    std::snprintf(
      item, sizeof(item), "%s %.1f ", info_.joints[i].name.c_str(),
      rad_to_deg(position_states_[i]));
    pose += item;
  }
  RCLCPP_INFO(logger(), "attivo, posa letta dall'eco (gradi URDF): %s", pose.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SesameSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  RCLCPP_INFO(
    logger(), "disattivato: comandi sospesi, il firmware si ferma nella posa corrente (watchdog)");
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------------------------
// read / write (thread del controller_manager, 50 Hz)
// ---------------------------------------------------------------------------------------------
return_type SesameSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::uint64_t count = 0;
  SteadyTime time;
  {
    std::lock_guard<std::mutex> lock(echo_mutex_);
    count = echo_count_;
    time = echo_time_;
  }

  if (count != last_echo_seen_ && count != 0) {
    update_states_from_echo(false);
    if (echo_lost_) {
      echo_lost_ = false;
      RCLCPP_INFO(logger(), "eco di nuovo presente");
    }
    return return_type::OK;
  }

  // Nessuna eco nuova: stati fermi all'ultima eco, velocita' a zero dopo echo_timeout_ms.
  const auto age_ms = count == 0 ? -1 :
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time)
    .count();
  if (active_ && (count == 0 || age_ms > echo_timeout_ms_)) {
    std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);
    if (!echo_lost_) {
      echo_lost_ = true;
      RCLCPP_WARN(
        logger(), "nessuna eco da %s da oltre %d ms: stati fermi all'ultima eco "
        "(firmware fermo per il watchdog, agente spento o cavo staccato?)", echo_topic_.c_str(),
        echo_timeout_ms_);
    }
    RCLCPP_WARN_THROTTLE(
      logger(), *steady_clock_, 5000, "ancora nessuna eco (ultima %ld ms fa)",
      static_cast<long>(age_ms));
  }
  return return_type::OK;
}

return_type SesameSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_ || !command_pub_) {
    return return_type::OK;
  }

  bool clamped_urdf = false;
  bool clamped_servo = false;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double command = position_commands_[i];
    if (!std::isfinite(command)) {
      // Senza comando valido non si pubblica: dopo 250 ms il firmware si ferma (watchdog).
      RCLCPP_WARN_THROTTLE(
        logger(), *steady_clock_, kWarnThrottleMs, "comando non finito su %s: non inviato",
        info_.joints[i].name.c_str());
      return return_type::OK;
    }
    const auto & cal = calibration_[i];
    const double urdf_deg = clamp_flag(rad_to_deg(command), cal.min_deg, cal.max_deg, clamped_urdf);
    const double servo_deg =
      clamp_flag(urdf_to_servo_deg(urdf_deg, cal), servo_min_deg_, servo_max_deg_, clamped_servo);
    command_msg_.data[cal.firmware_index] = static_cast<float>(servo_deg);
  }
  if (clamped_urdf) {
    RCLCPP_WARN_THROTTLE(
      logger(), *steady_clock_, kWarnThrottleMs, "comando oltre i limiti URDF: limitato");
  }
  if (clamped_servo) {
    RCLCPP_WARN_THROTTLE(
      logger(), *steady_clock_, kWarnThrottleMs,
      "comando fuori dal campo del servo dopo la calibrazione: limitato");
  }

  command_pub_->publish(command_msg_);
  return return_type::OK;
}

// ---------------------------------------------------------------------------------------------
// Funzioni interne
// ---------------------------------------------------------------------------------------------
void SesameSystemHardware::echo_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  bool valid = msg->data.size() == kJointCount;
  for (std::size_t i = 0; valid && i < kJointCount; ++i) {
    valid = std::isfinite(msg->data[i]);
  }

  std::lock_guard<std::mutex> lock(echo_mutex_);
  if (!valid) {
    ++echo_bad_;
    RCLCPP_WARN_THROTTLE(
      logger(), *steady_clock_, kWarnThrottleMs, "eco non valida ignorata (%zu valori, %lu in tutto)",
      msg->data.size(), static_cast<unsigned long>(echo_bad_));
    return;
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    echo_servo_deg_[i] = msg->data[i];
  }
  ++echo_count_;
  echo_time_ = std::chrono::steady_clock::now();
}

bool SesameSystemHardware::update_states_from_echo(bool reset_velocity)
{
  std::array<float, kJointCount> servo{};
  std::uint64_t count = 0;
  SteadyTime time;
  {
    std::lock_guard<std::mutex> lock(echo_mutex_);
    servo = echo_servo_deg_;
    count = echo_count_;
    time = echo_time_;
  }
  if (count == 0) {
    return false;
  }

  const double dt =
    std::chrono::duration<double>(time - last_echo_seen_time_).count();
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto & cal = calibration_[i];
    const double position =
      deg_to_rad(servo_to_urdf_deg(static_cast<double>(servo[cal.firmware_index]), cal));
    if (reset_velocity || last_echo_seen_ == 0) {
      velocity_states_[i] = 0.0;
    } else if (dt > 0.001) {
      velocity_states_[i] = (position - position_states_[i]) / dt;
    }
    position_states_[i] = position;
  }
  last_echo_seen_ = count;
  last_echo_seen_time_ = time;
  return true;
}

void SesameSystemHardware::stop_node()
{
  stop_spin_ = true;
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  echo_sub_.reset();
  command_pub_.reset();
  executor_.reset();
  node_.reset();
}

}  // namespace sesame_hardware

#pragma GCC diagnostic pop

PLUGINLIB_EXPORT_CLASS(sesame_hardware::SesameSystemHardware, hardware_interface::SystemInterface)
