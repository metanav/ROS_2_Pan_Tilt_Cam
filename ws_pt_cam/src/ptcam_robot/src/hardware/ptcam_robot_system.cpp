#include <string>
#include <sstream>
#include <cmath>
#include <iostream>
#include <algorithm> 
#include <PiPCA9685/PCA9685.h>

#include "rclcpp/rclcpp.hpp"
#include "ptcam_robot/ptcam_robot_system.hpp"

namespace ptcam_robot
{

  hardware_interface::CallbackReturn PTCamSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Get configuration parameters
    std::string i2c_bus = info_.hardware_parameters["i2c_bus"];
    int i2c_address = std::stoi(info_.hardware_parameters["i2c_address"], 0, 16);

    RCLCPP_INFO(rclcpp::get_logger("PTCamSystemHardware"), "Configured for I2C Bus: %s and Address: 0x%X", i2c_bus.c_str(), i2c_address);

    // Initialize the dedicated driver
    driver_ = std::make_unique<PiPCA9685::PCA9685>();
    driver_->set_pwm_freq(50.0);

    // Size the command and state arrays
    num_servos_ = info_.joints.size();
    hw_commands_.resize(num_servos_, std::numeric_limits<double>::quiet_NaN());
    hw_states_.resize(num_servos_, std::numeric_limits<double>::quiet_NaN());
    servo_configs_.resize(num_servos_);

    // Validate joint configuration and load servo limits (pulse width in us)
    for (size_t i = 0; i < num_servos_; ++i) {
      const auto & joint = info_.joints[i];

      // Check for command/state interfaces (same logic as before)
      if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces.size() != 1 || joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_FATAL(rclcpp::get_logger("PTCamSystemHardware"),
          "Joint '%s' interfaces must be exactly one 'position' for command and state.", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // Load specific servo parameters
      try {
        servo_configs_[i].channel = std::stoi(joint.parameters.at("channel"));
        servo_configs_[i].min_us = std::stod(joint.parameters.at("min_pulse_us"));
        servo_configs_[i].max_us = std::stod(joint.parameters.at("max_pulse_us"));
        RCLCPP_INFO(rclcpp::get_logger("PTCamSystemHardware"), "Joint '%s' configured: Channel %d, min_us %.1f, max_us %.1f",
          joint.name.c_str(), servo_configs_[i].channel, servo_configs_[i].min_us, servo_configs_[i].max_us);
      } catch (const std::out_of_range & e) {
        RCLCPP_FATAL(rclcpp::get_logger("PTCamSystemHardware"),
          "Missing required joint parameter (channel, min_pulse_us, max_pulse_us) for joint '%s'.", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // --- State and Command Interface Exports (Unchanged) ---
  std::vector<hardware_interface::StateInterface> PTCamSystemHardware::export_state_interfaces() 
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < num_servos_; ++i) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> PTCamSystemHardware::export_command_interfaces() 
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < num_servos_; ++i) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }
    return command_interfaces;
  }

  /**
   * @brief Method to activate the hardware interface.
   */
  hardware_interface::CallbackReturn PTCamSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("PTCamSystemHardware"), "Activating hardware interface...");

    // Initialize command and state buffers
    for (size_t i = 0; i < num_servos_; ++i) {
      // Set the initial command to a neutral position (0.0 rad)
      hw_commands_[i] = 0.0;
      hw_states_[i] = 0.0;
      const auto & config = servo_configs_[i];
      driver_->set_pwm_ms(config.channel, 1.5f); // set initial position to 90 degrees (1500us)
    }

    RCLCPP_INFO(rclcpp::get_logger("PTCamSystemHardware"), "Hardware interface successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Method to deactivate the hardware interface.
   */
  hardware_interface::CallbackReturn PTCamSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) 
  {
    RCLCPP_INFO(rclcpp::get_logger("PTCamSystemHardware"), "Deactivating hardware interface...");
    // Driver destructor handles closing the file descriptor.
    RCLCPP_INFO(rclcpp::get_logger("PTCamSystemHardware"), "Hardware interface successfully deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Reads state data from the hardware.
   */
  hardware_interface::return_type PTCamSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
  {
    for (size_t i = 0; i < num_servos_; ++i) {
      hw_states_[i] = hw_commands_[i];
    }
    return hardware_interface::return_type::OK;
  }

  /**
   * @brief Writes command data to the hardware using the dedicated driver.
   */
  hardware_interface::return_type PTCamSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
  {
    for (size_t i = 0; i < num_servos_; ++i) {
      double target_rad = hw_commands_[i];
      const auto & config = servo_configs_[i];
    
      if (hw_commands_[i] == hw_states_[i]) {
          continue;
      }

      RCLCPP_INFO(get_logger(), "i=%ld, target_rad=%f", i, target_rad);

      // Radians to Pulse Width (us) Mapping 
      // Assuming +/- M_PI/2 radians maps to min/max pulse widths
      //constexpr double min_rad = -M_PI_2; // -90 degrees
      constexpr double min_rad = 0;
      constexpr double max_rad = M_PI; 

      // Clamp target_rad
      target_rad = std::max(min_rad, std::min(max_rad, target_rad));

      // Linear interpolation:
      double normalized = (target_rad - min_rad) / (max_rad - min_rad);
      double pulse_us = config.min_us + normalized * (config.max_us - config.min_us);

      driver_->set_pwm_ms(config.channel, pulse_us / 1000.0f);
    }
    return hardware_interface::return_type::OK;
  }

}  // namespace 

// This macro exports the hardware interface class to be used by the ROS 2 Control manager.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ptcam_robot::PTCamSystemHardware,
  hardware_interface::SystemInterface
)

