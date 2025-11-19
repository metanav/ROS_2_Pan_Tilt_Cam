#ifndef PTCAM_ROBOT_SYSTEM_HPP_
#define PTCAM_ROBOT_SYSTEM_HPP_

#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
//#include "ptcam_robot/pca9685_driver.hpp"


namespace ptcam_robot
{

class PTCamSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PTCamSystemHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & /*info*/) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override;

  hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

  hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  std::unique_ptr<PiPCA9685::PCA9685> driver_;

  struct ServoConfig
  {
    int channel;  // PCA9685 channel (0-15)
    double min_us;  
    double max_us; 
  };

  size_t num_servos_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<ServoConfig> servo_configs_;

};

}  // namespace 


#endif //PTCAM_ROBOT_SYSTEM_HPP_
