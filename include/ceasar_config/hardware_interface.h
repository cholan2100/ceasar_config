#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <ros/ros.h>

#include <ceasar_config/hardware.h>

#include <i2cpwm_board/i2cpwm_lib.h>


namespace spotmicro_hardware_interface
{
class SpotHardwareInterface : public spotmicro_hardware_interface::SpotHardware
{
public:
    SpotHardwareInterface(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle);
    ~SpotHardwareInterface();
    void read(const ros::Time& time, const ros::Duration& elapsed_time);
    void write(const ros::Time& time, const ros::Duration& elapsed_time);

protected:
    std::unique_ptr<I2cPWMLib> pwm_controller_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Duration control_period_;
    hardware_interface::PositionJointInterface positionJointInterface;
    joint_limits_interface::PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
};
}  // namespace spotmicro_harware_interface
