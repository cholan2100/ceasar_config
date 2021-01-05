#include <ceasar_config/hardware_interface.h>

namespace spotmicro_hardware_interface
{
SpotHardwareInterface::SpotHardwareInterface(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle) 
: nh_(node_handle)
, pnh_(private_node_handle) {
    // Get joint names
    nh_.getParam("hardware_interface/joints", joint_names_);
    if (joint_names_.size() == 0) {
        ROS_FATAL_STREAM_NAMED("hardware_interface", "No joints found on parameter server for controller. Did you load the proper yaml file?");
    }
    num_joints_ = joint_names_.size();

    // Resize vectors
    joint_position_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_effort_.resize(num_joints_);
    joint_position_command_.resize(num_joints_);
    joint_velocity_command_.resize(num_joints_);
    joint_effort_command_.resize(num_joints_);
    // previous_command_.resize(num_joints_);

    // Initialize Controller 
    for (int i = 0; i < num_joints_; ++i) {
        // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);
        ROS_INFO_STREAM_NAMED("hardware_interface", "Registering joint " << joint_names_[i]);

        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        // joint_limits_interface::JointLimits limits;
        // joint_limits_interface::SoftJointLimits softLimits;

        // if (getJointLimits(joint_names_[i], nh_, limits) == false) {
        //     ROS_ERROR_STREAM("Cannot set limits for " << joint_names_[i]);
        // }
        // else {
        //     joint_limits_interface::PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
        //     positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
        // }
        position_joint_interface_.registerHandle(jointPositionHandle);

        // Create effort joint interface
        hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
        effort_joint_interface_.registerHandle(jointEffortHandle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&effort_joint_interface_);
    // registerInterface(&positionJointSoftLimitsInterface);

    pwm_controller_.reset(new I2cPWMLib(&nh_));
    pwm_controller_->publishServoConfiguration();

    ROS_INFO_NAMED("hardware_interface", "Loaded hardware_interface.");
}

SpotHardwareInterface::~SpotHardwareInterface() {

}

void SpotHardwareInterface::read(const ros::Time& time, const ros::Duration& elapsed_time) {
    for (int i = 0; i < num_joints_; i++) {
        joint_position_[i] = joint_position_command_[i];
    }
}

void SpotHardwareInterface::write(const ros::Time& time, const ros::Duration& elapsed_time) {
    // Ensure that the joint limits are respected
    // positionJointSoftLimitsInterface.enforceLimits(elapsed_time);        

    for (int i = 0; i < num_joints_; i++) {
        // Sending commands only when needed
        // if (std::abs(joint_position_command_[i] - previous_command_[i]) < std::numeric_limits<double>::epsilon())
        //     continue;

        // previous_command_[i] = joint_position_command_[i];

        double k = 1.0;
        if(joint_names_[i].find("right") != std::string::npos) k = -1.0;

        pwm_controller_->setServoCommand(joint_names_[i], k*joint_position_command_[i]);
    }
    pwm_controller_->publishServoProportionalCommand();
}
}  // namespace spotmicro_hardware_interface
