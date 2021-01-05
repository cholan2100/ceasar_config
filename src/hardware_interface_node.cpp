#include <chrono>

#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <ceasar_config/hardware_interface.h>

typedef std::chrono::duration<double> duration_t;
typedef std::chrono::system_clock::time_point time_point_t;

void update(spotmicro_hardware_interface::SpotHardwareInterface &hw, 
            controller_manager::ControllerManager &cm, 
            time_point_t &last_time) {
    // Get change in time
    time_point_t current_time = std::chrono::system_clock::now();
    duration_t time_span = std::chrono::duration_cast<duration_t>(current_time - last_time);
    ros::Duration elapsed_time = ros::Duration(time_span.count());
    last_time = current_time;
    
    // Input: get the hardware's state
    hw.read(ros::Time::now(), elapsed_time);
    // Control
    // Let the controller compute the new command (via the controller manager)
    cm.update(ros::Time::now(), elapsed_time);
    // Output
    // Send the new command to hardware
    hw.write(ros::Time::now(), elapsed_time);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hardware_interface");

    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");

    ros::CallbackQueue queue;
    ros::AsyncSpinner spinner(2, &queue);

    spotmicro_hardware_interface::SpotHardwareInterface shi(node_handle, private_node_handle);
    
    controller_manager::ControllerManager controller_manager(&shi, node_handle);

    time_point_t last_time = std::chrono::system_clock::now();

    double loop_hz;
    node_handle.param("hardware_interface/loop_hz", loop_hz, 0.1);
    ROS_INFO_STREAM_NAMED("hardware_interface", "Using loop frequency of " << loop_hz << " Hz");
    ros::TimerOptions control_timer(
        ros::Duration(1.0/loop_hz),
        std::bind(update, std::ref(shi), std::ref(controller_manager), std::ref(last_time)),
        &queue
    );
    ros::Timer non_realtime_loop = node_handle.createTimer(control_timer);

    spinner.start();
    ros::spin();

    return 0;
}
