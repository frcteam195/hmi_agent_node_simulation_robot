#include "hmi_agent_node.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <rio_control_node/Joystick_Status.h>
#include <rio_control_node/Robot_Status.h>
#include <hmi_agent_node/HMI_Signals.h>
#include <ck_utilities/Joystick.hpp>
#include "ck_utilities/CKMath.hpp"
#include <atomic>
#include <ck_utilities/ParameterHelper.hpp>
#define RATE (100)

ros::NodeHandle *node;
Joystick *operator_joystick;
std::map<int, uint8_t> button_clicks;

enum RobotState : int
{
    DISABLED = 0,
    TELEOP = 1,
    AUTONOMOUS = 2,
    TEST = 3,
};
std::atomic<RobotState> robot_state {DISABLED};

uint8_t debounce(int index, uint8_t value)
{

    if (button_clicks.count(index) == 0)
    {
        button_clicks[index] = value;
        return value;
    }

    if (button_clicks[index] != value)
    {
        button_clicks[index] = value;
        return value;
    }

    return 0;
}

void joystick_status_callback(const rio_control_node::Joystick_Status &joystick_status)
{
    Joystick::update(joystick_status);
    hmi_agent_node::HMI_Signals output_signals;

    output_signals.gauge_value = 1500 + (1500 * operator_joystick->getRawAxis(gauge_axis_id)); 

    output_signals.elevator_vertical = operator_joystick->getRawAxis(elevator_vertical_axis_id);
    output_signals.claw_open = operator_joystick->getButton(claw_open_button_id);

    static ros::Publisher signal_publisher = node->advertise<hmi_agent_node::HMI_Signals>("/HMISignals", 10);

    if (robot_state != RobotState::AUTONOMOUS)
    {
        signal_publisher.publish(output_signals);
    }
}

void robot_status_callback(const rio_control_node::Robot_Status &robot_status)
{
    robot_state = (RobotState)robot_status.robot_state;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hmi_agent_node");
    ros::NodeHandle n;
    ros::Rate rate(RATE);
    node = &n;

	bool required_params_found = true;

    // Operator
    required_params_found &= n.getParam(CKSP(gauge_axis_id), gauge_axis_id);
    required_params_found &= n.getParam(CKSP(elevator_vertical_axis_id), elevator_vertical_axis_id);
    required_params_found &= n.getParam(CKSP(claw_open_button_id), claw_open_button_id);

	if (!required_params_found)
	{
		ROS_ERROR("Missing required parameters for node %s. Please check the list and make sure all required parameters are included", ros::this_node::getName().c_str());
		return 1;
	}

    ros::Subscriber joystick_status_sub = node->subscribe("/JoystickStatus", 100, joystick_status_callback);
    ros::Subscriber robot_status_sub = node->subscribe("/RobotStatus", 1, robot_status_callback);
    
    operator_joystick = new Joystick(0);

    ros::spin();

    return 0;
}