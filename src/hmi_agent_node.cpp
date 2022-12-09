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
Joystick *drive_joystick;
Joystick *arm_joystick;
Joystick *button_box_1_joystick;
Joystick *button_box_2_joystick;
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
    static double turret_aim_degrees = 0;
    static double turret_hood_degrees = 0;
    static double turret_speed_rpm = 0;
    hmi_agent_node::HMI_Signals output_signals;

    if (button_box_2_joystick->getButton(bb2_set_near_button_id)) //near
    {
        turret_hood_degrees = 0;
        turret_speed_rpm = 1800;
    }
    if (button_box_2_joystick->getButton(bb2_set_near_mid_button_id)) //near mid
    {
        turret_hood_degrees = -6;
        turret_speed_rpm = 2100;
    }
    if (button_box_2_joystick->getButton(bb2_set_mid_button_id)) //mid
    {
        turret_hood_degrees = -12;
        turret_speed_rpm = 2400;
    }
    if (button_box_2_joystick->getButton(bb2_set_mid_far_button_id)) //mid far
    {
        turret_hood_degrees = -18;
        turret_speed_rpm = 2700;
    }
    if (button_box_2_joystick->getButton(bb2_set_far_button_id)) //far
    {
        turret_hood_degrees = -24;
        turret_speed_rpm = 3000;
    }


    output_signals.turret_manual = arm_joystick->getButton(arm_turret_manual_button_id);
    static constexpr double MAX_TURRET_DEG = 180;
    static constexpr double MIN_TURRET_DEG = -180;
    static constexpr double MAX_HOOD_DEG = 25;
    static constexpr double MIN_HOOD_DEG = 0;
    static constexpr double MAX_SHOOTER_RPM = 5000;
    static constexpr double MIN_SHOOTER_RPM = 0;

    int invert_arm_twist_axis = arm_twist_axis_inverted ? -1 : 1;

    if (output_signals.turret_manual)
    {
        if (invert_arm_twist_axis * arm_joystick->getFilteredAxis(arm_twist_axis_id, arm_axis_deadband) < 0) //aim left
        {
            turret_aim_degrees += 2;
        }
        else if (invert_arm_twist_axis * arm_joystick->getFilteredAxis(arm_twist_axis_id, arm_axis_deadband) > 0) //aim right
        {
            turret_aim_degrees -= 2;
        }
        
        turret_aim_degrees = std::min(std::max(turret_aim_degrees, MIN_TURRET_DEG), MAX_TURRET_DEG);



        //fix
        turret_aim_degrees = invert_arm_twist_axis * arm_joystick->getFilteredAxis(arm_twist_axis_id, arm_axis_deadband) / 4.0;
    }


    if (arm_joystick->getButton(arm_hood_up_button_id) || arm_joystick->getPOV(arm_pov_id) == arm_hood_up_pov_angle) //hood up
    {
        turret_hood_degrees -= 0.2;
    }
    if (arm_joystick->getButton(arm_hood_down_button_id)|| arm_joystick->getPOV(arm_pov_id) == arm_hood_down_pov_angle) //hood down
    {
        turret_hood_degrees += 0.2;
    }
    turret_hood_degrees = std::min(std::max(turret_hood_degrees, MIN_HOOD_DEG), MAX_HOOD_DEG);


    ////////////////NOT USED//////////////////////////////
    if (arm_joystick->getButton(arm_rpm_up_button_id)) //speed up
    {
        turret_speed_rpm += 60;
    }
    if (arm_joystick->getButton(arm_rpm_down_button_id)) //speed down
    {
        turret_speed_rpm -= 60;
    }
    turret_speed_rpm = std::min(std::max(turret_speed_rpm, MIN_SHOOTER_RPM), MAX_SHOOTER_RPM);
    ///////////////////////////////////////////////////////

    output_signals.drivetrain_brake = drive_joystick->getButton(drive_brake_button_id);
    int invert_axis_fwd_back = drive_fwd_back_axis_inverted ? -1 : 1;
    int invert_axis_turn = drive_turn_axis_inverted ? -1 : 1;
    output_signals.drivetrain_fwd_back = invert_axis_fwd_back * drive_joystick->getFilteredAxis(drive_fwd_back_axis_id, drive_axis_deadband);
    double turn = invert_axis_turn * drive_joystick->getFilteredAxis(drive_turn_axis_id, drive_axis_deadband);
    output_signals.drivetrain_left_right = 0.8 * turn;


    double x = output_signals.drivetrain_fwd_back;
    double y = turn;
    int invert_axis_z = drive_z_axis_inverted ? -1 : 1;
    double z = invert_axis_z * drive_joystick->getFilteredAxis(drive_z_axis_id, drive_z_axis_deadband);

    double r = ck::math::hypotenuse(x, y);
    double theta = ck::math::polar_angle_rad(x, y);

    output_signals.drivetrain_swerve_direction = theta;
    output_signals.drivetrain_swerve_percent_fwd_vel = ck::math::limit(r, 0.0, 1.0);
    output_signals.drivetrain_swerve_percent_angular_rot = z;

    // if (drive_joystick->getAxisActuated(3, 0.35))
    // {
    //     output_signals.drivetrain_left_right *= 0.4;
    //     output_signals.drivetrain_fwd_back *= 0.4;
    // }
    // output_signals.drivetrain_quickturn = drive_joystick->getAxisActuated(2, 0.35);
    output_signals.drivetrain_quickturn = drive_joystick->getButton(drive_quickturn_button_id);
    output_signals.turret_aim_degrees = turret_aim_degrees;
    output_signals.turret_hood_degrees = turret_hood_degrees;
    output_signals.turret_speed_rpm = turret_speed_rpm;
    output_signals.intake_rollers = button_box_2_joystick->getButton(bb2_intake_rollers_button_id) | drive_joystick->getButton(drive_intake_rollers_button_id) | arm_joystick->getButton(arm_intake_rollers_button_id) | arm_joystick->getAxisActuated(arm_intake_rollers_trigger_id, 0.45);
    output_signals.retract_intake = button_box_2_joystick->getButton(bb2_retract_intakes_button_id) | output_signals.intake_rollers | arm_joystick->getButton(arm_retract_intake_button_id) | arm_joystick->getAxisActuated(arm_retract_intake_trigger_id, 0.45) | (arm_joystick->getPOV(arm_pov_id) == arm_shoot_3ball_pov_angle);
    output_signals.manual_intake = button_box_2_joystick->getButton(bb2_manual_intake_button_id) | arm_joystick->getButton(arm_manual_intake_button_id);
    output_signals.manual_outake_back = button_box_2_joystick->getButton(bb2_manual_outtake_back_button_id) | (arm_joystick->getPOV(arm_pov_id) == arm_manual_outtake_back_pov_angle) | arm_joystick->getButton(arm_manual_outtake_back_button_id);
    output_signals.manual_outake_front = button_box_2_joystick->getButton(bb2_manual_outtake_front_button_id) | (arm_joystick->getPOV(arm_pov_id) == arm_manual_outtake_front_pov_angle) | arm_joystick->getButton(arm_manual_outtake_front_button_id);
    output_signals.stop_climber = button_box_1_joystick->getButton(bb1_stop_climber_button_id);
    output_signals.allow_shoot = arm_joystick->getButton(arm_allow_shoot_button_id) | drive_joystick->getButton(drive_allow_shoot_button_id)| (arm_joystick->getPOV(arm_pov_id) == arm_shoot_3ball_pov_angle);
    output_signals.shoot_3ball = (arm_joystick->getPOV(arm_pov_id) == arm_shoot_3ball_pov_angle);
    output_signals.increase_offset = button_box_1_joystick->getButton(bb1_increase_rpm_offset_button_id) | arm_joystick->getButton(arm_rpm_up_button_id);
    output_signals.decrease_offset = button_box_1_joystick->getButton(bb1_decrease_rpm_offset_button_id) | arm_joystick->getButton(arm_rpm_down_button_id);
    output_signals.deploy_hooks = button_box_1_joystick->getButton(bb1_deploy_hooks_button_id);
    output_signals.begin_climb = button_box_1_joystick->getButton(bb1_begin_climb_button_id);
    output_signals.retract_hooks = button_box_1_joystick->getButton(bb1_retract_hooks_button_id);
    output_signals.forced_reset_retract_hooks = false;  //DO NOT SET THIS SIGNAL HERE TO ANYTHING OTHER THAN FALSE
    output_signals.angle_increase_offset = button_box_1_joystick->getButton(bb1_increase_angle_button_id) | (arm_joystick->getPOV(arm_pov_id) == arm_hood_up_pov_angle);
    output_signals.angle_decrease_offset = button_box_1_joystick->getButton(bb1_decrease_angle_button_id) | (arm_joystick->getPOV(arm_pov_id) == arm_hood_down_pov_angle);
    output_signals.climber_retry_last_stage = button_box_1_joystick->getButton(bb1_retry_last_stage_climber_button_id); //DO NOT SET THIS SIGNAL HERE TO ANYTHING OTHER THAN FALSE
    output_signals.intake_do_not_eject = false;


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

    //Drive
	required_params_found &= n.getParam(CKSP(drive_fwd_back_axis_id), drive_fwd_back_axis_id);
	required_params_found &= n.getParam(CKSP(drive_fwd_back_axis_inverted), drive_fwd_back_axis_inverted);
	required_params_found &= n.getParam(CKSP(drive_turn_axis_id), drive_turn_axis_id);
	required_params_found &= n.getParam(CKSP(drive_turn_axis_inverted), drive_turn_axis_inverted);
    required_params_found &= n.getParam(CKSP(drive_z_axis_id), drive_z_axis_id);
    required_params_found &= n.getParam(CKSP(drive_z_axis_inverted), drive_z_axis_inverted);
	required_params_found &= n.getParam(CKSP(drive_axis_deadband), drive_axis_deadband);
	required_params_found &= n.getParam(CKSP(drive_z_axis_deadband), drive_z_axis_deadband);
	required_params_found &= n.getParam(CKSP(drive_brake_button_id), drive_brake_button_id);
	required_params_found &= n.getParam(CKSP(drive_quickturn_button_id), drive_quickturn_button_id);
	required_params_found &= n.getParam(CKSP(drive_intake_rollers_button_id), drive_intake_rollers_button_id);
	required_params_found &= n.getParam(CKSP(drive_allow_shoot_button_id), drive_allow_shoot_button_id);

    //Arm
    required_params_found &= n.getParam(CKSP(arm_twist_axis_id), arm_twist_axis_id);
    required_params_found &= n.getParam(CKSP(arm_twist_axis_inverted), arm_twist_axis_inverted);
    required_params_found &= n.getParam(CKSP(arm_axis_deadband), arm_axis_deadband);
    required_params_found &= n.getParam(CKSP(arm_turret_manual_button_id), arm_turret_manual_button_id);
    required_params_found &= n.getParam(CKSP(arm_hood_up_button_id), arm_hood_up_button_id);
    required_params_found &= n.getParam(CKSP(arm_hood_down_button_id), arm_hood_down_button_id);
    required_params_found &= n.getParam(CKSP(arm_rpm_up_button_id), arm_rpm_up_button_id);
    required_params_found &= n.getParam(CKSP(arm_rpm_down_button_id), arm_rpm_down_button_id);
    required_params_found &= n.getParam(CKSP(arm_intake_rollers_button_id), arm_intake_rollers_button_id);
    required_params_found &= n.getParam(CKSP(arm_manual_intake_button_id), arm_manual_intake_button_id);
    required_params_found &= n.getParam(CKSP(arm_manual_outtake_front_pov_angle), arm_manual_outtake_front_pov_angle);
    required_params_found &= n.getParam(CKSP(arm_manual_outtake_back_pov_angle), arm_manual_outtake_back_pov_angle);
    required_params_found &= n.getParam(CKSP(arm_retract_intake_button_id), arm_retract_intake_button_id);
    required_params_found &= n.getParam(CKSP(arm_manual_outtake_front_button_id), arm_manual_outtake_front_button_id);
    required_params_found &= n.getParam(CKSP(arm_manual_outtake_back_button_id), arm_manual_outtake_back_button_id);
    required_params_found &= n.getParam(CKSP(arm_hood_up_pov_angle), arm_hood_up_pov_angle);
    required_params_found &= n.getParam(CKSP(arm_hood_down_pov_angle), arm_hood_down_pov_angle);
    required_params_found &= n.getParam(CKSP(arm_intake_rollers_trigger_id), arm_intake_rollers_trigger_id);
    required_params_found &= n.getParam(CKSP(arm_retract_intake_trigger_id), arm_retract_intake_trigger_id);
    required_params_found &= n.getParam(CKSP(arm_allow_shoot_button_id), arm_allow_shoot_button_id);
    required_params_found &= n.getParam(CKSP(arm_shoot_3ball_pov_angle), arm_shoot_3ball_pov_angle);

    //ButtonBox1
    // required_params_found &= n.getParam(CKSP(bb1_allow_shoot_button_id), bb1_allow_shoot_button_id);
    required_params_found &= n.getParam(CKSP(bb1_deploy_hooks_button_id), bb1_deploy_hooks_button_id);
    required_params_found &= n.getParam(CKSP(bb1_begin_climb_button_id), bb1_begin_climb_button_id);
    required_params_found &= n.getParam(CKSP(bb1_retract_hooks_button_id), bb1_retract_hooks_button_id);
    required_params_found &= n.getParam(CKSP(bb1_increase_rpm_offset_button_id), bb1_increase_rpm_offset_button_id);
    required_params_found &= n.getParam(CKSP(bb1_decrease_rpm_offset_button_id), bb1_decrease_rpm_offset_button_id);
    required_params_found &= n.getParam(CKSP(bb1_stop_climber_button_id), bb1_stop_climber_button_id);
    required_params_found &= n.getParam(CKSP(bb1_increase_angle_button_id), bb1_increase_angle_button_id);
    required_params_found &= n.getParam(CKSP(bb1_decrease_angle_button_id), bb1_decrease_angle_button_id);
    required_params_found &= n.getParam(CKSP(bb1_retry_last_stage_climber_button_id), bb1_retry_last_stage_climber_button_id);

    //ButtonBox2
    required_params_found &= n.getParam(CKSP(bb2_set_near_button_id), bb2_set_near_button_id);
    required_params_found &= n.getParam(CKSP(bb2_set_near_mid_button_id), bb2_set_near_mid_button_id);
    required_params_found &= n.getParam(CKSP(bb2_set_mid_button_id), bb2_set_mid_button_id);
    required_params_found &= n.getParam(CKSP(bb2_set_mid_far_button_id), bb2_set_mid_far_button_id);
    required_params_found &= n.getParam(CKSP(bb2_set_far_button_id), bb2_set_far_button_id);
    required_params_found &= n.getParam(CKSP(bb2_intake_rollers_button_id), bb2_intake_rollers_button_id);
    required_params_found &= n.getParam(CKSP(bb2_retract_intakes_button_id), bb2_retract_intakes_button_id);
    required_params_found &= n.getParam(CKSP(bb2_manual_intake_button_id), bb2_manual_intake_button_id);
    required_params_found &= n.getParam(CKSP(bb2_manual_outtake_back_button_id), bb2_manual_outtake_back_button_id);
    required_params_found &= n.getParam(CKSP(bb2_manual_outtake_front_button_id), bb2_manual_outtake_front_button_id);
    

	if (!required_params_found)
	{
		ROS_ERROR("Missing required parameters for node %s. Please check the list and make sure all required parameters are included", ros::this_node::getName().c_str());
		return 1;
	}

    ros::Subscriber joystick_status_sub = node->subscribe("/JoystickStatus", 100, joystick_status_callback);
    ros::Subscriber robot_status_sub = node->subscribe("/RobotStatus", 1, robot_status_callback);
    drive_joystick = new Joystick(0);
    arm_joystick = new Joystick(1);
    button_box_1_joystick = new Joystick(2);
    button_box_2_joystick = new Joystick(3);

    ros::spin();

    return 0;
}