#pragma once
#include "hmi_agent/ActionNames.hpp"

//Drive
int drive_fwd_back_axis_id;
bool drive_fwd_back_axis_inverted;
int drive_turn_axis_id;
bool drive_turn_axis_inverted;
int drive_z_axis_id;
bool drive_z_axis_inverted;
double drive_axis_deadband;
double drive_z_axis_deadband;
int drive_brake_button_id;
int drive_quickturn_button_id;
int drive_intake_rollers_button_id;
int drive_allow_shoot_button_id;

//Arm
int arm_twist_axis_id;
bool arm_twist_axis_inverted;
double arm_axis_deadband;
int arm_pov_id;
int arm_manual_outtake_back_pov_angle;
int arm_manual_outtake_front_pov_angle;
int arm_shoot_3ball_pov_angle;
int arm_turret_manual_button_id;
int arm_hood_up_button_id;
int arm_hood_down_button_id;
int arm_rpm_up_button_id;
int arm_rpm_down_button_id;
int arm_intake_rollers_button_id;
int arm_manual_intake_button_id;
int arm_retract_intake_button_id;
int arm_manual_outtake_front_button_id;
int arm_manual_outtake_back_button_id;
int arm_hood_up_pov_angle;
int arm_hood_down_pov_angle;
int arm_intake_rollers_trigger_id;
int arm_retract_intake_trigger_id;
int arm_allow_shoot_button_id;

//ButtonBox1
// int bb1_allow_shoot_button_id;
int bb1_deploy_hooks_button_id;
int bb1_begin_climb_button_id;
int bb1_retract_hooks_button_id;
int bb1_increase_rpm_offset_button_id;
int bb1_decrease_rpm_offset_button_id;
int bb1_stop_climber_button_id;
int bb1_increase_angle_button_id;
int bb1_decrease_angle_button_id;
int bb1_retry_last_stage_climber_button_id;

//ButtonBox2
int bb2_set_near_button_id;
int bb2_set_near_mid_button_id;
int bb2_set_mid_button_id;
int bb2_set_mid_far_button_id;
int bb2_set_far_button_id;
int bb2_intake_rollers_button_id;
int bb2_retract_intakes_button_id;
int bb2_manual_intake_button_id;
int bb2_manual_outtake_back_button_id;
int bb2_manual_outtake_front_button_id;