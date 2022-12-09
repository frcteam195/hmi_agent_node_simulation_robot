#pragma once
#include <map>
#include <string>

enum class SubsystemCategories
{
    INVALID,
    TURRET,
    DRIVEBASE,
    CLIMBER,
    INTAKE
};

std::map<SubsystemCategories, std::string> categories = {
    {SubsystemCategories::TURRET, "Turret"},
    {SubsystemCategories::DRIVEBASE, "DriveBase"},
    {SubsystemCategories::CLIMBER, "Climber"},
    {SubsystemCategories::INTAKE, "Intake"},
};

enum class TurretActions
{
    SHOOT_TURRET,
    DEACTIVATE_AUTOMATIC,
    ACTIVATE_AUTOMATIC
};
std::map<TurretActions, std::string> turret_actions = {
    {TurretActions::SHOOT_TURRET, "ShootTurret"},
    {TurretActions::DEACTIVATE_AUTOMATIC, "DeactivateAutomatic"},
    {TurretActions::ACTIVATE_AUTOMATIC, "ActivateAutomatic"},
};
enum class DrivebaseActions
{
    DEACTIVATE_AUTOMATIC,
    RUN_TRAJECTORY_NAME
};

std::map<DrivebaseActions, std::string> drive_base_actions = {
    {DrivebaseActions::DEACTIVATE_AUTOMATIC, "DeactivateAutomatic"},
    {DrivebaseActions::RUN_TRAJECTORY_NAME, "RunTrajectoryName"},
};
enum class IntakeActions
{
    INTAKE_BALL,
    REJECT_BALL,
    DEACTIVATE_AUTOMATIC,
    ACTIVATE_AUTOMATIC
};

std::map<IntakeActions, std::string> intake_actions = {
    {IntakeActions::DEACTIVATE_AUTOMATIC, "DeactivateAutomatic"},
    {IntakeActions::ACTIVATE_AUTOMATIC, "ActivateAutomatic"},
    {IntakeActions::INTAKE_BALL, "IntakeBall"},
    {IntakeActions::REJECT_BALL, "RejectBall"},
};
enum class ClimbingActions
{
    EMERGENCY_STOP,
    DEPLOY_HOOKS,
    RETRACT_HOOKS,
    ACTIVATE_AUTO_CLIMBING
};
std::map<ClimbingActions, std::string> climbing_actions = {
    {ClimbingActions::EMERGENCY_STOP, "EmergencyStop"},
    {ClimbingActions::DEPLOY_HOOKS, "DeployHooks"},
    {ClimbingActions::RETRACT_HOOKS, "RetractHooks"},
    {ClimbingActions::ACTIVATE_AUTO_CLIMBING, "ActivateAutoClimbing"},
};