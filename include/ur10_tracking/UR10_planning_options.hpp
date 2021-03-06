
#pragma once

/** @namespace UR10_planning UR10_planning_options.hpp
 * "include/shadowlibs/shadow_planning_options.hpp"
 *  @brief Contains PlanningOptions to set necessary parameters for planning
 */
namespace UR10_planning {
/** @struct PlanningOptions
 * @brief Struct to set planning parameters for MoveIt planning
 */
struct PlanningOptions {
  /** @var set_planning_time
   *  @brief Planning time limit
   */
  double set_planning_time;
  /** @var num_attempts
   *  @brief Number of attempts for replanning
   */
  int num_attempts;
  /** @var allow_replanning
   *  @brief Allow replanning if solution not found
   */
  bool allow_replanning;
  /** @var velocity_scaling_factor
   *  @brief The scaling factor for velocity of UR-10
   */
  double velocity_scaling_factor;
  /** @var acceleration_scaling_factor
   *  @brief The scaling factor for acceleration of UR-10
   */
  double acceleration_scaling_factor;
  /** @var goal_position_tolerance
   *  @brief Tolerance in goal position when planning
   */
  double goal_position_tolerance;
  /** @var goal_orientation_tolerance
   *  @brief Tolerance in goal orientation when planning
   */
  double goal_orientation_tolerance;
  /** @var goal_joint_tolerance
   *  @brief Tolerance in goal joint angle when planning
   */
  double goal_joint_tolerance;
  /**
   * @var end_effector_name
   * @brief Name of end effector to plan for
   */
  std::string end_effector_name;

  /**
   * @brief Default constructor, sets some reasonable defaults
   */
  PlanningOptions()
      : set_planning_time(2.0), allow_replanning(false), num_attempts(1),
        end_effector_name(""), goal_position_tolerance(0.01),
        velocity_scaling_factor(0.75), acceleration_scaling_factor(1.0),
        goal_orientation_tolerance(0.1), goal_joint_tolerance(0.01){};

  /**
   * @brief Custom constructor, for setting user defined values
   * @param set_planning_time Set time limit for completing planning
   * @param allow_replanning Allow replanning if failed or solution not found
   * @param num_attempts The number of attempts allowed for replanning
   * @param goal_position_tolerance The position tolerance for reaching goal
   * @param goal_orientation_tolerance The orientation tolerance for reaching
   * goal
   * @param goal_joint_tolerance The joint angle tolerance for reaching goal
   * @param end_effector_name The name of the end effector being planned for
   * @param velocity_scaling_factor The scaling factor for velocity of UR-10
   * @param acceleration_scaling_factor The scaling factor for acceleration of
   * UR-10
   */
  PlanningOptions(double set_planning_time, bool allow_replanning,
                  int num_attempts, double goal_position_tolerance,
                  double velocity_scaling_factor,
                  double acceleration_scaling_factor,
                  double goal_orientation_tolerance,
                  double goal_joint_tolerance, std::string end_effector_name)
      : set_planning_time(set_planning_time),
        allow_replanning(allow_replanning), num_attempts(num_attempts),
        goal_position_tolerance(goal_position_tolerance),
        velocity_scaling_factor(velocity_scaling_factor),
        acceleration_scaling_factor(acceleration_scaling_factor),
        goal_orientation_tolerance(goal_orientation_tolerance),
        goal_joint_tolerance(goal_joint_tolerance){};
};
} // namespace UR10_planning