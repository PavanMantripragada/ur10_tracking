#include <ros/ros.h>
#include <math.h>
#include <numeric> 
#include <string>
#include <interactive_markers/interactive_marker_server.h>
#include "./arm_controller/UR10_planning_options.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_interface/planning_interface.h>


namespace pose_buffer
{

class buffer
{  
public:
    int size;
    bool status = false;
    double AVG_x = 200.0, AVG_y = 219.0, AVG_z = -2.3;
    geometry_msgs::Pose latest_pose;
    std::vector<double> x_pos, y_pos, z_pos; 
    buffer(int n)
    {
      size = n;
    }

    void add_pose(geometry_msgs::Pose new_pose)
    {
      latest_pose = new_pose;
      x_pos.insert(x_pos.begin(), new_pose.position.x);
      y_pos.insert(y_pos.begin(), new_pose.position.y);
      z_pos.insert(z_pos.begin(), new_pose.position.z);
      if (x_pos.size() > size)
      {
        x_pos.pop_back();
        y_pos.pop_back();
        z_pos.pop_back();
      }
    }

    void isStable()
    {
      double dx, dy, dz;
      status = false;
      if (x_pos.size() == size)
      {
        AVG_x = std::accumulate(x_pos.begin(), x_pos.end(), 0.0) / x_pos.size();
        AVG_y = std::accumulate(y_pos.begin(), y_pos.end(), 0.0) / y_pos.size();
        AVG_z = std::accumulate(z_pos.begin(), z_pos.end(), 0.0) / z_pos.size();
        dx = fabs(AVG_x-x_pos[0]);
        dy = fabs(AVG_y-y_pos[0]);
        dz = fabs(AVG_z-z_pos[0]);
        if (dx < 0.000001 && dy < 0.000001 && dz < 0.000001)
        {
          status = true;
        }
      } 
    }

};
}


// global variable
pose_buffer::buffer reference_position(20);


void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  geometry_msgs::Pose current_pose = feedback->pose;
  reference_position.add_pose(current_pose);
  reference_position.isStable();
  //ROS_INFO("x : %f, y : %f, z : %f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
}


bool planToPoseTarget(
    UR10_planning::PlanningOptions &options,
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    geometry_msgs::Pose &target_pose, std::string &reference_frame,
    moveit::planning_interface::MoveGroupInterface::Plan &plan,
    std::string &end_effector_name) {
  move_group_interface.clearPoseTargets();
  move_group_interface.setPlanningTime(options.set_planning_time);
  move_group_interface.allowReplanning(options.allow_replanning);
  move_group_interface.setNumPlanningAttempts(options.num_attempts);
  move_group_interface.setMaxAccelerationScalingFactor(options.acceleration_scaling_factor);
  move_group_interface.setMaxVelocityScalingFactor(options.velocity_scaling_factor);
  move_group_interface.setStartState(*move_group_interface.getCurrentState());
  move_group_interface.setPoseTarget(target_pose);
  if (reference_frame != "") {
    move_group_interface.setPoseReferenceFrame(reference_frame);
  }
  if (end_effector_name != "") {
    move_group_interface.setEndEffector(end_effector_name + "_ee");
  }
  move_group_interface.setPlannerId("TRRTkConfigDefault");
  ROS_INFO("Planning for: %s", move_group_interface.getEndEffector().c_str());

  // Do planning for entire group
  bool plan_success = false;
  plan_success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // Return success or failure
  if (plan_success) {
    ROS_INFO("Plan Succeeded");
  } else {
    ROS_INFO("Plan Failed");
  }
  ros::Duration(2.0).sleep();

  return plan_success;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_marker");
  ros::NodeHandle n;
  interactive_markers::InteractiveMarkerServer server("simple_marker");

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "tracker_marker";
  int_marker.description = "2-DOF Control";

  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.05;
  box_marker.scale.y = 0.05;
  box_marker.scale.z = 0.05;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl rotate_control;
  rotate_control.orientation.w = 1;
  rotate_control.orientation.x = 1;
  rotate_control.orientation.y = 0;
  rotate_control.orientation.z = 0;
  rotate_control.name = "move_x";
  rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(rotate_control);

  rotate_control.orientation.w = 1;
  rotate_control.orientation.x = 0;
  rotate_control.orientation.y = 1;
  rotate_control.orientation.z = 0;
  rotate_control.name = "move_y";
  rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(rotate_control);

  rotate_control.orientation.w = 1;
  rotate_control.orientation.x = 0;
  rotate_control.orientation.y = 0;
  rotate_control.orientation.z = 1;
  rotate_control.name = "move_z";
  rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(rotate_control);

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  UR10_planning::PlanningOptions default_options;
  std::string ref_frame = "world";
  std::string end_effector_name = "";
  moveit::planning_interface::MoveGroupInterface::Plan output_plan;

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  ros::AsyncSpinner spinner(1);
  spinner.start();
  server.insert(int_marker, &processFeedback);
  bool prev_status = false;
  while(n.ok())
  {
   bool plan_status = false;
   server.applyChanges();
   if (!prev_status && reference_position.status)
   {
      ROS_INFO("Stable at :::::::::: x : %f, y : %f, z : %f", reference_position.x_pos[0], reference_position.y_pos[0], reference_position.z_pos[0]);
      plan_status = planToPoseTarget(default_options,move_group,reference_position.latest_pose,ref_frame,output_plan,end_effector_name);
      if (plan_status)
      {
      	move_group.execute(output_plan);
      }
      else
      {
        ROS_INFO("couldn't plan a path");
      }
   }
   prev_status = reference_position.status;
  }

}