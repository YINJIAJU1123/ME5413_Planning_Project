/** path_tracker_node.hpp
 *
 * Copyright (C) 2024 Yin Jiaju , National University of Singapore
 *
 * MIT License
 *
 * Declarations for PathTrackerNode class
 */

#include "me5413_world/path_tracker_node.hpp"
#include "angles/angles.h"
#include "me5413_world/math_utils.hpp"
#include <cmath> 

namespace me5413_world 
{
double SPEED_TARGET;
double PID_Kp, PID_Ki, PID_Kd;
double ahead_distance; 
bool PARAMS_UPDATED;

void dynamicParamCallback(const me5413_world::path_trackerConfig& config, uint32_t level)
{
  SPEED_TARGET = config.speed_target;

  PID_Kp = config.PID_Kp;
  PID_Ki = config.PID_Ki;
  PID_Kd = config.PID_Kd;

  ahead_distance = 3;
  PARAMS_UPDATED = true;
}

PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_)
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
  sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);
  robot_frame_ = "base_link";
  world_frame_ = "world";
  pid_ = control::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd);
}

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  world_frame_ = odom->header.frame_id;
  robot_frame_ = odom->child_frame_id;
  odom_world_robot_ = *odom.get();
  return;
}

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  pub_cmd_vel_.publish(ControlOutputs(odom_world_robot_, path));
return;}

geometry_msgs::Twist PathTrackerNode::ControlOutputs(const nav_msgs::Odometry& odom_robot, const nav_msgs::Path::ConstPtr& path)
{

  tf2::Quaternion q_robot;
  tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  double roll, pitch, yaw_robot;
  m_robot.getRPY(roll, pitch, yaw_robot);
  
  tf2::Vector3 point_robot, point_goal;
  tf2::fromMsg(odom_robot.pose.pose.position, point_robot);
  tf2::Vector3 robot_vel;
  tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, robot_vel);
  const double velocity = robot_vel.length();
  geometry_msgs::Twist cmd_vel;
  if (PARAMS_UPDATED)
  {
    pid_.updateSettings(PID_Kp, PID_Ki, PID_Kd);
    PARAMS_UPDATED = false;
  }
  cmd_vel.linear.x = pid_.calculate(SPEED_TARGET, velocity);

  geometry_msgs::Point goal_point = calculateTargetPoint(point_robot, path, ahead_distance);
  double yaw_goal = atan2(goal_point.y - point_robot.y(), goal_point.x - point_robot.x());
  double yaw_error = angles::normalize_angle(yaw_goal - yaw_robot); // normalize it to [-pi, pi)
  cmd_vel.angular.z = 2* yaw_error;    
  return cmd_vel;
}

geometry_msgs::Point PathTrackerNode::calculateTargetPoint(const tf2::Vector3& currentPosition, const nav_msgs::Path::ConstPtr& navigationPath, double lookaheadDist)  
{
  auto lookaheadPoint = std::find_if(navigationPath->poses.begin(), navigationPath->poses.end(), [&](const geometry_msgs::PoseStamped& poseStamped) {
    
    tf2::Vector3 pointPos;
    tf2::fromMsg(poseStamped.pose.position, pointPos);

    double distToCurrentPos = sqrt(pow(pointPos.x() - currentPosition.x(), 2) +
                                   pow(pointPos.y() - currentPosition.y(), 2) +
                                   pow(pointPos.z() - currentPosition.z(), 2));
    return distToCurrentPos >= lookaheadDist;
  });

  if (lookaheadPoint != navigationPath->poses.end()) {
    return lookaheadPoint->pose.position;
  
  } else {
    return navigationPath->poses.back().pose.position;
  }}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_tracker_node");
  me5413_world::PathTrackerNode path_tracker_node;
  ros::spin();  
  return 0;
  }