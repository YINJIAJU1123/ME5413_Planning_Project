/** path_tracker_node.hpp
 *
 * Copyright (C) 2024 Yin Jiaju, National University of Singapore
 *
 * MIT License
 *
 * Declarations for PathTrackerNode class
 */

// path_tracker_node.hpp

#ifndef PATH_TRACKER_NODE_H_
#define PATH_TRACKER_NODE_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

namespace me5413_world
{

class PathTrackerNode
{
 public:
  PathTrackerNode();

 private:
  void robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void localPathCallback(const nav_msgs::Path::ConstPtr& path);
  double computeLateralError(const geometry_msgs::Point& robot_position, const nav_msgs::Path& local_path);
  double computeHeadingError(const geometry_msgs::Quaternion& robot_orientation, const nav_msgs::Path& local_path);
  double lqrControl(double lateral_error, double heading_error);
  static Eigen::MatrixXd calRicatti(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);

  ros::NodeHandle nh_;
  ros::Subscriber sub_robot_odom_;
  ros::Subscriber sub_local_path_;
  ros::Publisher pub_cmd_vel_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  nav_msgs::Odometry odom_world_robot_;
  nav_msgs::Path local_path_;
};

} // namespace me5413_world

#endif // PATH_TRACKER_NODE_H_
