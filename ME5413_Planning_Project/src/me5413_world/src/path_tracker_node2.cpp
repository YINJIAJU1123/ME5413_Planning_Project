/** path_tracker_node.hpp
 *
 * Copyright (C) 2024 Yin Jiaju , National University of Singapore
 *
 * MIT License
 *
 * Declarations for PathTrackerNode class
 */


#include "me5413_world/path_tracker_node.hpp"

namespace me5413_world
{

PathTrackerNode::PathTrackerNode(): tf2_listener_(tf2_buffer_, nh_)
{
  sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
  sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);
}

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  odom_world_robot_ = *odom;
}

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  if (path->poses.empty()) {
    ROS_WARN("Received empty path.");
    return;
  }
  local_path_ = *path;

  double lateral_error = computeLateralError(odom_world_robot_.pose.pose.position, local_path_);
  double heading_error = computeHeadingError(odom_world_robot_.pose.pose.orientation, local_path_);

  double control_input = lqrControl(lateral_error, heading_error);

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.3; 
  cmd_vel.angular.z = control_input; 

  pub_cmd_vel_.publish(cmd_vel);
}

double PathTrackerNode::computeLateralError(const geometry_msgs::Point& robot_position, const nav_msgs::Path& local_path)
{
  if (local_path.poses.empty()) {
    ROS_WARN("Received empty path.");
    return 0.0;
  }

  double min_distance = std::numeric_limits<double>::max();

  geometry_msgs::Point projection_point;

  for (const auto& pose : local_path.poses) {

    double dx = pose.pose.position.x - robot_position.x;
    double dy = pose.pose.position.y - robot_position.y;

    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < min_distance) {
      min_distance = distance;
      projection_point = pose.pose.position;
    }
  }

  double dx_proj = robot_position.x - projection_point.x;
  double dy_proj = robot_position.y - projection_point.y;

  double lateral_error = std::sqrt(dx_proj * dx_proj + dy_proj * dy_proj);

  return lateral_error;
}

double PathTrackerNode::computeHeadingError(const geometry_msgs::Quaternion& robot_orientation, const nav_msgs::Path& local_path)
{
  if (local_path.poses.size() < 2) {
    ROS_WARN("Not enough points in the local path to compute heading error.");
    return 0.0;
  }

  int lookahead_index = std::min(5, (int)local_path.poses.size() - 1); // adjustment points ahead

  const auto& next_point = local_path.poses[lookahead_index].pose.position;
  double path_angle = std::atan2(next_point.y - local_path.poses[0].pose.position.y, 
                                 next_point.x - local_path.poses[0].pose.position.x);

  tf2::Quaternion q_robot;
  tf2::fromMsg(robot_orientation, q_robot);
  double roll, pitch, robot_yaw;
  tf2::Matrix3x3(q_robot).getRPY(roll, pitch, robot_yaw);

  double heading_error = path_angle - robot_yaw;

  heading_error = atan2(sin(heading_error), cos(heading_error));

  return heading_error;
}

double PathTrackerNode::lqrControl(double lateral_error, double heading_error)
{

  Eigen::MatrixXd A(3, 3), B(2, 1), Q(2, 2), R(1, 1);

  // matrix adjustment
  A << 1, 0.1, 0.005,
     0, 1, 0.1,
     0, 0, 1;
  B << 0.01,
       0.1;
  Q << 1, 0,
       0, 10;
  R << 10;

  Eigen::MatrixXd P = calRicatti(A, B, Q, R);
  Eigen::MatrixXd K = -(R + B.transpose() * P * B).inverse() * B.transpose() * P * A;

  Eigen::MatrixXd X(2, 1);
  X << lateral_error,
       heading_error;

  Eigen::MatrixXd u = K * X;

  double control_input = u(0, 0); 

  return control_input;
}

Eigen::MatrixXd PathTrackerNode::calRicatti(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
{
    int N = 100; 
    double EPS = 1e-6; 

    Eigen::MatrixXd Qf = Q;
    Eigen::MatrixXd P = Qf;
    Eigen::MatrixXd P_;

    for (int i = 0; i < N; ++i) {
        P_ = A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;
        if ((P_ - P).norm() < EPS) {
            break;
        }  
        P = P_;
    }
    return P; 
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_tracker_node");
  me5413_world::PathTrackerNode path_publisher_node;
  ros::spin(); 
  return 0;
}
