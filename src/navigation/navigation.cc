//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "f1tenth_course/AckermannCurvatureDriveMsg.h"
#include "f1tenth_course/Pose2Df.h"
#include "f1tenth_course/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include <cmath>

using Eigen::Vector2f;
using Eigen::Matrix2f;
using f1tenth_course::AckermannCurvatureDriveMsg;
using f1tenth_course::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;
using namespace std;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n, float dist) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
  distance_left_ = dist;
  current_velocity_ = 0;
  set_odom_ = true;
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  // cout << "loc: " << loc << "\n";
  // cout << "angle: " << angle << "\n";
  if (!set_odom_) {

    Matrix2f angles;
    angles << cos(-1 * odom_angle_), -sin(-1 * odom_angle_),
              sin(-1 * odom_angle_), cos(-1 * odom_angle_);

    Vector2f delta = angles * (loc - odom_loc_);

    float distance_travelled = sqrt(pow(delta.x(), 2) + pow(delta.y(), 2));

    cout << "distance travelled: " << distance_travelled << "\n";

    distance_left_ -= distance_travelled;
  } else {
    set_odom_ = false;
  }
  // update odometry location
  odom_loc_ = loc;

  // update odometery angle
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
}

void Navigation::Run() {
  // Create Helper functions here
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.

  if (!set_odom_) {
    float max_vel = 1.0;
    float acceleration = 3.0;
    float time_length = 0.05;

    // Initialize msg
    AckermannCurvatureDriveMsg msg;
    msg.curvature = 0;

    // Helper function to determine if we should brake
    auto should_stop = [&](float vel) {
      float stopping_dist = pow(vel, 2) / (2 * acceleration);
      if (distance_left_ <= stopping_dist) {
        return true;
      }
      return false;
    };

    // float initial_velocity = current_velocity_;

    // Helper function to get distance travelled this time step
    // auto get_dist_travelled = [&](float acc) {
    //   float d = (initial_velocity * time_length) + (0.5 * acc * pow(time_length, 2));
    //   if (d < 0) {
    //     d = 0;
    //   }
    //   return d;
    // };

    if (should_stop(current_velocity_) || distance_left_ <= 0) {
      // Deceleration
      current_velocity_ -= acceleration * time_length;
      if (current_velocity_ < 0) {
        current_velocity_ = 0;
      }
      // cout << "decelerating " << current_velocity_ << "\n";
      // distance_left_ -= get_dist_travelled(-1 * acceleration);
    } else if (current_velocity_ < max_vel) {
      // Acceleration
      current_velocity_ += acceleration * time_length;
      if (current_velocity_ > max_vel) {
        current_velocity_ = max_vel;
      }
      // cout << "accelerating " << current_velocity_ << "\n";
      // distance_left_ -= get_dist_travelled(acceleration);
    } else {
      // Cruising
      // distance_left_ -= get_dist_travelled(0.0);
    }

    cout << "distance_left_ " << distance_left_ << "\n";
    cout << "current_velocity_ " << current_velocity_ << "\n";

    msg.velocity = current_velocity_;

    drive_pub_.publish(msg);
  }
}

}  // namespace navigation
