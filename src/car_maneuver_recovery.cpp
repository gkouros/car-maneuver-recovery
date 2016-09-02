/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, George Kouros.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:  George Kouros
 *********************************************************************/

#include "car_maneuver_recovery/car_maneuver_recovery.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Point.h>
#include <boost/foreach.hpp>

// register this class as a recovery behavior plugin
PLUGINLIB_DECLARE_CLASS(car_maneuver_recovery, CarManeuverRecovery,
  car_maneuver_recovery::CarManeuverRecovery, nav_core::RecoveryBehavior)

namespace car_maneuver_recovery
{

  CarManeuverRecovery::CarManeuverRecovery()
  : tfListener_(NULL), globalCostmapROS_(NULL), localCostmapROS_(NULL)
  , worldModel_(NULL), initialized_(false)
  {
  }


  CarManeuverRecovery::~CarManeuverRecovery()
  {
    delete worldModel_;
  }


  void CarManeuverRecovery::initialize(std::string name,
    tf::TransformListener* tfListener,
    costmap_2d::Costmap2DROS* globalCostmapROS,
    costmap_2d::Costmap2DROS* localCostmapROS)
  {
    if (initialized_)
    {
      ROS_ERROR("Plugin already initialized. Doing nothing.");
      return;
    }

    // store arguments
    name_ = name;
    tfListener_ = tfListener;
    globalCostmapROS_ = globalCostmapROS;
    localCostmapROS_ = localCostmapROS;

    // load parameters
    ros::NodeHandle pnh("~/" + name);

    pnh.param("recovery_speed", recoverySpeed_, 0.5);
    pnh.param("recovery_steering_angle", recoverySteeringAngle_, 0.5);
    pnh.param("wheelbase", wheelbase_, 0.5);
    pnh.param<bool>("four_wheel_steering", fourWheelSteering_, false);
    pnh.param<bool>("crab_steering", crabSteering_, false);
    pnh.param<double>("timeout", timeout_, 5.0);
    pnh.param<double>("extra_footprint_padding", extraFootprintPadding_, 0.01);
    pnh.param<bool>("display_costs", displayCosts_, false);

    twistPub_ = pnh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // get world model
    worldModel_ = new base_local_planner::CostmapModel(
      *localCostmapROS_->getCostmap());

    // set initialization state
    initialized_ = true;
  }


  void CarManeuverRecovery::runBehavior()
  {
    if (!initialized_)
    {
      ROS_ERROR("Plugin must be initialized before recovery behavior is run!");
      return;
    }

    if(globalCostmapROS_ == NULL || localCostmapROS_ == NULL)
    {
      ROS_ERROR("The costmaps passed to the CarManeuverRecovery object cannot "
        "be NULL. Doing nothing.");
      return;
    }

    ROS_WARN("Car Maneuver recovery behavior started.");

    ros::Rate loopRate(5);
    ros::Time time = ros::Time::now();

    while (ros::ok() && (ros::Time::now() - time).toSec() < timeout_)
    {
      // get robot footprint
      std::vector<geometry_msgs::Point> footprint;
      footprint = localCostmapROS_->getRobotFootprint();

      // pad footprint
      costmap_2d::padFootprint(footprint, extraFootprintPadding_);

      // transform robot footprint to oriented footprint
      tf::Stamped<tf::Pose> robotPose;
      localCostmapROS_->getRobotPose(robotPose);
      std::vector<geometry_msgs::Point> orientedFootprint;
      costmap_2d::transformFootprint(robotPose.getOrigin().getX(),
        robotPose.getOrigin().getY(), tf::getYaw(robotPose.getRotation()),
        footprint, orientedFootprint);
      footprint = orientedFootprint;

      double frontLineCost = ceil(lineCost(footprint[2], footprint[3]));
      double rearLineCost =  ceil(lineCost(footprint[0], footprint[1]));
      double leftLineCost =  ceil(lineCost(footprint[1], footprint[2]));
      double rightLineCost = ceil(lineCost(footprint[0], footprint[3]));

      ROS_INFO_STREAM_COND(displayCosts_, "Front side cost: " << frontLineCost);
      ROS_INFO_STREAM_COND(displayCosts_, "Rear side cost: " << rearLineCost);
      ROS_INFO_STREAM_COND(displayCosts_, "Left side cost: " << leftLineCost);
      ROS_INFO_STREAM_COND(displayCosts_, "Right side cost: " << rightLineCost);

      int front = frontLineCost < 128;
      int rear = rearLineCost < 128;
      int left = leftLineCost < 128;
      int right = rightLineCost < 128;

      if (front && rear && left && right)  // robot is free
      {
        ROS_INFO("Robot is not obstructed!");
        break;
      }
      else if (!front && !rear)  // robot cannot go sideways or turn in place
      {
        ROS_FATAL("Unable to recover!");
        break;
      }

      geometry_msgs::Twist cmd;

      double speed;

      if (front && rear)
        speed =  recoverySpeed_ * ((frontLineCost > rearLineCost) ? 1 : -1);
      else
        speed = (front - rear) * recoverySpeed_;

      if (fourWheelSteering_)  // 4WS steering
      {
        cmd.linear.x = speed;

        if (crabSteering_)  // 4WS crab steering
          cmd.linear.y = (left - right) * fabs(speed)
            * tan(recoverySteeringAngle_);
        else  // 4WS counter steering
        {
          double fsa =
            (speed > 0.0) ? (left - right) * recoverySteeringAngle_ : 0.0;
          double rsa =
            (speed < 0.0) ? (right - left) * recoverySteeringAngle_ : 0.0;
          double beta = atan((tan(fsa) + tan(rsa)) / 2);
          cmd.linear.x = speed;
          cmd.linear.y = cmd.linear.x * tan(beta);
          cmd.angular.z = speed * cos(beta) * (tan(fsa) - tan(rsa))/ wheelbase_;
        }
      }
      else  // ackermann steering
      {
        double fsa = (left - right) * recoverySteeringAngle_;
        double beta = atan(tan(fsa) / 2);
        cmd.linear.x = speed * sqrt(1 /  (1 + pow(tan(beta), 2)));
        cmd.linear.y = cmd.linear.x * tan(beta);
        cmd.angular.z = speed * cos(beta) * tan(fsa) / wheelbase_;
      }

      // publish cmd
      twistPub_.publish(cmd);

      loopRate.sleep();
    }

    if ((ros::Time::now() - time).toSec() > timeout_)
      ROS_WARN("Car Maneuver recovery behavior timed out!");

    ROS_WARN("Car maneuver recovery behavior finished.");
  }


  double CarManeuverRecovery::lineCost(geometry_msgs::Point point1,
    geometry_msgs::Point point2)
  {
    unsigned int x[3], y[3];
    localCostmapROS_->getCostmap()->worldToMap(point1.x, point1.y, x[0], y[0]);
    localCostmapROS_->getCostmap()->worldToMap(point2.x, point2.y, x[2], y[2]);

    x[1] = (x[0] + x[2]) / 2;
    y[1] = (y[0] + y[2]) / 2;

    double cost = 0.0;

    for (unsigned int i = 0; i < 3; i++)
      cost += static_cast<unsigned int>(
        localCostmapROS_->getCostmap()->getCost(x[i], y[i])) / 3.0;

    return cost;
  }

}  // namespace car_maneuver_recovery
