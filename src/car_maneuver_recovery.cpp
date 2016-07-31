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
    ros::NodeHandle pnh_("~/" + name);
    pnh_.param("max_speed", maxSpeed_, 0.5);
    pnh_.param("max_steering_angle", maxSteeringAngle_, 0.5);
    pnh_.param("wheelbase", wheelbase_, 0.5);

    twistPub_ = pnh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

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

    if(globalCostmapROS_ == NULL || localCostmapROS_ == NULL){
      ROS_ERROR("The costmaps passed to the RotateRecovery object cannot be "
        "NULL. Doing nothing.");
      return;
    }

    ROS_WARN("Car Maneuver recovery behavior started.");

    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.1;
    twistPub_.publish(cmd);
  }

}  // namespace car_maneuver_recovery
