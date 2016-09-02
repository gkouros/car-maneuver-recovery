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

#ifndef CAR_MANEUVER_RECOVERY_CAR_MANEUVER_RECOVERY_H
#define CAR_MANEUVER_RECOVERY_CAR_MANEUVER_RECOVERY_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/recovery_behavior.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Twist.h>

namespace car_maneuver_recovery
{

  /**
   * @class CarManeuverRecovery
   * @brief A move base plugin that implements car like maneuvers as a
   * recovery behavior
   */
  class CarManeuverRecovery : public nav_core::RecoveryBehavior
  {
    public:

      /**
       * @brief Constructor
       */
      CarManeuverRecovery();

      /**
       * @brief Destructor
       */
      ~CarManeuverRecovery();

      /**
       * @brief Initializes plugin
       * @param name: name of instance
       * @param tfListener: ptr to the tf transform listener of the node
       * @param globalCostmapROS: ptr to the global costmap of the node
       * @param localCostmapROS: ptr to the local costmap of the node
       */
      void initialize(std::string name, tf::TransformListener* tfListener,
        costmap_2d::Costmap2DROS* globalCostmapROS,
        costmap_2d::Costmap2DROS* localCostmapROS);

      /**
       * @brief Executes the car maneuver recovery behavior
       */
      void runBehavior();

      /**
       * @brief Calculates line cost
       * @param point1: line start point
       * @param point2: line end point
       */
      double lineCost(geometry_msgs::Point point1, geometry_msgs::Point point2);

    private:

      //! name of instance
      std::string name_;
      //! contains the initialization state of the plugin
      bool initialized_;

      //! twist publisher
      ros::Publisher twistPub_;

      //! tf transform listener
      tf::TransformListener* tfListener_;
      //! local costmap ros wrapper ptr
      costmap_2d::Costmap2DROS* localCostmapROS_;
      //! global costmap ros wrapper ptr
      costmap_2d::Costmap2DROS* globalCostmapROS_;
      //! costmap world model
      base_local_planner::CostmapModel* worldModel_;

      //! max speed of robot
      double recoverySpeed_;
      //! max steering angle of robot
      double recoverySteeringAngle_;
      //! wheelbase of robot
      double wheelbase_;
      //! four wheel steering status
      bool fourWheelSteering_;
      //! use crab steering instead of counter steering or ackermann steering
      bool crabSteering_;
      //! padding used to get costs around footprint and not on its border
      double extraFootprintPadding_;
      //! recovery behavior timeout
      double timeout_;
      //! if true plugin prints costmap costs of the sides of the footprint
      bool displayCosts_;

  };

}  // namespace car_maneuver_recovery

#endif  // CAR_MANEUVER_RECOVERY_CAR_MANEUVER_RECOVERY_H
