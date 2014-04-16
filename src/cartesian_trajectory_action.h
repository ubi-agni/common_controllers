// Copyright 2014 WUT
/*
 * cartesian_trajectory_action.h
 *
 *  Created on: 5 mar 2014
 *      Author: konradb3
 */

#ifndef CARTESIAN_TRAJECTORY_ACTION_H_
#define CARTESIAN_TRAJECTORY_ACTION_H_

#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"
#include "cartesian_trajectory_msgs/CartesianTrajectory.h"

class CartesianTrajectoryAction: public RTT::TaskContext {
 public:
  explicit CartesianTrajectoryAction(const std::string& name);
  virtual ~CartesianTrajectoryAction();

  bool startHook();
  void updateHook();

 private:
  RTT::OutputPort<cartesian_trajectory_msgs::CartesianTrajectoryConstPtr> port_cartesian_trajectory_command_;
  RTT::InputPort<cartesian_trajectory_msgs::CartesianTrajectory> port_cartesian_trajectory_;
};

#endif  // CARTESIAN_TRAJECTORY_ACTION_H_
