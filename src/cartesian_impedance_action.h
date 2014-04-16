// Copyright 2014 WUT
/*
 * cartesian_impedance_action.h
 *
 *  Created on: 5 mar 2014
 *      Author: konradb3
 */

#ifndef CARTESIAN_IMPEDANCE_ACTION_H_
#define CARTESIAN_IMPEDANCE_ACTION_H_

#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"
#include "cartesian_trajectory_msgs/CartesianImpedanceTrajectory.h"

class CartesianImpedanceAction: public RTT::TaskContext {
 public:
  explicit CartesianImpedanceAction(const std::string& name);
  virtual ~CartesianImpedanceAction();

  bool startHook();
  void updateHook();

 private:
  RTT::OutputPort<cartesian_trajectory_msgs::CartesianImpedanceTrajectoryConstPtr> port_cartesian_trajectory_command_;
  RTT::InputPort<cartesian_trajectory_msgs::CartesianImpedanceTrajectory> port_cartesian_trajectory_;
};

#endif  // CARTESIAN_IMPEDANCE_ACTION_H_
