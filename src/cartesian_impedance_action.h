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
#include "cartesian_trajectory_msgs/CartesianImpedance.h"
#include "cartesian_trajectory_msgs/CartesianImpedanceAction.h"
#include "cartesian_trajectory_msgs/CartesianImpedanceGoal.h"

#include "rtt_actionlib/rtt_actionlib.h"
#include "rtt_actionlib/rtt_action_server.h"

class CartesianImpedanceAction: public RTT::TaskContext {
 private:
  typedef actionlib::ServerGoalHandle<cartesian_trajectory_msgs::CartesianImpedanceAction> GoalHandle;
  typedef boost::shared_ptr<const cartesian_trajectory_msgs::CartesianImpedanceGoal> Goal;

 public:
  explicit CartesianImpedanceAction(const std::string& name);
  virtual ~CartesianImpedanceAction();

  bool startHook();
  void updateHook();

 private:
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);
 
  bool checkImpedance(const cartesian_trajectory_msgs::CartesianImpedance& imp);
 
  RTT::OutputPort<cartesian_trajectory_msgs::CartesianImpedanceTrajectoryConstPtr> port_cartesian_trajectory_command_;
  RTT::InputPort<cartesian_trajectory_msgs::CartesianImpedanceTrajectory> port_cartesian_trajectory_;
  rtt_actionlib::RTTActionServer<cartesian_trajectory_msgs::CartesianImpedanceAction> as_;
  GoalHandle active_goal_;
};

#endif  // CARTESIAN_IMPEDANCE_ACTION_H_
