// Copyright 2014 WUT
/*
 * torque_publisher.h
 *
 *  Created on: 8 aug 2014
 *      Author: dseredyn
 */

#ifndef TORQUE_PUBLISHER_H_
#define TORQUE_PUBLISHER_H_

#include <string>

#include <Eigen/Dense>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

#include "std_msgs/Float64MultiArray.h"

class TorquePublisher: public RTT::TaskContext {
 public:
  explicit TorquePublisher(const std::string& name);
  virtual ~TorquePublisher();
  virtual bool configureHook();
  virtual bool startHook();
  virtual void updateHook();

 private:
  RTT::InputPort<Eigen::VectorXd > port_in_joint_torque_;
  RTT::InputPort<Eigen::VectorXd > port_in_joint_torque_command_;
  RTT::InputPort<Eigen::VectorXd > port_in_grav_torque_;
  RTT::OutputPort<std_msgs::Float64MultiArray > port_out_torques_;

  Eigen::VectorXd joint_torque_;
  Eigen::VectorXd joint_torque_command_;
  Eigen::VectorXd grav_torque_;
  std_msgs::Float64MultiArray out_torques_;

  size_t N_;
};

#endif  // TORQUE_PUBLISHER_H_

