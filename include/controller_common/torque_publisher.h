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

#include "Eigen/Dense"

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

#include "std_msgs/Float64MultiArray.h"

template <unsigned DOFS>
class TorquePublisher: public RTT::TaskContext {
 public:
  explicit TorquePublisher(const std::string& name);
  virtual ~TorquePublisher();
  virtual bool configureHook();
  virtual bool startHook();
  virtual void updateHook();

 private:

  typedef Eigen::Matrix<double, DOFS, 1> Joints;

  RTT::InputPort<Joints > port_in_joint_torque_;
  RTT::InputPort<Joints > port_in_joint_torque_command_;
  RTT::InputPort<Joints > port_in_grav_torque_;
  RTT::OutputPort<std_msgs::Float64MultiArray > port_out_torques_;

  Joints joint_torque_;
  Joints joint_torque_command_;
  Joints grav_torque_;
  std_msgs::Float64MultiArray out_torques_;
};

template <unsigned DOFS>
TorquePublisher<DOFS>::TorquePublisher(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {
  this->ports()->addPort("InJointTorque", port_in_joint_torque_);
  this->ports()->addPort("InJointTorqueCommand", port_in_joint_torque_command_);
  this->ports()->addPort("InGravityTorque", port_in_grav_torque_);
  this->ports()->addPort("OutTorques", port_out_torques_);
}

template <unsigned DOFS>
TorquePublisher<DOFS>::~TorquePublisher() {
}

template <unsigned DOFS>
bool TorquePublisher<DOFS>::configureHook() {
  out_torques_.layout.dim.resize(2);
  out_torques_.layout.dim[0].label = "torques";
  out_torques_.layout.dim[0].size = 3;
  out_torques_.layout.dim[0].stride = 3 * DOFS;
  out_torques_.layout.dim[1].label = "joints";
  out_torques_.layout.dim[1].size = DOFS;
  out_torques_.layout.dim[1].stride = DOFS;

  out_torques_.data.resize(DOFS * 3);

  return true;
}

template <unsigned DOFS>
bool TorquePublisher<DOFS>::startHook() {
  return true;
}

template <unsigned DOFS>
void TorquePublisher<DOFS>::updateHook() {
  port_in_joint_torque_.readNewest(joint_torque_);
  port_in_joint_torque_command_.readNewest(joint_torque_command_);
  port_in_grav_torque_.readNewest(grav_torque_);

  for (size_t i = 0; i < DOFS; i++) {
    out_torques_.data[i] = joint_torque_[i];
    out_torques_.data[i+DOFS] = joint_torque_command_[i];
    out_torques_.data[i+2*DOFS] = grav_torque_[i];
  }

  port_out_torques_.write(out_torques_);
}

#endif  // TORQUE_PUBLISHER_H_
