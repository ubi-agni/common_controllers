// Copyright 2014 WUT
/*
 * torque_publisher.cpp
 *
 *  Created on: 8 aug 2014
 *      Author: dseredyn
 */

#include "torque_publisher.h"

#include <string>

#include "rtt_rosclock/rtt_rosclock.h"

TorquePublisher::TorquePublisher(const std::string& name)
    : RTT::TaskContext(name),
      N_(0) {
  this->ports()->addPort("InJointTorque", port_in_joint_torque_);
  this->ports()->addPort("InJointTorqueCommand", port_in_joint_torque_command_);
  this->ports()->addPort("InGravityTorque", port_in_grav_torque_);
  this->ports()->addPort("OutTorques", port_out_torques_);
}

TorquePublisher::~TorquePublisher() {
}

bool TorquePublisher::configureHook() {
  port_in_joint_torque_.getDataSample(joint_torque_);
  N_ = joint_torque_.size();

  if (N_ <= 0) {
    RTT::log(RTT::Error) << "Wrong size joint_torque vector: " << N_
                         << RTT::endlog();
    return false;
  }

  out_torques_.layout.dim.resize(2);
  out_torques_.layout.dim[0].label = "torques";
  out_torques_.layout.dim[0].size = 3;
  out_torques_.layout.dim[0].stride = 3*N_;
  out_torques_.layout.dim[1].label = "joints";
  out_torques_.layout.dim[1].size = N_;
  out_torques_.layout.dim[1].stride = N_;

  out_torques_.data.resize(N_*3);

  joint_torque_.resize(N_);
  joint_torque_command_.resize(N_);
  grav_torque_.resize(N_);
  return true;
}

bool TorquePublisher::startHook() {
  return true;
}

void TorquePublisher::updateHook() {
  port_in_joint_torque_.readNewest(joint_torque_);
  port_in_joint_torque_command_.readNewest(joint_torque_command_);
  port_in_grav_torque_.readNewest(grav_torque_);

  if (joint_torque_.size() != N_) {
    RTT::log(RTT::Error) << "Wrong size joint_torque_ vector: " << joint_torque_.size()
                         << RTT::endlog();
    return;
  }

  if (joint_torque_command_.size() != N_) {
    RTT::log(RTT::Error) << "Wrong size joint_torque_command_ vector: " << joint_torque_command_.size()
                         << RTT::endlog();
    return;
  }

  if (grav_torque_.size() != N_) {
    RTT::log(RTT::Error) << "Wrong size grav_torque_ vector: " << grav_torque_.size()
                         << RTT::endlog();
    return;
  }

  for (size_t i = 0; i < N_; i++) {
    out_torques_.data[i] = joint_torque_[i];
    out_torques_.data[i+N_] = joint_torque_command_[i];
    out_torques_.data[i+2*N_] = grav_torque_[i];
  }

  port_out_torques_.write(out_torques_);
}

