// Copyright 2014 WUT
/*
 * robot_mass_matrix.cpp
 *
 *  Created on: 12 mar 2014
 *      Author: konradb3
 */

#include "mass_test.h"

#include <string>

MassTest::MassTest(const std::string& name) : RTT::TaskContext(name, PreOperational) {
  this->ports()->addPort("JointPosition", port_joint_position_);
  this->ports()->addPort("MassMatrix", port_mass_matrix_);
  this->ports()->addPort("MassMatrixLeft", port_mass_matrix_left_);
  this->ports()->addPort("MassMatrixRight", port_mass_matrix_right_);
}

MassTest::~MassTest() {
}

bool MassTest::configureHook() {
  robot_ = this->getProvider<controller_common::Robot>("robot");
  if (!robot_) {
    RTT::log(RTT::Error) << "Unable to load RobotService"
                         << RTT::endlog();
    return false;
  }

  number_of_joints_ = robot_->dofs();
  number_of_effectors_ = robot_->effectors();

  joint_position_.resize(number_of_joints_);
  M_.resize(number_of_joints_, number_of_joints_);

  return true;
}

void MassTest::updateHook() {
  port_joint_position_.read(joint_position_);

  if (joint_position_.size() != number_of_joints_) {
    RTT::log(RTT::Error) << "Received joint position vector have invalid size. [read: "
                         << joint_position_.size() << " expected: " << number_of_joints_
                         << "]" << RTT::endlog();
    error();
  }

  robot_->inertia(M_, joint_position_, 0);

  port_mass_matrix_left_.read(Ml_);
  port_mass_matrix_right_.read(Mr_);

  M_.block<7, 7>(2, 2) = Mr_;
  M_.block<7, 7>(9, 9) = Ml_;

  port_mass_matrix_.write(M_);
}

