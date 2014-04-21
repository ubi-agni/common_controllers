// Copyright 2014 WUT
/*
 * robot_mass_matrix.cpp
 *
 *  Created on: 12 mar 2014
 *      Author: konradb3
 */

#include "robot_mass_matrix.h"

#include <string>

RobotMassMatrix::RobotMassMatrix(const std::string& name) : RTT::TaskContext(name, PreOperational) {
  number_of_joints_ = 0;
  number_of_effectors_ = 0;

  this->ports()->addPort("JointPosition", port_joint_position_);
  this->ports()->addPort("MassMatrix", port_mass_matrix_);
  this->ports()->addPort("MassMatrixInv", port_mass_matrix_inv_);
}

RobotMassMatrix::~RobotMassMatrix() {
}

bool RobotMassMatrix::configureHook() {
  robot_ = this->getProvider<controller_common::Robot>("robot");
  if (!robot_) {
    RTT::log(RTT::Error) << "Unable to load RobotService"
                         << RTT::endlog();
    return false;
  }

  number_of_joints_ = robot_->dofs();
  number_of_effectors_ = robot_->effectors();

  lu_ = Eigen::PartialPivLU<Eigen::MatrixXd>(number_of_joints_);

  joint_position_.resize(number_of_joints_);
  M_.resize(number_of_joints_, number_of_joints_);
  Mi_.resize(number_of_joints_, number_of_joints_);

  port_mass_matrix_.setDataSample(M_);
  port_mass_matrix_inv_.setDataSample(Mi_);

  return true;
}

void RobotMassMatrix::updateHook() {
  port_joint_position_.read(joint_position_);

  if (joint_position_.size() != number_of_joints_) {
    RTT::log(RTT::Error) << "Received joint position vector have invalid size. [read: "
                         << joint_position_.size() << " expected: " << number_of_joints_
                         << "]" << RTT::endlog();
    error();
  }

  robot_->inertia(M_, joint_position_, 0);

  lu_.compute(M_);
  Mi_ = lu_.inverse();

  port_mass_matrix_.write(M_);
  port_mass_matrix_inv_.write(Mi_);
}
