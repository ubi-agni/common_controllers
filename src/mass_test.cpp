// Copyright 2014 WUT
/*
 * robot_mass_matrix.cpp
 *
 *  Created on: 12 mar 2014
 *      Author: konradb3
 */

#include "mass_test.h"

#include <string>

MassTest::MassTest(const std::string& name) :
    RTT::TaskContext(name, PreOperational),
    port_joint_position_("JointPosition_INPORT"),
    port_mass_matrix_("MassMatrix_OUTPORT", false),
    port_mass_matrix_left_("MassMatrixLeft_INPORT"),
    port_mass_matrix_right_("MassMatrixRight_INPORT") {

  number_of_joints_ = 0;
  number_of_effectors_ = 0;

  this->ports()->addPort(port_joint_position_);
  this->ports()->addPort(port_mass_matrix_);
  this->ports()->addPort(port_mass_matrix_left_);
  this->ports()->addPort(port_mass_matrix_right_);
}

MassTest::~MassTest() {
}

bool MassTest::configureHook() {
  RTT::Logger::In in("MassTest::configureHook");

  robot_ = this->getProvider<controller_common::Robot>("robot");
  if (!robot_) {
    RTT::log(RTT::Error) << "Unable to load RobotService"
                         << RTT::endlog();
    return false;
  }

  number_of_joints_ = robot_->dofs();
  number_of_effectors_ = robot_->effectors();

  if (number_of_joints_ == 0) {
    RTT::log(RTT::Error) << "wrong number of joints: 0"
                         << RTT::endlog();
    return false;
  }

  if (number_of_effectors_ == 0) {
    RTT::log(RTT::Error) << "wrong number of effectors: 0"
                         << RTT::endlog();
    return false;
  }

  joint_position_.resize(number_of_joints_);
  M_.resize(number_of_joints_, number_of_joints_);

  port_mass_matrix_.setDataSample(M_);

  return true;
}

void MassTest::updateHook() {
  port_joint_position_.read(joint_position_);

  if (joint_position_.size() != number_of_joints_) {
    RTT::Logger::In in("MassTest::updateHook");
    RTT::log(RTT::Error) << "Received joint position vector have invalid size. [read: "
                         << joint_position_.size() << " expected: " << number_of_joints_
                         << "]" << RTT::endlog();
    error();
    return;
  }

  //robot_->inertia(M_, joint_position_, 0);

  M_.setZero();

  M_(0, 0) = 10.0;

  if (port_mass_matrix_left_.read(Ml_) != RTT::NewData) {
    RTT::Logger::In in("MassTest::updateHook");
    RTT::log(RTT::Error) << "could not receive data on port "
                         << port_mass_matrix_left_.getName()
                         << RTT::endlog();
    error();
    return;
  }
  if (Ml_.rows() != 7 || Ml_.cols() != 7) {
    RTT::Logger::In in("MassTest::updateHook");
    RTT::log(RTT::Error) << "invalid size of data received on port "
                         << port_mass_matrix_left_.getName()
                         << ": " << Ml_.rows() << " " << Ml_.cols()
                         << RTT::endlog();
    error();
    return;
  }

  if (port_mass_matrix_right_.read(Mr_) != RTT::NewData) {
    RTT::Logger::In in("MassTest::updateHook");
    RTT::log(RTT::Error) << "could not receive data on port "
                         << port_mass_matrix_left_.getName()
                         << RTT::endlog();
    error();
    return;
  }

  if (Mr_.rows() != 7 || Mr_.cols() != 7) {
    RTT::Logger::In in("MassTest::updateHook");
    RTT::log(RTT::Error) << "invalid size of data received on port "
                         << port_mass_matrix_right_.getName()
                         << ": " << Mr_.rows() << " " << Mr_.cols()
                         << RTT::endlog();
    error();
    return;
  }

  M_.block<7, 7>(1, 1) = Mr_;
  M_.block<7, 7>(8, 8) = Ml_;

  port_mass_matrix_.write(M_);
}

