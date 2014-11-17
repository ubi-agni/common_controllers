// Copyright 2014 WUT
/*
 * joint_limit_avoidance.cpp
 *
 *  Created on: 11 mar 2014
 *      Author: konradb3
 */

#include "joint_limit_avoidance.h"

#include <string>

JointLimitAvoidance::JointLimitAvoidance(const std::string& name) :
  RTT::TaskContext(name, PreOperational), number_of_joints_(0) {
  this->ports()->addPort("JointPosition", port_joint_position_);
  this->ports()->addPort("JointVelocity", port_joint_velocity_);
  this->ports()->addPort("MassMatrix", port_mass_matrix_);
  this->ports()->addPort("JointTorqueCommand", port_joint_torque_command_);
  this->ports()->addPort("NullSpaceTorqueCommand",
                         port_nullspace_torque_command_);

  this->properties()->addProperty("upper_limit", upper_limit_);
  this->properties()->addProperty("lower_limit", lower_limit_);
  this->properties()->addProperty("limit_range", limit_range_);
  this->properties()->addProperty("max_trq", max_trq_);
}

JointLimitAvoidance::~JointLimitAvoidance() {
}

bool JointLimitAvoidance::configureHook() {
  port_joint_position_.getDataSample(joint_position_);
  port_joint_velocity_.getDataSample(joint_velocity_);
  port_mass_matrix_.getDataSample(m_);

  number_of_joints_ = joint_position_.size();

  if (number_of_joints_ == 0) {
    RTT::log(RTT::Error) << "invalid joint position data sample" << std::endl;
    return false;
  }

  if ((upper_limit_.size() != number_of_joints_)
      && (lower_limit_.size() != number_of_joints_)
      && (limit_range_.size() != number_of_joints_)
      && (max_trq_.size() != number_of_joints_)) {
    RTT::log(RTT::Error) << "invalid configuration data size" << std::endl;
    return false;
  }
  joint_torque_command_.resize(number_of_joints_);
  nullspace_torque_command_ = Eigen::VectorXd::Zero(number_of_joints_);
  k_.resize(number_of_joints_);
  q_.resizeLike(m_);
  k0_.resizeLike(m_);
  es_ = Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd >(number_of_joints_);

  return true;
}

void JointLimitAvoidance::updateHook() {
  port_joint_position_.read(joint_position_);
  port_joint_velocity_.read(joint_velocity_);
  port_nullspace_torque_command_.read(nullspace_torque_command_);

  for (size_t i = 0; i < number_of_joints_; i++) {
    joint_torque_command_(i) = jointLimitTrq(upper_limit_[i],
                               lower_limit_[i], limit_range_[i], max_trq_[i],
                               joint_position_(i));
    if (abs(joint_torque_command_(i)) > 0.001) {
      k_(i) = max_trq_[i]/limit_range_[i];
    } else {
      k_(i) = 0.001;
    }
  }

  port_mass_matrix_.read(m_);
  tmpNN_ = k_.asDiagonal();
  es_.compute(tmpNN_, m_);
  q_ = es_.eigenvectors().inverse();
  k0_ = es_.eigenvalues();

  tmpNN_ = k0_.cwiseSqrt().asDiagonal();

  d_.noalias() = 2.0 * q_.adjoint() * 0.7 * tmpNN_ * q_;

  joint_torque_command_.noalias() -= d_ * joint_velocity_;

  joint_torque_command_.noalias() += nullspace_torque_command_;

  port_joint_torque_command_.write(joint_torque_command_);
}

void JointLimitAvoidance::stopHook() {
  for (int i = 0; i < joint_torque_command_.size(); i++) {
    joint_torque_command_(i) = 0.0;
  }
  port_joint_torque_command_.write(joint_torque_command_);
}

double JointLimitAvoidance::jointLimitTrq(double hl, double ll, double ls,
    double r_max, double q) {
  if (q > (hl - ls)) {
    return -1 * ((q - hl + ls) / ls) * ((q - hl + ls) / ls) * r_max;
  } else if (q < (ll + ls)) {
    return ((ll + ls - q) / ls) * ((ll + ls - q) / ls) * r_max;
  } else {
    return 0.0;
  }
}
