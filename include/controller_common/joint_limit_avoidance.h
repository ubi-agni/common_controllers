// Copyright 2014 WUT
/*
 * joint_limit_avoidance.h
 *
 *  Created on: 11 mar 2014
 *      Author: konradb3
 */

#ifndef JOINT_LIMIT_AVOIDANCE_H_
#define JOINT_LIMIT_AVOIDANCE_H_

#include <string>
#include <vector>

#include "eigen_patch/eigen_patch.h"

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define RESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(false)
#define UNRESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(true)
#else
#error EIGEN_RUNTIME_NO_MALLOC must be defined
#endif

template <unsigned DOFS>
class JointLimitAvoidance: public RTT::TaskContext {
 public:
  explicit JointLimitAvoidance(const std::string& name);
  virtual ~JointLimitAvoidance();

  bool configureHook();
  void updateHook();
  bool startHook();
  void stopHook();

 private:
  double jointLimitTrq(double hl, double ll, double ls, double r_max, double q);

  typedef Eigen::Matrix<double, DOFS, DOFS> Inertia;
  typedef Eigen::Matrix<double, DOFS, 1> Stiffness;
  typedef Eigen::Matrix<double, DOFS, 1> Joints;

  RTT::InputPort<Joints> port_joint_position_;
  RTT::InputPort<Joints> port_joint_velocity_;
  RTT::InputPort<Inertia> port_mass_matrix_;
  RTT::InputPort<Joints> port_nullspace_torque_command_;
  RTT::OutputPort<Joints> port_joint_torque_command_;

  Joints joint_position_, joint_velocity_, joint_torque_command_, nullspace_torque_command_;

  std::vector<double> upper_limit_, lower_limit_, max_trq_, limit_range_;

  Eigen::GeneralizedSelfAdjointEigenSolver<Inertia > es_;
  Stiffness k_, k0_;  // local stiffness
  Inertia m_, d_, q_, qt_;
  Inertia tmpNN_;
};

template <unsigned DOFS>
JointLimitAvoidance<DOFS>::JointLimitAvoidance(const std::string& name) :
  RTT::TaskContext(name, PreOperational)
{
  this->ports()->addPort("JointPosition_INPORT", port_joint_position_);
  this->ports()->addPort("JointVelocity_INPORT", port_joint_velocity_);
  this->ports()->addPort("MassMatrix_INPORT", port_mass_matrix_);
  this->ports()->addPort("JointTorqueCommand_OUTPORT", port_joint_torque_command_);
  this->ports()->addPort("NullSpaceTorqueCommand_INPORT",
                         port_nullspace_torque_command_);

  this->properties()->addProperty("upper_limit", upper_limit_);
  this->properties()->addProperty("lower_limit", lower_limit_);
  this->properties()->addProperty("limit_range", limit_range_);
  this->properties()->addProperty("max_trq", max_trq_);
}

template <unsigned DOFS>
JointLimitAvoidance<DOFS>::~JointLimitAvoidance() {
}

template <unsigned DOFS>
bool JointLimitAvoidance<DOFS>::configureHook() {
    RTT::Logger::In in("JointLimitAvoidance::configureHook");

    if ((upper_limit_.size() != DOFS)
        || (lower_limit_.size() != DOFS)
        || (limit_range_.size() != DOFS)
        || (max_trq_.size() != DOFS)) {
        RTT::log(RTT::Error) << "invalid configuration data size" << RTT::endlog();
        return false;
    }

    return true;
}

template <unsigned DOFS>
void JointLimitAvoidance<DOFS>::updateHook() {
  if (port_joint_position_.read(joint_position_) != RTT::NewData) {
    RTT::Logger::In in("JointLimitAvoidance::updateHook");
    error();
    RTT::log(RTT::Error) << "could not read port: " << port_joint_position_.getName() << RTT::endlog();
    return;
  }

  if (port_joint_velocity_.read(joint_velocity_) != RTT::NewData) {
    RTT::Logger::In in("JointLimitAvoidance::updateHook");
    error();
    RTT::log(RTT::Error) << "could not read port: " << port_joint_velocity_.getName() << RTT::endlog();
    return;
  }

  if (port_nullspace_torque_command_.read(nullspace_torque_command_) != RTT::NewData) {
    // this is acceptable
    nullspace_torque_command_.setZero();
  }

  for (size_t i = 0; i < DOFS; i++) {
    joint_torque_command_(i) = jointLimitTrq(upper_limit_[i],
                               lower_limit_[i], limit_range_[i], max_trq_[i],
                               joint_position_(i));
    if (abs(joint_torque_command_(i)) > 0.001) {
      k_(i) = max_trq_[i]/limit_range_[i];
    } else {
      k_(i) = 0.001;
    }
  }

  if (port_mass_matrix_.read(m_) != RTT::NewData) {
    RTT::Logger::In in("JointLimitAvoidance::updateHook");
    error();
    RTT::log(RTT::Error) << "could not read port: " << port_mass_matrix_.getName() << RTT::endlog();
    return;
  }

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

template <unsigned DOFS>
bool JointLimitAvoidance<DOFS>::startHook() {
  RESTRICT_ALLOC;
}

template <unsigned DOFS>
void JointLimitAvoidance<DOFS>::stopHook() {
  for (int i = 0; i < joint_torque_command_.size(); i++) {
    joint_torque_command_(i) = 0.0;
  }
  port_joint_torque_command_.write(joint_torque_command_);
  UNRESTRICT_ALLOC;
}

template <unsigned DOFS>
double JointLimitAvoidance<DOFS>::jointLimitTrq(double hl, double ll, double ls,
    double r_max, double q) {
  if (q > (hl - ls)) {
    return -1 * ((q - hl + ls) / ls) * ((q - hl + ls) / ls) * r_max;
  } else if (q < (ll + ls)) {
    return ((ll + ls - q) / ls) * ((ll + ls - q) / ls) * r_max;
  } else {
    return 0.0;
  }
}
#endif  // JOINT_LIMIT_AVOIDANCE_H_
