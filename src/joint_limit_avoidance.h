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

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

#include "Eigen/Dense"

class JointLimitAvoidance: public RTT::TaskContext {
 public:
  explicit JointLimitAvoidance(const std::string& name);
  virtual ~JointLimitAvoidance();

  bool configureHook();
  void updateHook();

 private:
  double jointLimitTrq(double hl, double ll, double ls, double r_max, double q);

  RTT::InputPort<Eigen::VectorXd> port_joint_position_;
  RTT::InputPort<Eigen::VectorXd> port_joint_velocity_;
  RTT::InputPort<Eigen::MatrixXd> port_mass_matrix_;
  RTT::InputPort<Eigen::VectorXd> port_nullspace_torque_command_;
  RTT::OutputPort<Eigen::VectorXd> port_joint_torque_command_;

  Eigen::VectorXd joint_position_, joint_velocity_, joint_torque_command_, nullspace_torque_command_;

  std::vector<double> upper_limit_, lower_limit_, max_trq_, limit_range_;

  size_t number_of_joints_;

  Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd > es_;
  Eigen::VectorXd k_;  // local stiffness
  Eigen::MatrixXd m_, d_, k0_, q_, qt_;
  Eigen::MatrixXd tmpNN_;
};

#endif  // JOINT_LIMIT_AVOIDANCE_H_
