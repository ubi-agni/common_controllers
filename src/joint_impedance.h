// Copyright 2014 WUT
/*
 * joint_limit_avoidance.h
 *
 *  Created on: 30 sep 2014
 *      Author: konradb3
 */

#ifndef JOINT_IMPEDANCE_H_
#define JOINT_IMPEDANCE_H_

#include <string>
#include <vector>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

#include "Eigen/Dense"

class JointImpedance: public RTT::TaskContext {
 public:
  explicit JointImpedance(const std::string& name);
  virtual ~JointImpedance();

  bool configureHook();
  bool startHook();
  void stopHook();
  void updateHook();

 private:
  RTT::InputPort<Eigen::VectorXd> port_joint_position_;
  RTT::InputPort<Eigen::VectorXd> port_joint_position_command_;
  RTT::InputPort<Eigen::VectorXd> port_joint_stiffness_command_;
  RTT::InputPort<Eigen::VectorXd> port_joint_velocity_;
  RTT::InputPort<Eigen::MatrixXd> port_mass_matrix_;
  RTT::InputPort<Eigen::VectorXd> port_nullspace_torque_command_;
  RTT::OutputPort<Eigen::VectorXd> port_joint_torque_command_;

  Eigen::VectorXd joint_position_;
  Eigen::VectorXd joint_position_command_;
  Eigen::VectorXd joint_error_;
  Eigen::VectorXd joint_velocity_;
  Eigen::VectorXd joint_torque_command_;
  Eigen::VectorXd nullspace_torque_command_;

  std::vector<double> initial_stiffness_;

  int number_of_joints_;

  Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd > es_;
  Eigen::VectorXd k_;
  Eigen::MatrixXd m_, d_, k0_, q_, qt_;
  Eigen::MatrixXd tmpNN_;
};

#endif  // JOINT_IMPEDANCE_H_
