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

#include "eigen_patch/eigen_patch.h"

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

namespace Eigen {
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::MatrixXd::Options, 30, 30 > LimitedMatrixXd;
};

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

//  Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd > es_;
//  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd > es_;
//  Eigen::EigenSolver<Eigen::MatrixXd > es_;
  Eigen::LLT<Eigen::MatrixXd> cholB_;
  Eigen::MatrixXd matC_;
  Eigen::MatrixXd eigenvectors_;

  Eigen::VectorXd m_eivalues_;
  Eigen::LimitedMatrixXd m_eivec_;    // matrix with limited allocation
  Eigen::Tridiagonalization<Eigen::MatrixXd >::SubDiagonalType m_subdiag_;
  Eigen::VectorXd hCoeffs_;
//  Eigen::VectorXd tmp_vec_;
  Eigen::Matrix<double, -1, 1,
             Eigen::AutoAlign|Eigen::ColMajor, -1, 1> tmp_vec_;

//  Eigen::MatrixXd mat2_;

  Eigen::VectorXd k_, k0_;
  Eigen::MatrixXd m_, d_, q_, qt_;  // k0_,
  Eigen::MatrixXd tmpNN_;
};

#endif  // JOINT_IMPEDANCE_H_
