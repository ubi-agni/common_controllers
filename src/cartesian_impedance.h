// Copyright 2014 WUT
/*
 * cartesian_impedance.h
 *
 *  Created on: 26 sty 2014
 *      Author: konrad
 */

#ifndef CARTESIAN_IMPEDANCE_H_
#define CARTESIAN_IMPEDANCE_H_

#include <cstring>

#include <vector>
#include <string>

#include <Eigen/Dense>

#include "rtt/RTT.hpp"
#include "rtt/os/TimeService.hpp"

#include "controller_common/robot.h"

#include "geometry_msgs/Pose.h"
#include "cartesian_trajectory_msgs/CartesianImpedance.h"

#include "eigen_conversions/eigen_msg.h"

class CartesianImpedance: public RTT::TaskContext {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit CartesianImpedance(const std::string &name);

  bool configureHook();

  bool startHook();

  void stopHook();

  void updateHook();

  typedef controller_common::Robot Robot;

  typedef Eigen::MatrixXd Jacobian;
  typedef Eigen::MatrixXd Inertia;
  typedef Eigen::VectorXd Joints;
  typedef Eigen::VectorXd Stiffness;
  typedef Eigen::VectorXd Spring;
  typedef Eigen::VectorXd Force;
  typedef Eigen::Matrix<double, 4, 1> ToolMass;
  typedef Eigen::Matrix<double, 7, 1> Tool;

 private:
  RTT::InputPort<Eigen::VectorXd> port_joint_position_;
  RTT::InputPort<Eigen::VectorXd> port_joint_velocity_;
  RTT::InputPort<Eigen::MatrixXd> port_mass_matrix_inv_;

  std::vector<RTT::InputPort<geometry_msgs::Pose>* > port_cartesian_position_command_;
  std::vector<RTT::OutputPort<geometry_msgs::Pose>* > port_cartesian_position_;
  std::vector<RTT::InputPort<geometry_msgs::Pose>* > port_tool_position_command_;
  std::vector<RTT::InputPort<cartesian_trajectory_msgs::CartesianImpedance>* > port_cartesian_impedance_command_;

  RTT::InputPort<Eigen::VectorXd> port_nullspace_torque_command_;

  RTT::OutputPort<Eigen::VectorXd> port_joint_torque_command_;

  Eigen::VectorXd joint_position_;
  Eigen::VectorXd joint_velocity_;

  Eigen::VectorXd joint_torque_command_;
  Eigen::VectorXd nullspace_torque_command_;

  boost::shared_ptr<Robot> robot_;

  Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> es_;

  Eigen::PartialPivLU<Eigen::MatrixXd> lu_, luKK_;

  std::vector<Tool> tools;
  std::vector<Eigen::Affine3d> r_cmd;
  Stiffness Kc, Dxi;
  Jacobian J, JT, Ji;
  Inertia M, Mi, P;
  Eigen::MatrixXd A, Q, Dc;
  Eigen::VectorXd K0;
  Spring p;
  Force F;
  Eigen::MatrixXd tmpNK_, tmpKK_, tmpKK2_, tmpNN_, tmpKN_;
  Eigen::VectorXd tmpK_;
  int N, K;
};

#endif  // CARTESIAN_IMPEDANCE_H_

