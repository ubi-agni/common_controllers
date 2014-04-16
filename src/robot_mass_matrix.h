// Copyright 2014 WUT
/*
 * robot_mass_matrix.h
 *
 *  Created on: 12 mar 2014
 *      Author: konradb3
 */

#ifndef ROBOT_MASS_MATRIX_H_
#define ROBOT_MASS_MATRIX_H_

#include <string>

#include "rtt/TaskContext.hpp"
#include "Eigen/Dense"
#include "Eigen/LU"

#include "controller_common/robot.h"

class RobotMassMatrix: public RTT::TaskContext {
 public:
  explicit RobotMassMatrix(const std::string& name);
  virtual ~RobotMassMatrix();

  bool configureHook();
  void updateHook();
 private:
  RTT::InputPort<Eigen::VectorXd> port_joint_position_;
  RTT::OutputPort<Eigen::MatrixXd> port_mass_matrix_;
  RTT::OutputPort<Eigen::MatrixXd> port_mass_matrix_inv_;

  boost::shared_ptr<controller_common::Robot> robot_;
  int number_of_joints_;
  int number_of_effectors_;

  Eigen::VectorXd joint_position_;
  Eigen::MatrixXd M_, Mi_;
  Eigen::PartialPivLU<Eigen::MatrixXd> lu_;
};

#endif  // ROBOT_MASS_MATRIX_H_
