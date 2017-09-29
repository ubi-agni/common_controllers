// Copyright 2014 WUT
/*
 * robot.h
 *
 *  Created on: 26 sty 2014
 *      Author: konrad
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <string>

#include <Eigen/Dense>

#include "rtt/RTT.hpp"

namespace controller_common {
class Robot : public RTT::ServiceRequester {
 public:
  explicit Robot(RTT::TaskContext *owner) :
    RTT::ServiceRequester("robot", owner),
    inertia("inertia"),
    jacobian("jacobian"),
    fkin("fkin"),
    dofs("dofs"),
    effectors("effectors") {
    this->addOperationCaller(inertia);
    this->addOperationCaller(jacobian);
    this->addOperationCaller(fkin);
    this->addOperationCaller(dofs);
    this->addOperationCaller(effectors);
  }

  typedef Eigen::MatrixXd Jacobian;
  typedef Eigen::MatrixXd Inertia;
  typedef Eigen::VectorXd Joints;
  typedef Eigen::Matrix<double, 4, 1> ToolMass;
  typedef Eigen::Matrix<double, 7, 1> Tool;

  RTT::OperationCaller<void(Inertia &, const Joints &, const ToolMass*)> inertia;
  RTT::OperationCaller<void(Jacobian &, const Joints &, const Tool*)> jacobian;
  RTT::OperationCaller<void(Eigen::Affine3d *, const Joints &, const Tool*)> fkin;
  RTT::OperationCaller<int(void)> dofs;
  RTT::OperationCaller<int(void)> effectors;
};
}  // namespace controller_common

#endif  // ROBOT_H_
