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

#include "rtt/RTT.hpp"
#include "rtt/os/TimeService.hpp"
#include "Eigen/Dense"
#include "Eigen/LU"

#include "controller_common/robot.h"

#include "geometry_msgs/Pose.h"
#include "cartesian_trajectory_msgs/CartesianImpedance.h"

#include "eigen_conversions/eigen_msg.h"

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define RESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(false)
#define UNRESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(true)
#else
#define RESTRICT_ALLOC
#define UNRESTRICT_ALLOC
#endif

class CartesianImpedance: public RTT::TaskContext {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit CartesianImpedance(const std::string &name) :
    RTT::TaskContext(name, PreOperational) {
    N = 0;
    K = 0;

    this->ports()->addPort("JointPosition", port_joint_position_);
    this->ports()->addPort("JointVelocity", port_joint_velocity_);
    this->ports()->addPort("MassMatrixInv", port_mass_matrix_inv_);

    this->ports()->addPort("JointTorqueCommand",
                           port_joint_torque_command_);
    this->ports()->addPort("NullSpaceTorqueCommand",
                           port_nullspace_torque_command_);
  }

  bool configureHook() {
    robot_ = this->getProvider<Robot>("robot");
    if (!robot_) {
      RTT::log(RTT::Error) << "Unable to load RobotService"
                           << RTT::endlog();
      return false;
    }

    N = robot_->dofs();
    K = robot_->effectors();

    port_cartesian_position_command_.resize(K);
    port_cartesian_position_.resize(K);
    port_cartesian_impedance_command_.resize(K);
    port_tool_position_command_.resize(K);

    for (size_t i = 0; i < K; i++) {
      char name[30];
      snprintf(name, sizeof(name), "CartesianPositionCommand%zu", i);
      port_cartesian_position_command_[i] = new typeof(*port_cartesian_position_command_[i]);
      this->ports()->addPort(name, *port_cartesian_position_command_[i]);

      snprintf(name, sizeof(name), "CartesianPosition%zu", i);
      port_cartesian_position_[i] = new typeof(*port_cartesian_position_[i]);
      this->ports()->addPort(name, *port_cartesian_position_[i]);

      snprintf(name, sizeof(name), "ToolPositionCommand%zu", i);
      port_tool_position_command_[i] = new typeof(*port_tool_position_command_[i]);
      this->ports()->addPort(name, *port_tool_position_command_[i]);

      snprintf(name, sizeof(name), "CartesianImpedanceCommand%zu", i);
      port_cartesian_impedance_command_[i] = new typeof(*port_cartesian_impedance_command_[i]);
      this->ports()->addPort(name, *port_cartesian_impedance_command_[i]);
    }

    tools.resize(K);
    r_cmd.resize(K);

    joint_position_.resize(N);
    joint_velocity_.resize(N);
    joint_torque_command_.resize(N);
    nullspace_torque_command_.resize(N),
                                     Kc.resize(K * 6);
    Dxi.resize(K * 6);
    J.resize(K * 6, N);
    JT.resize(N, K * 6);
    Ji.resize(N, K * 6);
    M.resize(N, N);
    Mi.resize(N, N);
    P.resize(N, N);
    A.resize(K * 6, K * 6);
    Q.resize(K * 6, K * 6);
    Dc.resize(K * 6, K * 6);
    K0.resize(K * 6);
    p.resize(K * 6);
    F.resize(K * 6);

    tmpNK_.resize(N, K * 6);
    tmpKK_.resize(K * 6, K * 6);
    tmpKK2_.resize(K * 6, K * 6);
    tmpK_.resize(K * 6);
    tmpNN_.resize(N, N);
    tmpKN_.resize(K * 6, N);

    lu_ = Eigen::PartialPivLU<Eigen::MatrixXd>(N);
    luKK_ = Eigen::PartialPivLU<Eigen::MatrixXd>(K*6);
    es_ = Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd>(K * 6);

    port_joint_torque_command_.setDataSample(joint_torque_command_);

    return true;
  }

  bool startHook() {
    RESTRICT_ALLOC;

    for (size_t i = 0; i < K; i++) {
      Kc(i * 6 + 0) = 1;
      Kc(i * 6 + 1) = 1;
      Kc(i * 6 + 2) = 1500;
      Kc(i * 6 + 3) = 150;
      Kc(i * 6 + 4) = 150;
      Kc(i * 6 + 5) = 150;

      Dxi(i * 6 + 0) = 0.7;
      Dxi(i * 6 + 1) = 0.7;
      Dxi(i * 6 + 2) = 0.7;
      Dxi(i * 6 + 3) = 0.7;
      Dxi(i * 6 + 4) = 0.7;
      Dxi(i * 6 + 5) = 0.7;

      tools[i](0) = 0;
      tools[i](1) = 0;
      tools[i](2) = 0;

      tools[i](3) = 1;
      tools[i](4) = 0;
      tools[i](5) = 0;
      tools[i](6) = 0;
    }

    for (size_t i = 0; i < N; i++) {
      nullspace_torque_command_(i) = 0.0;
    }

    if (port_joint_position_.read(joint_position_) == RTT::NewData) {
      robot_->fkin(&r_cmd[0], joint_position_, &tools[0]);

      for (size_t i = 0; i < K; i++) {
        geometry_msgs::Pose pos;
        tf::poseEigenToMsg(r_cmd[i], pos);
        port_cartesian_position_[i]->write(pos);
      }
    } else {
      return false;
    }

    UNRESTRICT_ALLOC;
    return true;
  }

  void updateHook() {
    RESTRICT_ALLOC;
    ToolMass toolsM[K];
    Eigen::Affine3d r[K];

    // read inputs
    port_joint_position_.read(joint_position_);
    port_joint_velocity_.read(joint_velocity_);
    port_nullspace_torque_command_.read(nullspace_torque_command_);

    for (size_t i = 0; i < K; i++) {
      geometry_msgs::Pose pos;
      if (port_cartesian_position_command_[i]->read(pos) == RTT::NewData) {
        tf::poseMsgToEigen(pos, r_cmd[i]);
      }

      if (port_tool_position_command_[i]->read(pos) == RTT::NewData) {
        tools[i](0) = pos.position.x;
        tools[i](1) = pos.position.y;
        tools[i](2) = pos.position.z;

        tools[i](3) = pos.orientation.w;
        tools[i](4) = pos.orientation.x;
        tools[i](5) = pos.orientation.y;
        tools[i](6) = pos.orientation.z;
      }

      cartesian_trajectory_msgs::CartesianImpedance impedance;
      if (port_cartesian_impedance_command_[i]->read(impedance)
          == RTT::NewData) {
        Kc(i * 6 + 0) = impedance.stiffness.force.x;
        Kc(i * 6 + 1) = impedance.stiffness.force.y;
        Kc(i * 6 + 2) = impedance.stiffness.force.z;
        Kc(i * 6 + 3) = impedance.stiffness.torque.x;
        Kc(i * 6 + 4) = impedance.stiffness.torque.y;
        Kc(i * 6 + 5) = impedance.stiffness.torque.z;

        Dxi(i * 6 + 0) = impedance.damping.force.x;
        Dxi(i * 6 + 1) = impedance.damping.force.y;
        Dxi(i * 6 + 2) = impedance.damping.force.z;
        Dxi(i * 6 + 3) = impedance.damping.torque.x;
        Dxi(i * 6 + 4) = impedance.damping.torque.y;
        Dxi(i * 6 + 5) = impedance.damping.torque.z;
      }
    }

    port_mass_matrix_inv_.read(M);

    // calculate robot data
    robot_->jacobian(J, joint_position_, &tools[0]);

    robot_->fkin(r, joint_position_, &tools[0]);

    JT = J.transpose();
    lu_.compute(M);
    Mi = lu_.inverse();

    // calculate stiffness component
    for (size_t i = 0; i < K; i++) {
      Eigen::Affine3d tmp;
      tmp = r[i].inverse() * r_cmd[i];

      p(i * 6) = tmp.translation().x();
      p(i * 6 + 1) = tmp.translation().y();
      p(i * 6 + 2) = tmp.translation().z();

      Eigen::Quaternion<double> quat = Eigen::Quaternion<double>(
                                         tmp.rotation());
      p(i * 6 + 3) = quat.x();
      p(i * 6 + 4) = quat.y();
      p(i * 6 + 5) = quat.z();
    }

    F.noalias() = (Kc.array() * p.array()).matrix();
    joint_torque_command_.noalias() = JT * F;

    // calculate damping component

#if 1
    tmpNK_.noalias() = J * Mi;
    A.noalias() = tmpNK_ * JT;
    luKK_.compute(A);
    A = luKK_.inverse();

    tmpKK_ = Kc.asDiagonal();
    UNRESTRICT_ALLOC;
    es_.compute(tmpKK_, A);
    RESTRICT_ALLOC;
    K0 = es_.eigenvalues();
    luKK_.compute(es_.eigenvectors());
    Q = luKK_.inverse();

    tmpKK_ = Dxi.asDiagonal();
    Dc.noalias() = Q.transpose() * tmpKK_;
    tmpKK_ = K0.cwiseSqrt().asDiagonal();
    tmpKK2_.noalias() = Dc *  tmpKK_;
    Dc.noalias() = tmpKK2_ * Q;
    tmpK_.noalias() = J * joint_velocity_;
    F.noalias() = Dc * tmpK_;
    joint_torque_command_.noalias() -= JT * F;
#else
    UNRESTRICT_ALLOC;
    tmpNN_.noalias() = JT * Kc.asDiagonal() * J;

    es_.compute(tmpNN_, M, Eigen::ComputeEigenvectors | Eigen::BAx_lx);

    K0 = es_.eigenvalues();
    Q = es_.eigenvectors();
    RESTRICT_ALLOC;

    tmpNN_ = K0.cwiseSqrt().asDiagonal();

    UNRESTRICT_ALLOC;
    Dc.noalias() = Q * 0.7 * tmpNN_ * Q.adjoint();
    RESTRICT_ALLOC;
    joint_torque_command_.noalias() -= Dc * joint_velocity_;
#endif

    // calculate null-space component

    tmpNK_.noalias() = J * Mi;
    tmpKK_.noalias() = tmpNK_ * JT;
    luKK_.compute(tmpKK_);
    tmpKK_ = luKK_.inverse();
    tmpKN_.noalias() = Mi * JT;
    Ji.noalias() = tmpKN_ * tmpKK_;

    P.noalias() = Eigen::MatrixXd::Identity(P.rows(), P.cols());
    Eigen::Matrix<double, 16, 1> tmp1;
    Eigen::Matrix<double, 1, 16> tmp2;
    for (size_t i = 0; i < (K*6); i++) {
      tmpNN_.noalias() = Eigen::MatrixXd::Identity(P.rows(), P.cols());
      tmp1 = J.row(i);
      tmp2 = Ji.col(i);
      tmp1.normalize();
      tmp2.normalize();
      tmpNN_.noalias() -= tmp1 * tmp2;
      UNRESTRICT_ALLOC;
      P = P * tmpNN_;
      RESTRICT_ALLOC;
    }

    joint_torque_command_.noalias() += P * nullspace_torque_command_;

    // write outputs
    UNRESTRICT_ALLOC;
    port_joint_torque_command_.write(joint_torque_command_);

    for (size_t i = 0; i < K; i++) {
      geometry_msgs::Pose pos;
      tf::poseEigenToMsg(r[i], pos);
      port_cartesian_position_[i]->write(pos);
    }
  }

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

