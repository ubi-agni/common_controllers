// Copyright 2014 WUT
/*
 * joint_impedance.cpp
 *
 *  Created on: 11 sep 2014
 *      Author: konradb3
 */

#include "joint_impedance.h"

#include <string>

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define RESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(false)
#define UNRESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(true)
#else
#error EIGEN_RUNTIME_NO_MALLOC must be defined
#endif

using namespace RTT;

JointImpedance::JointImpedance(const std::string& name) :
    RTT::TaskContext(name, PreOperational),
    number_of_joints_(0),
    port_joint_torque_command_("JointTorqueCommand_OUTPORT", false) {

  this->ports()->addPort("JointPosition_INPORT", port_joint_position_);
  this->ports()->addPort("JointPositionCommand_INPORT", port_joint_position_command_);
  this->ports()->addPort("JointStiffnessCommand_INPORT", port_joint_stiffness_command_);
  this->ports()->addPort("JointVelocity_INPORT", port_joint_velocity_);
  this->ports()->addPort("MassMatrix_INPORT", port_mass_matrix_);
  this->ports()->addPort(port_joint_torque_command_);
  this->ports()->addPort("NullSpaceTorqueCommand_INPORT",
                         port_nullspace_torque_command_);

  this->properties()->addProperty("number_of_joints", number_of_joints_);
  this->properties()->addProperty("initial_stiffness", initial_stiffness_);
}

JointImpedance::~JointImpedance() {
}

bool JointImpedance::configureHook() {
  Logger::In in("JointImpedance::configureHook");

  if (0 == number_of_joints_) {
    Logger::log() << Logger::Error << "wrong number of joints"
                    << Logger::endl;
    return false;
  }

  port_joint_position_.getDataSample(joint_position_);
  if (joint_position_.size() != number_of_joints_) {
    Logger::log() << Logger::Error << "wrong data sample on port "
                    << port_joint_position_.getName() << Logger::endl;
    return false;
  }

  port_joint_velocity_.getDataSample(joint_velocity_);
  if (joint_velocity_.size() != number_of_joints_) {
    Logger::log() << Logger::Error << "wrong data sample on port "
                    << port_joint_velocity_.getName() << Logger::endl;
    return false;
  }

  port_joint_position_command_.getDataSample(joint_position_command_);
  if (joint_position_command_.size() != number_of_joints_) {
    Logger::log() << Logger::Error << "wrong data sample on port "
                    << port_joint_position_command_.getName() << Logger::endl;
    return false;
  }

  port_mass_matrix_.getDataSample(m_);
  if (m_.cols() != number_of_joints_ || m_.rows() != number_of_joints_) {
    Logger::log() << Logger::Error << "wrong data sample on port "
                    << port_mass_matrix_.getName() << Logger::endl;
    return false;
  }

  if ((initial_stiffness_.size() != number_of_joints_)) {
    log(RTT::Error) << "invalid configuration data size" << Logger::endl;
    return false;
  }

  joint_torque_command_.resize(number_of_joints_);
  joint_error_.resize(number_of_joints_);
  nullspace_torque_command_ = Eigen::VectorXd::Zero(number_of_joints_);
  k_.resize(number_of_joints_);
  q_.resizeLike(m_);
//  k0_.resizeLike(m_);
  k0_.resize(number_of_joints_);
  d_.resizeLike(m_);
//  es_ = Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd >(number_of_joints_);
//  es_ = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd >(number_of_joints_);
//  es_ = Eigen::EigenSolver<Eigen::MatrixXd >(number_of_joints_);
  cholB_ = Eigen::LLT<Eigen::MatrixXd>(number_of_joints_);
  matC_.resizeLike(m_);

  eigenvectors_.resizeLike(m_);

  m_eivalues_.resize(number_of_joints_);
  m_eivec_.resizeLike(m_);
  m_subdiag_.resize(number_of_joints_-1);
  hCoeffs_.resize(m_.cols()-1);
  tmp_vec_.resize(number_of_joints_);
//  mat2_.resizeLike(m_);

  tmpNN_.resizeLike(m_);

  for (size_t i = 0; i < initial_stiffness_.size(); i++) {
    k_(i) = initial_stiffness_[i];
    if (k_(i) < 0.0) {
      return false;
    }
  }

  port_joint_torque_command_.setDataSample(joint_torque_command_);

  return true;
}

bool JointImpedance::startHook() {
//  if (port_joint_position_.read(joint_position_command_) != RTT::NewData) {
//    return false;
//  }
  return true;
}

void JointImpedance::stopHook() {
  for (int i = 0; i < joint_torque_command_.size(); i++) {
    joint_torque_command_(i) = 0.0;
  }

  port_joint_torque_command_.write(joint_torque_command_);
}

void JointImpedance::updateHook() {
  RESTRICT_ALLOC;

  if (port_joint_position_.read(joint_position_) != RTT::NewData) {
    error();
    Logger::In in("JointImpedance::updateHook");
    Logger::log() << Logger::Error << "no data on port "
                  << port_joint_position_.getName() << Logger::endl;
    return;
  }

  if (joint_position_.size() != number_of_joints_) {
    error();
    Logger::In in("JointImpedance::updateHook");
    Logger::log() << Logger::Error << "wrong size of data on port "
                    << port_joint_position_.getName() << Logger::endl;
    return;
  }

  if (port_joint_position_command_.read(joint_position_command_) != RTT::NewData) {
    error();
    Logger::In in("JointImpedance::updateHook");
    Logger::log() << Logger::Error << "no data on port "
                  << port_joint_position_command_.getName() << Logger::endl;
    return;
  }

  if (joint_position_command_.size() != number_of_joints_) {
    error();
    Logger::In in("JointImpedance::updateHook");
    Logger::log() << Logger::Error << "wrong size of data on port "
                    << port_joint_position_command_.getName() << Logger::endl;
    return;
  }


  port_joint_stiffness_command_.read(k_);
  port_joint_velocity_.read(joint_velocity_);
  port_nullspace_torque_command_.read(nullspace_torque_command_);

  joint_error_.noalias() = joint_position_command_ - joint_position_;
  joint_torque_command_.noalias() = k_.cwiseProduct(joint_error_);

  port_mass_matrix_.read(m_);

  tmpNN_ = k_.asDiagonal();

//  es_.compute(tmpNN_, m_);  // this allocates!

//  es_



//  Eigen::MatrixXd matA = tmpNN_;
//  Eigen::MatrixXd matB = m_;

  // code from eigen3/Eigen/src/Eigenvalues/GeneralizedSelfAdjointEigenSolver.h <
  // Compute the cholesky decomposition of m_ = L L' = U'U
  {
//  Eigen::LLT<Eigen::MatrixXd> cholB(m_);
    cholB_.compute(m_.derived());

    // compute C = inv(L) A inv(L')
    matC_ = tmpNN_.template selfadjointView<Eigen::Lower>();
    cholB_.matrixL().template solveInPlace<Eigen::OnTheLeft>(matC_);
    cholB_.matrixU().template solveInPlace<Eigen::OnTheRight>(matC_);
    
//  es_.compute(matC_);

    // code from eigen3/Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h <
    {
      const Eigen::MatrixXd &matrix(matC_.derived());
      int n = matrix.cols();
      //m_eivalues_.resize(n,1);

      // declare some aliases
      Eigen::VectorXd& diag = m_eivalues_;
      Eigen::LimitedMatrixXd& mat = m_eivec_;

      // map the matrix coefficients to [-1:1] to avoid over- and underflow.
      mat = matrix.template triangularView<Eigen::Lower>();
      double scale = mat.cwiseAbs().maxCoeff();
      if (scale==0) scale = 1;
      mat.template triangularView<Eigen::Lower>() /= scale;
      //m_subdiag.resize(n-1);

      Eigen::internal::tridiagonalization_inplace(mat, diag, m_subdiag_, true);
      // code from eigen3/Eigen/src/Eigenvalues/Tridiagonalization.h <
      /*{
        //CoeffVectorType hCoeffs_(mat.cols()-1);
        tridiagonalization_inplace(mat,hCoeffs_);
        diag = mat.diagonal().real();
        m_subdiag_ = mat.template diagonal<-1>().real();
        Eigen::Tridiagonalization<Eigen::LimitedMatrixXd >::HouseholderSequenceType hhs(mat, hCoeffs_.conjugate());
        hhs.setLength(mat.rows()-1);
        hhs.setShift(1);
        hhs.template evalTo<Eigen::LimitedMatrixXd, Eigen::VectorXd >(mat, tmp_vec_);
      }*/
      // > eigen3/Eigen/src/Eigenvalues/Tridiagonalization.h

      const int m_maxIterations = 30;

      //m_info = 
      Eigen::internal::computeFromTridiagonal_impl(diag, m_subdiag_, m_maxIterations, true, m_eivec_);

      // scale back the eigen values
      m_eivalues_ *= scale;
    } // > eigen3/Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h


  // transform back the eigen vectors: evecs = inv(U) * evecs
//  cholB_.matrixU().solve(es_.eigenvectors());

//TODO:uncomment
//  cholB_.matrixU().solveInPlace(es_.eigenvectors());
    cholB_.matrixU().solveInPlace(m_eivec_);
  }
  // > code from eigen3/Eigen/src/Eigenvalues/GeneralizedSelfAdjointEigenSolver.h








//  q_ = es_.eigenvectors().inverse();

//TODO:uncomment
//  eigenvectors_.noalias() = es_.eigenvectors();
//  q_.noalias() = eigenvectors_.inverse();
//  k0_ = es_.eigenvalues();
  eigenvectors_.noalias() = m_eivec_;
  q_.noalias() = m_eivec_.inverse();
  k0_ = m_eivalues_;

  tmpNN_ = k0_.cwiseAbs().cwiseSqrt().asDiagonal();

  d_.noalias() = tmpNN_ * q_;
  d_.noalias() = q_.transpose() * d_;
  d_.noalias() = 2.0 * 0.7 * d_;


//  d_.noalias() = 2.0 * q_.transpose() * 0.7 * tmpNN_ * q_;

  if (!joint_torque_command_.allFinite()) {
    RTT::Logger::In in("JointImpedance::updateHook");
    error();
    RTT::log(RTT::Error) << "Non finite output form stiffness" << Logger::endl;
    stop();
  }

  joint_torque_command_.noalias() -= d_ * joint_velocity_;

  if (!joint_torque_command_.allFinite()) {
    RTT::Logger::In in("JointImpedance::updateHook");
    error();
    RTT::log(RTT::Error) << "Non finite output form damping" << Logger::endl;
    stop();
  }

  joint_torque_command_.noalias() += nullspace_torque_command_;

  if (!joint_torque_command_.allFinite()) {
    RTT::Logger::In in("JointImpedance::updateHook");
    error();
    RTT::log(RTT::Error) << "Non finite output form nullspace" << Logger::endl;
    stop();
  }

  port_joint_torque_command_.write(joint_torque_command_);
  UNRESTRICT_ALLOC;
}

