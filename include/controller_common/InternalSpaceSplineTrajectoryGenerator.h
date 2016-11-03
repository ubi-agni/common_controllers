/*
 * Copyright (c) 2010, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * InternalSpaceSplineTrajectoryGenerator.h
 *
 * Generator for both the motor and joint spline interpolation
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz
 */

#ifndef INTERNALSPACESPLINETRAJECTORYGENERATOR_H_
#define INTERNALSPACESPLINETRAJECTORYGENERATOR_H_

#include <Eigen/Dense>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include "rtt_rosclock/rtt_rosclock.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <vector>
#include <exception>

#include "controller_common/velocityprofile_spline.hpp"

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define RESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(false)
#define UNRESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(true)
#else
#error EIGEN_RUNTIME_NO_MALLOC must be defined
#endif

template <unsigned NUMBER_OF_JOINTS>
class InternalSpaceSplineTrajectoryGenerator : public RTT::TaskContext {
 public:
  explicit InternalSpaceSplineTrajectoryGenerator(const std::string& name);
  virtual ~InternalSpaceSplineTrajectoryGenerator();

  virtual bool configureHook();
  virtual bool startHook();
  virtual void stopHook();
  virtual void updateHook();

 protected:
  typedef Eigen::Matrix<double, NUMBER_OF_JOINTS, 1>  VectorNd;

  RTT::InputPort<trajectory_msgs::JointTrajectoryConstPtr> port_trajectory_in_;

  RTT::OutputPort<VectorNd> port_internal_space_position_command_out_;
  RTT::InputPort<VectorNd> port_internal_space_position_measurement_in_;
  RTT::OutputPort<bool> port_generator_active_out_;
  RTT::InputPort<bool> port_is_synchronised_in_;

 private:
  bool last_point_not_set_;
  bool trajectory_active_;
  std::vector<KDL::VelocityProfile_Spline> vel_profile_;

  trajectory_msgs::JointTrajectoryPoint trajectory_old_;
  trajectory_msgs::JointTrajectoryPoint trajectory_new_;

  VectorNd des_jnt_pos_, setpoint_, old_point_;

  trajectory_msgs::JointTrajectoryConstPtr trajectory_;
  size_t trajectory_ptr_;
};

using namespace RTT;

template <unsigned NUMBER_OF_JOINTS>
InternalSpaceSplineTrajectoryGenerator<NUMBER_OF_JOINTS>::InternalSpaceSplineTrajectoryGenerator(
    const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      last_point_not_set_(false),
      trajectory_active_(false),
      trajectory_ptr_(0),
      port_trajectory_in_("trajectoryPtr_INPORT"),
      port_internal_space_position_command_out_("JointPositionCommand_OUTPORT", true),
      port_generator_active_out_("GeneratorActive_OUTPORT", true),
      port_internal_space_position_measurement_in_("JointPosition_INPORT"),
      port_is_synchronised_in_("IsSynchronised_INPORT") {
  this->ports()->addPort(port_trajectory_in_);
  this->ports()->addPort(port_internal_space_position_command_out_);
  this->ports()->addPort(port_internal_space_position_measurement_in_);
  this->ports()->addPort(port_generator_active_out_);
  this->ports()->addPort(port_is_synchronised_in_);

  return;
}

template <unsigned NUMBER_OF_JOINTS>
InternalSpaceSplineTrajectoryGenerator<NUMBER_OF_JOINTS>::~InternalSpaceSplineTrajectoryGenerator() {
  return;
}

template <unsigned NUMBER_OF_JOINTS>
bool InternalSpaceSplineTrajectoryGenerator<NUMBER_OF_JOINTS>::configureHook() {
  Logger::In in("InternalSpaceSplineTrajectoryGenerator::configureHook");

  vel_profile_.resize(NUMBER_OF_JOINTS);

  return true;
}

template <unsigned NUMBER_OF_JOINTS>
bool InternalSpaceSplineTrajectoryGenerator<NUMBER_OF_JOINTS>::startHook() {
  RESTRICT_ALLOC;

  bool is_synchronised = true;

  FlowStatus read_status = port_internal_space_position_measurement_in_.read(setpoint_);
  if (read_status == RTT::NoData) {
    Logger::In in("InternalSpaceSplineTrajectoryGenerator::startHook");
    Logger::log() << Logger::Error << "could not read data on port "
                  << port_internal_space_position_measurement_in_.getName() << Logger::endl;
    return false;
  }
  else if (read_status == RTT::OldData) {
    Logger::In in("InternalSpaceSplineTrajectoryGenerator::startHook");
    Logger::log() << Logger::Error << "could not read new data on port "
                  << port_internal_space_position_measurement_in_.getName() << Logger::endl;
    return false;
  }

  port_is_synchronised_in_.read(is_synchronised);

  if (!is_synchronised) {
    return false;
  }

  port_generator_active_out_.write(true);
  last_point_not_set_ = false;
  trajectory_active_ = false;
  return true;
}

template <unsigned NUMBER_OF_JOINTS>
void InternalSpaceSplineTrajectoryGenerator<NUMBER_OF_JOINTS>::stopHook() {
  port_generator_active_out_.write(false);
  UNRESTRICT_ALLOC;
}

template <unsigned NUMBER_OF_JOINTS>
void InternalSpaceSplineTrajectoryGenerator<NUMBER_OF_JOINTS>::updateHook() {
  port_generator_active_out_.write(true);

  trajectory_msgs::JointTrajectoryConstPtr trj_ptr_tmp;
  if (port_trajectory_in_.read(trj_ptr_tmp) == RTT::NewData) {
    trajectory_ = trj_ptr_tmp;
    trajectory_ptr_ = 0;
    old_point_ = setpoint_;
    last_point_not_set_ = true;
    trajectory_active_ = true;
  }

  ros::Time now = rtt_rosclock::host_now();
  if (trajectory_active_ && trajectory_ && (trajectory_->header.stamp < now)) {
    for (; trajectory_ptr_ < trajectory_->points.size(); trajectory_ptr_++) {
      ros::Time trj_time = trajectory_->header.stamp
          + trajectory_->points[trajectory_ptr_].time_from_start;
      if (trj_time > now) {
        for (unsigned int i = 0; i < NUMBER_OF_JOINTS; i++) {
          if (trajectory_ptr_ < 1) {
            if (trajectory_->points[trajectory_ptr_].accelerations.size() > 0
                && trajectory_->points[trajectory_ptr_].velocities.size() > 0) {
              vel_profile_[i].SetProfileDuration(
                  old_point_(i), 0.0, 0.0,
                  trajectory_->points[trajectory_ptr_].positions[i],
                  trajectory_->points[trajectory_ptr_].velocities[i],
                  trajectory_->points[trajectory_ptr_].accelerations[i],
                  trajectory_->points[trajectory_ptr_].time_from_start.toSec());
            } else if (trajectory_->points[trajectory_ptr_].velocities.size()
                > 0) {
              vel_profile_[i].SetProfileDuration(
                  old_point_(i), 0.0,
                  trajectory_->points[trajectory_ptr_].positions[i],
                  trajectory_->points[trajectory_ptr_].velocities[i],
                  trajectory_->points[trajectory_ptr_].time_from_start.toSec());
            } else {
              vel_profile_[i].SetProfileDuration(
                  old_point_(i),
                  trajectory_->points[trajectory_ptr_].positions[i],
                  trajectory_->points[trajectory_ptr_].time_from_start.toSec());
            }
          } else {
            if (trajectory_->points[trajectory_ptr_ - 1].accelerations.size()
                > 0
                && trajectory_->points[trajectory_ptr_].accelerations.size()
                    > 0) {
              vel_profile_[i].SetProfileDuration(
                  trajectory_->points[trajectory_ptr_ - 1].positions[i],
                  trajectory_->points[trajectory_ptr_ - 1].velocities[i],
                  trajectory_->points[trajectory_ptr_ - 1].accelerations[i],
                  trajectory_->points[trajectory_ptr_].positions[i],
                  trajectory_->points[trajectory_ptr_].velocities[i],
                  trajectory_->points[trajectory_ptr_].accelerations[i],
                  (trajectory_->points[trajectory_ptr_].time_from_start
                      - trajectory_->points[trajectory_ptr_ - 1].time_from_start)
                      .toSec());
            } else if (trajectory_->points[trajectory_ptr_ - 1].velocities.size()
                > 0
                && trajectory_->points[trajectory_ptr_].velocities.size() > 0) {
              vel_profile_[i].SetProfileDuration(
                  trajectory_->points[trajectory_ptr_ - 1].positions[i],
                  trajectory_->points[trajectory_ptr_ - 1].velocities[i],
                  trajectory_->points[trajectory_ptr_].positions[i],
                  trajectory_->points[trajectory_ptr_].velocities[i],
                  (trajectory_->points[trajectory_ptr_].time_from_start
                      - trajectory_->points[trajectory_ptr_ - 1].time_from_start)
                      .toSec());
            } else {
              vel_profile_[i].SetProfileDuration(
                  trajectory_->points[trajectory_ptr_ - 1].positions[i],
                  trajectory_->points[trajectory_ptr_].positions[i],
                  (trajectory_->points[trajectory_ptr_].time_from_start
                      - trajectory_->points[trajectory_ptr_ - 1].time_from_start)
                      .toSec());
            }
          }
        }
        break;
      }
    }

    if (trajectory_ptr_ < trajectory_->points.size()) {
      double t;
      if (trajectory_ptr_ < 1) {
        t = (now - trajectory_->header.stamp).toSec();
      } else {
        t = (now - trajectory_->header.stamp).toSec()
            - trajectory_->points[trajectory_ptr_ - 1].time_from_start.toSec();
      }

      for (unsigned int i = 0; i < NUMBER_OF_JOINTS; i++) {
        setpoint_(i) = vel_profile_[i].Pos(t);
        // setpoint_.setpoints[i].velocity = velProfile_[i].Vel(time * dt);
        // setpoint_.setpoints[i].acceleration = velProfile_[i].Acc(time * dt);
      }

    } else if (last_point_not_set_) {
      for (unsigned int i = 0; i < NUMBER_OF_JOINTS; i++) {
        setpoint_(i) = trajectory_->points[trajectory_->points.size() - 1]
            .positions[i];
      }
      trajectory_ = trajectory_msgs::JointTrajectoryConstPtr();
      last_point_not_set_ = false;
    }
  }

  port_internal_space_position_command_out_.write(setpoint_);
}

#endif  // INTERNALSPACESPLINETRAJECTORYGENERATOR_H_

