// Copyright 2014 WUT
/*
 * CartesianInterpolator.cpp
 *
 *  Created on: 27 lut 2014
 *      Author: konradb3
 */

#include "cartesian_impedance_interpolator.h"

#include <string>

#include "rtt_rosclock/rtt_rosclock.h"

using ::cartesian_trajectory_msgs::CartesianImpedance;
using ::cartesian_trajectory_msgs::CartesianImpedanceTrajectoryPoint;

CartesianImpedanceInterpolator::CartesianImpedanceInterpolator(
    const std::string& name) :
      RTT::TaskContext(name),
      trajectory_ptr_(0),
      last_point_not_set_(false),
      trajectory_active_(false),
      port_cartesian_impedance_command_("CartesianImpedanceCommand_OUTPORT", false),
      port_cartesian_impedance_("CartesianImpedance_INPORT"),
      port_trajectory_("CartesianImpedanceTrajectoryCommand_INPORT") {
  this->ports()->addPort(port_cartesian_impedance_);
  this->ports()->addPort(port_cartesian_impedance_command_);
  this->ports()->addPort(port_trajectory_);
}

CartesianImpedanceInterpolator::~CartesianImpedanceInterpolator() {
}

bool CartesianImpedanceInterpolator::configureHook() {
  return true;
}

bool CartesianImpedanceInterpolator::startHook() {
  setpoint_.stiffness.force.x = 1500;
  setpoint_.stiffness.force.y = 1500;
  setpoint_.stiffness.force.z = 1500;

  setpoint_.stiffness.torque.x = 150;
  setpoint_.stiffness.torque.y = 150;
  setpoint_.stiffness.torque.z = 150;

  setpoint_.damping.force.x = 0.7;
  setpoint_.damping.force.y = 0.7;
  setpoint_.damping.force.z = 0.7;

  setpoint_.damping.torque.x = 0.7;
  setpoint_.damping.torque.y = 0.7;
  setpoint_.damping.torque.z = 0.7;

  last_point_not_set_ = false;
  trajectory_active_ = false;

  return true;
}

void CartesianImpedanceInterpolator::updateHook() {
  if (port_trajectory_.read(trajectory_) == RTT::NewData) {
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
        break;
      }
    }

    if (trajectory_ptr_ < trajectory_->points.size()) {
      if (trajectory_ptr_ == 0) {
        cartesian_trajectory_msgs::CartesianImpedanceTrajectoryPoint p0;
        p0.time_from_start.fromSec(0.0);
        p0.impedance = old_point_;
        setpoint_ = interpolate(p0, trajectory_->points[trajectory_ptr_], now);
      } else {
        setpoint_ = interpolate(trajectory_->points[trajectory_ptr_ - 1],
                                trajectory_->points[trajectory_ptr_], now);
      }
    } else if (last_point_not_set_) {
      setpoint_ = trajectory_->points[trajectory_->points.size() - 1].impedance;
      last_point_not_set_ = false;
    }
  }
  port_cartesian_impedance_command_.write(setpoint_);
}

CartesianImpedance CartesianImpedanceInterpolator::interpolate(
    const CartesianImpedanceTrajectoryPoint& p0,
    const CartesianImpedanceTrajectoryPoint& p1, ros::Time t) {
  CartesianImpedance impedance;

  ros::Time t0 = trajectory_->header.stamp + p0.time_from_start;
  ros::Time t1 = trajectory_->header.stamp + p1.time_from_start;

  impedance.stiffness.force.x = interpolate(p0.impedance.stiffness.force.x,
                                            p1.impedance.stiffness.force.x,
                                            t0.toSec(), t1.toSec(), t.toSec());
  impedance.stiffness.force.y = interpolate(p0.impedance.stiffness.force.y,
                                            p1.impedance.stiffness.force.y,
                                            t0.toSec(), t1.toSec(), t.toSec());
  impedance.stiffness.force.z = interpolate(p0.impedance.stiffness.force.z,
                                            p1.impedance.stiffness.force.z,
                                            t0.toSec(), t1.toSec(), t.toSec());

  impedance.stiffness.torque.x = interpolate(p0.impedance.stiffness.torque.x,
                                             p1.impedance.stiffness.torque.x,
                                             t0.toSec(), t1.toSec(), t.toSec());
  impedance.stiffness.torque.y = interpolate(p0.impedance.stiffness.torque.y,
                                             p1.impedance.stiffness.torque.y,
                                             t0.toSec(), t1.toSec(), t.toSec());
  impedance.stiffness.torque.z = interpolate(p0.impedance.stiffness.torque.z,
                                             p1.impedance.stiffness.torque.z,
                                             t0.toSec(), t1.toSec(), t.toSec());

  impedance.damping.force.x = interpolate(p0.impedance.damping.force.x,
                                          p1.impedance.damping.force.x,
                                          t0.toSec(), t1.toSec(), t.toSec());
  impedance.damping.force.y = interpolate(p0.impedance.damping.force.y,
                                          p1.impedance.damping.force.y,
                                          t0.toSec(), t1.toSec(), t.toSec());
  impedance.damping.force.z = interpolate(p0.impedance.damping.force.z,
                                          p1.impedance.damping.force.z,
                                          t0.toSec(), t1.toSec(), t.toSec());

  impedance.damping.torque.x = interpolate(p0.impedance.damping.torque.x,
                                           p1.impedance.damping.torque.x,
                                           t0.toSec(), t1.toSec(), t.toSec());
  impedance.damping.torque.y = interpolate(p0.impedance.damping.torque.y,
                                           p1.impedance.damping.torque.y,
                                           t0.toSec(), t1.toSec(), t.toSec());
  impedance.damping.torque.z = interpolate(p0.impedance.damping.torque.z,
                                           p1.impedance.damping.torque.z,
                                           t0.toSec(), t1.toSec(), t.toSec());

  return impedance;
}

double CartesianImpedanceInterpolator::interpolate(double p0, double p1,
                                                   double t0, double t1,
                                                   double t) {
  return (p0 + (p1 - p0) * (t - t0) / (t1 - t0));
}
