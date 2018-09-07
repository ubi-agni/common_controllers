// Copyright 2014 WUT
/*
 * CartesianInterpolator.cpp
 *
 *  Created on: 27 lut 2014
 *      Author: konradb3
 */

#include "cartesian_interpolator.h"

#include <string>

#include "rtt_rosclock/rtt_rosclock.h"
#include "Eigen/Geometry"

using namespace RTT;

CartesianInterpolator::CartesianInterpolator(const std::string& name)
    : RTT::TaskContext(name),
      trajectory_ptr_(0),
      activate_pose_init_property_(false),
      last_point_not_set_(false),
      trajectory_active_(false),
      port_cartesian_position_("CartesianPosition_INPORT"),
      port_cartesian_command_("CartesianPositionCommand_OUTPORT", false),
      port_trajectory_("CartesianTrajectoryCommand_INPORT"),
      port_generator_active_("GeneratorActiveOut_OUTPORT", false),
      port_is_synchronised_("IsSynchronised_INPORT") {

  this->ports()->addPort(port_cartesian_position_);
  this->ports()->addPort(port_cartesian_command_);
  this->ports()->addPort(port_trajectory_);
  this->ports()->addPort(port_generator_active_);
  this->ports()->addPort(port_is_synchronised_);

  port_cartesian_command_.setDataSample(setpoint_);

  this->addProperty("activate_pose_init", activate_pose_init_property_);
  this->addProperty("init_setpoint", init_setpoint_property_);
}

CartesianInterpolator::~CartesianInterpolator() {
}

bool CartesianInterpolator::configureHook() {
  return true;
}

bool CartesianInterpolator::startHook() {
  bool is_synchronised = true;
  if (activate_pose_init_property_) {
    setpoint_.position.x = init_setpoint_property_[0];
    setpoint_.position.y = init_setpoint_property_[1];
    setpoint_.position.z = init_setpoint_property_[2];
    setpoint_.orientation.w = init_setpoint_property_[3];
    setpoint_.orientation.x = init_setpoint_property_[4];
    setpoint_.orientation.y = init_setpoint_property_[5];
    setpoint_.orientation.z = init_setpoint_property_[6];
  } else {
    if (port_cartesian_position_.read(setpoint_) != RTT::NewData) {
      Logger::In in("CartesianInterpolator::startHook");
      Logger::log() << Logger::Error << "could not read data on port "
                    << port_cartesian_position_.getName() << Logger::endl;
      
      return false;
    }
  }

  port_is_synchronised_.read(is_synchronised);

  if (!is_synchronised) {
    return false;
  }
  port_generator_active_.write(true);
  last_point_not_set_ = false;
  trajectory_active_ = false;
  return true;
}

void CartesianInterpolator::stopHook() {
  port_generator_active_.write(false);
}

void CartesianInterpolator::updateHook() {
  port_generator_active_.write(true);
  if (port_trajectory_.read(trajectory_) == RTT::NewData) {
    trajectory_ptr_ = 0;
    old_point_ = setpoint_;
    last_point_not_set_ = true;
    trajectory_active_ = true;
    RTT::Logger::log(RTT::Logger::Debug) << "Interpolator: received new trajectory." << RTT::endlog();
  }

  ros::Time now = rtt_rosclock::host_now();
  if (trajectory_active_ && trajectory_)
  {
    if (trajectory_->header.stamp < now)
    {
      // find current trajectory point regarding current time
      for (; trajectory_ptr_ < trajectory_->points.size(); trajectory_ptr_++) {
        ros::Time trj_time = trajectory_->header.stamp
            + trajectory_->points[trajectory_ptr_].time_from_start;
        // find first trajectory point in the future
        if (trj_time > now) {
          RTT::Logger::log(RTT::Logger::Debug) << "Interpolator: point " << trajectory_ptr_ << " at trj_time " 
                                               << trj_time.toSec() << " > now (" 
                                               << now.toSec() << ") will be used" << RTT::endlog();
          break;
        }
        if (trajectory_ptr_ + 1 == trajectory_->points.size())
        {
          RTT::Logger::log(RTT::Logger::Debug) << "Interpolator: all points in the past, last point at trj_time " 
                                               << trj_time.toSec() << " < now (" 
                                               << now.toSec() << ")" << RTT::endlog();
        }
      }
      

      if (trajectory_ptr_ < trajectory_->points.size()) {
        RTT::Logger::log(RTT::Logger::Debug) << "Interpolator: interpolating, idx: " << trajectory_ptr_ << RTT::endlog();
        if (trajectory_ptr_ == 0) {
          cartesian_trajectory_msgs::CartesianTrajectoryPoint p0;
          p0.time_from_start.fromSec(0.0);
          p0.pose = old_point_;
          setpoint_ = interpolate(p0, trajectory_->points[trajectory_ptr_], now);
        } else {
          setpoint_ = interpolate(trajectory_->points[trajectory_ptr_ - 1],
                                  trajectory_->points[trajectory_ptr_], now);
        }
      } else if (last_point_not_set_ && trajectory_->points.size() > 0) {
        setpoint_ = trajectory_->points[trajectory_->points.size() - 1].pose;
        last_point_not_set_ = false;
        RTT::Logger::log(RTT::Logger::Debug) << "Interpolator: end point reached : " << RTT::endlog();
      }
    }
    else
    {
      RTT::Logger::log(RTT::Logger::Debug) << "Interpolator: trajectory time in the future" << RTT::endlog();
    } 
  }
  port_cartesian_command_.write(setpoint_);
}

geometry_msgs::Pose CartesianInterpolator::interpolate(
    const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p0,
    const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p1,
    ros::Time t) {
  geometry_msgs::Pose pose;

  ros::Time t0 = trajectory_->header.stamp + p0.time_from_start;
  ros::Time t1 = trajectory_->header.stamp + p1.time_from_start;

  pose.position.x = interpolate(p0.pose.position.x, p1.pose.position.x,
                                t0.toSec(), t1.toSec(), t.toSec());
  pose.position.y = interpolate(p0.pose.position.y, p1.pose.position.y,
                                t0.toSec(), t1.toSec(), t.toSec());
  pose.position.z = interpolate(p0.pose.position.z, p1.pose.position.z,
                                t0.toSec(), t1.toSec(), t.toSec());

  Eigen::Quaterniond q0(p0.pose.orientation.w, p0.pose.orientation.x,
                        p0.pose.orientation.y, p0.pose.orientation.z);
  Eigen::Quaterniond q1(p1.pose.orientation.w, p1.pose.orientation.x,
                        p1.pose.orientation.y, p1.pose.orientation.z);

  double a = interpolate(0.0, 1.0, t0.toSec(), t1.toSec(), t.toSec());
  Eigen::Quaterniond q = q0.slerp(a, q1);
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();

  return pose;
}

double CartesianInterpolator::interpolate(double p0, double p1, double t0,
                                          double t1, double t) {
  return (p0 + (p1 - p0) * (t - t0) / (t1 - t0));
}
