// Copyright 2014 WUT
/*
 * CartesianImpedanceInterpolator.h
 *
 *  Created on: 27 lut 2014
 *      Author: konradb3
 */

#ifndef CARTESIAN_IMPEDANCE_INTERPOLATOR_H_
#define CARTESIAN_IMPEDANCE_INTERPOLATOR_H_

#include <string>

#include "eigen_patch/eigen_patch.h"

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

#include "cartesian_trajectory_msgs/CartesianImpedanceTrajectory.h"
#include "geometry_msgs/Pose.h"


class CartesianImpedanceInterpolator : public RTT::TaskContext {
 public:
  explicit CartesianImpedanceInterpolator(const std::string& name);
  virtual ~CartesianImpedanceInterpolator();
  virtual bool configureHook();
  virtual bool startHook();
  virtual void updateHook();

 private:
  cartesian_trajectory_msgs::CartesianImpedance interpolate(
      const cartesian_trajectory_msgs::CartesianImpedanceTrajectoryPoint& p0,
      const cartesian_trajectory_msgs::CartesianImpedanceTrajectoryPoint& p1,
      ros::Time t);
  double interpolate(double p0, double p1, double t0, double t1, double t);
  RTT::InputPort<cartesian_trajectory_msgs::CartesianImpedanceTrajectoryConstPtr> port_trajectory_;
  RTT::InputPort<cartesian_trajectory_msgs::CartesianImpedance> port_cartesian_impedance_;

  RTT::OutputPort<cartesian_trajectory_msgs::CartesianImpedance> port_cartesian_impedance_command_;

  cartesian_trajectory_msgs::CartesianImpedanceTrajectoryConstPtr trajectory_;
  cartesian_trajectory_msgs::CartesianImpedance setpoint_;
  cartesian_trajectory_msgs::CartesianImpedance old_point_;

  size_t trajectory_ptr_;
  bool last_point_not_set_;
  bool trajectory_active_;
};

#endif  // CARTESIAN_IMPEDANCE_INTERPOLATOR_H_
