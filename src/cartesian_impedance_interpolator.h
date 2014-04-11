/*
 * CartesianImpedanceInterpolator.h
 *
 *  Created on: 27 lut 2014
 *      Author: konradb3
 */

#ifndef CARTESIANIMPEDANCEINTERPOLATOR_H_
#define CARTESIANIMPEDANCEINTERPOLATOR_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <cartesian_trajectory_msgs/CartesianImpedanceTrajectory.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Dense>

class CartesianImpedanceInterpolator: public RTT::TaskContext {
public:
	CartesianImpedanceInterpolator(const std::string& name);
	virtual ~CartesianImpedanceInterpolator();
	virtual bool configureHook();
	virtual bool startHook();
	virtual void updateHook();

private:
	cartesian_trajectory_msgs::CartesianImpedance interpolate(const cartesian_trajectory_msgs::CartesianImpedanceTrajectoryPoint& p0, const cartesian_trajectory_msgs::CartesianImpedanceTrajectoryPoint& p1, ros::Time t);
	double interpolate(double p0, double p1, double t0, double t1, double t);
	RTT::InputPort<cartesian_trajectory_msgs::CartesianImpedanceTrajectoryConstPtr > port_trajectory_;
	RTT::InputPort<cartesian_trajectory_msgs::CartesianImpedance > port_cartesian_impedance_;

	RTT::OutputPort<cartesian_trajectory_msgs::CartesianImpedance > port_cartesian_impedance_command_;

	cartesian_trajectory_msgs::CartesianImpedanceTrajectoryConstPtr trajectory_;
	cartesian_trajectory_msgs::CartesianImpedance setpoint_;
	cartesian_trajectory_msgs::CartesianImpedance old_point_;

	size_t trajectory_ptr_;
};

#endif /* CARTESIANIMPEDANCEINTERPOLATOR_H_ */
