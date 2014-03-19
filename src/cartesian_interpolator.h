/*
 * CartesianInterpolator.h
 *
 *  Created on: 27 lut 2014
 *      Author: konradb3
 */

#ifndef CARTESIANINTERPOLATOR_H_
#define CARTESIANINTERPOLATOR_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <cartesian_trajectory_msgs/CartesianTrajectory.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Dense>

class CartesianInterpolator: public RTT::TaskContext {
public:
	CartesianInterpolator(const std::string& name);
	virtual ~CartesianInterpolator();
	virtual bool configureHook();
	virtual bool startHook();
	virtual void updateHook();

private:
	geometry_msgs::Pose interpolate(const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p0, const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p1, ros::Time t);
	double interpolate(double p0, double p1, double t0, double t1, double t);
	RTT::InputPort<cartesian_trajectory_msgs::CartesianTrajectoryConstPtr > port_trajectory_;
	RTT::InputPort<geometry_msgs::Pose > port_cartesian_position_;

	RTT::OutputPort<geometry_msgs::Pose > port_cartesian_command_;

	cartesian_trajectory_msgs::CartesianTrajectoryConstPtr trajectory_;
	geometry_msgs::Pose setpoint_;
	geometry_msgs::Pose old_point_;

	size_t trajectory_ptr_;
};

#endif /* CARTESIANINTERPOLATOR_H_ */
