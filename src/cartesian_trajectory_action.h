/*
 * cartesian_trajectory_action.h
 *
 *  Created on: 5 mar 2014
 *      Author: konradb3
 */

#ifndef CARTESIAN_TRAJECTORY_ACTION_H_
#define CARTESIAN_TRAJECTORY_ACTION_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <controller_common/CartesianTrajectory.h>

class CartesianTrajectoryAction: public RTT::TaskContext {
public:
	CartesianTrajectoryAction(const std::string& name);
	virtual ~CartesianTrajectoryAction();

	bool startHook();
	void updateHook();

private:
  RTT::OutputPort<controller_common::CartesianTrajectoryConstPtr> port_cartesian_trajectory_command_;
  RTT::InputPort<controller_common::CartesianTrajectory> port_cartesian_trajectory_;
};

#endif /* CARTESIAN_TRAJECTORY_ACTION_H_ */
