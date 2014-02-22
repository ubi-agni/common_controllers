/*
 * robot_service.h
 *
 *  Created on: 27 sty 2014
 *      Author: konradb3
 */

#ifndef ROBOT_SERVICE_H_
#define ROBOT_SERVICE_H_

#include <rtt/RTT.hpp>
#include <rtt/Service.hpp>
#include <rtt/Logger.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <Eigen/Dense>

namespace controller_common {
template <int N, int K>
class RobotService: public RTT::Service {
public:
	RobotService(RTT::TaskContext* owner) : RTT::Service("robot", owner) {
		this->addOperation("jacobian", &RobotService::jacobian, this, RTT::ClientThread);
		this->addOperation("inertia", &RobotService::inertia, this, RTT::ClientThread);
		this->addOperation("fkin", &RobotService::fkin, this, RTT::ClientThread);
	}

	typedef Eigen::Matrix<double, K * 6, N> Jacobian;
	typedef Eigen::Matrix<double, N, N> Inertia;
	typedef Eigen::Matrix<double, N, 1> Joints;
	typedef Eigen::Matrix<double, 4, 1> ToolMass;
	typedef Eigen::Matrix<double, 7, 1> Tool;

	virtual void jacobian(Jacobian &, const Joints &, const Tool[K]) = 0;
	virtual void inertia(Inertia &, const Joints &, const ToolMass[K]) = 0;
	virtual void fkin(Eigen::Affine3d *, const Joints &, const Tool[K]) = 0;
};
}

#endif /* ROBOT_SERVICE_H_ */
