/*
 * robot_mass_matrix.h
 *
 *  Created on: 12 mar 2014
 *      Author: konradb3
 */

#ifndef MASS_MATRIX_H_
#define MASS_MATRIX_H_

#include <rtt/TaskContext.hpp>
#include <Eigen/Dense>
#include <Eigen/LU>

#include <controller_common/robot.h>

class MassTest: public RTT::TaskContext {
public:
	MassTest(const std::string& name);
	virtual ~MassTest();

	bool configureHook();
	void updateHook();
private:
	RTT::InputPort<Eigen::VectorXd> port_joint_position_;
	RTT::OutputPort<Eigen::MatrixXd> port_mass_matrix_;
	RTT::InputPort<Eigen::Matrix<double, 7, 7> > port_mass_matrix_left_;
	RTT::InputPort<Eigen::Matrix<double, 7, 7> > port_mass_matrix_right_;

	boost::shared_ptr<controller_common::Robot> robot_;
	int number_of_joints_;
	int number_of_effectors_;

	Eigen::MatrixXd M_;
	Eigen::Matrix<double, 7, 7> Ml_, Mr_;
	Eigen::VectorXd joint_position_;
};

#endif /* MASS_MATRIX_H_ */
