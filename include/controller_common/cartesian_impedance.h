/*
 * cartesian_impedance.h
 *
 *  Created on: 26 sty 2014
 *      Author: konrad
 */

#ifndef CARTESIAN_IMPEDANCE_H_
#define CARTESIAN_IMPEDANCE_H_

#include <rtt/RTT.hpp>
#include <rtt/os/TimeService.hpp>
#include <Eigen/Dense>

#include <controller_common/robot.h>

#include <geometry_msgs/Pose.h>
#include <controller_common/CartesianImpedance.h>

template<int N, int K>
class CartesianImpedance: public RTT::TaskContext {
public:
	CartesianImpedance(const std::string &name) :
			RTT::TaskContext(name), joint_position_(N), joint_velocity_(N), joint_torque_command_(
					N), nullspace_torque_command_(N) {
		this->ports()->addPort("JointPosition", port_joint_position_);
		this->ports()->addPort("JointVelocity", port_joint_velocity_);

		this->ports()->addPort("JointTorqueCommand",
				port_joint_torque_command_);
		this->ports()->addPort("NullSpaceTorqueCommand",
				port_nullspace_torque_command_);

		for (size_t i = 0; i < K; i++) {
			char name[30];
			sprintf(name, "CartesianPositionCommand%zu", i);
			this->ports()->addPort(name, port_cartesian_position_command_[i]);

			sprintf(name, "ToolPositionCommand%zu", i);
			this->ports()->addPort(name, port_tool_position_command_[i]);
		}
	}

	bool configureHook() {
		robot_ = this->getProvider<Robot>("robot");
		if (!robot_) {
			RTT::log(RTT::Error) << "Unable to load RobotService"
					<< RTT::endlog();
			return false;
		}

		port_joint_torque_command_.setDataSample(joint_torque_command_);

		return true;
	}

	bool startHook() {
		for (size_t i = 0; i < K; i++) {
			Kc(i * 6 + 0) = 1500;
			Kc(i * 6 + 1) = 1500;
			Kc(i * 6 + 2) = 1500;
			Kc(i * 6 + 3) = 150;
			Kc(i * 6 + 4) = 150;
			Kc(i * 6 + 5) = 150;

			Dxi(i * 6 + 0) = 0.7;
			Dxi(i * 6 + 1) = 0.7;
			Dxi(i * 6 + 2) = 0.7;
			Dxi(i * 6 + 3) = 0.7;
			Dxi(i * 6 + 4) = 0.7;
			Dxi(i * 6 + 5) = 0.7;

			tools[i](0) = 0;
			tools[i](1) = 0;
			tools[i](2) = 0;

			tools[i](3) = 1;
			tools[i](4) = 0;
			tools[i](5) = 0;
			tools[i](6) = 0;
		}

		for(size_t i = 0; i < N; i++) {
			nullspace_torque_command_(i) = 0.0;
		}

		if(port_joint_position_.read(joint_position_) == RTT::NewData) {
			//std::cout << "q0 " << joint_position_ << std::endl;
			robot_->fkin(r_cmd, joint_position_, tools);

			//std::cout << "r_cmd[0] : " << r_cmd[0].matrix() << std::endl << "r_cmd[1] : " << r_cmd[1].matrix() << std::endl;
		} else {
			return false;
		}

		//set_is_malloc_allowed(false);

		return true;
	}

	void updateHook() {
		Jacobian J;
		JacobianT JT, Ji;
		Inertia M, Mi, P;
		Spring p;
		Force F;
		ToolMass toolsM[K];
		Eigen::Affine3d r[K];

		RTT::os::TimeService::ticks tim = RTT::os::TimeService::Instance()->ticksGet();

		// read inputs
		port_joint_position_.read(joint_position_);
		port_joint_velocity_.read(joint_velocity_);
		port_nullspace_torque_command_.read(nullspace_torque_command_);

		for (size_t i = 0; i < K; i++) {
			geometry_msgs::Pose pos;
			if (port_cartesian_position_command_[i].read(pos) == RTT::NewData) {
				r_cmd[i].translation().x() = pos.position.x;
				r_cmd[i].translation().y() = pos.position.y;
				r_cmd[i].translation().z() = pos.position.z;

				r_cmd[i].linear() =
						Eigen::Quaterniond(pos.orientation.w, pos.orientation.x,
								pos.orientation.y, pos.orientation.z).toRotationMatrix();
				//std::cout << "dupaaaa" << std::endl;
			}

			if (port_tool_position_command_[i].read(pos) == RTT::NewData) {
				tools[i](0) = pos.position.x;
				tools[i](1) = pos.position.y;
				tools[i](2) = pos.position.z;

				tools[i](3) = pos.orientation.w;
				tools[i](4) = pos.orientation.x;
				tools[i](5) = pos.orientation.y;
				tools[i](6) = pos.orientation.z;
			}

			controller_common::CartesianImpedance impedance[K];
			if (port_cartesian_impedance_command_[i].read(impedance[i])
					== RTT::NewData) {
				Kc(i * 6 + 0) = impedance[i].stiffness.force.x;
				Kc(i * 6 + 1) = impedance[i].stiffness.force.y;
				Kc(i * 6 + 2) = impedance[i].stiffness.force.z;
				Kc(i * 6 + 3) = impedance[i].stiffness.torque.x;
				Kc(i * 6 + 4) = impedance[i].stiffness.torque.y;
				Kc(i * 6 + 5) = impedance[i].stiffness.torque.z;

				Dxi(i * 6 + 0) = impedance[i].damping.force.x;
				Dxi(i * 6 + 1) = impedance[i].damping.force.y;
				Dxi(i * 6 + 2) = impedance[i].damping.force.z;
				Dxi(i * 6 + 3) = impedance[i].damping.torque.x;
				Dxi(i * 6 + 4) = impedance[i].damping.torque.y;
				Dxi(i * 6 + 5) = impedance[i].damping.torque.z;
			}
		}

		// calculate robot data
		robot_->inertia(M, joint_position_, toolsM);
		robot_->jacobian(J, joint_position_, tools);
		//std::cout << "q1 " << joint_position_ << std::endl;
		//std::cout << "tool :" << tools[0] << std::endl;
		robot_->fkin(r, joint_position_, tools);

		JT = J.transpose();
		Mi = M.inverse();

		// calculate stiffness component
		for (size_t i = 0; i < K; i++) {
			Eigen::Affine3d tmp;
			tmp = r[i].inverse() * r_cmd[i];
			//std::cout << "r :" << r[i].matrix() << std::endl;
			//std::cout << "r_cmd :" << r_cmd[i].matrix() << std::endl;
			p(i * 6) = tmp.translation().x();
			p(i * 6 + 1) = tmp.translation().y();
			p(i * 6 + 2) = tmp.translation().z();

			Eigen::Quaternion<double> quat = Eigen::Quaternion<double>(
					tmp.rotation());
			p(i * 6 + 3) = quat.x();
			p(i * 6 + 4) = quat.y();
			p(i * 6 + 5) = quat.z();
		}

		F.noalias() = (Kc.array() * p.array()).matrix();
		//std::cout << "p :" << p << std::endl;
		//std::cout << "Kc :" << Kc << std::endl;
		//std::cout << "F spring:" << F << std::endl;

		// calculate damping component
		Eigen::GeneralizedSelfAdjointEigenSolver<
				Eigen::Matrix<double, K * 6, K * 6> > es;
		Eigen::Matrix<double, K * 6, K * 6> A, Q, Dc;
		Eigen::Matrix<double, K * 6, 1> K0;
		A = (J * Mi * JT).inverse();
		es.compute(Kc.asDiagonal(), A);
		K0 = es.eigenvalues();
		Q = es.eigenvectors().inverse();

		Dc.noalias() = Q.transpose() * Dxi.asDiagonal()
				* K0.cwiseSqrt().asDiagonal() * Q;
		F.noalias() -= Dc * J * joint_velocity_;

		// calculate null-space component
		Ji.noalias() = Mi * JT * (J * Mi * JT).inverse();
		P.noalias() = Eigen::Matrix<double, N, N>::Identity() - Ji * J;
		joint_torque_command_.noalias() += P * nullspace_torque_command_;

		// write outputs
		joint_torque_command_.noalias() = JT * F;
		//std::cout << "F :" << F << std::endl;
		//std::cout << "tau : " << joint_torque_command_ << std::endl;
		port_joint_torque_command_.write(joint_torque_command_);

		//std::cout << "q : " << joint_position_ << std::endl;
		double sec = RTT::os::TimeService::Instance()->secondsSince(tim);

		//std::cout << "time :" << sec << std::endl;
	}

	typedef controller_common::Robot<N, K> Robot;
	typedef Eigen::Matrix<double, N, K * 6> JacobianT;
	typedef Eigen::Matrix<double, K * 6, N> Jacobian;
	typedef Eigen::Matrix<double, N, N> Inertia;
	typedef Eigen::Matrix<double, N, 1> Joints;
	typedef Eigen::Matrix<double, 4, 1> ToolMass;
	typedef Eigen::Matrix<double, 7, 1> Tool;
	typedef Eigen::Matrix<double, K * 6, 1> Stiffness;
	typedef Eigen::Matrix<double, K * 6, 1> Spring;
	typedef Eigen::Matrix<double, K * 6, 1> Force;

private:
	RTT::InputPort<Eigen::VectorXd> port_joint_position_;
	RTT::InputPort<Eigen::VectorXd> port_joint_velocity_;

	RTT::InputPort<geometry_msgs::Pose> port_cartesian_position_command_[K];
	RTT::InputPort<geometry_msgs::Pose> port_tool_position_command_[K];
	RTT::InputPort<controller_common::CartesianImpedance> port_cartesian_impedance_command_[K];

	RTT::InputPort<Eigen::VectorXd> port_nullspace_torque_command_;

	RTT::OutputPort<Eigen::VectorXd> port_joint_torque_command_;

	Eigen::VectorXd joint_position_;
	Eigen::VectorXd joint_velocity_;

	Eigen::VectorXd joint_torque_command_;
	Eigen::VectorXd nullspace_torque_command_;

	boost::shared_ptr<Robot> robot_;

	Tool tools[K];
	Eigen::Affine3d r_cmd[K];
	Stiffness Kc, Dxi;

};

#endif /* CARTESIAN_IMPEDANCE_H_ */
