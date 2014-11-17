// Copyright 2014 WUT
/*
 * simple_transmision_inv.h
 *
 *  Created on: 8 aug 2014
 *      Author: dseredyn
 */

#ifndef SIMPLE_TRANSMISION_INV_H_
#define SIMPLE_TRANSMISION_INV_H_

#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

class SimpleTransmisionInv: public RTT::TaskContext {
 public:
  explicit SimpleTransmisionInv(const std::string& name);
  virtual ~SimpleTransmisionInv();
  virtual bool configureHook();
  virtual bool startHook();
  virtual void updateHook();

 private:
  RTT::OutputPort<double> port_motor_position_;
  RTT::OutputPort<double> port_motor_velocity_;
  RTT::OutputPort<double> port_motor_current_;
  RTT::InputPort<double> port_joint_position_;
  RTT::InputPort<double> port_joint_velocity_;
  RTT::InputPort<double> port_joint_torque_;

  double gear_;
  double encoder_res_;
  double motor_offset_;
  double joint_offset_;
  double motor_constant_;
};

#endif  // SIMPLE_TRANSMISION_INV_H_

