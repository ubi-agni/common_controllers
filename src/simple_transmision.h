// Copyright 2014 WUT
/*
 * simple_transmision.h
 *
 *  Created on: 8 aug 2014
 *      Author: dseredyn
 */

#ifndef SIMPLE_TRANSMISION_H_
#define SIMPLE_TRANSMISION_H_

#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

class SimpleTransmision: public RTT::TaskContext {
 public:
  explicit SimpleTransmision(const std::string& name);
  virtual ~SimpleTransmision();
  virtual bool configureHook();
  virtual bool startHook();
  virtual void updateHook();

 private:
  RTT::InputPort<double> port_motor_position_in_;
  RTT::InputPort<double> port_motor_velocity_in_;
  RTT::OutputPort<double> port_joint_position_out_;
  RTT::OutputPort<double> port_joint_velocity_out_;

  double gear_;
  double encoder_res_;
  double motor_offset_;
  double joint_offset_;

  bool enable_position_;
  bool enable_velocity_;
};

#endif  // SIMPLE_TRANSMISION_H_

