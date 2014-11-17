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
  RTT::InputPort<double> port_motor_position_;
  RTT::InputPort<double> port_motor_velocity_;
  RTT::OutputPort<double> port_joint_position_;
  RTT::OutputPort<double> port_joint_velocity_;

  double gear_;
  double encoder_res_;
  double motor_offset_;
  double joint_offset_;
};

#endif  // SIMPLE_TRANSMISION_H_

