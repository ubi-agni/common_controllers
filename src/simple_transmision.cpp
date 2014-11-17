// Copyright 2014 WUT
/*
 * simple_transmision.cpp
 *
 *  Created on: 8 aug 2014
 *      Author: dseredyn
 */

#include "simple_transmision.h"

#include <string>

#include "rtt_rosclock/rtt_rosclock.h"

SimpleTransmision::SimpleTransmision(const std::string& name)
  : RTT::TaskContext(name),
    gear_(1.0),
    encoder_res_(1.0),
    motor_offset_(0.0),
    joint_offset_(0.0) {
  this->ports()->addPort("MotorPosition", port_motor_position_);
  this->ports()->addPort("MotorVelocity", port_motor_velocity_);
  this->ports()->addPort("JointPosition", port_joint_position_);
  this->ports()->addPort("JointVelocity", port_joint_velocity_);
  
  this->addProperty("gear", gear_);
  this->addProperty("encoder_res", encoder_res_);
  this->addProperty("motor_offset", motor_offset_);
  this->addProperty("joint_offset", joint_offset_);
}

SimpleTransmision::~SimpleTransmision() {
}

bool SimpleTransmision::configureHook() {
  return true;
}

bool SimpleTransmision::startHook() {
  return true;
}

void SimpleTransmision::updateHook() {
  double mpos, mvel;
  if (port_motor_position_.read(mpos) == RTT::NewData) {
    double jpos =  ((mpos - motor_offset_)/(encoder_res_ * gear_) * M_PI * 2) + joint_offset_;
    port_joint_position_.write(jpos);
  }

  if (port_motor_velocity_.read(mvel) == RTT::NewData) {
    double jvel =  ((mvel)/(encoder_res_ * gear_) * M_PI * 2);
    port_joint_velocity_.write(jvel);
  }
}

