// Copyright 2014 WUT
/*
 * simple_transmision_inv.cpp
 *
 *  Created on: 8 aug 2014
 *      Author: dseredyn
 */

#include "simple_transmision_inv.h"

#include <string>

SimpleTransmisionInv::SimpleTransmisionInv(const std::string& name)
  : RTT::TaskContext(name),
    gear_(1.0),
    encoder_res_(1.0),
    motor_offset_(0.0),
    joint_offset_(0.0),
    motor_constant_(0.0) {
  this->ports()->addPort("MotorPosition", port_motor_position_);
  this->ports()->addPort("MotorVelocity", port_motor_velocity_);
  this->ports()->addPort("JointPosition", port_joint_position_);
  this->ports()->addPort("JointVelocity", port_joint_velocity_);
  this->ports()->addPort("MotorCurrent", port_motor_current_);
  this->ports()->addPort("JointTorque", port_joint_torque_);

  this->addProperty("gear", gear_);
  this->addProperty("encoder_res", encoder_res_);
  this->addProperty("motor_offset", motor_offset_);
  this->addProperty("joint_offset", joint_offset_);
  this->addProperty("motor_constant_", motor_constant_);
}

SimpleTransmisionInv::~SimpleTransmisionInv() {
}

bool SimpleTransmisionInv::configureHook() {
  return true;
}

bool SimpleTransmisionInv::startHook() {
  return true;
}

void SimpleTransmisionInv::updateHook() {
  double jpos, jvel, jtrq;
  if (port_joint_position_.read(jpos) == RTT::NewData) {
    double mpos =  (((jpos - joint_offset_)*(encoder_res_ * gear_))/(M_PI * 2)) + motor_offset_;
    port_motor_position_.write(mpos);
  }

  if (port_joint_velocity_.read(jvel) == RTT::NewData) {
    double mvel =  ((jvel * (encoder_res_ * gear_))/(M_PI * 2));
    port_motor_velocity_.write(mvel);
  }

  if (port_joint_torque_.read(jtrq) == RTT::NewData) {
    double mvel =  (jtrq/gear_)/motor_constant_;
    port_motor_current_.write(mvel);
  }
}

