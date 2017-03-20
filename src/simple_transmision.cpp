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
    gear_(0.0),
    encoder_res_(1.0),
    motor_offset_(0.0),
    joint_offset_(0.0),
    port_motor_position_("MotorPosition_INPORT"),
    port_motor_velocity_("MotorVelocity_INPORT"),
    port_joint_position_("JointPosition_OUTPORT", false),
    port_joint_velocity_("JointVelocity_OUTPORT", false) {

  this->ports()->addPort(port_motor_position_);
  this->ports()->addPort(port_motor_velocity_);
  this->ports()->addPort(port_joint_position_);
  this->ports()->addPort(port_joint_velocity_);

  this->addProperty("gear", gear_);
  this->addProperty("encoder_res", encoder_res_);
  this->addProperty("motor_offset", motor_offset_);
  this->addProperty("joint_offset", joint_offset_);
}

SimpleTransmision::~SimpleTransmision() {
}

bool SimpleTransmision::configureHook() {
    if (gear_ == 0.0) {
        RTT::Logger::In in("SimpleTransmision::configureHook: " + getName());
        RTT::Logger::log() << RTT::Logger::Error << "property \'gear\' is not set" << RTT::Logger::endl;
        return false;
    }
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

