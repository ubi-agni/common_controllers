// Copyright 2014 WUT
/*
 * simple_transmision_inv.cpp
 *
 *  Created on: 8 aug 2014
 *      Author: dseredyn
 */

#include "simple_transmision_inv.h"
#include <rtt/Logger.hpp>

#include <string>

SimpleTransmisionInv::SimpleTransmisionInv(const std::string& name)
  : RTT::TaskContext(name),
    gear_(0.0),
    encoder_res_(1.0),
    motor_offset_(0.0),
    joint_offset_(0.0),
    motor_constant_(0.0),
    port_motor_position_("MotorPosition_OUTPORT", false),
    port_motor_velocity_("MotorVelocity_OUTPORT", false),
    port_joint_position_("JointPosition_INPORT"),
    port_joint_velocity_("JointVelocity_INPORT"),
    port_motor_current_("MotorCurrent_OUTPORT", false),
    port_joint_torque_("JointTorque_INPORT") {

  this->ports()->addPort(port_motor_position_);
  this->ports()->addPort(port_motor_velocity_);
  this->ports()->addPort(port_joint_position_);
  this->ports()->addPort(port_joint_velocity_);
  this->ports()->addPort(port_motor_current_);
  this->ports()->addPort(port_joint_torque_);

  this->addProperty("gear", gear_);
  this->addProperty("encoder_res", encoder_res_);
  this->addProperty("motor_offset", motor_offset_);
  this->addProperty("joint_offset", joint_offset_);
  this->addProperty("motor_constant", motor_constant_);
}

SimpleTransmisionInv::~SimpleTransmisionInv() {
}

bool SimpleTransmisionInv::configureHook() {
    if (gear_ == 0.0) {
        RTT::Logger::In in("SimpleTransmisionInv::configureHook: " + getName());
        RTT::Logger::log() << RTT::Logger::Error << "property \'gear\' is not set" << RTT::Logger::endl;
        return false;
    }
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

