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
    : RTT::TaskContext(name, PreOperational)
    , gear_(0.0)
    , encoder_res_(1.0)
    , motor_offset_(0.0)
    , joint_offset_(0.0)
    , port_motor_position_in_("MotorPosition_INPORT")
    , port_motor_velocity_in_("MotorVelocity_INPORT")
    , port_joint_position_out_("JointPosition_OUTPORT", false)
    , port_joint_velocity_out_("JointVelocity_OUTPORT", false)
    , enable_position_(false)
    , enable_velocity_(false)
{
  this->addProperty("gear", gear_);
  this->addProperty("encoder_res", encoder_res_);
  this->addProperty("motor_offset", motor_offset_);
  this->addProperty("joint_offset", joint_offset_);
  this->addProperty("enable_position", enable_position_);
  this->addProperty("enable_velocity", enable_velocity_);
}

SimpleTransmision::~SimpleTransmision() {
}

bool SimpleTransmision::configureHook() {
    RTT::Logger::In in("SimpleTransmision::configureHook: " + getName());
    if (gear_ == 0.0) {
        RTT::Logger::log() << RTT::Logger::Error << "property \'gear\' is not set" << RTT::Logger::endl;
        return false;
    }

    if (!enable_position_ && !enable_velocity_) {
        RTT::Logger::log() << RTT::Logger::Error << "parameters 'enable_position' and 'enable_velocity' are not set" << RTT::Logger::endl;
        return false;
    }

    if (enable_position_) {
      this->ports()->addPort(port_motor_position_in_);
      this->ports()->addPort(port_joint_position_out_);
    }

    if (enable_velocity_) {
      this->ports()->addPort(port_motor_velocity_in_);
      this->ports()->addPort(port_joint_velocity_out_);
    }

    return true;
}

bool SimpleTransmision::startHook() {
  return true;
}

void SimpleTransmision::updateHook() {
  double mpos, mvel;
  if (enable_position_) {
    if (port_motor_position_in_.read(mpos) == RTT::NewData) {
      double jpos =  ((mpos - motor_offset_)/(encoder_res_ * gear_) * M_PI * 2) + joint_offset_;
      port_joint_position_out_.write(jpos);
    }
    else {
      RTT::Logger::log() << RTT::Logger::Error << "could not read port '" << port_motor_position_in_.getName() << "'" << RTT::Logger::endl;
      error();
      return;
    }
  }

  if (enable_velocity_) {
    if (port_motor_velocity_in_.read(mvel) == RTT::NewData) {
      double jvel =  ((mvel)/(encoder_res_ * gear_) * M_PI * 2);
      port_joint_velocity_out_.write(jvel);
    }
    else {
      RTT::Logger::log() << RTT::Logger::Error << "could not read port '" << port_motor_velocity_in_.getName() << "'" << RTT::Logger::endl;
      error();
      return;
    }
  }
}

