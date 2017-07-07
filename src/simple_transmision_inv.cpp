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
    : RTT::TaskContext(name, PreOperational)
    , gear_(0.0)
    , encoder_res_(1.0)
    , motor_offset_(0.0)
    , joint_offset_(0.0)
    , motor_constant_(0.0)
    , port_motor_position_("MotorPosition_OUTPORT", false)
    , port_motor_velocity_("MotorVelocity_OUTPORT", false)
    , port_joint_position_("JointPosition_INPORT")
    , port_joint_velocity_("JointVelocity_INPORT")
    , port_motor_current_("MotorCurrent_OUTPORT", false)
    , port_joint_torque_("JointTorque_INPORT")
    , enable_position_(false)
    , enable_velocity_(false)
    , enable_torque_(false)
 {
  this->addProperty("gear", gear_);
  this->addProperty("encoder_res", encoder_res_);
  this->addProperty("motor_offset", motor_offset_);
  this->addProperty("joint_offset", joint_offset_);
  this->addProperty("motor_constant", motor_constant_);
  this->addProperty("enable_position", enable_position_);
  this->addProperty("enable_velocity", enable_velocity_);
  this->addProperty("enable_torque", enable_torque_);
}

SimpleTransmisionInv::~SimpleTransmisionInv() {
}

bool SimpleTransmisionInv::configureHook() {
    RTT::Logger::In in("SimpleTransmisionInv::configureHook: " + getName());
    if (gear_ == 0.0) {
        RTT::Logger::log() << RTT::Logger::Error << "property \'gear\' is not set" << RTT::Logger::endl;
        return false;
    }

    if (!enable_position_ && !enable_velocity_ && !enable_torque_) {
        RTT::Logger::log() << RTT::Logger::Error << "parameters 'enable_position', 'enable_velocity' and 'enable_torque' are not set" << RTT::Logger::endl;
        return false;
    }

    if (enable_position_) {
      this->ports()->addPort(port_motor_position_);
      this->ports()->addPort(port_joint_position_);
    }

    if (enable_velocity_) {
      this->ports()->addPort(port_motor_velocity_);
      this->ports()->addPort(port_joint_velocity_);
    }

    if (enable_torque_) {
      this->ports()->addPort(port_motor_current_);
      this->ports()->addPort(port_joint_torque_);
    }

    return true;
}

bool SimpleTransmisionInv::startHook() {
  return true;
}

void SimpleTransmisionInv::updateHook() {
  double jpos, jvel, jtrq;
  if (enable_position_) {
    if (port_joint_position_.read(jpos) == RTT::NewData) {
      double mpos =  (((jpos - joint_offset_)*(encoder_res_ * gear_))/(M_PI * 2)) + motor_offset_;
      port_motor_position_.write(mpos);
    }
    else {
      RTT::Logger::log() << RTT::Logger::Error << "could not read port '" << port_joint_position_.getName() << "'" << RTT::Logger::endl;
      error();
      return;
    }
  }

  if (enable_velocity_) {
    if (port_joint_velocity_.read(jvel) == RTT::NewData) {
      double mvel =  ((jvel * (encoder_res_ * gear_))/(M_PI * 2));
      port_motor_velocity_.write(mvel);
    }
    else {
      RTT::Logger::log() << RTT::Logger::Error << "could not read port '" << port_joint_velocity_.getName() << "'" << RTT::Logger::endl;
      error();
      return;
    }
  }

  if (enable_torque_) {
    if (port_joint_torque_.read(jtrq) == RTT::NewData) {
      double mvel =  (jtrq/gear_)/motor_constant_;
      port_motor_current_.write(mvel);
    }
    else {
      RTT::Logger::log() << RTT::Logger::Error << "could not read port '" << port_joint_torque_.getName() << "'" << RTT::Logger::endl;
      error();
      return;
    }
  }
}

