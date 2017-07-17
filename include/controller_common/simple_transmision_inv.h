/*
 Copyright (c) 2017, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef SIMPLE_TRANSMISION_INV_H_
#define SIMPLE_TRANSMISION_INV_H_

#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

template <typename TYPE_OUT1, typename TYPE_OUT2 >
class SimpleTransmisionInv: public RTT::TaskContext {
 public:
    explicit SimpleTransmisionInv(const std::string& name)
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

    virtual ~SimpleTransmisionInv() {
    }

    virtual bool configureHook() {
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

    virtual bool startHook() {
        return true;
    }

    virtual void updateHook() {
      double jpos, jvel, jtrq;
      if (enable_position_) {
        if (port_joint_position_.read(jpos) == RTT::NewData) {
          TYPE_OUT1 mpos =  (((jpos - joint_offset_)*(encoder_res_ * gear_))/(M_PI * 2)) + motor_offset_;
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
          TYPE_OUT1 mvel =  ((jvel * (encoder_res_ * gear_))/(M_PI * 2));
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
          TYPE_OUT2 mcur =  (jtrq/gear_)/motor_constant_;
          port_motor_current_.write(mcur);
        }
        else {
          RTT::Logger::log() << RTT::Logger::Error << "could not read port '" << port_joint_torque_.getName() << "'" << RTT::Logger::endl;
          error();
          return;
        }
      }
    }


 private:
  RTT::OutputPort<TYPE_OUT1 > port_motor_position_;
  RTT::OutputPort<TYPE_OUT1 > port_motor_velocity_;
  RTT::OutputPort<TYPE_OUT2 > port_motor_current_;
  RTT::InputPort<double> port_joint_position_;
  RTT::InputPort<double> port_joint_velocity_;
  RTT::InputPort<double> port_joint_torque_;

  double gear_;
  double encoder_res_;
  double motor_offset_;
  double joint_offset_;
  double motor_constant_;

  bool enable_position_;
  bool enable_velocity_;
  bool enable_torque_;
};

#endif  // SIMPLE_TRANSMISION_INV_H_

