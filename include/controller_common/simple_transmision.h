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

#ifndef SIMPLE_TRANSMISION_H_
#define SIMPLE_TRANSMISION_H_

#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

template <typename INPUT_TYPE >
class SimpleTransmision: public RTT::TaskContext {
public:
    explicit SimpleTransmision(const std::string& name)
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

    virtual ~SimpleTransmision() {
    }

    virtual bool configureHook() {
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

    virtual bool startHook() {
        return true;
    }

    virtual void updateHook() {
      INPUT_TYPE mpos, mvel;
      if (enable_position_) {
        if (port_motor_position_in_.read(mpos) == RTT::NewData) {
          double jpos =  ((static_cast<double >(mpos) - motor_offset_)/(encoder_res_ * gear_) * M_PI * 2) + joint_offset_;
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
          double jvel =  (static_cast<double >(mvel)/(encoder_res_ * gear_) * M_PI * 2);
          port_joint_velocity_out_.write(jvel);
        }
        else {
          RTT::Logger::log() << RTT::Logger::Error << "could not read port '" << port_motor_velocity_in_.getName() << "'" << RTT::Logger::endl;
          error();
          return;
        }
      }
    }

private:
  RTT::InputPort<INPUT_TYPE> port_motor_position_in_;
  RTT::InputPort<INPUT_TYPE> port_motor_velocity_in_;
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

