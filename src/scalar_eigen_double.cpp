/*
 Copyright (c) 2014-2017, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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

#include "Eigen/Dense"

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <vector>
#include <string>

class ScalarEigenToDouble : public RTT::TaskContext {
 public:
  explicit ScalarEigenToDouble(const std::string & name)
    : RTT::TaskContext(name)
    , port_in_("sc_INPORT")
    , port_out_("sc_OUTPORT")
  {
    this->ports()->addPort(port_in_);
    this->ports()->addPort(port_out_);
  }

  ~ScalarEigenToDouble() {
  }

  void updateHook() {
    if (port_in_.read(in_) == RTT::NewData) {
        out_ = in_(0);
        port_out_.write(out_);
    }
  }

 private:

  typedef Eigen::Matrix<double, 1, 1> ScalarIn;
  typedef double ScalarOut;

  RTT::InputPort<ScalarIn > port_in_;
  RTT::OutputPort<ScalarOut > port_out_;

  ScalarIn in_;
  ScalarOut out_;
};


class ScalarDoubleToEigen : public RTT::TaskContext {
 public:
  explicit ScalarDoubleToEigen(const std::string & name)
    : RTT::TaskContext(name)
    , port_in_("sc_INPORT")
    , port_out_("sc_OUTPORT")
  {
    this->ports()->addPort(port_in_);
    this->ports()->addPort(port_out_);
  }

  ~ScalarDoubleToEigen() {
  }

  void updateHook() {
    if (port_in_.read(in_) == RTT::NewData) {
        out_(0) = in_;
        port_out_.write(out_);
    }
  }

 private:

  typedef double ScalarIn;
  typedef Eigen::Matrix<double, 1, 1> ScalarOut;

  RTT::InputPort<ScalarIn > port_in_;
  RTT::OutputPort<ScalarOut > port_out_;

  ScalarIn in_;
  ScalarOut out_;
};

ORO_LIST_COMPONENT_TYPE(ScalarEigenToDouble)

ORO_LIST_COMPONENT_TYPE(ScalarDoubleToEigen)

