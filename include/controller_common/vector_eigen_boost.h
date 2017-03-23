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

#ifndef VECTOR_EIGEN_BOOST_H_
#define VECTOR_EIGEN_BOOST_H_

#include "Eigen/Dense"

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <vector>
#include <string>

template <int N>
class VectorEigenToBoost : public RTT::TaskContext {
 public:
  explicit VectorEigenToBoost(const std::string & name)
    : RTT::TaskContext(name)
    , port_in_("vec_INPORT")
    , port_out_("vec_OUTPORT")
  {
    this->ports()->addPort(port_in_);
    this->ports()->addPort(port_out_);
  }

  ~VectorEigenToBoost() {
  }

  void updateHook() {
    if (port_in_.read(in_) == RTT::NewData) {
        for (int i = 0; i < N; ++i) {
            out_[i] = in_(i);
        }
        port_out_.write(out_);
    }
  }

 private:

  typedef Eigen::Matrix<double, N, 1> VectorIn;
  typedef boost::array<double, N> VectorOut;

  RTT::InputPort<VectorIn > port_in_;
  RTT::OutputPort<VectorOut > port_out_;

  VectorIn in_;
  VectorOut out_;
};

template <int N>
class VectorBoostToEigen : public RTT::TaskContext {
 public:
  explicit VectorBoostToEigen(const std::string & name)
    : RTT::TaskContext(name)
    , port_in_("vec_INPORT")
    , port_out_("vec_OUTPORT")
  {
    this->ports()->addPort(port_in_);
    this->ports()->addPort(port_out_);
  }

  ~VectorBoostToEigen() {
  }

  void updateHook() {
    if (port_in_.read(in_) == RTT::NewData) {
        for (int i = 0; i < N; ++i) {
            out_(i) = in_[i];
        }
        port_out_.write(out_);
    }
  }

 private:

  typedef boost::array<double, N> VectorIn;
  typedef Eigen::Matrix<double, N, 1> VectorOut;

  RTT::InputPort<VectorIn > port_in_;
  RTT::OutputPort<VectorOut > port_out_;

  VectorIn in_;
  VectorOut out_;
};

#endif  // VECTOR_EIGEN_BOOST_H_

