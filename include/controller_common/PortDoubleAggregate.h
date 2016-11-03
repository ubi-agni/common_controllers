/*
 * Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PORTDOUBLEAGGREGATE_H_
#define PORTDOUBLEAGGREGATE_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <Eigen/Dense>
#include <string>
#include <vector>

template <unsigned N>
class PortDoubleAggregate : public RTT::TaskContext {
 public:
  explicit PortDoubleAggregate(const std::string& name);
  virtual ~PortDoubleAggregate();

  bool configureHook();
  void updateHook();
 private:

  typedef Eigen::Matrix<double, N, 1> VectorNd;

  // ports
  RTT::OutputPort<VectorNd> output_port_;
  boost::array<RTT::InputPort<double>*, N> port_input_list_;

  VectorNd data_;
};

template <unsigned N>
PortDoubleAggregate<N>::PortDoubleAggregate(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      output_port_("OutputPort", false)
{
  this->ports()->addPort(output_port_);
}

template <unsigned N>
PortDoubleAggregate<N>::~PortDoubleAggregate() {
}

template <unsigned N>
bool PortDoubleAggregate<N>::configureHook() {
  for (size_t i = 0; i < N; i++) {
    char port_name[16];
    snprintf(port_name, sizeof(port_name), "InputPort_%zu", i);
    port_input_list_[i] = new typeof(*port_input_list_[i]);
    this->ports()->addPort(port_name, *port_input_list_[i]);
  }

  return true;
}

template <unsigned N>
void PortDoubleAggregate<N>::updateHook() {
  bool new_data = true;
  for (int i = 0; i < N; i++) {
    if (RTT::NewData != port_input_list_[i]->read(data_(i))) {
      new_data = false;
    }
  }

  if (new_data) {
    output_port_.write(data_);
  }
}

#endif  // PORTDOUBLEAGGREGATE_H_
