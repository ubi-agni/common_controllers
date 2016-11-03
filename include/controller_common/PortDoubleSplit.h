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

#ifndef PORTDOUBLESPLIT_H_
#define PORTDOUBLESPLIT_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <Eigen/Dense>
#include <string>
#include <vector>

template <unsigned N>
class PortDoubleSplit : public RTT::TaskContext {
 public:
  explicit PortDoubleSplit(const std::string& name);
  virtual ~PortDoubleSplit();

  bool configureHook();
  void updateHook();
 private:

  typedef Eigen::Matrix<double, N, 1> VectorNd;

  // ports
  RTT::InputPort<VectorNd> input_port_;
  boost::array<RTT::OutputPort<double>*, N> port_output_list_;

  VectorNd data_;
};

template <unsigned N>
PortDoubleSplit<N>::PortDoubleSplit(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {
  this->ports()->addPort("InputPort", input_port_);
}

template <unsigned N>
PortDoubleSplit<N>::~PortDoubleSplit() {
}

template <unsigned N>
bool PortDoubleSplit<N>::configureHook() {
  for (size_t i = 0; i < N; i++) {
    char port_name[16];
    snprintf(port_name, sizeof(port_name), "OutputPort_%zu", i);
    port_output_list_[i] = new typeof(*port_output_list_[i]);
    this->ports()->addPort(port_name, *port_output_list_[i]);
    port_output_list_[i]->keepLastWrittenValue(false);
  }

  return true;
}

template <unsigned N>
void PortDoubleSplit<N>::updateHook() {
  if (RTT::NewData == input_port_.read(data_)) {
    for (int i = 0; i < N; i++) {
      port_output_list_[i]->write(data_[i]);
    }
  }
}

#endif  // PORTDOUBLESPLIT_H_
