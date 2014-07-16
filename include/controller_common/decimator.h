// Copyright 2014 WUT
#ifndef DECIMATOR_H_
#define DECIMATOR_H_

#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

template <class T >
class Decimator : public RTT::TaskContext {
 public:
  explicit Decimator(const std::string & name) : TaskContext(name, PreOperational) {
    this->ports()->addPort("Out", port_output_);
    this->ports()->addPort("In", port_input_);
  }

  ~Decimator() {
  }

  bool configureHook() {
    port_input_.getDataSample(input_);
    port_output_.setDataSample(input_);

    return true;
  }

  bool startHook() {
    return true;
  }

  void updateHook() {
    port_input_.readNewest(input_);
    port_output_.write(input_);
  }

 private:
  RTT::InputPort<T > port_input_;

  RTT::OutputPort<T > port_output_;

  T input_;
};
#endif  // DECIMATOR_H_

