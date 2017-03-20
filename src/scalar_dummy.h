// Copyright 2014 WUT
/*
 * simple_transmision.h
 *
 *  Created on: 8 aug 2014
 *      Author: dseredyn
 */

#ifndef CONTROLLER_COMMON_SCALAR_DUMMY_H_
#define CONTROLLER_COMMON_SCALAR_DUMMY_H_

#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

class ScalarDummy: public RTT::TaskContext {
 public:
  explicit ScalarDummy(const std::string& name);
  virtual ~ScalarDummy();
  virtual bool startHook();
  virtual void updateHook();

 private:
  RTT::OutputPort<double> port_out_;
};

#endif  // CONTROLLER_COMMON_SCALAR_DUMMY_H_

