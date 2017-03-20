// Copyright 2014 WUT
/*
 * simple_transmision.cpp
 *
 *  Created on: 8 aug 2014
 *      Author: dseredyn
 */

#include "scalar_dummy.h"

#include <string>

#include "rtt_rosclock/rtt_rosclock.h"

ScalarDummy::ScalarDummy(const std::string& name)
    : RTT::TaskContext(name)
    , port_out_("scalar_OUTPORT", false) {

  this->ports()->addPort(port_out_);
}

ScalarDummy::~ScalarDummy() {
}

bool ScalarDummy::startHook() {
  return true;
}

void ScalarDummy::updateHook() {
  port_out_.write(0);
}

