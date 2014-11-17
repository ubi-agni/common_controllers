// Copyright 2014 WUT
/*
 * tf_publisher.cpp
 *
 *  Created on: 16 jul 2014
 *      Author: dseredyn
 */

#include "tf_publisher.h"

#include <string>

#include "rtt_rosclock/rtt_rosclock.h"

TfPublisher::TfPublisher(const std::string& name)
    : RTT::TaskContext(name),
      N_(0) {
  this->ports()->addPort("OutTf", port_out_tf_);
  this->addProperty("frame_ids", frame_ids_);
  this->addProperty("child_frame_ids", child_frame_ids_);
}

TfPublisher::~TfPublisher() {
}

bool TfPublisher::configureHook() {
  if (frame_ids_.size() != child_frame_ids_.size()) {
    RTT::log(RTT::Error) << "Wrong size of input vectors"
                         << RTT::endlog();
    return false;
  }

  N_ = frame_ids_.size();

  if (N_ == 0) {
    RTT::log(RTT::Error) << "Input vectors is null"
                         << RTT::endlog();
    return false;
  }

  message_.transforms.resize(N_);
  port_in_.resize(N_);

  for (size_t i = 0; i < N_; i++) {
    char name[30];
    snprintf(name, sizeof(name), "In%zu", i);
    port_in_[i] = new typeof(*port_in_[i]);
    this->ports()->addPort(name, *port_in_[i]);
  }

  pose_.position.x = 0;
  pose_.position.y = 0;
  pose_.position.z = 0;
  pose_.orientation.x = 0;
  pose_.orientation.y = 0;
  pose_.orientation.z = 0;
  pose_.orientation.w = 0;

  return true;
}

bool TfPublisher::startHook() {
  return true;
}

void TfPublisher::updateHook() {
  for (size_t i = 0; i < N_; i++) {
    port_in_[i]->readNewest(pose_);
    message_.transforms[i].header.stamp = rtt_rosclock::host_now();
    message_.transforms[i].header.frame_id = frame_ids_[i];
    message_.transforms[i].child_frame_id = child_frame_ids_[i];
    message_.transforms[i].transform.translation.x = pose_.position.x;
    message_.transforms[i].transform.translation.y = pose_.position.y;
    message_.transforms[i].transform.translation.z = pose_.position.z;
    message_.transforms[i].transform.rotation = pose_.orientation;
  }

  port_out_tf_.write(message_);
}

