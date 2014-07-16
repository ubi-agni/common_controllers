// Copyright 2014 WUT
/*
 * tf_publisher.h
 *
 *  Created on: 16 jul 2014
 *      Author: dseredyn
 */

#ifndef TF_PUBLISHER_H_
#define TF_PUBLISHER_H_

#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

#include "geometry_msgs/Pose.h"
#include "tf/tfMessage.h"

class TfPublisher: public RTT::TaskContext {
 public:
  explicit TfPublisher(const std::string& name);
  virtual ~TfPublisher();
  virtual bool configureHook();
  virtual bool startHook();
  virtual void updateHook();

 private:
  std::vector<RTT::InputPort<geometry_msgs::Pose>* > port_in_;
  RTT::OutputPort<tf::tfMessage > port_out_tf_;

  geometry_msgs::Pose pose_;
  tf::tfMessage message_;

  std::vector<std::string > frame_ids_;
  std::vector<std::string > child_frame_ids_;

  size_t N_;
};

#endif  // TF_PUBLISHER_H_

