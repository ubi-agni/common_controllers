// Copyright 2014 WUT
/*
 * pose_transform.h
 *
 *  Created on: 11 apr 2014
 *      Author: mwalecki
 */

#ifndef POSE_TRANSFORM_H_
#define POSE_TRANSFORM_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <geometry_msgs/Pose.h>

#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>
#include <string>
#include <vector>

enum pose_status{
    pose_none = 0,
    pose_old = 1,
    pose_new = 2
};

class PoseTransform: public RTT::TaskContext {
 public:
  explicit PoseTransform(const std::string &name);
  virtual ~PoseTransform();
  virtual bool configureHook();
  virtual bool startHook();
  virtual void updateHook();

 private:
  std::vector<geometry_msgs::Pose> primary_frame_pose_;
  std::vector<pose_status> primary_frame_status;
  std::vector<KDL::Frame> primary_frame;

  geometry_msgs::Pose primary_target_pose_;
  pose_status primary_target_status;
  KDL::Frame primary_target;

  geometry_msgs::Pose secondary_target_pose_;
  KDL::Frame secondary_target;
  int primary_frame_selector;

  std::vector<RTT::InputPort<geometry_msgs::Pose>* > port_primary_frame_pose_;
  RTT::InputPort<geometry_msgs::Pose> port_primary_target_pose_;
  RTT::InputPort<int> port_primary_frame_selector_;
  RTT::OutputPort<geometry_msgs::Pose> port_secondary_target_pose_;

  int input_frames_;
};

#endif  // POSE_TRANSFORM_H_

