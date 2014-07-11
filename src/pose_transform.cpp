// Copyright 2014 WUT
/*
 * pose_transform.cpp
 *
 *  Created on: 11 apr 2014
 *      Author: mwalecki
 */

#include "pose_transform.h"

#include <string>

PoseTransform::PoseTransform(const std::string &name)
    : RTT::TaskContext(name, PreOperational) {
  this->addProperty("input_frames", input_frames_);
  // variable init
  primary_frame_selector = 0;
  primary_target_status = pose_none;
}

PoseTransform::~PoseTransform() {
}

bool PoseTransform::configureHook() {
  port_primary_frame_pose_.resize(input_frames_ + 1);
  primary_frame_pose_.resize(input_frames_ + 1);
  primary_frame.resize(input_frames_ + 1);
  primary_frame_status.resize(input_frames_ + 1);

  for (size_t i = 1; i <= input_frames_; i++) {
    // port "0" will not be initialized - "0" is base frame
    char port_name[16];
    snprintf(port_name, sizeof(port_name), "PrimaryFrame%zu", i);  // "PrimaryFrame0" is base frame
    port_primary_frame_pose_[i] = new typeof(*port_primary_frame_pose_[i]);
    this->ports()->addPort(port_name, *port_primary_frame_pose_[i]);
  }
  this->ports()->addPort("PrimaryTargetPoint", port_primary_target_pose_).doc(
      "");
  this->ports()->addPort("PrimaryFrameSelector", port_primary_frame_selector_)
      .doc("");
  this->ports()->addPort("SecondaryTargetPoint", port_secondary_target_pose_)
      .doc("");

  for (size_t i = 0; i <= input_frames_; i++) {
    primary_frame_status[i] = pose_none;
  }
  primary_target_status = pose_none;

  // Initialize base frame
  primary_frame_pose_[0].position.x = 0;
  primary_frame_pose_[0].position.y = 0;
  primary_frame_pose_[0].position.z = 0;
  primary_frame_pose_[0].orientation.x = 0;
  primary_frame_pose_[0].orientation.y = 0;
  primary_frame_pose_[0].orientation.z = 0;
  primary_frame_pose_[0].orientation.w = 1;
  primary_frame_status[0] = pose_old;
  // Default primary frame is base
  primary_frame_selector = 0;
  port_secondary_target_pose_.setDataSample(secondary_target_pose_);
  return true;
}

void PoseTransform::updateHook() {
  geometry_msgs::Pose pos;

  if (port_primary_frame_selector_.read(primary_frame_selector)
      == RTT::NewData) {
  }

  if (primary_frame_selector < 0) {
    // Say what?
  } else if (primary_frame_selector == 0) {
    primary_frame_status[0] = pose_new;  // Base frame to base frame transform
  } else if (primary_frame_selector <= input_frames_) {
    if (port_primary_frame_pose_[primary_frame_selector]->read(
        primary_frame_pose_[primary_frame_selector]) == RTT::NewData) {
      primary_frame_status[primary_frame_selector] = pose_new;
      tf::poseMsgToKDL(primary_frame_pose_[primary_frame_selector],
                       primary_frame[primary_frame_selector]);
    }
  }

  if (port_primary_target_pose_.read(primary_target_pose_) == RTT::NewData) {
    primary_target_status = pose_new;
    /*std::cout << "PT: p: ";
     std::cout << primary_target_pose_.position.x << " ";
     std::cout << primary_target_pose_.position.y << " ";
     std::cout << primary_target_pose_.position.z << " o: ";
     std::cout << primary_target_pose_.orientation.x << " ";
     std::cout << primary_target_pose_.orientation.y << " ";
     std::cout << primary_target_pose_.orientation.z << " ";
     std::cout << primary_target_pose_.orientation.w << std::endl;*/
    tf::poseMsgToKDL(primary_target_pose_, primary_target);
  }

  // On any change of primary_frame or primary_target
  if ((primary_frame_status[primary_frame_selector] + primary_target_status)
      >= (pose_new + pose_old)) {
    primary_frame_status[primary_frame_selector] = pose_old;
    primary_target_status = pose_old;

    // If you have a Frame F_A_B that expresses the pose of frame B wrt frame A,
    // and a Frame F_B_C that expresses the pose of frame C wrt to frame B,
    // the calculation of Frame F_A_C that expresses the pose of frame C wrt to frame A is as follows:
    // Frame F_A_C = F_A_B * F_B_C;
    secondary_target = primary_frame[primary_frame_selector] * primary_target;

    tf::poseKDLToMsg(secondary_target, secondary_target_pose_);
    /*std::cout << "ST: p: ";
     std::cout << secondary_target_pose_.position.x << " ";
     std::cout << secondary_target_pose_.position.y << " ";
     std::cout << secondary_target_pose_.position.z << " o: ";
     std::cout << secondary_target_pose_.orientation.x << " ";
     std::cout << secondary_target_pose_.orientation.y << " ";
     std::cout << secondary_target_pose_.orientation.z << " ";
     std::cout << secondary_target_pose_.orientation.w << std::endl << std::endl;*/
    port_secondary_target_pose_.write(secondary_target_pose_);
  }
}

