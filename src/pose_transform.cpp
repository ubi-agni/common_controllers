// Copyright 2014 WUT
/*
 * pose_transform.cpp
 *
 *  Created on: 11 kwi 2014
 *      Author: w
 */

#include "pose_transform.h"

using namespace KDL;

PoseTransform::PoseTransform(const std::string &name) :
		RTT::TaskContext(name, PreOperational)
{
	this->addProperty("input_frames", input_frames_);
	port_primary_frame_pose_.resize(input_frames_);
	primary_frame_pose_.resize(input_frames_);
	primary_frame.resize(input_frames_);
	primary_frame_status.resize(input_frames_);
	
	for(size_t i = 0; i < input_frames_; i++) {
		char port_name[10];
//		sprintf(port_name, "PrimaryFrame%zu", i+1); // "PrimaryFrame0" is base frame
//		this->ports()->addPort(port_name, port_primary_frame_pose_[i]).doc("");
		
		snprintf(port_name, sizeof(port_name), "PrimaryFrame%zu", i); // "PrimaryFrame0" is base frame
		port_primary_frame_pose_[i] = new typeof(*port_primary_frame_pose_[i]);
		this->ports()->addPort(port_name, *port_primary_frame_pose_[i]);
	}
	this->ports()->addPort("", port_primary_target_pose_).doc("");
	this->ports()->addPort("", port_primary_frame_selector_).doc("");
	this->ports()->addPort("", port_secondary_target_pose_).doc("");
}

PoseTransform::~PoseTransform()
{
}

bool PoseTransform::configureHook() {
	for(size_t i = 0; i < input_frames_; i++) {
		primary_frame_status[i] = pose_none;
	}
	primary_target_status = pose_none;
	port_secondary_target_pose_.setDataSample(secondary_target_pose_);
	return true;
}

bool PoseTransform::startHook() {

}

void PoseTransform::updateHook() {
		geometry_msgs::Pose pos;

	if (port_primary_frame_selector_.read(primary_frame_selector) == RTT::NewData) {
	}
	
	if (port_primary_frame_pose_[primary_frame_selector]->read(primary_frame_pose_[primary_frame_selector]) == RTT::NewData){
		primary_frame_status[primary_frame_selector] = pose_new;
		tf::poseMsgToKDL(primary_frame_pose_[primary_frame_selector], primary_frame[primary_frame_selector]);
	}
	
	if (port_primary_target_pose_.read(primary_target_pose_) == RTT::NewData){
		primary_target_status = pose_new;
		tf::poseMsgToKDL(primary_target_pose_, primary_target);
	}

	// On any change of primary_frame or primary_target
	if((primary_frame_status[primary_frame_selector] + primary_target_status) >= (pose_new + pose_old)){
		primary_frame_status[primary_frame_selector] = pose_old;
		primary_target_status = pose_old;
	
		// If you have a Frame F_A_B that expresses the pose of frame B wrt frame A,
		// and a Frame F_B_C that expresses the pose of frame C wrt to frame B,
		// the calculation of Frame F_A_C that expresses the pose of frame C wrt to frame A is as follows:
		// Frame F_A_C = F_A_B * F_B_C;
		secondary_target = primary_frame[primary_frame_selector] * primary_target;
		
		tf::poseKDLToMsg(secondary_target, secondary_target_pose_);
		port_secondary_target_pose_.write(secondary_target_pose_);
		
	}
			

}

//ORO_CREATE_COMPONENT(PoseTransform)

