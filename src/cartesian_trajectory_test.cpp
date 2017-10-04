#include "cartesian_trajectory_test.hpp"
#include <rtt/Component.hpp>
#include "rtt_rosclock/rtt_rosclock.h"
#include <iostream>

CartTrajTest::CartTrajTest(std::string const& name) :
    RTT::TaskContext(name, PreOperational),
    mOutPortCartTraj("DesiredCartTrajectory")
{
  ports()->addPort(mOutPortCartTraj);

  addOperation("execTraj", &CartTrajTest::execTraj,this).doc("send the trajectory");
  addOperation("addVia", (void (CartTrajTest::*)(const std::vector<double>&))&CartTrajTest::addVia,this).doc("add via points to the trajectory (x,y,z, a,b,c, time_from_start)");
  addOperation("clearTraj", &CartTrajTest::clearTraj, this).doc("clear trajectory");
  //addOperation("goInit", &CartTrajTest::goInit, this).doc("goto initial pose");
  std::cout << "CartTrajTest constructed !" <<std::endl;
}

bool CartTrajTest::configureHook(){
  // initialize varialbles
  std::cout << "CartTrajTest configured !" <<std::endl;
  mCartTraj.target_frame = "world";
  return true;
}

bool CartTrajTest::startHook(){
  std::cout << "CartTrajTest started !" <<std::endl;
  return true;
}

void CartTrajTest::updateHook(){

}

void CartTrajTest::stopHook() {
  std::cout << "CartTrajTest executes stopping !" <<std::endl;
}

void CartTrajTest::cleanupHook() {
    std::cout << "CartTrajTest cleaning up !" <<std::endl;
}


void CartTrajTest::addVia(const std::vector<double>& val) {
  if (val.size() == 7)
  {
    // if there is already a point, check monotonous time increase
    bool valid = true;
    if (mCartTraj.points.size())
    {
      double tfs = mCartTraj.points.back().time_from_start.toSec();
      if (val[6] <= tfs)
      {
        valid = false;
        std::cout << "Received via point with non increasing time_from_start. Found " << val[6]
                  << "s, expected >" << tfs << "s" << std::endl;
      }
      
    }
    if (valid)
    {
      mPoint.pose.position.x = val[0];
      mPoint.pose.position.y = val[1];
      mPoint.pose.position.z = val[2];
      Eigen::Quaterniond target_q(Eigen::AngleAxisd(val[5], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(val[4],  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(val[3], Eigen::Vector3d::UnitX()));
      mPoint.pose.orientation.w = target_q.w();
      mPoint.pose.orientation.x = target_q.x();
      mPoint.pose.orientation.y = target_q.y();
      mPoint.pose.orientation.z = target_q.z();

      mPoint.time_from_start = ros::Duration(val[6]);
      mCartTraj.points.push_back(mPoint);
      std::cout << "via point added" <<std::endl;
    }
  }
  else
  {
    std::cout << "Received targets with invalid size : "<< val.size()<< " expected : " << 7 << std::endl;
  }
}

void CartTrajTest::execTraj() {
  mCartTraj.header.stamp = rtt_rosclock::host_now();
  mOutPortCartTraj.write(mCartTraj);
}

void CartTrajTest::clearTraj() {
  mCartTraj.points.clear();
}


/*
void CartTrajTest::goInit() {
  mCartTraj.points.clear();
  mCartTraj.points.push_back(mPoseInit)
  execTraj();
  std::cout << "Initial pose sent" <<std::endl;
}
*/

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(CartTrajTest)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(CartTrajTest)
