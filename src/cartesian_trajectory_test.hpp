#ifndef OROCOS_CARTMOTIONGENERATOR_COMPONENT_HPP
#define OROCOS_CARTMOTIONGENERATOR_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cartesian_trajectory_msgs/CartesianTrajectory.h>
#include <cartesian_trajectory_msgs/CartesianTrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "ros/time.h"
#include "string"


class CartTrajTest : public RTT::TaskContext{
  public:
    CartTrajTest(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

protected:
    RTT::OutputPort<cartesian_trajectory_msgs::CartesianTrajectory> mOutPortCartTraj;

private:
    cartesian_trajectory_msgs::CartesianTrajectoryPoint mPoint;
    cartesian_trajectory_msgs::CartesianTrajectory mCartTraj;
    
    
    void addVia(const std::vector<double>& val);
    void execTraj(void);
    void clearTraj(void);
    //void goInit(void);
};
#endif
