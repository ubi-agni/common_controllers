// Copyright 2014 WUT
#ifndef VECTOR_SPLIT_H_
#define VECTOR_SPLIT_H_

#include <vector>
#include <string>

#include "Eigen/Dense"

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

template <int n1, int n2, int n3, int n4>
class VectorSplit : public RTT::TaskContext {
 public:
  explicit VectorSplit(const std::string & name) :
    TaskContext(name),
    port_input_("In"),
    port_output1_("Out0", true),
    port_output2_("Out1", true),
    port_output3_("Out2", true),
    port_output4_("Out3", true) {

    this->ports()->addPort(port_input_);

    if (n1 > 0) {
        this->ports()->addPort(port_output1_);
    }

    if (n2 > 0) {
        this->ports()->addPort(port_output2_);
    }

    if (n3 > 0) {
        this->ports()->addPort(port_output3_);
    }

    if (n4 > 0) {
        this->ports()->addPort(port_output4_);
    }
  }

  ~VectorSplit() {
  }

  bool startHook() {
    return true;
  }

  void updateHook() {
    if (port_input_.read(input_) == RTT::NewData) {

      if (n1 > 0) {
          output1_ = input_.template block<n1, 1>(0, 0);
          port_output1_.write(output1_);
      }

      if (n2 > 0) {
          output2_ = input_.template block<n2, 1>(n1, 0);
          port_output2_.write(output2_);
      }

      if (n3 > 0) {
          output3_ = input_.template block<n3, 1>(n1+n2, 0);
          port_output3_.write(output3_);
      }

      if (n4 > 0) {
          output4_ = input_.template block<n4, 1>(n1+n2+n3, 0);
          port_output4_.write(output4_);
      }
    }
  }

 private:

  typedef Eigen::Matrix<double, n1, 1> VectorOut1;
  typedef Eigen::Matrix<double, n2, 1> VectorOut2;
  typedef Eigen::Matrix<double, n3, 1> VectorOut3;
  typedef Eigen::Matrix<double, n4, 1> VectorOut4;
  RTT::OutputPort<VectorOut1 > port_output1_;
  RTT::OutputPort<VectorOut2 > port_output2_;
  RTT::OutputPort<VectorOut3 > port_output3_;
  RTT::OutputPort<VectorOut4 > port_output4_;
  VectorOut1 output1_;
  VectorOut2 output2_;
  VectorOut3 output3_;
  VectorOut4 output4_;

  typedef Eigen::Matrix<double, n1+n2+n3+n4, 1> VectorIn;
  RTT::InputPort<VectorIn > port_input_;
  VectorIn input_;
};
#endif  // VECTOR_SPLIT_H_

