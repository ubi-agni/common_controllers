#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <Eigen/Dense>

#include <vector>
#include <string>

template <int N>
class VectorConcate : public RTT::TaskContext {
public:
  VectorConcate(const std::string & name) : RTT::TaskContext(name, PreOperational) {
    for(size_t i = 0; i < N; i++) {
      char port_name[10];
      sprintf(port_name, "In%zu", i);
      this->ports()->addPort(port_name, port_inputs_[i]);
    }
    
    this->ports()->addPort("Out", port_output_);
  }

  ~VectorConcate(){
  }

  bool configureHook() {
    size_t size = 0;
    for(size_t i = 0; i < N; i++) {
      port_inputs_[i].getDataSample(inputs_[i]);
      size += inputs_[i].size();
    }
    
    output_.resize(size);
    port_output_.setDataSample(output_);
    
    return true;
  }

  void updateHook() {
    size_t k = 0;
    bool new_data = false;
    for(size_t i = 0; i < N; i++) {
      if(port_inputs_[i].read(inputs_[i]) == RTT::NewData)
        new_data = true;
      for(size_t j = 0; j < inputs_[i].size(); j++) {
        output_[k++] = inputs_[i][j];
      }
    }
    if(new_data)
      port_output_.write(output_);
  }

private:

  RTT::InputPort<Eigen::VectorXd > port_inputs_[N];
  
  RTT::OutputPort<Eigen::VectorXd > port_output_;

  Eigen::VectorXd inputs_[N];
  Eigen::VectorXd output_;
};
