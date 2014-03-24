#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <Eigen/Dense>

#include <vector>

template <int N>
class VectorSplit : public RTT::TaskContext {
public:
  VectorSplit(const std::string & name) : TaskContext(name, PreOperational), size_(0) {
  
    this->addProperty("outputs", out_);
    for(size_t i = 0; i < N; i++) {
      char port_name[10];
      sprintf(port_name, "Out%zu", i);
      this->ports()->addPort(port_name, port_outputs_[i]);
    }
    
    this->ports()->addPort("In", port_input_);
  }

  ~VectorSplit(){
  }

  bool configureHook() {
    
    if(!(out_.size() == N)) {
      RTT::log(RTT::Error) << "config error " << out_.size() << RTT::endlog();
      return false;
    }
    port_input_.getDataSample(input_);
    
    size_ = 0;
    for(size_t i = 0; i < N; i++) {
      size_ += out_[i];
      outputs_[i].resize(out_[i]);
      port_outputs_[i].setDataSample(outputs_[i]);
    }
    
   // if(!(input_.size() == size_))
   //   return false;
    
    return true;
  }

  bool startHook() {
	  return true;
  }

  void updateHook() {
    size_t k = 0;
    
    port_input_.read(input_);
    
    if(!(input_.size() == size_)) {
      RTT::Logger::log(RTT::Logger::Error) << "Input vector size (" << input_.size() << ") does not mach the expected (" << size_ << ")" << RTT::endlog();
      return ;
    }
    
    for(size_t i = 0; i < N; i++) {
      for(size_t j = 0; j < outputs_[i].size(); j++) {
        outputs_[i][j] = input_[k++];
      }
      port_outputs_[i].write(outputs_[i]);
    }
  }

private:

  RTT::InputPort<Eigen::VectorXd > port_input_;
  
  RTT::OutputPort<Eigen::VectorXd > port_outputs_[N];

  Eigen::VectorXd input_;
  Eigen::VectorXd outputs_[N];
  std::vector<int> out_;
  
  size_t size_;
};
