/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef COMMON_CONTROLLERS_BYPASS_COMPONENT_H__
#define COMMON_CONTROLLERS_BYPASS_COMPONENT_H__

#include <vector>
#include <string>
#include "rtt/RTT.hpp"

namespace controller_common {

class BypassComponent: public RTT::TaskContext {
public:
    explicit BypassComponent(const std::string &name);

    bool addInputPort(std::shared_ptr<RTT::base::InputPortInterface > p);

private:
    bool configureHook();

    bool startHook();

    void updateHook();

    std::vector<std::shared_ptr<RTT::base::InputPortInterface > > input_ports_;

    std::vector<RTT::base::DataSourceBase::shared_ptr > ds_;
    std::vector<RTT::base::InputPortInterface* > ipi_;
    std::vector<RTT::base::OutputPortInterface* > opi_;
};

BypassComponent::BypassComponent(const std::string &name)
    : TaskContext(name, PreOperational)
{
    this->addOperation("addInputPort", &BypassComponent::addInputPort, this, RTT::ClientThread);
}

bool BypassComponent::addInputPort(std::shared_ptr<RTT::base::InputPortInterface > p) {
    if (!p) {
        return false;
    }

    input_ports_.push_back( p );
    this->ports()->addPort( *p );
    return true;
}

bool BypassComponent::configureHook() {
    RTT::DataFlowInterface::Ports ports = this->ports()->getPorts();
    for (int i = 0; i < ports.size(); ++i) {
        RTT::base::InputPortInterface* ipi = dynamic_cast<RTT::base::InputPortInterface* >(ports[i]);
        if (ipi) {
            RTT::base::OutputPortInterface* opi( dynamic_cast<RTT::base::OutputPortInterface* >(ipi->antiClone()) );
            std::string name = opi->getName();
            name = name + "_OUTPORT";
            opi->setName(name);
            this->ports()->addPort(*opi);
            ipi_.push_back(ipi);
            opi_.push_back(opi);
            ds_.push_back(RTT::base::DataSourceBase::shared_ptr(ipi->getDataSource()));
            ipi->setName(ipi->getName() + "_INPORT");
        }
    }

    return true;
}

bool BypassComponent::startHook() {
    return true;
}

void BypassComponent::updateHook() {
    for (int i = 0; i < ipi_.size(); ++i) {
        if (ipi_[i]->read(ds_[i]) == RTT::NewData) {
            opi_[i]->write(ds_[i]);
        }
    }
}

}   //namespace controller_common

#endif  // COMMON_CONTROLLERS_BYPASS_COMPONENT_H__

