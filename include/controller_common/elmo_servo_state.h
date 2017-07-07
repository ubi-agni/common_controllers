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

#ifndef COMMON_CONTROLLERS_ELMO_SERVO_STATE_H__
#define COMMON_CONTROLLERS_ELMO_SERVO_STATE_H__

#include <vector>
#include <string>
#include <cstdint>

namespace controller_common {

namespace elmo_servo {
/*
const uint16_t SW_ReadyToSwitchOn_Mask = 0x0001;
const uint16_t SW_SwitchedOn_Mask = 0x0002;
const uint16_t SW_OperationEnabled_Mask = 0x0004;
const uint16_t SW_Fault_Mask = 0x0008;
const uint16_t SW_VoltageEnabled_Mask = 0x0010;
const uint16_t SW_QuickStop_Mask = 0x0020;
const uint16_t SW_SwitchOnDisabled_Mask = 0x0040;
const uint16_t SW_Warning_Mask = 0x0080;
const uint16_t SW_ManufactureSpecific_Mask = 0x0100;
const uint16_t SW_Remote_Mask = 0x0200;
const uint16_t SW_TargetReached_Mask = 0x0400;
const uint16_t SW_InternalLimitActive_Mask = 0x0800;
const uint16_t SW_HomingComplete_Mask = 0x1000;

const uint16_t NotReadyToSwitchOn_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_SwitchOnDisabled_Mask;
const uint16_t NotReadyToSwitchOn_Pattern = 0x0000;

const uint16_t SwitchOnDisabled_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_SwitchOnDisabled_Mask;
const uint16_t SwitchOnDisabled_Pattern = SW_SwitchOnDisabled_Mask;

const uint16_t ReadyToSwitchOn_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_QuickStop_Mask | SW_SwitchOnDisabled_Mask;
const uint16_t ReadyToSwitchOn_Pattern = SW_ReadyToSwitchOn_Mask
    | SW_QuickStop_Mask;

const uint16_t SwitchedOn_Mask = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask
    | SW_OperationEnabled_Mask | SW_Fault_Mask | SW_QuickStop_Mask
    | SW_SwitchOnDisabled_Mask;
const uint16_t SwitchedOn_Pattern = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask
    | SW_QuickStop_Mask;

const uint16_t OperationEnabled_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_QuickStop_Mask | SW_SwitchOnDisabled_Mask;
const uint16_t OperationEnabled_Pattern = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_QuickStop_Mask;

const uint16_t Fault_Reaction_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_SwitchOnDisabled_Mask;
const uint16_t Fault_Reaction_Pattern = SW_Fault_Mask | SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask;

const uint16_t Fault_Mask = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask
    | SW_OperationEnabled_Mask | SW_Fault_Mask | SW_SwitchOnDisabled_Mask;
const uint16_t Fault_Pattern = SW_Fault_Mask;

const uint16_t QuickStopActive_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_QuickStop_Mask | SW_SwitchOnDisabled_Mask;
const uint16_t QuickStopActive_Pattern = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask;

const std::string servoStateNames[9];
*/

enum class ServoState {
    INVALID = 0,
    NOT_READY_TO_SWITCH_ON = 1,
    SWITCH_ON_DISABLED = 2,
    READY_TO_SWITCH_ON = 3,
    SWITCH_ON = 4,
    OPERATION_ENABLED = 5,
    QUICK_STOP_ACTIVE = 6,
    FAULT_REACTION_ACTIVE = 7,
    FAULT = 8
};

ServoState getServoState(uint16_t statusword);

uint16_t getStatusWord(ServoState s);

const std::string& getServoStateStr(ServoState s);

}   // namespace elmo_servo

}   // namespace controller_common

#endif  // COMMON_CONTROLLERS_ELMO_SERVO_STATE_H__

