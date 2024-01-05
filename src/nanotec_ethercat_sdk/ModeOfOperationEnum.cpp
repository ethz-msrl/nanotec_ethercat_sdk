// clang-format off
/*
** Copyright 2021 Robotic Systems Lab - ETH Zurich:
** Linghao Zhang, Jonas Junger, Lennart Nachtigall
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**
** 1. Redistributions of source code must retain the above copyright notice,
**    this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above copyright notice,
**    this list of conditions and the following disclaimer in the documentation
**    and/or other materials provided with the distribution.
**
** 3. Neither the name of the copyright holder nor the names of its contributors
**    may be used to endorse or promote products derived from this software without
**    specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
// clang-format on

#include "nanotec_ethercat_sdk/ModeOfOperationEnum.hpp"
#include <unordered_map>

std::ostream& operator<<(std::ostream& os,
                         const nanotec::ModeOfOperationEnum modeOfOperation) {
  std::unordered_map<nanotec::ModeOfOperationEnum, std::string> mode2strMap = {
    {nanotec::ModeOfOperationEnum::AutoSetup, "AutoSetup"},
    {nanotec::ModeOfOperationEnum::ClockDirectionMode, "ClockDirectionMode"},
    {nanotec::ModeOfOperationEnum::NA, "NA"},
    {nanotec::ModeOfOperationEnum::ProfilePositionMode, "ProfilePositionMode"},
    {nanotec::ModeOfOperationEnum::VelocityMode, "VelocityMode"},
    {nanotec::ModeOfOperationEnum::ProfileVelocityMode, "ProfileVelocityMode"},
    {nanotec::ModeOfOperationEnum::ProfileTorqueMode, "ProfileTorqueMode"},
    {nanotec::ModeOfOperationEnum::HomingMode, "HomingMode"},
    {nanotec::ModeOfOperationEnum::InterpolatedPositionMode, "InterpolatedPositionMode"},
    {nanotec::ModeOfOperationEnum::CyclicSynchronousPositionMode, "CyclicSynchronousPositionMode"},
    {nanotec::ModeOfOperationEnum::CyclicSynchronousVelocityMode, "CyclicSynchronousVelocityMode"},
    {nanotec::ModeOfOperationEnum::CyclicSynchronousTorqueMode, "CyclicSynchronousTorqueMode"},
  };
  os << mode2strMap[modeOfOperation];
  return os;
}