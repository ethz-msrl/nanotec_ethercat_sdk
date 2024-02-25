// clang-format off
/*
** Copyright (c) 2024, Multi-Scale Robotics Lab ETH Zurich
** Neelaksh Singh, Claas Ehmke
**
** Copyright (c) 2021, Robotic Systems Lab ETH Zurich
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

#pragma once

#include <cstdint>
#include <iostream>

namespace nanotec {
enum class ModeOfOperationEnum : int8_t {
  AutoSetup = -2,
  ClockDirectionMode = -1,
  NA = 0,
  ProfilePositionMode = 1,
  VelocityMode = 2,
  ProfileVelocityMode = 3,
  ProfileTorqueMode = 4,
  HomingMode = 6,
  InterpolatedPositionMode = 7,
  CyclicSynchronousPositionMode = 8,
  CyclicSynchronousVelocityMode = 9,
  CyclicSynchronousTorqueMode = 10
};

}  // namespace nanotec

std::ostream& operator<<(std::ostream& os, const nanotec::ModeOfOperationEnum modeOfOperation);
