#pragma once
#include <ostream>

namespace nanotec {
enum class DriveState : uint8_t {
  NotReadyToSwitchOn,
  SwitchOnDisabled,
  ReadyToSwitchOn,
  SwitchedOn,
  OperationEnabled,
  QuickStopActive,
  FaultReactionActive,
  Fault,
  NA
};

enum class StateTransition : uint8_t {
  _2,
  _3,
  _4,
  _5,
  _6,
  _7,
  _8,
  _9,
  _10,
  _11,
  _12,
  _15L,
  _15H,
  _16L,
  _16H
};

}  // namespace nanotec

std::ostream& operator<<(std::ostream& os, const nanotec::DriveState& driveState);
