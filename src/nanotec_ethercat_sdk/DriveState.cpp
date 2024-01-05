#include "nanotec_ethercat_sdk/DriveState.hpp"

std::ostream& operator<<(std::ostream& os,
                         const nanotec::DriveState& driveState) {
  switch (driveState) {
    case nanotec::DriveState::NotReadyToSwitchOn:
      os << "NotReadyToSwitchOn";
      break;
    case nanotec::DriveState::SwitchOnDisabled:
      os << "SwitchOnDisabled";
      break;
    case nanotec::DriveState::ReadyToSwitchOn:
      os << "ReadyToSwitchOn";
      break;
    case nanotec::DriveState::SwitchedOn:
      os << "SwitchedOn";
      break;
    case nanotec::DriveState::OperationEnabled:
      os << "OperationEnabled";
      break;
    case nanotec::DriveState::QuickStopActive:
      os << "QuickStopActive";
      break;
    case nanotec::DriveState::FaultReactionActive:
      os << "FaultReactionActive";
      break;
    case nanotec::DriveState::Fault:
      os << "Fault";
      break;
    case nanotec::DriveState::NA:
      os << "NA";
      break;
  }
  return os;
}
