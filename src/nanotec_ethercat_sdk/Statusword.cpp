#include "nanotec_ethercat_sdk/Statusword.hpp"
#include "message_logger/message_logger.hpp"

namespace nanotec {
std::ostream& operator<<(std::ostream& os, const Statusword& statusword) { //DONE
 using std::setfill;
  using std::setw;
  std::string driveStateString = statusword.getDriveStateString();
  int gapSize2 = driveStateString.size() + 1;
  if (gapSize2 < 6) {
    gapSize2 = 6;
  }
  os << std::left << std::boolalpha << setw(gapSize2 + 27) << setfill('-')
     << "|"
     << "|\n"
     << setw(gapSize2 + 27) << setfill(' ') << "| Statusword"
     << "|\n"
     << setw(gapSize2 + 27) << setfill('-') << "|"
     << "|\n"
     << setw(25) << setfill(' ') << "| Name of Bit" << setw(gapSize2 + 2)
     << "| Value"
     << "|\n"
     << setw(25) << setfill('-') << "|" << setw(gapSize2 + 2) << "+"
     << "|\n"
     << setfill(' ') <<

      setw(25) << "| Ready to switch on:"
     << "| " << setw(gapSize2) << statusword.readyToSwitchOn_ << "|\n"
     << setw(25) << "| Switched on:"
     << "| " << setw(gapSize2) << statusword.switchedOn_ << "|\n"
     << setw(25) << "| Operation enabled:"
     << "| " << setw(gapSize2) << statusword.operationEnabled_ << "|\n"
     << setw(25) << "| Fault:"
     << "| " << setw(gapSize2) << statusword.fault_ << "|\n"
     << setw(25) << "| Voltage enabled:"
     << "| " << setw(gapSize2) << statusword.voltageEnabled_ << "|\n"
     << setw(25) << "| Quick stop:"
     << "| " << setw(gapSize2) << statusword.quickStop_ << "|\n"
     << setw(25) << "| Switch on disabled:"
     << "| " << setw(gapSize2) << statusword.switchOnDisabled_ << "|\n"
     << setw(25) << "| Warning:"
     << "| " << setw(gapSize2) << statusword.warning_ << "|\n"
     << setw(25) << "| Target reached:"
     << "| " << setw(gapSize2) << statusword.targetReached_ << "|\n"
     << setw(25) << "| Operation Mode Specific  Bit 1:"
     << "| " << setw(gapSize2) << statusword.operationModeSpecific1_ << "|\n"
     << setw(25) << "| Operation Mode Specific  Bit 2:"
     << "| " << setw(gapSize2) << statusword.operationModeSpecific2_ << "|\n"
     << setw(25) << "| Closed Loop Active:"
     << "| " << setw(gapSize2) << statusword.closedLoopActive_ << "|\n"
     << setw(25) << setfill('-') << "|" << setw(gapSize2 + 2) << "+"
     << "|\n"
     << setfill(' ') << setw(25) << "| Resulting Drive State:"
     << "| " << setw(gapSize2) << driveStateString << "|\n"
     << setw(25) << setfill('-') << "|" << setw(gapSize2 + 2) << "+"
     << "|" <<

      std::noboolalpha << std::right << setfill(' ');
 
  return os;
}

void Statusword::setFromRawStatusword(uint16_t status) { //DONE
  readyToSwitchOn_ = static_cast<bool>(status & 1 << (0));
  switchedOn_ = static_cast<bool>(status & 1 << (1));
  operationEnabled_ = static_cast<bool>(status & 1 << (2));
  fault_ = static_cast<bool>(status & 1 << (3));
  voltageEnabled_ = static_cast<bool>(status & 1 << (4));
  quickStop_ = static_cast<bool>(status & 1 << (5));
  switchOnDisabled_ = static_cast<bool>(status & 1 << (6));
  warning_ = static_cast<bool>(status & 1 << (7));
  remote_ = static_cast<bool>(status & 1 << (9));
  targetReached_ = static_cast<bool>(status & 1 << (10));
  internalLimitActive_ = static_cast<bool>(status & 1 << (11));
  operationModeSpecific1_ = static_cast<bool>(status & 1 << (12));
  operationModeSpecific2_ = static_cast<bool>(status & 1 << (13));
  closedLoopActive_ = static_cast<bool>(status & 1 << (15));

  rawStatusword_ = status;
}

uint16_t Statusword::getRawStatusWord() {
  return rawStatusword_;
}

DriveState Statusword::getDriveState() const {
  DriveState driveState = DriveState::NA;

  if ((rawStatusword_ & 0b0000000001001111) == 0b0000000000000000) {
    driveState = DriveState::NotReadyToSwitchOn;
    //LOG("Current Drivestate: NotReadyToSwitchOn");

  } else if ((rawStatusword_ & 0b0000000001001111) == 0b0000000001000000) {
    driveState = DriveState::SwitchOnDisabled;
    //LOG("Current Drivestate: SwitchOnDisabled");

  } else if ((rawStatusword_ & 0b0000000001101111) == 0b0000000000100001) {
    driveState = DriveState::ReadyToSwitchOn;
    //LOG("Current Drivestate: ReadyToSwitchOn");

  } else if ((rawStatusword_ & 0b0000000001101111) == 0b0000000000100011) {
    driveState = DriveState::SwitchedOn;
    //LOG("Current Drivestate: SwitchedOn");

  } else if ((rawStatusword_ & 0b0000000001101111) == 0b0000000000100111) {
    driveState = DriveState::OperationEnabled;
    //LOG("Current Drivestate: OperationEnabled");

  } else if ((rawStatusword_ & 0b0000000001101111) == 0b0000000000000111) {
    driveState = DriveState::QuickStopActive;
    //LOG("Current Drivestate: QuickStopActive");

  } else if ((rawStatusword_ & 0b0000000001101111) == 0b0000000000001111) {
    driveState = DriveState::FaultReactionActive;
    //LOG("Current Drivestate: FaultReactionActive");

  } else if ((rawStatusword_ & 0b0000000001001111) == 0b0000000000001000) {
    driveState = DriveState::Fault;
    //LOG("Current Drivestate: Fault");
  }else {
    MELO_ERROR("Current Drivestate: NA");
  }

  return driveState;
}


std::string Statusword::getDriveStateString() const {
  DriveState driveState = getDriveState();
  switch (driveState) {
    case DriveState::SwitchOnDisabled:
      return "switch on disabled";
      break;
    case DriveState::ReadyToSwitchOn:
      return "ready to switch on";
      break;
    case DriveState::SwitchedOn:
      return "switched on";
      break;
    case DriveState::OperationEnabled:
      return "operation enabled";
      break;
    case DriveState::QuickStopActive:
      return "quick stop active";
      break;
    case DriveState::Fault:
      return "fault_";
      break;
    case DriveState::FaultReactionActive:
      return "fault_ reaction active";
    case DriveState::NotReadyToSwitchOn:
      return "not ready to switch on";
    default:
      return "N/A";
  }
}


}  // namespace nanotec
