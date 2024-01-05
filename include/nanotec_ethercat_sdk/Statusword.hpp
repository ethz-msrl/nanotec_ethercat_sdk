#pragma once

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>

#include "nanotec_ethercat_sdk/DriveState.hpp"

namespace nanotec {
class Statusword {
 private:
  bool readyToSwitchOn_{false};             // bit 0
  bool switchedOn_{false};                  // bit 1
  bool operationEnabled_{false};            // bit 2
  bool fault_{false};                       // bit 3
  bool voltageEnabled_{false};              // bit 4
  bool quickStop_{false};                   // bit 5
  bool switchOnDisabled_{false};            // bit 6
  bool warning_{false};                     // bit 7
  bool synchronization_{false};            // bit 8
  bool remote_{false};                      // bit 9
  bool targetReached_{false};               // bit 10
  bool internalLimitActive_{false};         // bit 11
  bool operationModeSpecific1_{false};       // bit 12   
  bool operationModeSpecific2_ {false};    // bit 13  
  bool closedLoopActive_{false};            // bit 15

  // the raw statusword
  uint16_t rawStatusword_{0};

 public:
  friend std::ostream& operator<<(std::ostream& os,
                                  const Statusword& statusword);
  void setFromRawStatusword(uint16_t status);
  uint16_t getRawStatusWord();
  DriveState getDriveState() const;
  std::string getDriveStateString() const;

  bool getReadyToSwitchOnValue(){ return readyToSwitchOn_;};
  bool getSwitchedOnValue(){ return switchedOn_;};
  bool getOperationEnabledValue(){ return operationEnabled_;};
  bool getFaultValue(){ return fault_;};
  bool getVoltageEnabledValue(){ return voltageEnabled_;};
  bool getQuickStopValue(){ return quickStop_;};
  bool getSwitchOnDisabledValue(){ return switchOnDisabled_;};
  bool getWarningValue(){ return warning_;};
  bool getSynchronizationValue(){ return synchronization_;};
  bool getRemoteValue(){ return remote_;};
  bool getTargetReachedValue(){ return targetReached_;};
  bool getInternalLimitActiveValue(){ return internalLimitActive_;};
  bool getOperationModeSpecific1Value(){ return operationModeSpecific1_;};
  bool getOperationModeSpecific2Value(){ return operationModeSpecific2_;};
  bool getClosedLoopActiveValue(){ return closedLoopActive_;};
  
};

}  // namespace nanotec
