#pragma once

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <ethercat_sdk_master/EthercatDevice.hpp>
#include <mutex>
#include <string>

#include "nanotec_ethercat_sdk/Command.hpp"
#include "nanotec_ethercat_sdk/Controlword.hpp"
#include "nanotec_ethercat_sdk/DriveState.hpp"
#include "nanotec_ethercat_sdk/Reading.hpp"
#include "ethercat_motor_msgs/ethercat_motor_msgs.h"

namespace nanotec {
class Nanotec : public ecat_master::EthercatDevice {
 public:
  typedef std::shared_ptr<Nanotec> SharedPtr; // Shared Ptr (a smart pointer) to the Nanotec Object

  // create Nanotec Drive from setup file
  static SharedPtr deviceFromFile(const std::string& configFile,
                                  const std::string& name,
                                  const uint32_t address);
                                  
  // constructor
  Nanotec() = default;
  Nanotec(const std::string& name, const uint32_t address);

  // pure virtual overwrites
 public:
  bool startup() override;
  void preShutdown() override;
  void shutdown() override;
  void updateWrite() override;
  void updateRead() override;
  /*bool putIntoOperation() {
    bool success;
    bus_->setState(EC_STATE_OPERATIONAL, getAddress());
    success =
        bus_->waitForState(EC_STATE_OPERATIONAL, getAddress(), 1000, 0.001);
    return success;
  }*/
  PdoInfo getCurrentPdoInfo() const override { return pdoInfo_; }

 public:
  void stageCommand(const Command& command);
  Reading getReading() const;
  void getReading(Reading& reading) const;

  bool loadConfigFile(const std::string& fileName);
  bool loadConfigNode(YAML::Node configNode);
  bool loadConfiguration(const Configuration& configuration);
  Configuration getConfiguration() const;

  bool disableNanoJ(); 

  // SDO
 public:
  bool getStatuswordViaSdo(Statusword& statusword);
  bool setControlwordViaSdo(Controlword& controlword);
  bool setDriveStateViaSdo(const DriveState& driveState);

  bool setStartCommand(bool setOMS0, bool setOMS1, bool setOMS2, bool setOMS3, bool setHALT);
  
 protected:
  bool stateTransitionViaSdo(const StateTransition& stateTransition);

  // PDO
 public:
  bool setDriveStateViaPdo(const DriveState& driveState,
                           const bool waitForState);
  bool lastPdoStateChangeSuccessful() const { return stateChangeSuccessful_; }

 protected:
  void engagePdoStateMachine();
  bool mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum);
  bool configParam();
  Controlword getNextStateTransitionControlword(
      const DriveState& requestedDriveState,
      const DriveState& currentDriveState);
  void autoConfigurePdoSizes();

  uint16_t getTxPdoSize();
  uint16_t getRxPdoSize();

  bool isAllowedModeCombination(const std::vector<ModeOfOperationEnum> modes);
  std::pair<RxPdoTypeEnum, TxPdoTypeEnum> getMixedPdoType(
      std::vector<ModeOfOperationEnum> modes);
      
  // Control
  bool autoSetup();
  bool homing();
  // bool actuateProfilePositionMode(ProfilePositionModeSettings profilePositionModeSettings);

  // Errors
 protected:
  void addErrorToReading(const ErrorType& errorType);

 public:
  void printErrorCode();

 public:
  Configuration configuration_;

 protected:
  Command stagedCommand_;
  Reading reading_;
  RxPdoTypeEnum rxPdoTypeEnum_{RxPdoTypeEnum::NA};
  TxPdoTypeEnum txPdoTypeEnum_{TxPdoTypeEnum::NA};
  Controlword controlword_;
  PdoInfo pdoInfo_;
  bool hasRead_{false};
  bool conductStateChange_{false};
  bool edgeTransitionLowSet_{false};
  DriveState targetDriveState_{DriveState::NA};
  std::chrono::time_point<std::chrono::steady_clock> driveStateChangeTimePoint_;
  uint16_t numberOfSuccessfulTargetStateReadings_{0};
  std::atomic<bool> stateChangeSuccessful_{false};

  // Configurable parameters
 protected:
  bool allowModeChange_{false};
  ModeOfOperationEnum modeOfOperation_{ModeOfOperationEnum::NA};
  bool changeModeOfOpertion(ModeOfOperationEnum setModeOfOperation, bool *alreadySatisfied);

 protected:
  mutable std::recursive_mutex stagedCommandMutex_;  // TODO required?
  mutable std::recursive_mutex readingMutex_;        // TODO required?
  mutable std::recursive_mutex mutex_;               // TODO: change name!!!!
};
}  // namespace nanotec
