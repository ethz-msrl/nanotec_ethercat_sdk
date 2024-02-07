#include "nanotec_ethercat_sdk/ConfigurationParser.hpp"
#include "nanotec_ethercat_sdk/ObjectDictionary.hpp"
#include "nanotec_ethercat_sdk/RxPdo.hpp"
#include "nanotec_ethercat_sdk/TxPdo.hpp"
#include "nanotec_ethercat_sdk/Nanotec.hpp"

#include <chrono>
#include <cmath>
#include <map>
#include <thread>
#include <algorithm>

namespace nanotec {
std::string binstring(uint16_t var) {
  std::string s = "0000000000000000";
  for (int i = 0; i < 16; i++) {
    if (var & (1 << (15 - i))) {
      s[i] = '1';
    }
  }
  return s;
}
std::string binstring(int8_t var) {
  std::string s = "00000000";
  for (int i = 0; i < 8; i++) {
    if (var & (1 << (7 - i))) {
      s[i] = '1';
    }
  }
  return s;
}

Nanotec::SharedPtr Nanotec::deviceFromFile(const std::string& configFile,
                                       const std::string& name,
                                       const uint32_t address) {
  auto nanotec = std::make_shared<Nanotec>(name, address);
  nanotec->loadConfigFile(configFile);
  return nanotec;
}

Nanotec::Nanotec(const std::string& name, const uint32_t address) {
  address_ = address;
  name_ = name;
}

bool Nanotec::startup() {
  bool success = true;
  success &= bus_->waitForState(EC_STATE_PRE_OP, address_, 50, 0.05);
  // success &= bus_->setState(EC_STATE_SAFE_OP, address_);
  bus_->syncDistributedClock0(address_, true, timeStep_, timeStep_ / 2.f); //

  // Might not needed
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  success &= disableNanoJ(); // Error is thrown within the function on failure.
  // Claas' code worked fine without this, so removing this line of code.
  // Set PDO Mapping in Rx and Tx PDO map registers
  success &= mapPdos(rxPdoTypeEnum_, txPdoTypeEnum_);

  // Assigning no mode of operation
  success &= sdoVerifyWrite(OD_INDEX_MODES_OF_OPERATION, 0x00, false,
                            static_cast<int8_t>(ModeOfOperationEnum::NA),
                            configuration_.configRunSdoVerifyTimeout);

  // To be on the safe side: set currect PDO sizes
  autoConfigurePdoSizes();
  // write the configuration parameters via Sdo
  success &= configParam();
  if(configuration_.autoSetupEn)
  {
    success &= autoSetup();
    if(!success){
      MELO_ERROR("[nanotec_ethercat_sdk:Nanotec::startup] AutoSetup failed! Aborting startup sequence.");
      addErrorToReading(ErrorType::ConfigurationError);
      return success;
    }
  }

  // Before disabling the brake, for the sake of safety, we set the position 
  // velocity, and torques to current values; and offsets to 0. Not doing this
  // may for e.g.: lead to quick rush to 0 position when master code is started
  // since all the PDO structs will contain a default of 0.
  bool readSuccess = true;
  int32_t current_position;
  readSuccess &= sendSdoRead(OD_INDEX_POSITION_ACTUAL, 0, false, current_position);
  stagedCommand_.setTargetPositionRaw(current_position);
  reading_.setActualPosition(current_position);
  stagedCommand_.setPositionOffsetRaw(0);
  /* Velocity and Torque zeroed at startup, uncomment to hold at startup */
  // int32_t current_velocity;
  // readSuccess &= sendSdoRead(OD_INDEX_VELOCITY_ACTUAL, 0, false, current_velocity);
  // stagedCommand_.setTargetVelocityRaw(0);
  // reading_.setActualVelocity(current_velocity);
  // stagedCommand_.setVelocityOffsetRaw(0);
  // int16_t current_torque;
  // readSuccess &= sendSdoRead(OD_INDEX_TORQUE_ACTUAL, 0, false, current_torque);
  // stagedCommand_.setTargetTorqueRaw(current_torque);
  // reading_.setActualTorque(current_torque);
  // stagedCommand_.setTorqueOffsetRaw(0);
  if(!readSuccess){
    MELO_ERROR("[nanotec_ethercat_sdk:Nanotec::startup] Reading current position, velocity, and torque failed! Aborting startup sequence.");
    addErrorToReading(ErrorType::ConfigurationError);
    return false;
  }

  //disable brake
  success &= sdoVerifyWrite(OD_INDEX_DIGITAL_OUTPUTS, 0x01, false,
                  0x00000000,
                  configuration_.configRunSdoVerifyTimeout);

  if(configuration_.homingEn)
  {
    success &= homing();
  }

  if (!success) {
    MELO_ERROR_STREAM(
        "[nanotec_ethercat_sdk:Nanotec::preStartupOnlineConfiguration] "
        "hardware configuration of '"
        << name_ << "' not successful!");
    addErrorToReading(ErrorType::ConfigurationError);
    return success;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  return success;
}

void Nanotec::preShutdown() {
  setDriveStateViaSdo(DriveState::QuickStopActive);
  setDriveStateViaSdo(DriveState::SwitchOnDisabled);
}

void Nanotec::shutdown() { bus_->setState(EC_STATE_INIT, address_); }

void Nanotec::updateWrite() {

  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // Check if the Mode of Operation has been set properly
  
  if (modeOfOperation_ == ModeOfOperationEnum::NA) {
    reading_.addError(ErrorType::ModeOfOperationError);
    MELO_ERROR_STREAM(
        "[nanotec_ethercat_sdk:Nanotec::updateWrite]"
        " Mode of operation for '"
        << name_ << "' has not been set.");
    return;
  }

  // engage the state machine if a state change is requested
   
  if (conductStateChange_ && hasRead_) {
    engagePdoStateMachine();
  }

  switch (rxPdoTypeEnum_) {
    case RxPdoTypeEnum::RxPdoStandard: {
      RxPdoStandard rxPdo{};
      rxPdo.modeOfOperation_ = static_cast<int8_t>(modeOfOperation_);
      rxPdo.controlWord_ = controlword_.getRawControlword();

      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
      break;
    }
    case RxPdoTypeEnum::RxPdoCSP: {
      RxPdoCSP rxPdo{};
      {
        std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
        rxPdo.targetPosition_ = stagedCommand_.getTargetPositionRaw();
        rxPdo.positionOffset_ = stagedCommand_.getPositionOffsetRaw();
        // @todo: Add Velocity Offset
        rxPdo.torqueOffset_ = stagedCommand_.getTorqueOffsetRaw();

        // Extra data
        rxPdo.controlWord_ = controlword_.getRawControlword();
        rxPdo.modeOfOperation_ = static_cast<int8_t>(modeOfOperation_);
      }

      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
      break;
    }
    case RxPdoTypeEnum::RxPdoCST: {
      RxPdoCST rxPdo{};
      {
        std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
        rxPdo.targetTorque_ = stagedCommand_.getTargetTorqueRaw();
        rxPdo.torqueOffset_ = stagedCommand_.getTorqueOffsetRaw();

        // Extra data
        rxPdo.controlWord_ = controlword_.getRawControlword();
        rxPdo.modeOfOperation_ = static_cast<int8_t>(modeOfOperation_);
      }

      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
      break;
    }
    case RxPdoTypeEnum::RxPdoCSV: {
      RxPdoCSV rxPdo{};
      {
        std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
        rxPdo.targetVelocity_ = stagedCommand_.getTargetVelocityRaw();
        rxPdo.velocityOffset_ = stagedCommand_.getVelocityOffsetRaw();

        // Extra data
        rxPdo.controlWord_ = controlword_.getRawControlword();
        rxPdo.modeOfOperation_ = static_cast<int8_t>(modeOfOperation_);
      }

      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
      break;
    }
    case RxPdoTypeEnum::RxPdoCSTCSP: {
      RxPdoCSTCSP rxPdo{};
      {
        std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
        rxPdo.targetPosition_ = stagedCommand_.getTargetPositionRaw();
        rxPdo.positionOffset_ = stagedCommand_.getPositionOffsetRaw();
        rxPdo.targetTorque_ = stagedCommand_.getTargetTorqueRaw();
        rxPdo.torqueOffset_ = stagedCommand_.getTorqueOffsetRaw();

        // Extra data
        rxPdo.controlWord_ = controlword_.getRawControlword();
        rxPdo.modeOfOperation_ = static_cast<int8_t>(modeOfOperation_);
      }

      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
      break;
    }
    case RxPdoTypeEnum::RxPdoCSTCSPCSV: {
      RxPdoCSTCSPCSV rxPdo{};
      {
        std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
        rxPdo.targetPosition_ = stagedCommand_.getTargetPositionRaw();
        rxPdo.positionOffset_ = stagedCommand_.getPositionOffsetRaw();
        rxPdo.targetTorque_ = stagedCommand_.getTargetTorqueRaw();
        rxPdo.torqueOffset_ = stagedCommand_.getTorqueOffsetRaw();
        rxPdo.targetVelocity_ = stagedCommand_.getTargetVelocityRaw();
        rxPdo.velocityOffset_ = stagedCommand_.getVelocityOffsetRaw();

        // Extra data
        rxPdo.controlWord_ = controlword_.getRawControlword();
        rxPdo.modeOfOperation_ = static_cast<int8_t>(modeOfOperation_);
      }

      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
      break;
    }
    case RxPdoTypeEnum::RxPdoPVM: {
      RxPdoPVM rxPdo{};
      {
        std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
        rxPdo.controlWord_ = controlword_.getRawControlword();
        rxPdo.targetVelocity_ = stagedCommand_.getTargetVelocityRaw();
        rxPdo.profileAccel_ = stagedCommand_.getProfileAccelRaw();
        rxPdo.profileDeccel_ = stagedCommand_.getProfileDeccelRaw();
        rxPdo.motionProfileType_ = stagedCommand_.getMotionProfileType();
      }

      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
      break;
    }
    default:
      MELO_ERROR_STREAM(
          "[nanotec_ethercat_sdk:Nanotec::updateWrite] "
          " Unsupported Rx Pdo type for '"
          << name_ << "'");
      addErrorToReading(ErrorType::RxPdoTypeError);
  }
}

bool Nanotec::changeModeOfOpertion(ModeOfOperationEnum setModeOfOperation, bool *alreadySatisfied) { 
  bool success = true;
  if (modeOfOperation_ != setModeOfOperation)
  {
    MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::changeModeOfOperation] changing to " << setModeOfOperation);
    success &= sdoVerifyWrite(OD_INDEX_MODES_OF_OPERATION, 0x00, false,
                  static_cast<int8_t>(setModeOfOperation),
                  configuration_.configRunSdoVerifyTimeout);
    *alreadySatisfied = false;
    modeOfOperation_ = setModeOfOperation;
    if(success){
      MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::changeModeOfOperation] change to " << setModeOfOperation << " successful!");
    }
    else{
      MELO_ERROR_STREAM("[nanotec_ethercat_sdk:Nanotec::changeModeOfOperation] change to " << setModeOfOperation << " failed!");
    }
  }
  else
  {
    *alreadySatisfied = true;
  }
  return success;
}

bool Nanotec::autoSetup() {
  Statusword currentStatusword_;
  bool success = true;
  Controlword controlword_;
  MELO_INFO("[nanotec_ethercat_sdk:Nanotec::AutoSetup] Perform Auto Setup Begin");
  // Reset Control Word
  controlword_.setAllFalse();
  success &= setControlwordViaSdo(controlword_);
  // Set to Drive State "ReadyToSwitchOn"
  success &= setDriveStateViaSdo(DriveState::ReadyToSwitchOn);
  // Set Mode of Operation to "AutoSetup"
  bool alreadySatisfied;
  success &= changeModeOfOpertion(ModeOfOperationEnum::AutoSetup, &alreadySatisfied);
  // Set to Drive State "OperationEnabled"
  success &= setDriveStateViaSdo(DriveState::OperationEnabled);
  getStatuswordViaSdo(currentStatusword_);
  // Run auto setup
  success &= setStartCommand(true, false, false, false, false);
  // Wait until auto setup done
  auto start_time = std::chrono::high_resolution_clock::now();
  uint32_t rawStatusWord;
  while (1) { 
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    getStatuswordViaSdo(currentStatusword_);
    rawStatusWord = currentStatusword_.getRawStatusWord();
    if ((rawStatusWord & 0b0001001000110111) == 0b0001001000110111) {
      if((rawStatusWord & 0b0000010000000000) == 0b0000010000000000){
        MELO_INFO("[nanotec_ethercat_sdk:Nanotec::AutoSetup] Encoder index was found.")
      }
      else{
        MELO_INFO("[nanotec_ethercat_sdk:Nanotec::AutoSetup] Encoder index was not found.")
      }
      break;
    }

    auto cur_time = std::chrono::high_resolution_clock::now();
    auto diff_time = cur_time - start_time;
    if (diff_time/std::chrono::milliseconds(1)>configuration_.autoSetupTimeoutms)
    {
      MELO_ERROR("[nanotec_ethercat_sdk:Nanotec::AutoSetup] AutoSetup timeout. RawStatusWord: 0x%04X", rawStatusWord);
      return false;
    }
  }
  // Reset Control Word
  controlword_.setAllFalse();
  success &= setControlwordViaSdo(controlword_);

  MELO_INFO("nanotec_ethercat_sdk:Nanotec::AutoSetup] Perform Auto Setup... DONE!");
  MELO_INFO("nanotec_ethercat_sdk:Nanotec::AutoSetup] Save Auto Setup Parameter...");
  int32_t saveCommand = 0x65766173;
  uint8_t savingCategory = 0x01;
  bool tmpSuccess = sendSdoWrite(OD_INDEX_STORE_PARAMETER, savingCategory, false, saveCommand);
  MELO_DEBUG("nanotec_ethercat_sdk:Nanotec::AutoSetup] Save Command Issued Status: %u", tmpSuccess);
  success &= tmpSuccess;
  uint32_t savingVerification = 0;
  auto save_start_time = std::chrono::high_resolution_clock::now();
  while (1) { 
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    sendSdoRead(OD_INDEX_STORE_PARAMETER, savingCategory, false, savingVerification);

    if (savingVerification == 0x01) {
      break;
    }

    auto cur_time = std::chrono::high_resolution_clock::now();
    auto diff_time = cur_time - save_start_time;
    if (diff_time/std::chrono::milliseconds(1)>configuration_.autoSetupTimeoutms)
    {
      MELO_ERROR("nanotec_ethercat_sdk:Nanotec::AutoSetup] AutoSetup Save timed out. RawStatusWord: 0x%04X,\
       Saving Verification: 0x%02X", rawStatusWord, savingVerification);
      return false;
    }

  }
  MELO_INFO("[nanotec_ethercat_sdk:Nanotec::AutoSetup] Save Auto Setup Parameter... DONE!");

  return success;
}

bool Nanotec::homing(){
  Statusword currentStatusword_;
  bool success = true;
  Controlword controlword_;
  MELO_INFO("[nanotec_ethercat_sdk:Nanotec::Homing] Performing Homing...");
  // Reset Control Word
  controlword_.setAllFalse();
  success &= setControlwordViaSdo(controlword_);
  // Set to Drive State "ReadyToSwitchOn"
  success &= setDriveStateViaSdo(DriveState::ReadyToSwitchOn);
  // Set Mode of Operation to "Homing"
  bool alreadySatisfied;
  success &= changeModeOfOpertion(ModeOfOperationEnum::HomingMode, &alreadySatisfied);
  // Set to Drive State to "OperationEnabled"
  success &= setDriveStateViaSdo(DriveState::OperationEnabled);
  // Run auto setup
  success &= setStartCommand(true, false, false, false, false);
  // Wait until auto setup done
  auto start_time = std::chrono::high_resolution_clock::now();
  uint32_t rawStatusWord;
  while (1) { 
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    getStatuswordViaSdo(currentStatusword_);
    rawStatusWord = currentStatusword_.getRawStatusWord();
    if ((rawStatusWord & 0b0001001000110111) == 0b0001001000110111) {
      break;
    }
    
    auto cur_time = std::chrono::high_resolution_clock::now();
    auto diff_time = cur_time - start_time;
    if (diff_time/std::chrono::milliseconds(1)>configuration_.homingTimeoutms)
    {
      MELO_ERROR("[nanotec_ethercat_sdk:Nanotec::Homing] Homing timeout. RawStatusWord: 0x%04X", rawStatusWord);
      return false;
    }
  }

  // Reset Control Word
  controlword_.setAllFalse();
  success &= setControlwordViaSdo(controlword_);

  MELO_INFO("[nanotec_ethercat_sdk:Nanotec::Homing] Homing DONE!");
  return success;
}

void Nanotec::updateRead() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // TODO(duboisf): implement some sort of time stamp
  switch (txPdoTypeEnum_) {
    case TxPdoTypeEnum::TxPdoStandard: {
      TxPdoStandard txPdo{};
      // reading from the bus
      bus_->readTxPdo(address_, txPdo);
      reading_.setStatusword(txPdo.statusword_);
      break;
    }
    case TxPdoTypeEnum::TxPdoCSP: {
      TxPdoCSP txPdo{};
      // reading from the bus
      bus_->readTxPdo(address_, txPdo);
      { 
        std::lock_guard<std::recursive_mutex> lock(readingMutex_);
        reading_.setStatusword(txPdo.statusword_);
        reading_.setActualFollowingError(txPdo.actualFollowingError_);
        reading_.setActualTorque(txPdo.actualTorque_);
        reading_.setActualVelocity(txPdo.actualVelocity_);
        reading_.setActualPosition(txPdo.actualPosition_);
        reading_.setErrorCode(txPdo.errorCode_);
        reading_.setErrorRegister(txPdo.errorRegister_);
      }
      break;
    }
    case TxPdoTypeEnum::TxPdoCST: {
      TxPdoCST txPdo{};
      // reading from the bus
      bus_->readTxPdo(address_, txPdo);
      {
        std::lock_guard<std::recursive_mutex> lock(readingMutex_);
        reading_.setStatusword(txPdo.statusword_);
        reading_.setActualTorque(txPdo.actualTorque_);
        reading_.setDemandTorque(txPdo.demandTorque_);
        reading_.setActualVelocity(txPdo.actualVelocity_);
        reading_.setActualPosition(txPdo.actualPosition_);
        reading_.setErrorCode(txPdo.errorCode_);
        reading_.setErrorRegister(txPdo.errorRegister_);
      }
      break;
    }
    case TxPdoTypeEnum::TxPdoCSV: {
      TxPdoCSV txPdo{};
      // reading from the bus
      bus_->readTxPdo(address_, txPdo);
      {
        std::lock_guard<std::recursive_mutex> lock(readingMutex_);
        reading_.setStatusword(txPdo.statusword_);
        reading_.setActualTorque(txPdo.actualTorque_);
        reading_.setActualVelocity(txPdo.actualVelocity_);
        reading_.setActualPosition(txPdo.actualPosition_);
        reading_.setErrorCode(txPdo.errorCode_);
        reading_.setErrorRegister(txPdo.errorRegister_);
      }
      break;
    }
    case TxPdoTypeEnum::TxPdoCSTCSP: {
      TxPdoCSTCSP txPdo{};
      // reading from the bus
      bus_->readTxPdo(address_, txPdo);
      {
        std::lock_guard<std::recursive_mutex> lock(readingMutex_);
        reading_.setStatusword(txPdo.statusword_);
        reading_.setActualFollowingError(txPdo.actualFollowingError_);
        reading_.setDemandTorque(txPdo.demandTorque_);
        reading_.setActualTorque(txPdo.actualTorque_);
        reading_.setActualVelocity(txPdo.actualVelocity_);
        reading_.setActualPosition(txPdo.actualPosition_);
        reading_.setErrorCode(txPdo.errorCode_);
        reading_.setErrorRegister(txPdo.errorRegister_);
      }
      break;
    }
    case TxPdoTypeEnum::TxPdoCSTCSPCSV: {
      TxPdoCSTCSPCSV txPdo{};
      // reading from the bus
      bus_->readTxPdo(address_, txPdo);
      {
        std::lock_guard<std::recursive_mutex> lock(readingMutex_);
        reading_.setStatusword(txPdo.statusword_);
        reading_.setActualFollowingError(txPdo.actualFollowingError_);
        reading_.setDemandTorque(txPdo.demandTorque_);
        reading_.setActualTorque(txPdo.actualTorque_);
        reading_.setActualVelocity(txPdo.actualVelocity_);
        reading_.setActualPosition(txPdo.actualPosition_);
        reading_.setErrorCode(txPdo.errorCode_);
        reading_.setErrorRegister(txPdo.errorRegister_);
      }
      break;
    }
    case TxPdoTypeEnum::TxPdoPVM: {
      TxPdoPVM txPdo{};
      // reading from the bus
      bus_->readTxPdo(address_, txPdo);
      {
        std::lock_guard<std::recursive_mutex> lock(readingMutex_);
        reading_.setDemandVelocity(txPdo.demandVelocity_);
        reading_.setStatusword(txPdo.statusword_);
        reading_.setErrorCode(txPdo.errorCode_);
        reading_.setErrorRegister(txPdo.errorRegister_);
      }
      break;
    }
    default:
      MELO_ERROR_STREAM(
          "[nanotec_ethercat_sdk:Nanotec::updateRead] Unsupported Tx Pdo "
          "type for '"
          << name_ << "'");
      reading_.addError(ErrorType::TxPdoTypeError);
  }

  // set the hasRead_ variable to true since a nes reading was read
  if (!hasRead_) {
    hasRead_ = true;
  }

  // Print warning if drive is in FaultReactionAcrive state.
  if (reading_.getDriveState() == DriveState::FaultReactionActive) {
    MELO_ERROR_STREAM("[nanotec_ethercat_sdk:Nanotec::updateRead] '"
                      << name_ << "' is in drive state 'FaultReactionAcrive'");
  }

  // Print warning if drive is in Fault state.
  if (reading_.getDriveState() == DriveState::Fault) {
    MELO_ERROR_STREAM("[nanotec_ethercat_sdk:Nanotec::updateRead] '"
                      << name_ << "' is in drive state 'Fault'");
    MELO_ERROR_STREAM("[nanotec_ethercat_sdk:Nanotec::updateRead] '"
                      << name_ << "' Error Code: " << std::hex << reading_.getErrorCode());
    MELO_ERROR_STREAM("[nanotec_ethercat_sdk:Nanotec::updateRead] '"
                      << name_ << "' Error Register: " << std::hex << reading_.getErrorRegister());

  }
}

void Nanotec::stageCommand(const Command& command) {
  std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
  stagedCommand_ = command;
  // stagedCommand_.setPositionFactorRadToInteger(
  //     static_cast<double>(configuration_.positionEncoderResolution) /
  //     (2.0 * M_PI));

  // double currentFactorAToInt = 1000.0 / configuration_.ratedCurrentmA;
  // stagedCommand_.setCurrentFactorAToInteger(currentFactorAToInt);
  // stagedCommand_.setTorqueFactorNmToInteger(
  //     1000.0 /
  //     (configuration_.ratedCurrentmA * configuration_.torqueConstantNmA));
  // stagedCommand_.doUnitConversion();

  const auto targetMode = command.getModeOfOperation();
  if (std::find(configuration_.modesOfOperation.begin(),
                configuration_.modesOfOperation.end(),
                targetMode) != configuration_.modesOfOperation.end()) {
    modeOfOperation_ = targetMode;
  } else {
    MELO_ERROR_STREAM(
        "[nanotec_ethercat_sdk:Nanotec::stageCommand] "
        "Target mode of operation '"
        << targetMode << "' for device '" << name_ << "' not allowed");
  }
}


Reading Nanotec::getReading() const {
  std::lock_guard<std::recursive_mutex> lock(readingMutex_);
  return reading_;
}

void Nanotec::getReading(Reading& reading) const {
  std::lock_guard<std::recursive_mutex> lock(readingMutex_);
  reading = reading_;
}

bool Nanotec::loadConfigFile(const std::string& fileName) {
  ConfigurationParser configurationParser(fileName);
  return loadConfiguration(configurationParser.getConfiguration());
}

bool Nanotec::loadConfigNode(YAML::Node configNode) {
  ConfigurationParser configurationParser(configNode);
  return loadConfiguration(configurationParser.getConfiguration());
}

bool Nanotec::loadConfiguration(const Configuration& configuration) {
  reading_.configureReading(configuration);
  modeOfOperation_ = configuration.modesOfOperation[0];
  const auto pdoTypeSolution = configuration.getPdoTypeSolution();
  rxPdoTypeEnum_ = pdoTypeSolution.first;
  txPdoTypeEnum_ = pdoTypeSolution.second;
  configuration_ = configuration;

  MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::loadConfiguration] Sanity check for '" << name_
                                                                  << "':");
  return configuration.sanityCheck();
}

Configuration Nanotec::getConfiguration() const { return configuration_; }

bool Nanotec::getStatuswordViaSdo(Statusword& statusword) {
  uint16_t statuswordValue = 0;
  bool success = sendSdoRead(OD_INDEX_STATUSWORD, 0, false, statuswordValue);
  statusword.setFromRawStatusword(statuswordValue);
  return success;
}

bool Nanotec::setControlwordViaSdo(Controlword& controlword) {
  bool success = sendSdoWrite(OD_INDEX_CONTROLWORD, 0, false,
                      controlword.getRawControlword());
  
  if(!success){
    MELO_ERROR("nanotec_ethercat_sdk:Nanotec::setControlwordViaSdo] Setting Controlword 0x%04X failed!", controlword.getRawControlword());
  }
  return success;
}


bool Nanotec::setDriveStateViaSdo(const DriveState& driveState) {
  bool success = true;
  Statusword currentStatusword;
  success &= getStatuswordViaSdo(currentStatusword);
  DriveState currentDriveState = currentStatusword.getDriveState();

  // do the adequate state changes (via sdo) depending on the requested and
  // current drive states
  switch (driveState) {
    // Target: switch on disabled
    // This is the lowest state in which the state machine can be brought over
    // EtherCAT
    case DriveState::SwitchOnDisabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= true;
          break;
        case DriveState::ReadyToSwitchOn:
          success &= stateTransitionViaSdo(StateTransition::_7);
          break;
        case DriveState::SwitchedOn:
          success &= stateTransitionViaSdo(StateTransition::_10);
          break;
        case DriveState::OperationEnabled:
          success &= stateTransitionViaSdo(StateTransition::_9);
          break;
        case DriveState::QuickStopActive:
          success &= stateTransitionViaSdo(StateTransition::_12);
          break;
        case DriveState::Fault:
          // Fault reset by high edge on bit 7
          success &= stateTransitionViaSdo(StateTransition::_15L);
          // @todo: remove manual delay if not needed
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          success &= stateTransitionViaSdo(StateTransition::_15H);
          break;
        default:
          MELO_ERROR_STREAM(
              "[nanotec_ethercat_sdk:Nanotec::setDriveStateViaSdo] State "
              "Transition not implemented");
          addErrorToReading(ErrorType::SdoStateTransitionError);
          success = false;
      }
      break;

    case DriveState::ReadyToSwitchOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= stateTransitionViaSdo(StateTransition::_7);
          break;
        case DriveState::ReadyToSwitchOn:
          success &= true;
          break;
        case DriveState::SwitchedOn:
          success &= stateTransitionViaSdo(StateTransition::_10);
          break;

        case DriveState::OperationEnabled:
          success &= stateTransitionViaSdo(StateTransition::_8);
          break;
        case DriveState::QuickStopActive:
          success &= stateTransitionViaSdo(StateTransition::_12);
          success &= stateTransitionViaSdo(StateTransition::_2);
          break;
        case DriveState::Fault:
          success &= stateTransitionViaSdo(StateTransition::_15L);
          // @todo: remove manual delay if not needed
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          success &= stateTransitionViaSdo(StateTransition::_15H);
          success &= stateTransitionViaSdo(StateTransition::_2);
          break;
        default:
          MELO_ERROR_STREAM(
              "[nanotec_ethercat_sdk:Nanotec::setDriveStateViaSdo] State "
              "Transition not implemented");
          addErrorToReading(ErrorType::SdoStateTransitionError);
          success = false;
      }
      break;

 case DriveState::SwitchedOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          break;
        case DriveState::ReadyToSwitchOn:
          success &= stateTransitionViaSdo(StateTransition::_3);
          break;
        case DriveState::SwitchedOn:
          success &= true;
          break;
        case DriveState::OperationEnabled:
          success &= stateTransitionViaSdo(StateTransition::_5);
          break;
        case DriveState::QuickStopActive:
          success &= stateTransitionViaSdo(StateTransition::_12);
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          break;
        case DriveState::Fault:
          success &= stateTransitionViaSdo(StateTransition::_15L);
          // @todo: remove manual delay if not needed
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          success &= stateTransitionViaSdo(StateTransition::_15H);
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          break;
        default:
          MELO_ERROR_STREAM(
              "[nanotec_ethercat_sdk:Nanotec::setDriveStateViaSdo] State "
              "Transition not implemented");
          addErrorToReading(ErrorType::SdoStateTransitionError);
          success = false;
      }
      break;

 case DriveState::OperationEnabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          break;
        case DriveState::ReadyToSwitchOn:
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          break;
        case DriveState::SwitchedOn:
          success &= stateTransitionViaSdo(StateTransition::_4);
          break;
        case DriveState::OperationEnabled:
          success &= true;
          break;
        case DriveState::QuickStopActive:
          // Exiting Quick Stop Active by a high edge on Bit 2
          success &= stateTransitionViaSdo(StateTransition::_16L);
          // @todo: remove manual delay if not needed
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          success &= stateTransitionViaSdo(StateTransition::_16H);
          break;
        case DriveState::Fault:
          success &= stateTransitionViaSdo(StateTransition::_15L);
          // @todo: remove manual delay if not needed
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          success &= stateTransitionViaSdo(StateTransition::_15H);
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          break;
        default:
          MELO_ERROR_STREAM(
              "[nanotec_ethercat_sdk:Nanotec::setDriveStateViaSdo] State "
              "Transition not implemented");
          addErrorToReading(ErrorType::SdoStateTransitionError);
          success = false;
      }
      break;

 case DriveState::QuickStopActive:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        case DriveState::ReadyToSwitchOn:
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        case DriveState::SwitchedOn:
          success &= stateTransitionViaSdo(StateTransition::_4);
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        case DriveState::OperationEnabled:
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        case DriveState::QuickStopActive:
          success &= true;
          break;
        case DriveState::Fault:
          success &= stateTransitionViaSdo(StateTransition::_15L);
          // @todo: remove manual delay if not needed
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          success &= stateTransitionViaSdo(StateTransition::_15H);
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        default:
          MELO_ERROR_STREAM(
              "[nanotec_ethercat_sdk:Nanotec::setDriveStateViaSdo] State "
              "Transition not implemented");
          addErrorToReading(ErrorType::SdoStateTransitionError);
          success = false;
      }
      break;

    default:
      MELO_ERROR_STREAM(
          "[nanotec_ethercat_sdk:Nanotec::setDriveStateViaSdo] State "
          "Transition not implemented");
      addErrorToReading(ErrorType::SdoStateTransitionError);
      success = false;
  }

  return success;
}

bool Nanotec::stateTransitionViaSdo(const StateTransition& stateTransition) {
  Controlword controlword;
  switch (stateTransition) {
    case StateTransition::_2:
      controlword.setStateTransition2();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_3:
      controlword.setStateTransition3();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_4:
      controlword.setStateTransition4();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_5:
      controlword.setStateTransition5();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_6:
      controlword.setStateTransition6();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_7:
      controlword.setStateTransition7();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_8:
      controlword.setStateTransition8();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_9:
      controlword.setStateTransition9();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_10:
      controlword.setStateTransition10();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_11:
      controlword.setStateTransition11();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_12:
      controlword.setStateTransition12();
      return setControlwordViaSdo(controlword);
      break;
    // Transition 13 doesn't exist and 14 is automatic (not admissible), so not programmed
    case StateTransition::_15L:
      controlword.setStateTransition15Low();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_15H:
      controlword.setStateTransition15High();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_16L:
      controlword.setStateTransition16Low();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_16H:
      controlword.setStateTransition16High();
      return setControlwordViaSdo(controlword);
      break;
    default:
      MELO_ERROR_STREAM(
          "[nanotec_ethercat_sdk:Nanotec::stateTransitionViaSdo] State "
          "Transition not implemented");
      addErrorToReading(ErrorType::SdoStateTransitionError);
      return false;
  }
}

bool Nanotec::setDriveStateViaPdo(const DriveState& driveState, const bool waitForState) {
                                  
  bool success = false;
 /*
  ** Skipping lock guard declaration since manual locking and unlocking
  ** of the mutex is required for allowing periodic PDO writes (and 
  ** thus state changes).
  */
  mutex_.lock();

  // reset the "stateChangeSuccessful_" flag to false such that a new successful
  // state change can be detected
  stateChangeSuccessful_ = false;

  // make the state machine realize that a state change will have to happen
  conductStateChange_ = true;

  // overwrite the target drive state
  targetDriveState_ = driveState;

  // set the hasRead flag to false such that at least one new reading will be
  // available when starting the state change
  hasRead_ = false;

  // set the time point of the last pdo change to now
  driveStateChangeTimePoint_ = std::chrono::steady_clock::now();

  // set a temporary time point to prevent getting caught in an infinite loop
  auto driveStateChangeStartTimePoint = std::chrono::steady_clock::now();

  // return if no waiting is requested
  if (!waitForState) {
    // unlock the mutex
    mutex_.unlock();
    // return true if no waiting is requested
    return true;
  }

  // Wait for the state change to be successful
  // during the waiting time the mutex MUST be unlocked!

  while (true) {
    // break loop as soon as the state change was successful
    if (stateChangeSuccessful_) {
      success = true;
      break;
    }

    // break the loop if the state change takes too long
    // this prevents a freezing of the end user's program if the hardware is not
    // able to change it's state.
    if ((std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now() - driveStateChangeStartTimePoint))
            .count() > configuration_.driveStateChangeMaxTimeout) {
      break;
    }
    // unlock the mutex during sleep time
    mutex_.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // lock the mutex to be able to check the success flag
    mutex_.lock();
  }
  // unlock the mutex one last time
  mutex_.unlock();
  return success;
  
}


Controlword Nanotec::getNextStateTransitionControlword(
    const DriveState& requestedDriveState,
    const DriveState& currentDriveState) {
      
  Controlword controlword;
  controlword.setAllFalse();
  edgeTransitionLowSet_ = false;
  switch (requestedDriveState) {
    case DriveState::SwitchOnDisabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          MELO_ERROR_STREAM("[nanotec_ethercat_sdk:Nanotec::getNextStateTransitionControlword] "
              << "drive state has already been reached for '" << name_ << "'");
          addErrorToReading(ErrorType::PdoStateTransitionError);
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition7();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition10();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition9();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          if(!edgeTransitionLowSet_){
            controlword.setStateTransition15Low();
            edgeTransitionLowSet_ = true;}
          else {
            controlword.setStateTransition15High();
            edgeTransitionLowSet_ = false;
          }
          break;
        default:
          MELO_ERROR_STREAM(
              "[nanotec_ethercat_sdk:Nanotec::getNextStateTransitionControlword] "
              << "PDO state transition not implemented for '" << name_ << "'\n"
              << "Current: " << currentDriveState << "\n"
              << "Requested: " << requestedDriveState);
          addErrorToReading(ErrorType::PdoStateTransitionError);
      }
      break;

    case DriveState::ReadyToSwitchOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          MELO_ERROR_STREAM(
              "[nanotec_ethercat_sdk:Nanotec::getNextStateTransitionControlword] "
              << "drive state has already been reached for '" << name_ << "'");
          addErrorToReading(ErrorType::PdoStateTransitionError);
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition6();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition8();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          if(!edgeTransitionLowSet_){
            controlword.setStateTransition15Low();
            edgeTransitionLowSet_ = true;}
          else {
            controlword.setStateTransition15High();
            edgeTransitionLowSet_ = false;
          }
          break;
        default:
          MELO_ERROR_STREAM(
              "[nanotec_ethercat_sdk:Nanotec::getNextStateTransitionControlword] "
              << "PDO state transition not implemented for '" << name_ << "'\n"
              << "Current: " << currentDriveState << "\n"
              << "Requested: " << requestedDriveState);
          addErrorToReading(ErrorType::PdoStateTransitionError);
      }
      break;

    case DriveState::SwitchedOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          MELO_ERROR_STREAM(
              "[nanotec_ethercat_sdk:Nanotec::getNextStateTransitionControlword] "
              << "drive state has already been reached for '" << name_ << "'");
          addErrorToReading(ErrorType::PdoStateTransitionError);
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition5();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          if(!edgeTransitionLowSet_){
            controlword.setStateTransition15Low();
            edgeTransitionLowSet_ = true;}
          else {
            controlword.setStateTransition15High();
            edgeTransitionLowSet_ = false;
          }
          break;
        default:
          MELO_ERROR_STREAM(
              "[nanotec_ethercat_sdk:Nanotec::getNextStateTransitionControlword] "
              << "PDO state transition not implemented for '" << name_ << "'\n"
              << "Current: " << currentDriveState << "\n"
              << "Requested: " << requestedDriveState);
          addErrorToReading(ErrorType::PdoStateTransitionError);
      }
      break;

    case DriveState::OperationEnabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition4();
          break;
        case DriveState::OperationEnabled:
          MELO_ERROR_STREAM(
              "[nanotec_ethercat_sdk:Nanotec::getNextStateTransitionControlword] "
              << "drive state has already been reached for '" << name_ << "'");
          addErrorToReading(ErrorType::PdoStateTransitionError);
          break;
        case DriveState::QuickStopActive:
          if(!edgeTransitionLowSet_){
            controlword.setStateTransition16Low();
            edgeTransitionLowSet_ = true;}
          else {
            controlword.setStateTransition16High();
            edgeTransitionLowSet_ = false;
          }
          break;
        case DriveState::Fault:
          if(!edgeTransitionLowSet_){
            controlword.setStateTransition15Low();
            edgeTransitionLowSet_ = true;}
          else {
            controlword.setStateTransition15High();
            edgeTransitionLowSet_ = false;
          }
          break;
        default:
          MELO_ERROR_STREAM(
              "[nanotec_ethercat_sdk:Nanotec::getNextStateTransitionControlword] "
              << "PDO state transition not implemented for '" << name_ << "'\n"
              << "Current: " << currentDriveState << "\n"
              << "Requested: " << requestedDriveState);
          addErrorToReading(ErrorType::PdoStateTransitionError);
      }
      break;

    case DriveState::QuickStopActive:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition4();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition11();
          break;
        case DriveState::QuickStopActive:
          MELO_ERROR_STREAM(
              "[nanotec_ethercat_sdk:Nanotec::getNextStateTransitionControlword] "
              << "drive state has already been reached for '" << name_ << "'");
          addErrorToReading(ErrorType::PdoStateTransitionError);
          break;
        case DriveState::Fault:
          if(!edgeTransitionLowSet_){
            controlword.setStateTransition15Low();
            edgeTransitionLowSet_ = true;}
          else {
            controlword.setStateTransition15High();
            edgeTransitionLowSet_ = false;
          }
          break;
        default:
          MELO_ERROR_STREAM(
              "[nanotec_ethercat_sdk:Nanotec::getNextStateTransitionControlword] "
              << "PDO state transition not implemented for '" << name_ << "'\n"
              << "Current: " << currentDriveState << "\n"
              << "Requested: " << requestedDriveState);
          addErrorToReading(ErrorType::PdoStateTransitionError);
      }
      break;

    default:
      MELO_ERROR_STREAM("[nanotec_ethercat_sdk:Nanotec::getNextStateTransitionControlword] "
          << "PDO state cannot be reached for '" << name_ << "'");
      addErrorToReading(ErrorType::PdoStateTransitionError);
  }

  return controlword;
  
}


void Nanotec::autoConfigurePdoSizes() {
  auto pdoSizes = bus_->getHardwarePdoSizes(static_cast<uint16_t>(address_));
  pdoInfo_.rxPdoSize_ = pdoSizes.first;
  pdoInfo_.txPdoSize_ = pdoSizes.second;
}

uint16_t Nanotec::getTxPdoSize() { return pdoInfo_.txPdoSize_; }

uint16_t Nanotec::getRxPdoSize() { return pdoInfo_.rxPdoSize_; }

void Nanotec::engagePdoStateMachine() {
  
  // locking the mutex
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // elapsed time since the last new controlword
  auto microsecondsSinceChange =
      (std::chrono::duration_cast<std::chrono::microseconds>(
           std::chrono::steady_clock::now() - driveStateChangeTimePoint_))
          .count();

  // get the current state
  // since we wait until "hasRead" is true, this is guaranteed to be a newly
  // read value
  const DriveState currentDriveState = reading_.getDriveState();
  // check if the state change already was successful:
  if (currentDriveState == targetDriveState_) {
    numberOfSuccessfulTargetStateReadings_++;
    if (numberOfSuccessfulTargetStateReadings_ >=
        configuration_.minNumberOfSuccessfulTargetStateReadings) {
      // disable the state machine
      conductStateChange_ = false;
      numberOfSuccessfulTargetStateReadings_ = 0;
      stateChangeSuccessful_ = true;
      return;
    }
  } else if (microsecondsSinceChange >
             configuration_.driveStateChangeMinTimeout) {
    // get the next controlword from the state machine
    controlword_ =
        getNextStateTransitionControlword(targetDriveState_, currentDriveState);
    driveStateChangeTimePoint_ = std::chrono::steady_clock::now();
  }

  // set the "hasRead" variable to false such that there will definitely be a
  // new reading when this method is called again
  hasRead_ = false;
  
}

bool Nanotec::disableNanoJ() {
  bool success = true;
  MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::disableNanoJ] Disabling NanoJ for '"
            << name_ << "'");
  success &= sdoVerifyWrite(OD_INDEX_NANOJ_CONTROL, 0x00, false,
                                  static_cast<uint32_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);
  if (!success)
  {
    MELO_ERROR_STREAM("[nanotec_ethercat_sdk:Nanotec::disableNanoJ] Disabling NanoJ for '"
            << name_ << "' failed");
  }
  return success;
}

bool Nanotec::setStartCommand(bool setOMS0, bool setOMS1, bool setOMS2, bool setOMS3, bool setHALT) {
  Controlword controlword;
  bool success = true;
  // controlword.setAllFalse();
  controlword.setStateTransition4();
  if (setOMS0)
  {
    controlword.setControlWordOMS0();
  }
  if (setOMS1)
  {
    controlword.setControlWordOMS1();
  }
  if (setOMS2)
  {
    controlword.setControlWordOMS2();
  }
  if (setOMS3)
  {
    controlword.setControlWordOMS3();
  }
  if (setHALT)
  {
    controlword.setControlWordHALT();
  }

  success &= setControlwordViaSdo(controlword);

  if(!success){
    MELO_ERROR("nanotec_ethercat_sdk:Nanotec::setStartCommand] Setting start command failed! Raw Controlword: 0x%04X", controlword.getRawControlword());
  }
  return success;
}

}  // namespace nanotec
