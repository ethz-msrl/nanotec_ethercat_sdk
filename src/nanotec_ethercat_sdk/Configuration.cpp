
#pragma once
#include "nanotec_ethercat_sdk/Configuration.hpp"

#include <iomanip>
#include <vector>
#include <map>
#include <algorithm>
#include <utility>
#include <ros/ros.h>

#define LOG(format, ...) ROS_INFO("[CONFIGURATION] " format, ##__VA_ARGS__)
#define LOG_STREAM(format, ...) ROS_INFO_STREAM("[CONFIGURATION] " << format, ##__VA_ARGS__)
#define LOG_WARN(format, ...) ROS_WARN("[CONFIGURATION] " format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) ROS_ERROR("[CONFIGURATION] " format, ##__VA_ARGS__)
#define LOG_ERROR_STREAM(format, ...) ROS_ERROR_STREAM("[CONFIGURATION] " << format, ##__VA_ARGS__)

namespace nanotec {

std::string modeOfOperationString(ModeOfOperationEnum modeOfOperation_) {
  switch (modeOfOperation_) {
    case ModeOfOperationEnum::AutoSetup:
      return "AutoSetup Mode";
    case ModeOfOperationEnum::ClockDirectionMode:
      return "Clock Direction Mode";
    case ModeOfOperationEnum::ProfilePositionMode:
      return "Profiled Position Mode";
    case ModeOfOperationEnum::VelocityMode:
      return "Velocity Mode";
    case ModeOfOperationEnum::ProfileVelocityMode:
      return "Profiled Velocity Mode";
    case ModeOfOperationEnum::ProfileTorqueMode:
      return "Profiled Torque Mode";
    case ModeOfOperationEnum::HomingMode:
      return "Homing Mode";
    case ModeOfOperationEnum::InterpolatedPositionMode:
      return "Interpolated Position Mode";
    case ModeOfOperationEnum::CyclicSynchronousPositionMode:
      return "Cyclic Synchronous Position Mode";
    case ModeOfOperationEnum::CyclicSynchronousVelocityMode:
      return "Cyclic Synchronous Velocity Mode";
    case ModeOfOperationEnum::CyclicSynchronousTorqueMode:
      return "Cyclic Synchronous Torque Mode";
    default:
      return "Unsupported Mode of Operation";
  }
}

  
std::string rxPdoString(RxPdoTypeEnum rxPdo) {
  switch (rxPdo) {
    case RxPdoTypeEnum::NA:
      return "NA";
    case RxPdoTypeEnum::RxPdoStandard:
      return "Rx PDO Standard";
    case RxPdoTypeEnum::RxPdoCSP:
      return "Rx PDO CSP";
    case RxPdoTypeEnum::RxPdoCST:
      return "Rx PDO CST";
    case RxPdoTypeEnum::RxPdoCSV:
      return "Rx PDO CSV";
    case RxPdoTypeEnum::RxPdoCSTCSP:
      return "Rx PDO CST/CSP mixed mode";
    case RxPdoTypeEnum::RxPdoCSTCSPCSV:
      return "Rx PDO CST/CSP/CSV mixed mode";
    case RxPdoTypeEnum::RxPdoPVM:
      return "Rx PDO PVM";
    default:
      return "Unsupported Type";
  }
}

std::string txPdoString(TxPdoTypeEnum txPdo) {
  switch (txPdo) {
    case TxPdoTypeEnum::NA:
      return "NA";
    case TxPdoTypeEnum::TxPdoCSP:
      return "Tx PDO CSP";
    case TxPdoTypeEnum::TxPdoCST:
      return "Tx PDO CST";
    case TxPdoTypeEnum::TxPdoCSV:
      return "Tx PDO CSV";
    case TxPdoTypeEnum::TxPdoCSTCSP:
      return "Tx PDO CST/CSP mixed mode";
    case TxPdoTypeEnum::TxPdoCSTCSPCSV:
      return "Rx PDO CST/CSP/CSV mixed mode";
    case TxPdoTypeEnum::TxPdoPVM:
      return "Tx PDO PVM";
    case TxPdoTypeEnum::TxPdoStandard:
      return "Tx PDO Standard";
    default:
      return "Unsupported Type";
  }
}

std::ostream& operator<<(std::ostream& os, const Configuration& configuration) {
  std::string modeOfOperation_ =
     modeOfOperationString(configuration.modesOfOperation[0]);
  unsigned int tmp3 = modeOfOperation_.size();
  unsigned int len2 = tmp3;
  len2++;

  os << std::boolalpha << std::left << std::setw(43) << std::setfill('-') << "|"
     << std::setw(len2 + 2) << "-"
     << "|\n"
     << std::setfill(' ') << std::setw(43 + len2 + 2) << "| Configuration"
     << "|\n"
     << std::setw(43) << std::setfill('-') << "|" << std::setw(len2 + 2) << "+"
     << "|\n"
     << std::setfill(' ') << std::setw(43) << "| 1st Mode of Operation:"
     << "| " << std::setw(len2) << modeOfOperation_ << "|\n"
     << std::setw(43) << "| Config Run SDO verify timeout:"
     << "| " << std::setw(len2) << configuration.configRunSdoVerifyTimeout
     << "|\n"
     << std::setw(43) << "| Drive State Change Min Timeout:"
     << "| " << std::setw(len2) << configuration.driveStateChangeMinTimeout
     << "|\n"
     << std::setw(43) << "| Drive State Change Max Timeout:"
     << "| " << std::setw(len2) << configuration.driveStateChangeMaxTimeout
     << "|\n"
     << std::setw(43) << "| Min Successful Target State Readings:"
     << "| " << std::setw(len2)
     << configuration.minNumberOfSuccessfulTargetStateReadings << "|\n"
     << std::setw(43) << std::setfill('-') << "|" << std::setw(len2 + 2) << "+"
     << "|\n"
     << std::setfill(' ') << std::noboolalpha << std::right;
  return os;
}


std::pair<RxPdoTypeEnum, TxPdoTypeEnum> Configuration::getPdoTypeSolution()
    const {
  // clang-format off
  // {ModeOfOperationEnum1, ..., ModeOfOperationEnumN} -> {RxPdoTypeEnum, TxPdoTypeEnum}
  const std::map<std::vector<ModeOfOperationEnum>, std::pair<RxPdoTypeEnum, TxPdoTypeEnum>> modes2PdoTypeMap = {
      {
        { ModeOfOperationEnum::CyclicSynchronousTorqueMode, ModeOfOperationEnum::CyclicSynchronousPositionMode },
        { RxPdoTypeEnum::RxPdoCSTCSP, TxPdoTypeEnum::TxPdoCSTCSP }
      },
      {
        { ModeOfOperationEnum::CyclicSynchronousTorqueMode, ModeOfOperationEnum::CyclicSynchronousPositionMode,
          ModeOfOperationEnum::CyclicSynchronousVelocityMode },
        { RxPdoTypeEnum::RxPdoCSTCSPCSV, TxPdoTypeEnum::TxPdoCSTCSPCSV }
      },
      {
        { ModeOfOperationEnum::CyclicSynchronousPositionMode },
        { RxPdoTypeEnum::RxPdoCSP, TxPdoTypeEnum::TxPdoCSP }
      },
      {
        { ModeOfOperationEnum::CyclicSynchronousTorqueMode },
        { RxPdoTypeEnum::RxPdoCST, TxPdoTypeEnum::TxPdoCST }
      },
      {
        { ModeOfOperationEnum::CyclicSynchronousVelocityMode },
        { RxPdoTypeEnum::RxPdoCSV, TxPdoTypeEnum::TxPdoCSV }
      },
      {
        { ModeOfOperationEnum::HomingMode },
        { RxPdoTypeEnum::NA, TxPdoTypeEnum::NA }
      },
      {
        { ModeOfOperationEnum::ProfilePositionMode },
        { RxPdoTypeEnum::NA, TxPdoTypeEnum::NA }
      },
      {
        { ModeOfOperationEnum::ProfileVelocityMode },
        { RxPdoTypeEnum::RxPdoPVM, TxPdoTypeEnum::TxPdoPVM }
      },
      {
        { ModeOfOperationEnum::NA },
        { RxPdoTypeEnum::NA, TxPdoTypeEnum::NA }
      },
  };
  // clang-format on

  bool setsAreEqual;
  for (const auto& modes2PdoTypeEntry : modes2PdoTypeMap) {
    setsAreEqual = true;
    for (const auto& modeOfOperation : modesOfOperation)
      setsAreEqual &=
          std::find(modes2PdoTypeEntry.first.begin(),
                    modes2PdoTypeEntry.first.end(),
                    modeOfOperation) != modes2PdoTypeEntry.first.end();
    for (const auto& modeOfOperation : modes2PdoTypeEntry.first)
      setsAreEqual &=
          std::find(modesOfOperation.begin(), modesOfOperation.end(),
                    modeOfOperation) != modesOfOperation.end();
    if (setsAreEqual) return modes2PdoTypeEntry.second;
  }
  return std::pair<RxPdoTypeEnum, TxPdoTypeEnum>{RxPdoTypeEnum::NA,
                                                 TxPdoTypeEnum::NA};
}

bool Configuration::sanityCheck(bool silent) const {
  bool success = true;
  std::string message = "\n";

  auto check_and_inform = [&message,
                           &success](std::pair<bool, std::string> test) {
    if (test.first) {
      message += "\033[32mOK\t";
      message += test.second;
      message += "\033[m\n";
      success &= true;
    } else {
      message += "\033[31mX\t";
      message += test.second;
      message += "\033[m\n";
      success = false;
    }
  };
  auto pdoTypePair = getPdoTypeSolution();
  // clang-format off
  const std::vector<std::pair<bool, std::string>> sanity_tests = {
      {
        (polePairs > 0),
        "pole_pairs > 0"
      },
      {
        (maxMotorSpeed > 0),
        "maxMotorSpeed > 0"
      },
      {
        (maxMotorCurrentmA > 0),
        "max_motor_current_A > 0"
      },
      {
        (ratedCurrentmA > 0),
        "nominal_current_A > 0"
      },
      {
        (I2tMaxDurationOfPeakms > 0),
        "i2t_max_duration_of_peak_ms > 0"
      },
      {
        (driveStateChangeMinTimeout <= driveStateChangeMaxTimeout),
        "drive_state_change_min_timeout ≤ drive_state_change_max_timeout"
      },
      {
        (pdoTypePair.first != RxPdoTypeEnum::NA && pdoTypePair.second != TxPdoTypeEnum::NA),
        "modes of operation combination allowed"
      },
  };
  // clang-format on

  std::for_each(sanity_tests.begin(), sanity_tests.end(), check_and_inform);

  if (!silent) {
   LOG_STREAM(message);
  }
  return success;
}
}  // namespace nanotec