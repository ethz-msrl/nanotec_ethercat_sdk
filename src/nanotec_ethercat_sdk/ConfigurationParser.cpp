
#include <cstdint>

#include "nanotec_ethercat_sdk/ConfigurationParser.hpp"

namespace nanotec {

  std::string modeOfOperationEnumToString(ModeOfOperationEnum modeOfOperation_) {
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


/*!
 * Function template for convenience
 * @param[in] yamlNode	the current node containing the requested variable
 * @param[in] varName	The name of the variable
 * @param[out] var	The variable which shall be read
 * @return	true on success
 */
template <typename T>
bool getValueFromFile(YAML::Node& yamlNode, const std::string& varName,
                      T& var) {
  if (!yamlNode[varName].IsDefined()) {
    MELO_WARN_STREAM(
        "[nanotec_ethercat_sdk:ConfigurationParser::parseConfiguration]: "
        "field '"
        << varName << "' is missing. Default value will be used.");
    return false;
  }
  try {
    T tmpVar = yamlNode[varName].as<T>();
    var = tmpVar;
    return true;
  } catch (...) {
    MELO_ERROR_STREAM(
        "[getValueFromFile] Error "
        "while parsing value \""
        << varName << "\", default values will be used");
    return false;
  }
}

/*!
 * Function to read a Modes of Operation enum from the yaml file
 * @param[in] yamlNode	the node containing the requested value
 * @param [in] varName	The name of the variable
 * @param [out] mode	The read mode of operation
 * @return	true on success
 */
bool getModesFromFile(YAML::Node& yamlNode, const std::string& varName,
                      std::vector<ModeOfOperationEnum>& modes) {
  if (!yamlNode[varName].IsDefined()) {
    MELO_WARN_STREAM(
        "[parseConfiguration]: "
        "field '"
        << varName << "' is missing. Default value will be used.");
    return false;
  }
  try {
    const std::map<std::string, ModeOfOperationEnum> str2ModeMap = {
        {"AutoSetup", ModeOfOperationEnum::AutoSetup},
        {"ClockDirectionMode", ModeOfOperationEnum::ClockDirectionMode},
        {"ProfilePositionMode", ModeOfOperationEnum::ProfilePositionMode},
        {"VelocityMode", ModeOfOperationEnum::VelocityMode},
        {"ProfileVelocityMode", ModeOfOperationEnum::ProfileVelocityMode},
        {"ProfileTorqueMode", ModeOfOperationEnum::ProfileTorqueMode},
        {"HomingMode", ModeOfOperationEnum::HomingMode},
        {"InterpolatedPositionMode", ModeOfOperationEnum::InterpolatedPositionMode},
        {"CyclicSynchronousPositionMode", ModeOfOperationEnum::CyclicSynchronousPositionMode},
        {"CyclicSynchronousVelocityMode", ModeOfOperationEnum::CyclicSynchronousVelocityMode},
        {"CyclicSynchronousTorqueMode", ModeOfOperationEnum::CyclicSynchronousTorqueMode}
    };

    std::vector<std::string> strModes =  yamlNode[varName].as<std::vector<std::string>>();
    for (const auto& strMode : strModes) {
      if (str2ModeMap.find(strMode) != str2ModeMap.end()) {
        modes.push_back(str2ModeMap.at(strMode));
        MELO_INFO("ModeOfOperation: %s", strMode.c_str());
      } else {
        MELO_ERROR_STREAM(
            "[nanotec_ethercat_sdk:ConfigurationParser::parseConfiguration]"
            << "Mode '" << strMode << "' Does not exist.");
        return false;
      }
    }
    return true;
  } catch (...) {
    MELO_ERROR_STREAM(
        "[nanotec_ethercat_sdk:ConfigurationParser::getModeFromFile] Error "
        "while parsing value \""
        << varName << "\", default values will be used");
    return false;
  }
}

ConfigurationParser::ConfigurationParser(const std::string& filename) {
  YAML::Node configNode;
  try {
    configNode = YAML::LoadFile(filename);
  } catch (...) {
    MELO_ERROR_STREAM(
        "[nanotec_ethercat_sdk:ConfigurationParser::ConfigurationParser] "
        "Loading YAML configuration file '"
        << filename << "' failed.");
  }
  parseConfiguration(configNode);
}

ConfigurationParser::ConfigurationParser(YAML::Node configNode) {
  parseConfiguration(configNode);
}

void ConfigurationParser::parseConfiguration(YAML::Node configNode) {
  std::string message = "\n";

  if (configNode["Nanotec"].IsDefined()) {
    /// A new node for the NanotecEthercat class
    YAML::Node nanotecNode = configNode["Nanotec"];
    message += "\033[34m";
    message += "[NANOTEC]";
    message += "\033[m\n";

    uint32_t configRunSdoVerifyTimeout;
    if (getValueFromFile(nanotecNode, "config_run_sdo_verify_timeout",
                         configRunSdoVerifyTimeout)) {
      configuration_.configRunSdoVerifyTimeout = configRunSdoVerifyTimeout;
      message += "\033[32m\t";
      message += "config_run_sdo_verify_timeout = ";
      message +=  std::to_string(configRunSdoVerifyTimeout);
      message += " ms";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "config_run_sdo_verify_timeout = ";
      message +=  std::to_string(configRunSdoVerifyTimeout);
      message += " ms";
      message += "\033[m\n";
    }

      

    double driveStateChangeMinTimeout;
    if (getValueFromFile(nanotecNode, "drive_state_change_min_timeout", driveStateChangeMinTimeout)) {
      configuration_.driveStateChangeMinTimeout = driveStateChangeMinTimeout;
      message += "\033[32m\t";
      message += "drive_state_change_min_timeout = ";
      message +=  std::to_string(driveStateChangeMinTimeout);
      message += " ms";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "drive_state_change_min_timeout = ";
      message +=  std::to_string(driveStateChangeMinTimeout);
      message += " ms";
      message += "\033[m\n";
    }

    double driveStateChangeMaxTimeout;
    if (getValueFromFile(nanotecNode, "drive_state_change_max_timeout", driveStateChangeMaxTimeout)) {
      configuration_.driveStateChangeMaxTimeout = driveStateChangeMaxTimeout;
      message += "\033[32m\t";
      message += "drive_state_change_max_timeout = ";
      message +=  std::to_string(driveStateChangeMaxTimeout);
      message += " ms";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "drive_state_change_max_timeout = ";
      message +=  std::to_string(driveStateChangeMaxTimeout);
      message += " ms";
      message += "\033[m\n";
    }

    double minNumberOfSuccessfulTargetStateReadings;
    if (getValueFromFile(nanotecNode, "min_number_of_successful_target_state_readings", minNumberOfSuccessfulTargetStateReadings)) {
      configuration_.minNumberOfSuccessfulTargetStateReadings = minNumberOfSuccessfulTargetStateReadings;
      message += "\033[32m\t";
      message += "min_number_of_successful_target_state_readings = ";
      message +=  std::to_string(minNumberOfSuccessfulTargetStateReadings);
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "min_number_of_successful_target_state_readings = ";
      message +=  std::to_string(minNumberOfSuccessfulTargetStateReadings);
      message += "\033[m\n";
    }

    uint32_t interpolationTimePeriodmS;
    if (getValueFromFile(nanotecNode, "interpolation_time_period_ms",
                         interpolationTimePeriodmS)) {
      configuration_.interpolationTimePeriodmS = interpolationTimePeriodmS;
      message += "\033[32m\t";
      message += "interpolation_time_period_ms = ";
      message +=  std::to_string(interpolationTimePeriodmS);
      message += " ms";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "interpolation_time_period_ms = ";
      message +=  std::to_string(interpolationTimePeriodmS);
      message += " ms";
      message += "\033[m\n";
    }
  }
 

  /// The configuration options for the nanotec::ethercat::Reading class
  if (configNode["Reading"].IsDefined()) {
    YAML::Node readingNode = configNode["Reading"];
    message += "\033[34m";
    message += "[READING]";
    message += "\033[m\n";
  }

  /// The configuration options for the Nanotec servo drive ("hardware")
  if (configNode["Hardware"].IsDefined()) {
    YAML::Node hardwareNode = configNode["Hardware"];
    message += "\033[34m";
    message += "[HARDWARE]";
    message += "\033[m\n";

    std::vector<ModeOfOperationEnum> modesOfOperation;
    if (getModesFromFile(hardwareNode, "mode_of_operation", modesOfOperation)) {
      configuration_.modesOfOperation = modesOfOperation;
        message += "\033[32m\t";
        message += "modes_of_operation:";
        message += "\033[m\n";
      for(const ModeOfOperationEnum &mode : modesOfOperation){
        message += "\033[32m\t";
        message += "  - ";
        message +=  modeOfOperationEnumToString(mode);
        message += "\033[m\n";
      }
    }

    uint32_t polePairs;
    if (getValueFromFile(hardwareNode, "pole_pairs", polePairs)) {
      configuration_.polePairs = polePairs;
      message += "\033[32m\t";
      message += "pole_pairs = ";
      message +=  std::to_string(polePairs);
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "pole_pairs = ";
      message +=  std::to_string(polePairs);
      message += "\033[m\n";
    }

    uint32_t maxMotorCurrentmA;
    if (getValueFromFile(hardwareNode, "max_motor_current_mA", maxMotorCurrentmA)) {
      configuration_.maxMotorCurrentmA = maxMotorCurrentmA;
      message += "\033[32m\t";
      message += "max_motor_current = ";
      message +=  std::to_string(maxMotorCurrentmA);
      message += " mA";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "max_motor_current = ";
      message +=  std::to_string(maxMotorCurrentmA);
      message += " mA";
      message += "\033[m\n";
    }

    uint32_t ratedCurrentmA;
    if (getValueFromFile(hardwareNode, "rated_current_mA", ratedCurrentmA)) {
      configuration_.ratedCurrentmA = ratedCurrentmA;
      message += "\033[32m\t";
      message += "rated_current_mA = ";
      message +=  std::to_string(ratedCurrentmA);
      message += " mA";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "rated_current_mA = ";
      message +=  std::to_string(ratedCurrentmA);
      message += " mA";
      message += "\033[m\n";
    }

    uint32_t maxMotorSpeed;
    if (getValueFromFile(hardwareNode, "max_motor_speed", maxMotorSpeed)) {
      configuration_.maxMotorSpeed = maxMotorSpeed;
      message += "\033[32m\t";
      message += "max_motor_speed = ";
      message +=  std::to_string(maxMotorSpeed);
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "max_motor_speed = ";
      message +=  std::to_string(maxMotorSpeed);
      message += "\033[m\n";
    }

    uint16_t maxCurrentPercentage;
    if (getValueFromFile(hardwareNode, "max_current_percentage", maxCurrentPercentage)) {
      configuration_.maxCurrentPercentage = maxCurrentPercentage;
      message += "\033[32m\t";
      message += "max_current_percentage = ";
      message +=  std::to_string(maxCurrentPercentage/10);
      message += " %%";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "max_current_percentage = ";
      message +=  std::to_string(maxCurrentPercentage/10);
      message += " %%";
      message += "\033[m\n";
    }

    uint32_t I2tMaxDurationOfPeakms;
    if (getValueFromFile(hardwareNode, "i2t_max_duration_of_peak_ms", I2tMaxDurationOfPeakms)) {
      configuration_.I2tMaxDurationOfPeakms = I2tMaxDurationOfPeakms;
      message += "\033[32m\t";
      message += "i2t_max_duration_of_peak_ms = ";
      message +=  std::to_string(I2tMaxDurationOfPeakms);
      message += " ms";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "i2t_max_duration_of_peak_ms = ";
      message +=  std::to_string(I2tMaxDurationOfPeakms);
      message += " ms";
      message += "\033[m\n";
    }

    int32_t clockDirectionMultiplier;
    if (getValueFromFile(hardwareNode, "clock_direction_multiplier", clockDirectionMultiplier)) {
      configuration_.clockDirectionMultiplier = clockDirectionMultiplier;
      message += "\033[32m\t";
      message += "clock_direction_multiplier = ";
      message +=  std::to_string(clockDirectionMultiplier);
      message += "";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "clock_direction_multiplier = ";
      message +=  std::to_string(clockDirectionMultiplier);
      message += " ms";
      message += "\033[m\n";
    }

    int32_t clockDirectionDivider;
    if (getValueFromFile(hardwareNode, "clock_direction_divider", clockDirectionDivider)) {
      configuration_.clockDirectionDivider = clockDirectionDivider;
      message += "\033[32m\t";
      message += "clock_direction_divider = ";
      message +=  std::to_string(clockDirectionDivider);
      message += "";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "clock_direction_divider = ";
      message +=  std::to_string(clockDirectionDivider);
      message += "";
      message += "\033[m\n";
    }

    bool limitSwitchNegativeEn;
    if (getValueFromFile(hardwareNode, "limit_switch_negative_en", limitSwitchNegativeEn)) {
      configuration_.limitSwitchNegativeEn = limitSwitchNegativeEn;
      message += "\033[32m\t";
      message += "limit_switch_negative_en = ";
      message +=  std::to_string(limitSwitchNegativeEn);
      message += "";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "limit_switch_negative_en = ";
      message +=  std::to_string(limitSwitchNegativeEn);
      message += "";
      message += "\033[m\n";
    }


    bool limitSwitchPositiveEn;
    if (getValueFromFile(hardwareNode, "limit_switch_positive_en", limitSwitchPositiveEn)) {
      configuration_.limitSwitchPositiveEn = limitSwitchPositiveEn;
      message += "\033[32m\t";
      message += "limit_switch_positive_en = ";
      message +=  std::to_string(limitSwitchPositiveEn);
      message += "";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "limit_switch_positive_en = ";
      message +=  std::to_string(limitSwitchPositiveEn);
      message += "";
      message += "\033[m\n";
    }

    bool limitSwitchHomingEn;
    if (getValueFromFile(hardwareNode, "limit_switch_homing_en", limitSwitchHomingEn)) {
      configuration_.limitSwitchHomingEn = limitSwitchHomingEn;
      message += "\033[32m\t";
      message += "limit_switch_homing_en = ";
      message +=  std::to_string(limitSwitchHomingEn);
      message += "";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "limit_switch_homing_en = ";
      message +=  std::to_string(limitSwitchHomingEn);
      message += "";
      message += "\033[m\n";
    }

    uint32_t SIUnitPosition;
    if (getValueFromFile(hardwareNode, "position_unit", SIUnitPosition)) {
      configuration_.SIUnitPosition = SIUnitPosition;
      message += "\033[32m\t";
      message += "position_unit = ";
      message +=  std::to_string(SIUnitPosition);
      message += "";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "position_unit = ";
      message +=  std::to_string(configuration_.SIUnitPosition);
      message += "";
      message += "\033[m\n";
    }

    uint32_t SIUnitVelocity;
    if (getValueFromFile(hardwareNode, "velocity_unit", SIUnitVelocity)) {
      configuration_.SIUnitVelocity = SIUnitVelocity;
      message += "\033[32m\t";
      message += "velocity_unit = ";
      message +=  std::to_string(SIUnitVelocity);
      message += "";
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "velocity_unit = ";
      message +=  std::to_string(configuration_.SIUnitVelocity);
      message += "";
      message += "\033[m\n";
    }
  }

  
  if (configNode["AutoSetup"].IsDefined()) {
    YAML::Node autoSetupNode = configNode["AutoSetup"];
    message += "\033[34m";
    message += "[AutoSetup]";
    message += "\033[m\n";

    bool autoSetupEn;
    if (getValueFromFile(autoSetupNode, "auto_setup_en", autoSetupEn)) {
      configuration_.autoSetupEn = autoSetupEn;
      message += "\033[32m\t";
      message += "auto_setup_en = ";
      message +=  std::to_string(autoSetupEn);
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "auto_setup_en = ";
      message +=  std::to_string(configuration_.autoSetupEn);
      message += "\033[m\n";
    }

    uint32_t autoSetupTimeoutms;
    if (getValueFromFile(autoSetupNode, "auto_setup_timeout_ms", autoSetupTimeoutms)) {
      configuration_.autoSetupTimeoutms = autoSetupTimeoutms;
      message += "\033[32m\t";
      message += "auto_setup_timeout_ms = ";
      message +=  std::to_string(autoSetupTimeoutms);
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "auto_setup_timeout_ms = ";
      message +=  std::to_string(configuration_.autoSetupTimeoutms);
      message += "\033[m\n";
    }


  }
  

  if (configNode["Homing"].IsDefined()) {
    YAML::Node homingNode = configNode["Homing"];
    message += "\033[34m";
    message += "[HOMING]";
    message += "\033[m\n";
    MELO_INFO_STREAM("homingNode: " << homingNode);

    bool homingEn;
    if (getValueFromFile(homingNode, "homing_en", homingEn)) {
      configuration_.homingEn = homingEn;
      message += "\033[32m\t";
      message += "homing_en = ";
      message +=  std::to_string(homingEn);
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "homing_en = ";
      message +=  std::to_string(configuration_.homingEn);
      message += "\033[m\n";
    }

    int32_t homeOffset;
    if (getValueFromFile(homingNode, "home_offset", homeOffset)) {
      configuration_.homeOffset = homeOffset;
      message += "\033[32m\t";
      message += "home_offset = ";
      message +=  std::to_string(homeOffset);
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "home_offset = ";
      message +=  std::to_string(homeOffset);
      message += "\033[m\n";
    }

    int32_t homingMethod;
    if (getValueFromFile(homingNode, "homing_method", homingMethod)) {
      configuration_.homingMethod = homingMethod;
      message += "\033[32m\t";
      message += "homing_method = ";
      message +=  std::to_string(homingMethod);
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "homing_method = ";
      message +=  std::to_string(homingMethod);
      message += "\033[m\n";
    }

    int32_t homingSpeedZeroSearch;
    if (getValueFromFile(homingNode, "homing_speed_zero_search", homingSpeedZeroSearch)) {
      configuration_.homingSpeedZeroSearch = homingSpeedZeroSearch;
      message += "\033[32m\t";
      message += "homing_speed_zero_search = ";
      message +=  std::to_string(homingSpeedZeroSearch);
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "homing_speed_zero_search = ";
      message +=  std::to_string(homingSpeedZeroSearch);
      message += "\033[m\n";
    }

    int32_t homingSpeedSwitchSearch;
    if (getValueFromFile(homingNode, "homing_speed_switch_search", homingSpeedSwitchSearch)) {
      configuration_.homingSpeedSwitchSearch = homingSpeedSwitchSearch;
      message += "\033[32m\t";
      message += "homing_speed_switch_search = ";
      message +=  std::to_string(homingSpeedSwitchSearch);
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "homing_speed_switch_search = ";
      message +=  std::to_string(homingSpeedSwitchSearch);
      message += "\033[m\n";
    }

    uint32_t homingAcceleration;
    if (getValueFromFile(homingNode, "homing_acceleration", homingAcceleration)) {
      configuration_.homingAcceleration = homingAcceleration;
      message += "\033[32m\t";
      message += "homing_acceleration = ";
      message +=  std::to_string(homingAcceleration);
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "homing_acceleration = ";
      message +=  std::to_string(homingAcceleration);
      message += "\033[m\n";
    }

    int32_t homingMinimumCurrentForBlockDetectionmA;
    if (getValueFromFile(homingNode, "minimum_current_for_block_detection_mA", homingMinimumCurrentForBlockDetectionmA)) {
      configuration_.homingMinimumCurrentForBlockDetectionmA = homingMinimumCurrentForBlockDetectionmA;
      message += "\033[32m\t";
      message += "minimum_current_for_block_detection_mA = ";
      message +=  std::to_string(homingMinimumCurrentForBlockDetectionmA);
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "minimum_current_for_block_detection_mA = ";
      message +=  std::to_string(homingMinimumCurrentForBlockDetectionmA);
      message += "\033[m\n";
    }

    int32_t homingPeriodForBlockingmS;
    if (getValueFromFile(homingNode, "period_of_blocking_ms", homingPeriodForBlockingmS)) {
      configuration_.homingPeriodForBlockingmS = homingPeriodForBlockingmS;
      message += "\033[32m\t";
      message += "period_of_blocking_ms = ";
      message +=  std::to_string(homingPeriodForBlockingmS);
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "period_of_blocking_ms = ";
      message +=  std::to_string(homingPeriodForBlockingmS);
      message += "\033[m\n";
    }

    uint32_t homingTimeoutms;
    if (getValueFromFile(homingNode, "homing_timeout_ms", homingTimeoutms)) {
      configuration_.homingTimeoutms = homingTimeoutms;
      message += "\033[32m\t";
      message += "homing_timeout_ms = ";
      message +=  std::to_string(homingTimeoutms);
      message += "\033[m\n";
    } else
    {
      message += "\033[31m\t";
      message += "homing_timeout_ms = ";
      message +=  std::to_string(homingTimeoutms);
      message += "\033[m\n";
    }

  }
  MELO_INFO_STREAM(message);
}

Configuration ConfigurationParser::getConfiguration() const {
  return configuration_;
}

}  // namespace nanotec
