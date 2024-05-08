#pragma once

#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

#include "nanotec_ethercat_sdk/ModeOfOperationEnum.hpp"
#include "nanotec_ethercat_sdk/PdoTypeEnum.hpp"
#include "ethercat_motor_msgs/ethercat_motor_msgs.h"



namespace nanotec {
class Configuration {
public:

    std::vector<ModeOfOperationEnum> modesOfOperation = {ModeOfOperationEnum::NA};
    // Nanotec
    uint32_t configRunSdoVerifyTimeout{20000}; // SDO Read Write verify timeout in micro-seconds
    uint32_t driveStateChangeMinTimeout{20000};
    uint32_t driveStateChangeMaxTimeout{300000};
    uint32_t minNumberOfSuccessfulTargetStateReadings{10};
    uint8_t interpolationTimePeriodmS{1};

    // Hardware
    uint32_t polePairs{50};
    uint32_t maxMotorCurrentmA{2000};
    uint16_t maxCurrentPercentage{1000}; // 1000 (default) is 100% for this param
    uint32_t ratedCurrentmA{1000};
    uint32_t maxMotorSpeed{1000};
    uint32_t I2tMaxDurationOfPeakms{100};
    // int32_t positionEncoderResolution{0};
    // int32_t motorRevsPerEncoderRev{0};
    // std::pair<uint32_t, uint32_t> gearRatio{1, 1}; // std::pair of motor revolutions, shaft revolutions
    // double motorConstant{1};
    // double workingVoltage{24.0};
    // double speedConstant{1};
    // double torqueConstantNmA{1};
    // double motorRatedTorqueNm{1};
    // double maxPosition{0}; // disable checks default
    // double minPosition{0};
    // double quickStopDeceleration{1000};
    // double profileDeceleration{1000};
    // uint32_t followErrorWindow{0};

    int32_t clockDirectionMultiplier{128};
    int32_t clockDirectionDivider{1};

    bool limitSwitchNegativeEn{false};
    bool limitSwitchPositiveEn{false};
    bool limitSwitchHomingEn{false};
    bool limitSwitchNegativeFunctionInverted{false};
    bool limitSwitchPositiveFunctionInverted{false};
    bool limitSwitchHomingFunctionInverted{false};

    int32_t softwarePositionLimitMin{0};
    int32_t softwarePositionLimitMax{0};

    int32_t positionRangeLimitMin{0};
    int32_t positionRangeLimitMax{0};

    int32_t gearRatioMotorRevolutions{0};
    int32_t gearRatioShaftRevolutions{0};


    uint32_t SIUnitPosition{0xFF410000}; // @brief defaults to `0xFF410000` = tenths of a degree i.e. 3600 = 1 mechanical revolution
    uint32_t SIUnitVelocity{0x00B44700}; // @brief defaults to `0x00B44700` = RPM

    // Motor type params
    uint32_t encoderConfiguration{0}; // Bit 1: 0 (0: Differential encoder; 1: Single-Ended encoder)
    
    /*! @brief Set motor drive submode
     *  @note
     *  Bit 0: 1 (0: Open loop control; 1: Closed loop control)
     *  Bit 1: 0 (0: V-Ctrl with S-Ramp Disabled; 1: V-Ctrl with S-Ramp Enabled)
     *  Bit 2: 0 (0: Auto Brake Ctrl Disabled; 1: Auto Brake Ctrl Enabled)
     *  Bit 3: 0 (0: Current reduction in open loop disabled; 1: Current reduction in open loop Enabled)
     *  Bit 4: 1 (0: Auto alignment in closed loop disabled; 1: Auto alignment in closed loop Enabled)
     *  Bit 5: 0 (0: M-controller disabled; 1: AM-controller enabled)
     *  Bit 6: 0 (0: Motor type: stepper; 1: Motor type: BLDC)
     *  Bit 7: 0 (0: Slow speed mode in closed loop disabled; 1: Slow speed mode in closed loop Enabled)
     *
     */
    uint32_t motorDriveSubmode{0x11}; 

    // AutoSetup
    bool autoSetupEn{false};
    uint32_t autoSetupTimeoutms{5000};

    // Homing
    bool homingEn{false};
    int32_t homeOffset{0};
    int8_t homingMethod{0x23};
    // uint32_t homingSpeed{100};
    uint32_t homingSpeedZeroSearch{0};
    uint32_t homingSpeedSwitchSearch{0};
    uint32_t homingAcceleration{100};
    int32_t homingMinimumCurrentForBlockDetectionmA{100};
    int32_t homingPeriodForBlockingmS{500};
    uint32_t homingTimeoutms{500};

    ProfilePositionModeSettings profilePositionModeSettings;  

    // Reading
    bool forceAppendEqualError{true};
    bool forceAppendEqualFault{false};
    unsigned int errorStorageCapacity{100};
    unsigned int faultStorageCapacity{100};

 public:
    // stream operator
    friend std::ostream& operator<<(std::ostream& os, const Configuration& configuration);

    /*!
    * @brief Check whether the parameters are sane.
    * Prints a list of the checks and whether they failed or passed.
    * @param[in] silent If true: Do not print. Only return the success of the
    * test.
    * @return true if the checks are successful.
    */
    bool sanityCheck(bool silent = false) const;

    std::pair<RxPdoTypeEnum, TxPdoTypeEnum> getPdoTypeSolution() const;
};

}  // namespace nanotec
