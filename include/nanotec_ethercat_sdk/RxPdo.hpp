/*!
 * @file	RxPdo.hpp
 * @brief	This file contains the PDOs which are sent to the hardware
 * (RxPdo) Note: each struct MUST contain the controlWord_ variable!
 */
#pragma once

#include <cstdint>

namespace nanotec {

// Ordering of the variables is important! Because
// they are packed enums and their contiguous bits
// will directly be copied into the memory. And the
// order has to be the same as mapped in mapPdos() in
// ConfigureParameters.cpp

/*!
 * Standard Rx PDO type.
 */
struct RxPdoStandard {
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

/*!
 * CSP Rx PDO type.
 */
struct RxPdoCSP {
  int32_t targetPosition_;
  int32_t positionOffset_;
  int16_t torqueOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

/*!
 * CST Rx PDO type.
 */
struct RxPdoCST {
  int16_t targetTorque_;
  int16_t torqueOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

/*!
 * CSV Rx PDO type.
 */
struct RxPdoCSV {
  int32_t targetVelocity_;
  int32_t velocityOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

// Mixed operation mode for CST and CSP
struct RxPdoCSTCSP {
  int16_t targetTorque_;
  int16_t torqueOffset_;
  int32_t targetPosition_;
  int32_t positionOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

// Mixed operation mode for CST, CSP and CSV
struct RxPdoCSTCSPCSV {
  int16_t targetTorque_;
  int16_t torqueOffset_;
  int32_t targetPosition_;
  int32_t positionOffset_;
  int32_t targetVelocity_;
  int32_t velocityOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

struct RxPdoPVM {
  uint16_t controlWord_;
  int32_t targetVelocity_;
  uint32_t profileAccel_;
  uint32_t profileDeccel_;
  int16_t motionProfileType_;
} __attribute__((packed));

}  // namespace nanotec
