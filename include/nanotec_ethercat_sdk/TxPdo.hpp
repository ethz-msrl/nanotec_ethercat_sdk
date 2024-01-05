/*!
 * @brief	This file contains the different Tx Pdo structs. Each struct
 * must contain a statusword_, or else the state changes won't work! Each struct
 * can contain either the actual torque or the actual current but not both.
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
 * Standard Tx Pdo type
 */
struct TxPdoStandard {
  uint16_t statusword_;
} __attribute__((packed));

/*!
 * CST Tx PDO type
 * Includes padding_ byte for firmware version 01.01.15.00 (october 2018)
 */
struct TxPdoCSP {
  uint16_t statusword_;
  int32_t actualFollowingError_;
  int16_t actualTorque_;
  int32_t actualVelocity_;
  int32_t actualPosition_;
} __attribute__((packed));

struct TxPdoCST {
  uint16_t statusword_;
  int16_t demandTorque_;
  int16_t actualTorque_;
  int32_t actualVelocity_;
  int32_t actualPosition_;
} __attribute__((packed));

struct TxPdoCSV {
  uint16_t statusword_;
  int16_t actualTorque_;
  int32_t actualVelocity_;
  int32_t actualPosition_;
} __attribute__((packed));

// Mixed operation mode for CST and CSP
struct TxPdoCSTCSP {
  uint16_t statusword_;
  int32_t actualFollowingError_;
  int16_t demandTorque_;
  int16_t actualTorque_;
  int32_t actualVelocity_;
  int32_t actualPosition_;
} __attribute__((packed));

// Mixed operation mode for CST, CSP, and CSV
struct TxPdoCSTCSPCSV {
  uint16_t statusword_;
  int32_t actualFollowingError_;
  int16_t demandTorque_;
  int16_t actualTorque_;
  int32_t actualVelocity_;
  int32_t actualPosition_;
} __attribute__((packed));

struct TxPdoPVM {
  uint16_t statusword_;
  int32_t demandVelocity_;
} __attribute__((packed));

}  // namespace nanotec
