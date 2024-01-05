#pragma once

#include <cstdint>
#include <ostream>

namespace nanotec {
// different RxPdo Types 0x1C12
enum class RxPdoTypeEnum : int8_t {
  NA = 0,
  RxPdoStandard,
  RxPdoCSP,
  RxPdoCST,
  RxPdoCSV,
  RxPdoCSTCSP,
  RxPdoCSTCSPCSV,
  RxPdoPVM
};

// different TxPdo Types 0x1C13
enum class TxPdoTypeEnum : int8_t {
  NA = -128,
  TxPdoStandard,
  TxPdoCSP,
  TxPdoCST,
  TxPdoCSV,
  TxPdoCSTCSP,
  TxPdoCSTCSPCSV,
  TxPdoPVM
};

}  // namespace nanotec

std::ostream& operator<<(std::ostream& os,
                         const nanotec::TxPdoTypeEnum& txPdoTypeEnum);
std::ostream& operator<<(std::ostream& os,
                         const nanotec::RxPdoTypeEnum& rxPdoTypeEnum);
