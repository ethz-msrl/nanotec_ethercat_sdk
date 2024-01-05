#include "nanotec_ethercat_sdk/PdoTypeEnum.hpp"

std::ostream& operator<<(std::ostream& os,
                         const nanotec::TxPdoTypeEnum& txPdoTypeEnum) {
  switch (txPdoTypeEnum) {
    case nanotec::TxPdoTypeEnum::NA:
      os << "NA";
      break;
    case nanotec::TxPdoTypeEnum::TxPdoStandard:
      os << "TxPdoStandard";
      break;
    case nanotec::TxPdoTypeEnum::TxPdoCSP:
      os << "TxPdoCSP";
      break;
    case nanotec::TxPdoTypeEnum::TxPdoCST:
      os << "TxPdoCST";
      break;
    case nanotec::TxPdoTypeEnum::TxPdoCSV:
      os << "TxPdoCSV";
      break;
    default:
      break;
  }
  return os;
}
std::ostream& operator<<(std::ostream& os,
                         const nanotec::RxPdoTypeEnum& rxPdoTypeEnum) {
  switch (rxPdoTypeEnum) {
    case nanotec::RxPdoTypeEnum::NA:
      os << "NA";
      break;
    case nanotec::RxPdoTypeEnum::RxPdoStandard:
      os << "RxPdoStandard";
      break;
    case nanotec::RxPdoTypeEnum::RxPdoCSP:
      os << "RxPdoCSP";
      break;
    case nanotec::RxPdoTypeEnum::RxPdoCST:
      os << "RxPdoCST";
      break;
    case nanotec::RxPdoTypeEnum::RxPdoCSV:
      os << "RxPdoCSV";
      break;
    case nanotec::RxPdoTypeEnum::RxPdoPVM:
      os << "RxPdoPVM";
      break;
    default:
      break;
  }
  return os;
}
