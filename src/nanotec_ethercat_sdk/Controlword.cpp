
#include <iomanip>

#include "nanotec_ethercat_sdk/Controlword.hpp"

namespace nanotec {
std::ostream& operator<<(std::ostream& os, const Controlword& controlword) {
  using std::setfill;
  using std::setw;

  os << std::left << std::boolalpha << setw(40) << setfill('-') << "|"
     << "|\n"
     << setw(40) << setfill(' ') << "| Controlword"
     << "|\n"
     << setw(25) << setfill('-') << "|" << setw(8) << "+" << setw(7) << "+"
     << "|"
     << "\n"
     << setw(25) << setfill(' ') << "| Name"
     << "| Value | Mode |"
     << "\n"
     << setw(25) << setfill('-') << "|" << setw(8) << "+" << setw(7) << "+"
     << "|"
     << "\n"
     << setw(25) << setfill(' ') << "| switch on:"
     << "| " << setw(6) << controlword.switchOn_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| enable voltage:"
     << "| " << setw(6) << controlword.enableVoltage_ << "|" << setw(6)
     << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| quick stop:"
     << "| " << setw(6) << controlword.quickStop_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| enable operation:"
     << "| " << setw(6) << controlword.enableOperation_ << "|" << setw(6)
     << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| operation Mode Specific 0:"
     << "| " << setw(6) << controlword.operationModeSpecific0_ << "|" << setw(6) << " pp"
     << "|\n"
     << setw(25) << setfill(' ') << "| operation Mode Specific 1:"
     << "| " << setw(6) << controlword.operationModeSpecific1_ << "|" << setw(6)
     << " hm"
     << "|\n"
     << setw(25) << setfill(' ') << "| operation Mode Specific 2:"
     << "| " << setw(6) << controlword.operationModeSpecific2_ << "|" << setw(6)
     << " pp"
     << "|\n"
     << setw(25) << setfill(' ') << "| operation Mode Specific 3:"
     << "| " << setw(6) << controlword.operationModeSpecific3_ << "|" << setw(6) << " pp "
     << "|\n"
     << setw(25) << setfill(' ') << "| fault_ reset:"
     << "| " << setw(6) << controlword.faultReset_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| halt_:"
     << "| " << setw(6) << controlword.halt_ << "|" << setw(6)
     << " all "
     //  << "|\n"
     //  << setw(25) << setfill(' ') << "| endless movement_:"
     //  << "| " << setw(6) << controlword.endlessMovement_ << "|" << setw(6) <<
     //  "  pp"
     << "|\n"
     <<

      setw(25) << setfill('-') << "|" << setw(8) << "+" << setw(7) << "+"
     << "|" << std::right << std::noboolalpha;
  return os;
}

uint16_t Controlword::getRawControlword() {
  uint16_t rawControlword = 0;

  if (switchOn_) {
    rawControlword |= (1 << 0);
  }
  if (enableVoltage_) {
    rawControlword |= (1 << 1);
  }
  if (quickStop_) {
    rawControlword |= (1 << 2);
  }
  if (enableOperation_) {
    rawControlword |= (1 << 3);
  }
  if (operationModeSpecific0_) {
    rawControlword |= (1 << 4);
  }
  if (operationModeSpecific1_) {
    rawControlword |= (1 << 5);
  }
  if (operationModeSpecific2_) {
    rawControlword |= (1 << 6);
  }
  if (faultReset_) {
    rawControlword |= (1 << 7);
  }
  if (halt_) {
    rawControlword |= (1 << 8);
  }
  if (operationModeSpecific3_) {
    rawControlword |= (1 << 9);
  }

  return rawControlword;
}

void Controlword::setStateTransition2() {
  //DONE
  setAllFalse();
  switchOn_ = false;
  enableVoltage_ = true;
  quickStop_ = true;
  faultReset_ = false;
}

void Controlword::setStateTransition3() {
  //DONE
  setAllFalse();
  switchOn_ = true;
  enableVoltage_ = true;
  quickStop_ = true;
  enableOperation_ = false;
  faultReset_ = false;
}

void Controlword::setStateTransition4() {
  //DONE
  setAllFalse();
  switchOn_ = true;
  enableVoltage_ = true;
  quickStop_ = true;
  enableOperation_ = true;
  faultReset_ = false;
}

void Controlword::setStateTransition5() {
  //DONE
  setAllFalse();
  switchOn_ = true;
  enableVoltage_ = true;
  quickStop_ = true;
  enableOperation_ = false;
  faultReset_ = false;
}

void Controlword::setStateTransition6() {
  //DONE
  setAllFalse();
  switchOn_ = false;
  enableVoltage_ = true;
  quickStop_ = true;
  faultReset_ = false;
}

void Controlword::setStateTransition7() {
  //DONE
  setAllFalse();
  enableVoltage_ = false;
  faultReset_ = false;
}

void Controlword::setStateTransition8() {
  //DONE
  setAllFalse();
  switchOn_ = false;
  enableVoltage_ = true;
  quickStop_ = true;
  faultReset_ = false;
}

void Controlword::setStateTransition9() {
  //DONE
  setAllFalse();
  enableVoltage_ = false;
  faultReset_ = false;
}

void Controlword::setStateTransition10() {
  //DONE
  setAllFalse();
  enableVoltage_ = false;
  faultReset_ = false;
}

void Controlword::setStateTransition11() {
  //DONE
  setAllFalse();
  enableVoltage_ = true;
  quickStop_ = false;
  faultReset_ = false;
}

void Controlword::setStateTransition12() {
  //DONE
  setAllFalse();
  enableVoltage_ = false;
  faultReset_ = false;
}

void Controlword::setStateTransition15Low() {
  //DONE
  setAllFalse();
}

void Controlword::setStateTransition15High() {
  //DONE
  setAllFalse();
  faultReset_ = true;
}

void Controlword::setStateTransition16Low() {
  //DONE
  setAllFalse();
  switchOn_ = true;
  enableVoltage_ = true;
  quickStop_ = false;
  enableOperation_ = true;
  faultReset_ = false;
}

void Controlword::setStateTransition16High() {
  //DONE
  setAllFalse();
  switchOn_ = true;
  enableVoltage_ = true;
  quickStop_ = true;
  enableOperation_ = true;
  faultReset_ = false;
}


void Controlword::setAllFalse() {
  switchOn_ = false;
  enableVoltage_ = false;
  quickStop_ = false;
  enableOperation_ = false;
  operationModeSpecific0_ = false;
  operationModeSpecific1_ = false;
  operationModeSpecific2_ = false;
  faultReset_ = false;
  halt_ = false;
  operationModeSpecific3_ = false;
  // endlessMovement_ = false;
}

void Controlword::setControlWordOMS0() {
  operationModeSpecific0_ = true;
}

void Controlword::setControlWordOMS1() {
  operationModeSpecific1_ = true;
}

void Controlword::setControlWordOMS2() {
  operationModeSpecific2_ = true;
}

void Controlword::setControlWordOMS3() {
  operationModeSpecific3_ = true;
}

void Controlword::setControlWordHALT() {
  halt_ = true;
}


void Controlword::setInit() { setStateTransition2(); }

}  // namespace nanotec
