#include "nanotec_ethercat_sdk/ObjectDictionary.hpp"
#include <unordered_map>
/*
std::ostream& operator<<(std::ostream& os, const nanotec::od_mode_of_operation_e modeOfOperation) {
  std::unordered_map<nanotec::od_mode_of_operation_e, std::string> mode2strMap =
      {
          {nanotec::od_mode_of_operation_e::NA, "NA"},
          {nanotec::od_mode_of_operation_e::ProfilePositionMode, "ProfilePositionMode"},
          {nanotec::od_mode_of_operation_e::ProfileVelocityMode, "ProfileVelocityMode"},
          {nanotec::od_mode_of_operation_e::HomingMode, "HomingMode"},
          {nanotec::od_mode_of_operation_e::CyclicSynchronousPositionMode, "CyclicSynchronousPositionMode"},
          {nanotec::od_mode_of_operation_e::CyclicSynchronousVelocityMode, "CyclicSynchronousVelocityMode"},
          {nanotec::od_mode_of_operation_e::CyclicSynchronousTorqueMode, "CyclicSynchronousTorqueMode"},
  };
  os << mode2strMap[modeOfOperation];
  return os;
}
*/