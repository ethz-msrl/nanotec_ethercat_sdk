#include "nanotec_ethercat_sdk/Nanotec.hpp"
#include "nanotec_ethercat_sdk/ObjectDictionary.hpp"

namespace nanotec {
// Print errors
void Nanotec::addErrorToReading(const ErrorType& errorType) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  reading_.addError(errorType);
}

/*
** Print error code
** See firmware documentation for meaning
*/
void Nanotec::printErrorCode() {
  
  uint16_t errorcode = 0;
  bool error_read_success =
      sendSdoRead(OD_INDEX_ERROR_CODE, 0x00, false, errorcode);
  if (error_read_success) {
    MELO_ERROR_STREAM("[nanotec_ethercat_sdk:Nanotec::printErrorCode] "
                      << "Error code: " << std::hex << errorcode);
  } else {
    MELO_ERROR_STREAM(
        "[nanotec_ethercat_sdk:Nanotec::printErrorCode] read error code "
        "uncessuful.")
  }
}

}  // namespace nanotec
