#pragma once

namespace nanotec {
/*!
 * An enum containing all the possible Error types.
 * Note that Errors and Faults are not the same thing.
 * Errors occur during setup, configuration and SDO reading / writing
 * Faults occur during PDO communication when the drive state jumps to "FAULT".
 */
enum class ErrorType {
  ConfigurationError,
  SdoWriteError,
  SdoReadError,
  ErrorReadingError,
  SdoStateTransitionError,
  PdoMappingError,
  RxPdoMappingError,
  TxPdoMappingError,
  RxPdoTypeError,
  TxPdoTypeError,
  PdoStateTransitionError,
  ModeOfOperationError
};

}  // namespace nanotec
