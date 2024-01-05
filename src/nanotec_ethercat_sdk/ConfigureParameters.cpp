#include <array>
#include <thread>

#include "nanotec_ethercat_sdk/Nanotec.hpp"
#include "nanotec_ethercat_sdk/ObjectDictionary.hpp"

namespace nanotec {

bool Nanotec::mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum) {
  uint8_t subIndex;
  bool rxSuccess = true;
  switch (rxPdoTypeEnum) {
    case RxPdoTypeEnum::RxPdoStandard: {
      MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::mapPdos] Rx Pdo: Standard Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write number of objects
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoCSP: {
      MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Position Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      // Filling the RX PDO Mapping 3 Object with the following objects:
      std::array<uint32_t, 5> objects{
          (OD_INDEX_TARGET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_OFFSET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
              sizeof(int8_t) * 8,
      };
      // Thus, specifying the data that will be sent over, and in what order.

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      // Nanotec CE allows upto 8 objects to be stored in RX PDO Mapping 3 Object.
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoCST: {
      MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Troque Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
          (OD_INDEX_TARGET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
              sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoCSV: {
      MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Veloctity Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
          (OD_INDEX_TARGET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_OFFSET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
              sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoCSTCSP: {
      MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Toruqe/Position Mixed Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 6> objects{
          (OD_INDEX_TARGET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_TARGET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_OFFSET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
              sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoCSTCSPCSV: {
      MELO_INFO_STREAM(
          "[nanotec_ethercat_sdk:Nanotec::mapPdos] Rx Pdo: "
          << "Cyclic Synchronous Toruqe/Position/Velocity Mixed Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 8> objects{
          (OD_INDEX_TARGET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_TARGET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_OFFSET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_TARGET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_OFFSET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
              sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoPVM: {
      MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::mapPdos] Rx Pdo: "
                       << "Profile Velocity Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 5> objects{
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_TARGET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_PROFILE_ACCELERATION << 16) | (0x00 << 8) |
              sizeof(uint32_t) * 8,
          (OD_INDEX_PROFILE_DECELERATION << 16) | (0x00 << 8) |
              sizeof(uint32_t) * 8,
          (OD_INDEX_MOTION_PROFILE_TYPE << 16) | (0x00 << 8) |
              sizeof(int16_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::NA:
      MELO_ERROR_STREAM(
          "[nanotec_ethercat_sdk:Nanotec::mapPdos] Cannot map "
          "RxPdoTypeEnum::NA, PdoType not configured properly");
      addErrorToReading(ErrorType::PdoMappingError);
      rxSuccess = false;
      break;
    default:  // Non-implemented type
      MELO_ERROR_STREAM(
          "[nanotec_ethercat_sdk:Nanotec::mapPdos] Cannot map unimplemented "
          "RxPdo, PdoType not configured properly");
      addErrorToReading(ErrorType::PdoMappingError);
      rxSuccess = false;
      break;
  }

  bool txSuccess = true;
  switch (txPdoTypeEnum) {
    case TxPdoTypeEnum::TxPdoStandard: {
      MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::mapPdos] Tx Pdo: "
                       << "Standard Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write number of objects
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoCSP: {
      MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Position Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 5> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_FOLLOWING_ERROR_ACTUAL_VALUE << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoCST: {
      MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Torque Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 5> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_TORQUE_DEMAND << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoCSV: {
      MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Velocity Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoCSTCSP: {
      MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Torque/Position Mixed Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 6> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_FOLLOWING_ERROR_ACTUAL_VALUE << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_TORQUE_DEMAND << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoCSTCSPCSV: {
      MELO_INFO_STREAM(
          "[nanotec_ethercat_sdk:Nanotec::mapPdos] Tx Pdo: "
          << "Cyclic Synchronous Torque/Position/Velocity Mixed Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 6> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_FOLLOWING_ERROR_ACTUAL_VALUE << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_TORQUE_DEMAND << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoPVM: {
      // (OD_INDEX_TORQUE_ACTUAL << 16) | (0x01 << 8) | sizeof(int16_t) * 8

      MELO_INFO_STREAM("[nanotec_ethercat_sdk:Nanotec::mapPdos] Tx Pdo: "
                       << "Profile Velocity Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 2> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_VELOCITY_DEMAND << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::NA:
      MELO_ERROR_STREAM(
          "[nanotec_ethercat_sdk:Nanotec::mapPdos] Cannot map "
          "TxPdoTypeEnum::NA, PdoType not configured properly");
      addErrorToReading(ErrorType::TxPdoMappingError);
      txSuccess = false;
      break;
    default:  // if any case was forgotten
      MELO_ERROR_STREAM(
          "[nanotec_ethercat_sdk:Nanotec::mapPdos] Cannot map undefined "
          "TxPdo, PdoType not configured properly");
      addErrorToReading(ErrorType::TxPdoMappingError);
      txSuccess = false;
      break;
  }
  return (txSuccess && rxSuccess);
}

bool Nanotec::configParam() {
  bool configSuccess = true;

  // Setting the interpolation time period; therefore, the cycle time.
  configSuccess &= sdoVerifyWrite(OD_INDEX_INTERPOLATION_TIME_PERIOD, 0x01, false,
                            configuration_.interpolationTimePeriodmS,
                            configuration_.configRunSdoVerifyTimeout);
  MELO_INFO("-->      Set OD_INDEX_INTERPOLATION_TIME_PERIOD in mS = %u", configuration_.interpolationTimePeriodmS);

  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_INTERPOLATION_TIME_PERIOD. - Magnitude");
    return configSuccess;
  }

  configSuccess &= sdoVerifyWrite(OD_INDEX_INTERPOLATION_TIME_PERIOD, 0x02, false,
                            static_cast<int8_t>(-3),
                            configuration_.configRunSdoVerifyTimeout); // Setting unit of interpolation time period to ms, 10^(-3)
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_INTERPOLATION_TIME_PERIOD. - Scale");
    return configSuccess;
  }

  //write Pole Pairs-->      
  configSuccess &= sdoVerifyWrite(OD_INDEX_POLE_PAIR_COUNT, 0x00, false, configuration_.polePairs, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_POLE_PAIR_COUNT.");
    return configSuccess;
  }
  MELO_INFO("-->      OD_INDEX_POLE_PAIR_COUNT = %u", configuration_.polePairs);
  

  //write Max Rated Current
  configSuccess &= sdoVerifyWrite(OD_INDEX_MAX_MOTOR_CURRENT, 0x00, false, configuration_.maxMotorCurrentmA, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_MAX_MOTOR_CURRENT.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_MAX_MOTOR_CURRENT = %u mA", configuration_.maxMotorCurrentmA);
  

  //write Rated Current
  configSuccess &= sdoVerifyWrite(OD_INDEX_MOTOR_RATED_CURRENT, 0x00, false, configuration_.ratedCurrentmA, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_MOTOR_RATED_CURRENT.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_MOTOR_RATED_CURRENT = %u mA", configuration_.ratedCurrentmA);
  

  //write Max Current - 100% with stepper motor
  configSuccess &= sdoVerifyWrite(OD_INDEX_MAX_CURRENT, 0x00, false, configuration_.maxCurrentPercentage, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_MAX_CURRENT.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_MAX_CURRENT = %u %%", configuration_.maxCurrentPercentage/10);
  
  //write Maximum Duration of Maximum Current - 100ms recomemnded
  configSuccess &= sdoVerifyWrite(OD_INDEX_I2T_PARAMETERS, 0x02, false, configuration_.I2tMaxDurationOfPeakms, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_I2T_PARAMETERS.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_I2T_PARAMETERS = %u ms", configuration_.I2tMaxDurationOfPeakms);
  
  //write Motor Type 
  // Bit 0: 1 (0: Open loop control; 1: Closed loop control)
  // Bit 1: 0 (0: V-Ctrl with S-Ramp Disabled; 1: V-Ctrl with S-Ramp Enabled)
  // Bit 2: 0 (0: Auto Brake Ctrl Disabled; 1: Auto Brake Ctrl Enabled)
  // Bit 3: 0 (0: Current reduction in open loop disabled; 1: Current reduction in open loop Enabled)
  // Bit 4: 1 (0: Auto alignment in closed loop disabled; 1: Auto alignment in closed loop Enabled)
  // Bit 5: 0 (0: M-controller disabled; 1: AM-controller enabled)
  // Bit 6: 0 (0: Motor type: stepper; 1: Motor type: BLDC)
  // Bit 7: 0 (0: Slow speed mode in closed loop disabled; 1: Slow speed mode in closed loop Enabled)
  configSuccess &= sdoVerifyWrite(OD_INDEX_MOTOR_DRIVE_SUBMODE_SELECT, 0x00, false, configuration_.motorDriveSubmode, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_MOTOR_DRIVE_SUBMODE_SELECT.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_MOTOR_DRIVE_SUBMODE_SELECT = 0x%02X", configuration_.motorDriveSubmode);
  
  //write Encoder Configuration
  // Bit 1: 0 (0: Differential encoder; 1: Single-Ended encoder)
  configSuccess &= sdoVerifyWrite(OD_INDEX_ENCODER_CONFIGURATION, 0x00, false, 0x00, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_ENCODER_CONFIGURATION.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_ENCODER_CONFIGURATION = 0x00");
  
  //write MaxMotorSpeed
  configSuccess &= sdoVerifyWrite(OD_INDEX_MAX_MOTOR_SPEED, 0x00, false, configuration_.maxMotorSpeed, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_MAX_MOTOR_SPEED.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_MAX_MOTOR_SPEED = %i", configuration_.maxMotorSpeed);
  
  //write ClockDirectionMultiplier
  configSuccess &= sdoVerifyWrite(OD_INDEX_CLOCK_DIRECTION_MULTIPLIER, 0x00, false, configuration_.clockDirectionMultiplier, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_CLOCK_DIRECTION_MULTIPLIER.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_CLOCK_DIRECTION_MULTIPLIER = %i", configuration_.clockDirectionMultiplier);

  //write ClockDirectionDivider
  configSuccess &= sdoVerifyWrite(OD_INDEX_CLOCK_DIRECTION_DIVIDER, 0x00, false, configuration_.clockDirectionDivider, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_CLOCK_DIRECTION_DIVIDER.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_CLOCK_DIRECTION_DIVIDER = %i", configuration_.clockDirectionDivider);


  //write digitalInputsControl
  uint32_t specialFunctionsEnable = ((uint32_t)configuration_.limitSwitchNegativeEn | ((uint32_t) configuration_.limitSwitchPositiveEn<<1) | ((uint32_t) configuration_.limitSwitchHomingEn<<2));
  configSuccess &= sdoVerifyWrite(OD_INDEX_DIGITAL_INPUT_CONTROL, 0x01, false, specialFunctionsEnable, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_DIGITAL_INPUT_CONTROL.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_DIGITAL_INPUT_CONTROL = %u", specialFunctionsEnable);

  ///////////////////////////HOMING
  MELO_INFO("         [HOMING]");
  //write homeOffset
  configSuccess &= sdoVerifyWrite(OD_INDEX_HOME_OFFSET, 0x00, false, configuration_.homeOffset, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_HOME_OFFSET.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_HOME_OFFSET = %i", configuration_.homeOffset);

  //write homingMethod
  configSuccess &= sdoVerifyWrite(OD_INDEX_HOMING_METHOD, 0x00, false, configuration_.homingMethod, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_HOMING_METHOD.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_HOMING_METHOD = %i", configuration_.homingMethod);
  
  //write homingSpeedSwitchSearch
  configSuccess &= sdoVerifyWrite(OD_INDEX_HOMING_SPEED, 0x01, false, configuration_.homingSpeedSwitchSearch, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_HOMING_SPEED - SWITCH SEARCH.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_HOMING_SPEED - SWITCH SEARCH = %u", configuration_.homingSpeedSwitchSearch);

  //write homingSpeedZeroSearch
  configSuccess &= sdoVerifyWrite(OD_INDEX_HOMING_SPEED, 0x02, false, configuration_.homingSpeedZeroSearch, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_HOMING_SPEED - ZERO SEARCH.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_HOMING_SPEED - ZERO SEARCH = %u", configuration_.homingSpeedZeroSearch);

  //write homingAcceleration
  configSuccess &= sdoVerifyWrite(OD_INDEX_HOMING_ACCELERATION, 0x00, false, configuration_.homingAcceleration, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_HOMING_ACCELERATION.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_HOMING_ACCELERATION = %u", configuration_.homingAcceleration);

  //write homingMinimumCurrentForBlockDetectionmA
  configSuccess &= sdoVerifyWrite(OD_INDEX_HOMING_ON_BLOCK_CONFIGURATION, 0x01, false, configuration_.homingMinimumCurrentForBlockDetectionmA, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_HOMING_ON_BLOCK_CONFIGURATION.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_HOMING_ON_BLOCK_CONFIGURATION - Current for block detection = %u", configuration_.homingMinimumCurrentForBlockDetectionmA);

  //write homingPeriodForBlockingmS
  configSuccess &= sdoVerifyWrite(OD_INDEX_HOMING_ON_BLOCK_CONFIGURATION, 0x02, false, configuration_.homingPeriodForBlockingmS, configuration_.configRunSdoVerifyTimeout);
  if (!configSuccess) {
    MELO_ERROR("Failed set OD_INDEX_HOMING_ON_BLOCK_CONFIGURATION.");
    return configSuccess;
  }
  MELO_INFO("-->      Set OD_INDEX_HOMING_ON_BLOCK_CONFIGURATION - Period of blocking = %u", configuration_.homingMinimumCurrentForBlockDetectionmA);

  //////////// Setting the correct SI Units
  configSuccess &= sdoVerifyWrite(OD_INDEX_SI_UNIT_POSITION, 0x00, false,
                            configuration_.SIUnitPosition,
                            configuration_.configRunSdoVerifyTimeout);
  
  configSuccess &= sdoVerifyWrite(OD_INDEX_SI_UNIT_VELOCITY, 0x00, false,
                            configuration_.SIUnitVelocity,
                            configuration_.configRunSdoVerifyTimeout);

  if(configSuccess){ MELO_INFO("[nanotec_ethercat_sdk::Nanotec::ConfigParam] Configuration parameters written successfully."); }
  else { MELO_ERROR("[nanotec_ethercat_sdk::Nanotec::ConfigParam] Configuration parameters could not be written successfully."); }

  return configSuccess;
}
}  // namespace nanotec
