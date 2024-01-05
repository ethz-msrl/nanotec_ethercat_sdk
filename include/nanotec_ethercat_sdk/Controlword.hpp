
#pragma once

#include <cstdint>
#include <iostream>

namespace nanotec {
struct Controlword {
  bool switchOn_{false};                // bit 0
  bool enableVoltage_{false};           // bit 1
  bool quickStop_{false};               // bit 2
  bool enableOperation_{false};         // bit 3
  bool operationModeSpecific0_{false};  // bit 4
  bool operationModeSpecific1_{false};  // bit 5
  bool operationModeSpecific2_{false};  // bit 6
  bool faultReset_{false};              // bit 7
  bool halt_{false};                    // bit 8
  bool operationModeSpecific3_{false};  // bit 9

  /*!
   * get the control word as a 16 bit unsigned integer
   * THIS DOES NOT RESPECT THE MODE SPECIFIC OPTIONS!
   * The usually used cyclic modes do not need mode specific options.
   * @return	the raw controlword
   */
  uint16_t getRawControlword();

  /*!
   * State transition 2
   * This corresponds to a "shutdown" Controlword
   */
  void setStateTransition2();


  /*!
   * State transition 3
   * This corresponds to a "switch on" Controlword
   * Initialize current sensor. Current offset calibration.
   */
  void setStateTransition3();

  /*!
   * State transition 4
   * This corresponds to a "enable operation" Controlword
   * Enable drive function (enable current controller and, if needed, position
   * or velocity controller)
   */
  void setStateTransition4();

  /*!
   * State transition 5
   * This corresponds to a "disable operation" Controlword
   * Stop movement according to "Disable operation option code". Disable drive
   * function.
   */
  void setStateTransition5();

  /*!
   * State transition 6
   * This corresponds to a "shutdown" Controlword
   * Disable power section
   */
  void setStateTransition6();

  /*!
   * State transition 7
   * This corresponds to a "disable voltage" Controlword
   */
  void setStateTransition7();

  /*!
   * State transition 8
   * This corresponds to a "shutdown" Controlword
   * Disable power section
   */
  void setStateTransition8();

  /*!
   * State transition 9
   * This corresponds to a "disable voltage" Controlword
   */
  void setStateTransition9();

  /*!
   * State transition 10
   * This corresponds to a "disable voltage" Controlword
   */
  void setStateTransition10();

  /*!
   * State transition 11
   * This corresponds to a "quick stop" Controlword
   */
  void setStateTransition11();

  /*!
   * State transition 12
   * This corresponds to a "disable voltage" Controlword
   */
  void setStateTransition12();

  /*!
   * State transition 15
   * A fault has occurred
   * Start fault reaction. This needs a high edge on Bit 7 (Fault Reset),
   * Therefore, set faultReset_ to false first using this function. Then
   * use the setStateTransition15High() function to set the faultReset_ to true
   * and send a high edge on Bit 7.
   */
  void setStateTransition15Low();

  /*!
   * State transition 15
   * A fault has occurred
   * Start fault reaction. This needs a high edge on Bit 7 (Fault Reset),
   * Therefore, set faultReset_ to false first using setSTateTransition15Low()
   * function. Then use this function to set the faultReset_ to true
   * and send a high edge on Bit 7.
   */
  void setStateTransition15High();

  /*!
   * State transition 16
   * Enable operation after quick stop.
   * This needs a high edge on Bit 2 (Quick Stop). So set quickStop_ to false
   * first using this function. Then use the setStateTransition16High() function.
   */
  void setStateTransition16Low();

  /*!
   * State transition 16
   * Enable operation after quick stop.
   * This needs a high edge on Bit 2 (Quick Stop). So set quickStop_ to false
   * first using the setStateTransition16Low() function. Then use this function.
   * This one will turn out to be the same as making Transition 4.
   */
  void setStateTransition16High();


  /*!
   * Sets all bools of this struct to false
   */
  void setAllFalse();

  void setControlWordOMS0();
  void setControlWordOMS1();
  void setControlWordOMS2();
  void setControlWordOMS3();
  void setControlWordHALT();

  /*!
   * goes to the init state
   * Alias for state transition 2
   */
  void setInit();

  friend std::ostream& operator<<(std::ostream& os,
                                  const Controlword& controlword);
};

}  // namespace nanotec
