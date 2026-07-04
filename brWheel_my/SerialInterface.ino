#include "Config.h"
#include "debug.h"

//--------------------------------------------------------------------------------------------------------

u8 toUpper(u8 c) {
  if ((c >= 'a') && (c <= 'z'))
    return (c + 'A' - 'a');
  return (c);
}

// dustin's rig, added - shared by the 'O' (new CPR) and 'G' (new rotation degrees) handlers,
// which used to carry two identical copies of this read-angle/rescale/rewrite-encoder block
// (saves a good chunk of flash). Keeps the current wheel angle while ROTATION_DEG/CPR change.
static void reindexEncoder(int16_t newDeg, int32_t newCpr) {
  s32 enc;
#ifdef USE_AS5600
#ifdef USE_TCA9548
  TcaChannelSel(baseTCA0, 0); // milos, select 1st i2C channel for AS5600(0x36) on x-axis
#endif // end of tca
  enc = as5600x.getCumulativePosition() - ROTATION_MID;
#else // if no as5600
#ifdef USE_QUADRATURE_ENCODER
  enc = myEnc.Read() - ROTATION_MID + brWheelFFB.offset;
#else // milos, if no digital encoders
  enc = 0;
#endif // end of quad enc
#endif // end of as5600
  f32 wheelAngle = f32(enc) * f32(ROTATION_DEG) / f32(ROTATION_MAX); // milos, current wheel angle
  ROTATION_DEG = newDeg;
  CPR = newCpr;
  ROTATION_MAX = int32_t(f32(CPR) / 360.0 * f32(ROTATION_DEG));
  ROTATION_MID = ROTATION_MAX >> 1;
  enc = int32_t(wheelAngle * f32(ROTATION_MAX) / f32(ROTATION_DEG)); // milos, here we recover the old wheel angle
#ifdef USE_AS5600
#ifdef USE_TCA9548
  TcaChannelSel(baseTCA0, 0);
#endif // end of tca
  as5600x.resetCumulativePosition(enc + ROTATION_MID);
#ifdef USE_TCA9548
  TcaChannelSel(baseTCA0, 1); // milos, select 2nd i2C channel for AS5600(0x36) on y-axis
  as5600y.resetCumulativePosition(enc + ROTATION_MID);
#endif // end of tca
#else // if no as5600
#ifdef USE_QUADRATURE_ENCODER
  myEnc.Write(enc + ROTATION_MID - brWheelFFB.offset);
#endif // end of quad enc
#endif // end of as5600
}

void configCDC() { // milos, virtual serial port firmware configuration interface
  if (CONFIG_SERIAL.available() > 0) {
    u8 c = toUpper(CONFIG_SERIAL.read());
    //DEBUG_SERIAL.println(c);
    s32 temp;
    u8 ffb_temp;
    switch (c) {
      case 'U': { // milos, send all firmware settings
          // dustin's rig - one space-separated loop instead of 17 print pairs (every value here
          // is non-negative, so printing via u32 emits the exact same characters as before)
          const u32 uVals[] = {
            (u32)ROTATION_DEG, configGeneralGain, configDamperGain, configFrictionGain,
            configConstantGain, configPeriodicGain, configSpringGain, configInertiaGain,
            configCenterGain, configStopGain, MM_MIN_MOTOR_TORQUE, LC_scaling,
            effstate, MM_MAX_MOTOR_TORQUE, (u32)CPR, pwmstate,
            axisInvertMask, axisDisableMask // dustin's rig, added - always present now (see USE_AXIS_TWEAKS removal note)
          };
          for (uint8_t i = 0; i < sizeof(uVals) / sizeof(uVals[0]); i++) {
            if (i) CONFIG_SERIAL.print(' ');
            CONFIG_SERIAL.print(uVals[i]);
          }
          CONFIG_SERIAL.println();
          break;
        }
      case 'V':
        CONFIG_SERIAL.print("fw-v");
        CONFIG_SERIAL.print(FIRMWARE_VERSION, DEC);
        // milos, firmware options
        // dustin's rig - the option letters are known at compile time, so adjacent string
        // literals concatenate into ONE flash string and one print call (was ~20 calls)
        CONFIG_SERIAL.print(
#ifdef USE_AUTOCALIB
          "a"
#endif
#ifdef USE_TWOFFBAXIS
          "b"
#endif
#ifdef USE_ZINDEX
          "z"
#endif
#ifdef USE_CENTERBTN
          "c"
#endif
#ifndef USE_QUADRATURE_ENCODER
          "d" // milos, if no optical encoder
#endif
#ifdef USE_AS5600
          "w"
#endif
#ifdef USE_TCA9548
          "u" // milos, if tca9548
#endif
#ifdef USE_HATSWITCH
          "h"
#endif
#ifdef USE_ADS1015
          "s"
#endif
#ifdef AVG_INPUTS
          "i"
#endif
#ifdef USE_BTNMATRIX
          "t"
#endif
#ifdef USE_XY_SHIFTER
          "f"
#endif
#ifdef USE_EXTRABTN
          "e"
#endif
#ifdef USE_SPLITAXIS
          "k"
#endif
#ifdef USE_ANALOGFFBAXIS
          "x"
#endif
#ifdef USE_SHIFT_REGISTER
          "n"
#endif
#ifdef USE_SN74ALS166N
          "r"
#endif
#ifdef USE_LOAD_CELL
          "l"
#endif
#ifdef USE_MCP4725
          "g"
#endif
#ifndef USE_EEPROM
          "p"
#endif
          // no 'v' letter anymore - axis invert/disable is always compiled in, not a build option
          "\r\n");
        break;
      case 'X': // dustin's rig, added - CI-injected build number (see FW_BUILD_ID in Config.h), for the control panel to compare against the latest GitHub release
        CONFIG_SERIAL.println(FW_BUILD_ID, DEC);
        break;
      case 'T': // dustin's rig, added - freeze-under-load diagnostics: reset cause + worst-ever loop() gap, see brWheel_my.ino
        CONFIG_SERIAL.print(mcusrMirror, HEX);
        CONFIG_SERIAL.print(' ');
        CONFIG_SERIAL.println(maxLoopGapUs);
        break;
      case 'S':
        CONFIG_SERIAL.println(brWheelFFB.state, DEC);
        break;
      case 'R':
        brWheelFFB.calibrate();
        break;
      case 'B': // milos, added to adjust brake load cell pressure
        ffb_temp = CONFIG_SERIAL.parseInt();
        LC_scaling = constrain(ffb_temp, 1, 255);
        CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_BRK_PRES, LC_scaling); // milos, update EEPROM
        break;
      case 'P': // milos, added to recalibrate pedals
#ifdef USE_AUTOCALIB
        accel.min = Z_AXIS_LOG_MAX, accel.max = 0;
        brake.min = Z_AXIS_LOG_MAX, brake.max = 0;
        clutch.min = RX_AXIS_LOG_MAX, clutch.max = 0;
        hbrake.min = RY_AXIS_LOG_MAX, hbrake.max = 0;
        CONFIG_SERIAL.println(1);
#else // if manual calib
        CONFIG_SERIAL.println(0);
#endif // end of autocalib
        break;
      case 'O': // milos, added to adjust optical encoder CPR
        temp = CONFIG_SERIAL.parseInt();
        temp = constrain(temp, 4, 600000); // milos, extended to 32bit (100000*6)
        reindexEncoder(ROTATION_DEG, temp); // dustin's rig - shared helper, see above
        CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_ENC_CPR, CPR); // milos, update EEPROM
        break;
      case 'C':
#ifdef USE_ZINDEX
        brWheelFFB.offset = ROTATION_MID - myEnc.Read();
        //CONFIG_SERIAL.println(brWheelFFB.offset); // milos, saving some bytes in flash memory
        CONFIG_SERIAL.println(1);
#else // if no zindex
#ifdef USE_AS5600 // milos, with AS5600
#ifdef USE_TCA9548
        TcaChannelSel(baseTCA0, 0); // milos, select 1st i2C channel for AS5600(0x36) on x-axis
#endif // end of tca
        as5600x.resetCumulativePosition(ROTATION_MID);
#ifdef USE_TCA9548
        TcaChannelSel(baseTCA0, 1); // milos, select 2nd i2C channel for AS5600(0x36) on y-axis
        as5600y.resetCumulativePosition(ROTATION_MID);
#endif // end of tca
#else // milos, if no as5600
#ifdef USE_QUADRATURE_ENCODER
        myEnc.Write(ROTATION_MID); // milos, just set to zero angle
#endif // end of quad enc
#endif // end of use as5600
        CONFIG_SERIAL.println(0);
#endif // end of use z index
        break;
      case 'Z': // milos, hard reset the z-index offset
#ifdef USE_ZINDEX
        brWheelFFB.offset = 0;
        SetParam(PARAM_ADDR_ENC_OFFSET, brWheelFFB.offset); // milos, update EEPROM right away
        CONFIG_SERIAL.println(1);
#else // if no zindex
        CONFIG_SERIAL.println(0);
#endif // end of zindex
        break;
      case 'G': // milos, set new rotation angle
        temp = CONFIG_SERIAL.parseInt();
        temp = constrain(temp, 30, 1800); // milos
        reindexEncoder(temp, CPR); // dustin's rig - shared helper, see above
        CONFIG_SERIAL.println(1);
        //SetParam(PARAM_ADDR_ROTATION_DEG, temp);// milos, update EEPROM
        break;
      case 'E': //milos, added - turn desktop effects and ffb monitor on/off
        // dustin's rig - the old constrain(0,255) + 8x bitWrite loop was byte-for-byte
        // identical to a plain u8 assignment
        effstate = CONFIG_SERIAL.parseInt();
        CONFIG_SERIAL.println(effstate, BIN);
        break;
      case 'I': // dustin's rig, added - invert axes: bit0=X(steering),bit1=Y(brake),bit2=Z(throttle),bit3=RX(clutch),bit4=RY(handbrake); use 'A' command to save to EEPROM
        ffb_temp = CONFIG_SERIAL.parseInt();
        axisInvertMask = constrain(ffb_temp, 0, 31);
        CONFIG_SERIAL.println(axisInvertMask, BIN);
        break;
      case 'D': // dustin's rig, added - disable (force to neutral) axes: same bit layout as 'I'; use 'A' command to save to EEPROM
        ffb_temp = CONFIG_SERIAL.parseInt();
        axisDisableMask = constrain(ffb_temp, 0, 31);
        CONFIG_SERIAL.println(axisDisableMask, BIN);
        break;
      case 'W': //milos, added - configure PWM settings and frequency
#ifdef USE_EEPROM
        pwmstate = CONFIG_SERIAL.parseInt(); // dustin's rig - same u8 assignment as the old constrain+bit-copy loop
        SetParam(PARAM_ADDR_PWM_SET, pwmstate); // milos, update EEPROM with new pwm settings
        temp = calcTOP(pwmstate) * minTorquePP; // milos, recalculate new min torque for curent min torque %
        {
          u16 minTorq16 = (u16)constrain(temp, 0, 65535); // milos, fixed - PARAM_ADDR_MIN_TORQ is a 2-byte EEPROM slot (MM_MIN_MOTOR_TORQUE is uint16_t); writing the raw s32 'temp' here used to overwrite the adjacent PARAM_ADDR_MAX_TORQ bytes
          MM_MIN_MOTOR_TORQUE = minTorq16; // milos, added - apply at runtime too (was previously only written to EEPROM, so it only took effect after a reboot)
          SetParam(PARAM_ADDR_MIN_TORQ, minTorq16); // milos, update min torque in EEPROM
        }
        ReconfigurePWMMode(); // milos, fixed - apply the new PWM mode/frequency immediately; previously this required a manual Arduino restart because Timer1/Timer3 registers were only ever configured once at boot
        CONFIG_SERIAL.println(calcTOP(pwmstate));
        //CONFIG_SERIAL.println(1);
#else // milos, if no eeprom we can't configure pwm settings durring runtime, but we can set it manualy in setup loop
        CONFIG_SERIAL.println(0);
#endif // end of eeprom
        break;
      case 'H': // milos, added - configure the XY shifter calibration
#ifdef USE_XY_SHIFTER
        c = toUpper(CONFIG_SERIAL.read());
        // dustin's rig - 'A'..'E' had identical parse/constrain/assign bodies, they now index
        // shifter.cal[] directly; 'F'/'G'/'R' keep their own paths
        if (c >= 'A' && c <= 'E') {
          temp = CONFIG_SERIAL.parseInt();
          shifter.cal[c - 'A'] = constrain(temp, 0, 1023);
          CONFIG_SERIAL.println(1);
        } else if (c == 'F') {
          temp = CONFIG_SERIAL.parseInt();
          shifter.cfg = constrain(temp, 0, 255);
          CONFIG_SERIAL.println(1);
        } else if (c == 'G') {
          for (uint8_t i = 0; i < 5; i++) {
            CONFIG_SERIAL.print(shifter.cal[i]);
            CONFIG_SERIAL.print(' ');
          }
          CONFIG_SERIAL.println(shifter.cfg);
        } else if (c == 'R') {
          CONFIG_SERIAL.print(shifter.x);
          CONFIG_SERIAL.print(' ');
          CONFIG_SERIAL.println(shifter.y);
        }
#else // if no xy shifter
        CONFIG_SERIAL.read(); // dustin's rig, fixed - consume the subcommand char: it used to stay in the RX buffer and then executed as a NEW command (e.g. "HG" left a dangling 'G' = set rotation degrees, with parseInt timing out to 0 -> constrained to 30 deg!)
        CONFIG_SERIAL.println(0);
#endif // end of xy shifter
        break;
      case 'Y': // milos, added - configure manual calibration for pedals
#ifndef USE_AUTOCALIB // milos, if manual pedal calibration
        c = toUpper(CONFIG_SERIAL.read());
        // dustin's rig - 8 identical parse/constrain/assign bodies collapsed into one shared
        // body behind a letter->target-pointer switch ('R' readout stays its own path)
        if (c == 'R') {
          const s16 yVals[] = { brake.min, brake.max, accel.min, accel.max, clutch.min, clutch.max, hbrake.min, hbrake.max };
          for (uint8_t i = 0; i < sizeof(yVals) / sizeof(yVals[0]); i++) {
            if (i) CONFIG_SERIAL.print(' ');
            CONFIG_SERIAL.print(yVals[i]);
          }
          CONFIG_SERIAL.println();
        } else {
          s16* cal = NULL;
          switch (c) {
            case 'A': cal = &brake.min; break;
            case 'B': cal = &brake.max; break;
            case 'C': cal = &accel.min; break;
            case 'D': cal = &accel.max; break;
            case 'E': cal = &clutch.min; break;
            case 'F': cal = &clutch.max; break;
            case 'G': cal = &hbrake.min; break;
            case 'H': cal = &hbrake.max; break;
          }
          if (cal != NULL) {
            temp = CONFIG_SERIAL.parseInt();
            *cal = constrain(temp, 0, 4095);
            CONFIG_SERIAL.println(1);
          }
        }
#else // if autocalib
        CONFIG_SERIAL.read(); // dustin's rig, fixed - same as 'H' above: a dangling 'R' from "YR" used to trigger the 'R' command = motor calibration run!
        CONFIG_SERIAL.println(0);
#endif // end of autocalib
        break;
      /*case 'Q': //milos, read and print out EEPROM contents
        uint8_t temp;
        for (uint8_t i = 0; i < 64; i++) {
          for (uint8_t j = 0; j < 16; j++) {
            GetParam(i * 16 + j, temp);
            if (j == 0) {
              Serial.print("0x");
              if (i < 16) Serial.print("0");
              Serial.print(i, HEX);
              Serial.print(": ");
            }
            if (temp < 16) Serial.print("0");
            if (j < 15) {
              Serial.print(temp, HEX);
              Serial.print(" ");
              if (j == 7) Serial.print(" ");
            } else {
              Serial.println(temp, HEX);
            }
          }
        }
        break;*/
      /*case 'W': //milos, clear EEPROM contents
        ClearEEPROMConfig();
        Serial.println("EEPROM cleared");
        break;*/
      case 'A': //milos, save all firmware settings in EEPROM
#ifdef USE_EEPROM
        SaveEEPROMConfig ();
        CONFIG_SERIAL.println(1);
#else // if no eeprom
        CONFIG_SERIAL.println(0);
#endif // end of eeprom
        break;
      case 'F': { // milos, FFB gain knobs
          c = toUpper(CONFIG_SERIAL.read());
          if (c == 'J') { // milos, min torque needs its own scaling math
            ffb_temp = CONFIG_SERIAL.parseInt();
            ffb_temp = constrain(ffb_temp, 0, 255); //milos
            minTorquePP = (f32)ffb_temp * 0.001; // milos, max is 25.5% or 0.255
            MM_MIN_MOTOR_TORQUE = (u16)(minTorquePP * (f32)MM_MAX_MOTOR_TORQUE); // milos, we can set it during run time
            CONFIG_SERIAL.println(1);
          } else {
            // dustin's rig - the 9 gain subcommands had identical parse/assign bodies; one
            // shared body behind a letter->gain-pointer switch. ffb_temp is u8, so the old
            // constrain(ffb_temp, 0, 255) was a no-op after the same u8 truncation.
            u8* gain = NULL;
            switch (c) {
              case 'G': gain = &configGeneralGain; break;
              case 'C': gain = &configConstantGain; break;
              case 'D': gain = &configDamperGain; break;
              case 'F': gain = &configFrictionGain; break;
              case 'S': gain = &configPeriodicGain; break;
              case 'M': gain = &configSpringGain; break;
              case 'I': gain = &configInertiaGain; break;
              case 'A': gain = &configCenterGain; break;
              case 'B': gain = &configStopGain; break;
            }
            if (gain != NULL) {
              *gain = CONFIG_SERIAL.parseInt();
              CONFIG_SERIAL.println(1);
            }
          }
          break;
        }
    }
  }
}
