#include "Config.h"
#ifdef USE_EEPROM
#include <EEPROM.h> // milos, re-implemented
#endif

//-------------------------------------------------------------------------------------------------------

void getParam (u16 offset, u8 *addr_to, u8 size) {
#ifdef USE_EEPROM
  for (u8 i = 0; i < size; i++) {
    addr_to[i] = EEPROM.read(offset + i);
  }
#endif
}

void setParam (u16 offset, u8 *addr_to, u8 size) {
#ifdef USE_EEPROM
  for (u8 i = 0; i < size; i++) {
    //EEPROM.write(offset + i, addr_to[i]);
    EEPROM.update(offset + i, addr_to[i]); //milos, re-write only when neccessary
  }
#endif
}

// dustin's rig - Load/Save/SetDefault used to be ~85 hand-written Get/SetParam call sites
// (~1.3KB of flash just in call setup). They now share one PROGMEM address<->variable map;
// Load and Save iterate it in opposite directions of transfer, and SetDefault assigns the
// default values to the live variables and then saves them through the same table.
typedef struct {
  u8 addr;  // EEPROM offset (PARAM_ADDR_*)
  u8 size;  // parameter size in bytes
  u8* ptr;  // live firmware variable backing this parameter
} EEVar;

static const EEVar eeVars[] PROGMEM = {
  { PARAM_ADDR_ROTATION_DEG, sizeof(ROTATION_DEG), (u8*)&ROTATION_DEG },
#ifdef USE_ZINDEX
  { PARAM_ADDR_ENC_OFFSET, sizeof(brWheelFFB.offset), (u8*)&brWheelFFB.offset },
#endif
  { PARAM_ADDR_ENC_CPR, sizeof(CPR), (u8*)&CPR },
  { PARAM_ADDR_GEN_GAIN, 1, &configGeneralGain },
  { PARAM_ADDR_DMP_GAIN, 1, &configDamperGain },
  { PARAM_ADDR_FRC_GAIN, 1, &configFrictionGain },
  { PARAM_ADDR_CNT_GAIN, 1, &configConstantGain },
  { PARAM_ADDR_PER_GAIN, 1, &configPeriodicGain },
  { PARAM_ADDR_SPR_GAIN, 1, &configSpringGain },
  { PARAM_ADDR_INR_GAIN, 1, &configInertiaGain },
  { PARAM_ADDR_CTS_GAIN, 1, &configCenterGain },
  { PARAM_ADDR_STP_GAIN, 1, &configStopGain },
  { PARAM_ADDR_BRK_PRES, 1, &LC_scaling },
  { PARAM_ADDR_DSK_EFFC, 1, &effstate },
  { PARAM_ADDR_AXIS_INVERT, 1, &axisInvertMask },
  { PARAM_ADDR_AXIS_DISABLE, 1, &axisDisableMask },
  { PARAM_ADDR_MIN_TORQ, sizeof(MM_MIN_MOTOR_TORQUE), (u8*)&MM_MIN_MOTOR_TORQUE },
  { PARAM_ADDR_MAX_TORQ, sizeof(MM_MAX_MOTOR_TORQUE), (u8*)&MM_MAX_MOTOR_TORQUE },
  { PARAM_ADDR_MAX_DAC, sizeof(MAX_DAC), (u8*)&MAX_DAC },
  // pwmstate is in the shared table even though the old SaveEEPROMConfig skipped it: the 'W'
  // command is the only mutator and it writes EEPROM immediately, so RAM == EEPROM whenever
  // 'A' runs and the extra EEPROM.update() is a verified no-op write of the same byte.
  { PARAM_ADDR_PWM_SET, 1, &pwmstate },
#ifdef USE_XY_SHIFTER
  { PARAM_ADDR_SHFT_X0, sizeof(shifter.cal[0]), (u8*)&shifter.cal[0] },
  { PARAM_ADDR_SHFT_X1, sizeof(shifter.cal[1]), (u8*)&shifter.cal[1] },
  { PARAM_ADDR_SHFT_X2, sizeof(shifter.cal[2]), (u8*)&shifter.cal[2] },
  { PARAM_ADDR_SHFT_Y0, sizeof(shifter.cal[3]), (u8*)&shifter.cal[3] },
  { PARAM_ADDR_SHFT_Y1, sizeof(shifter.cal[4]), (u8*)&shifter.cal[4] },
  { PARAM_ADDR_SHFT_CFG, 1, &shifter.cfg },
#endif // end of xy shifter
#ifndef USE_AUTOCALIB // milos, manual min/max cal values for all pedal axis live in EEPROM
  { PARAM_ADDR_ACEL_LO, sizeof(accel.min), (u8*)&accel.min },
  { PARAM_ADDR_ACEL_HI, sizeof(accel.max), (u8*)&accel.max },
  { PARAM_ADDR_BRAK_LO, sizeof(brake.min), (u8*)&brake.min },
  { PARAM_ADDR_BRAK_HI, sizeof(brake.max), (u8*)&brake.max },
  { PARAM_ADDR_CLUT_LO, sizeof(clutch.min), (u8*)&clutch.min },
  { PARAM_ADDR_CLUT_HI, sizeof(clutch.max), (u8*)&clutch.max },
  { PARAM_ADDR_HBRK_LO, sizeof(hbrake.min), (u8*)&hbrake.min },
  { PARAM_ADDR_HBRK_HI, sizeof(hbrake.max), (u8*)&hbrake.max },
#endif // end of autocalib
};

void SetDefaultEEPROMConfig() { // milos - store default firmware settings in EEPROM
  u16 v16 = FIRMWARE_VERSION;
  SetParam(PARAM_ADDR_FW_VERSION, v16);
  s32 v32 = 0; // milos, z-index offset is always zeroed in EEPROM, even when USE_ZINDEX is off (and eeVars has no entry for it)
  SetParam(PARAM_ADDR_ENC_OFFSET, v32);
  ROTATION_DEG = 1080; //milos, default degrees of rotation
  configGeneralGain = 100;
  configConstantGain = 100;
  configPeriodicGain = 100;
  configStopGain = 100;
  configSpringGain = 100;
  configDamperGain = 50;
  configFrictionGain = 50;
  configInertiaGain = 50;
  configCenterGain = 70;
  MM_MIN_MOTOR_TORQUE = 0;
  MM_MAX_MOTOR_TORQUE = 2047; // milos, for PWM signals
  MAX_DAC = 4095; // milos, for 12bit DAC
#ifdef USE_LOAD_CELL
  LC_scaling = 45; // milos, default max brake pressure
#else
  LC_scaling = 128; // milos, ffb balance center value
#endif
  effstate = 0b00000001; // milos, autocenter spring on
  axisInvertMask = 0; // dustin's rig - no axes inverted/disabled by default
  axisDisableMask = 0;
#ifndef USE_AS5600
  CPR = 2400; // milos, default CPR value for optical encoder (this is for 600PPR)
#else
  CPR = 4096; // milos, default CPR value for 12bit magnetic encoder AS5600
#endif
#ifndef USE_MCP4725
  pwmstate = 0b00001100; // milos, PWM out enabled, fast pwm, pwm+-, 7.8kHz, TOP 11bit (2047)
#else
  pwmstate = 0b10000000; // milos, DAC out enabled, DAC+- mode
#endif
#ifdef USE_ZINDEX
  brWheelFFB.offset = 0;
#endif
#ifdef USE_XY_SHIFTER
  shifter.cal[0] = 255;
  shifter.cal[1] = 511;
  shifter.cal[2] = 767;
  shifter.cal[3] = 255;
  shifter.cal[4] = 511;
  shifter.cfg = 0; // milos, default is rev gear in 6th, no inv x-axis, no inv y-axis, no inv rev gear button
#endif // end of xy shifter
#ifndef USE_AUTOCALIB //milos, default min/max manual cal values for all pedal axis
  accel.min = 0;
  brake.min = 0;
  clutch.min = 0;
  hbrake.min = 0;
  accel.max = maxCal;
  brake.max = maxCal;
  clutch.max = maxCal;
  hbrake.max = maxCal;
#endif // end of autocalib
  SaveEEPROMConfig(); // dustin's rig - defaults now flow into EEPROM through the shared eeVars table
}

void SetEEPROMConfig() { // milos, changed FIRMWARE_VERSION to 16bit from 32bit
  u16 v16;
  GetParam(PARAM_ADDR_FW_VERSION, v16);
  if (v16 != FIRMWARE_VERSION) { // milos, first time run, or version change - set default values for safety
    //ClearEEPROMConfig(); // milos, clear EEPROM before loading defaults
    SetDefaultEEPROMConfig(); // milos, set default firmware settings
  }
}

void LoadEEPROMConfig () { //milos, added - updates all firmware parameters from EEPROM
  for (u8 i = 0; i < sizeof(eeVars) / sizeof(eeVars[0]); i++) {
    EEVar v;
    memcpy_P(&v, &eeVars[i], sizeof(v));
    getParam(v.addr, v.ptr, v.size);
  }
#ifdef USE_MCP4725
  MM_MAX_MOTOR_TORQUE = MAX_DAC;
#endif
#ifdef USE_AUTOCALIB // milos, added - reset autocalibration
  accel.min = Z_AXIS_LOG_MAX;
  accel.max = 0;
  brake.min = s16(Z_AXIS_LOG_MAX);
  brake.max = 0;
  clutch.min = RX_AXIS_LOG_MAX;
  clutch.max = 0;
  hbrake.min = RY_AXIS_LOG_MAX;
  hbrake.max = 0;
#endif
}

void SaveEEPROMConfig () { //milos, added - saves all firmware parameters in EEPROM
  for (u8 i = 0; i < sizeof(eeVars) / sizeof(eeVars[0]); i++) {
    EEVar v;
    memcpy_P(&v, &eeVars[i], sizeof(v));
    setParam(v.addr, v.ptr, v.size);
  }
}

void ClearEEPROMConfig() { //milos, added - clears EEPROM (1KB on ATmega32U4)
#ifdef USE_EEPROM
  uint8_t zero;
  zero = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    SetParam(i, zero);
  }
#endif
}

// dustin's rig, removed - dead code: SetCPR() (never called, and its wheelAngle math used
// newCpr instead of the encoder position) and myMap() (never called anywhere).
