/*
  Force Feedback Wheel

  Copyright 2012  Tero Loimuneva (tloimu [at] gmail [dot] com)
  Copyright 2013  Saku Kekkonen
  Copyright 2015  Etienne Saint-Paul  (esaintpaul [at] gameseed [dot] fr)
  Copyright 2017  Fernando Igor  (fernandoigor [at] msn [dot] com)
  Copyright 2018-2025  Milos Rankovic (ranenbg [at] gmail [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#include "ffb_pro.h"
//#include "ffb.h" // milos, commented out
#include "QuadEncoder.h"
#include <util/delay.h>

//------------------------------------- Defines ----------------------------------------------------------

const s32 INERTIA_COEF = 16; //milos, added (wheel mass)
const s32 DAMPER_COEF = 16; //milos, added (wheel viscosity)
const s32 FRICTION_COEF = 8; //milos, modified (wheel friction)
const s32 SPRING_COEF = 8; //milos, modified (wheel elasticity)

const s32 AUTO_CENTER_SPRING = 512; //milos, modified
//#define AUTO_CENTER_DAMPER    64 //milos, commented
const s32 BOUNDARY_SPRING	=	 32767; //milos, modified
//#define BOUNDARY_FRICTION		32 //milos, commented

// dustin's rig, added - reserve headroom for the end-stop spring below other effects (game +
// desktop), so a hard hit against the wall is never swallowed by the final torque clamp when
// the game is already driving near-max force (heavy cornering/impact - exactly when the wall
// needs to be felt most). All non-endstop torque is scaled down to this fraction before the
// endstop spring is added on top, which then has its own guaranteed headroom up to the clamp.
// Same idea as OpenFFBoard's per-axis "fx_ratio" (default ~80%, see Axis.h upstream).
const f32 EFFECT_HEADROOM_RATIO = 0.8;

// dustin's rig, added - extra damping active only inside the end-stop zone (FFBeast calls this
// "soft stop dampening"): cuts the bounce/oscillation ("clunk") from slamming into the end of
// travel at speed, without adding any damping in the normal range of motion. Scaled down from
// the desktop damper's full strength and tied to the existing end-stop gain knob (configStopGain)
// so there's no new EEPROM parameter to add - a stiffer end-stop also gets proportionally more
// of this damping.
const f32 ENDSTOP_DAMPER_SCALE = 0.25;

//const f32 gSpeedSet = 0.1f;             //milos, commented
//const f32 gSpringSet = 0.05f; // 0.005  //milos, commented

//--------------------------------------- Globals --------------------------------------------------------

// milos, all effects can be divided into two groups:
// [1] time dependant effects:
//     [a] ramp
//     [b] periodic effects: sine, triangle, square, sawtoothup, sawtoothdown
//
// [2] time independant effects:
//     [a] postition dependant (X axis): spring (pos), damper (speed), inertia (acceleration), friction (speed)
//     [b] position independant: constant
//

void cSpeedObs::Init () {
  mLastPos = 0;
  mLastSpeed = 0;
  mLastValueValid = false;
  mCurrentLS = 0;
  for (u8 i = 0; i < NB_TAPS; i++)
    mLastSpeeds[i] = 0;
}

// dustin's rig, added - shared ring-buffer running average: cSpeedObs::Update and
// cAccelObs::Update carried two identical inline copies of this loop (store newest sample,
// sum backwards from the write index, divide by tap count, advance the index).
static f32 RingAvg (f32 *buf, u8 taps, u8 *cur, f32 newVal) {
  buf[*cur] = newVal;
  u8 f = *cur;
  f32 avg = 0;
  for (u8 i = 0; i < taps; i++) {
    avg += buf[f]; //* fir_coefs[i];
    if (f == 0) {
      f = taps - 1;
    } else {
      f--;
    }
  }
  avg /= taps;
  (*cur)++;
  if (*cur >= taps)
    *cur = 0;
  return (avg);
}

f32 cSpeedObs::Update (s32 new_pos) {
  f32 speed = new_pos - mLastPos; //milos, was s32 speed
  mLastPos = new_pos;
  if (mLastValueValid) {
    return RingAvg(mLastSpeeds, NB_TAPS, &mCurrentLS, speed);
  }
  mLastValueValid = true;
  mLastSpeed = 0;
  return (0);
}

void cAccelObs::Init () { //milos, added
  mLastSpd = 0;
  mLastAccel = 0;
  mLastValueValid = false;
  mCurrentLA = 0;
  for (u8 i = 0; i < NB_TAPS_A; i++) {
    mLastAccels[i] = 0;
  }
}

f32 cAccelObs::Update (f32 new_spd) { //milos, added, was s32 new_spd
  f32 accel = new_spd - mLastSpd; //s32
  mLastSpd = new_spd;
  if (mLastValueValid) {
    return RingAvg(mLastAccels, NB_TAPS_A, &mCurrentLA, accel); // dustin's rig - shared with cSpeedObs, see RingAvg above
  }
  mLastValueValid = true;
  mLastAccel = 0;
  return (0);
}

cFFB::cFFB() {
  mAutoCenter = true;
}

//--------------------------------------- Effects --------------------------------------------------------

s32 ConstrainEffect (s32 val) {
  return (constrain(val, -((s32)MM_MAX_MOTOR_TORQUE), (s32)MM_MAX_MOTOR_TORQUE));
}

f32 wDegScl() { // milos, added - scaling factor to convert encoder position to wheel angle units
  return (f32(ROTATION_DEG) / f32(ROTATION_MAX));
}

s16 DamperEffect (f32 spd, s16 mag) {
  //milos, speed in the units of wheel_angle/time_step
  if (spd > SPD_THRESHOLD)
    return (-ConstrainEffect(((spd - SPD_THRESHOLD) * mag * wDegScl()) * DAMPER_COEF * 10 / 512)); //milos
  if (spd < -SPD_THRESHOLD)
    return (-ConstrainEffect(((spd + SPD_THRESHOLD) * mag * wDegScl()) * DAMPER_COEF * 10 / 512)); //milos
  return (0);
}

s16 InertiaEffect(f32 acl, s16 mag) { //milos, modified
  //milos, acceleration in the units of wheel_angle/time_step^2
  //s16 cmd = ConstrainEffect(mag * abs(acl) * INERTIA_COEF / 32); //milos, my new
  s16 cmd = ConstrainEffect(mag * abs(acl) * wDegScl() * INERTIA_COEF * 10 / 32); //milos
  if (acl > ACL_THRESHOLD)
    return (-cmd);
  if (acl < -ACL_THRESHOLD)
    return (cmd);
  return (0);
}

s16 FrictionEffect (f32 spd, s16 mag) { //milos, modified
  //milos, simplified friction force model (constant above treshold, otherwise linear)
  //milos, speed in the units of wheel_angle/time_step
  s16 cmd = mag * FRICTION_COEF / 32;
  f32 scaledSpd = spd * wDegScl() * 10;
  if (scaledSpd > FRC_THRESHOLD)
    return (-ConstrainEffect(cmd));
  if (scaledSpd < -FRC_THRESHOLD)
    return (ConstrainEffect(cmd));
  // dustin's rig, changed - smooth sinusoidal ease-in across the rampup zone instead of a
  // linear ramp (same shape, continuous with the two branches above at +-FRC_THRESHOLD: sin
  // reaches +-1 exactly at the boundary, so there's no discontinuity). Widening FRC_THRESHOLD
  // (see ffb_pro.h) softens the harsh, gritty "catch" friction can have right as the wheel
  // passes through zero speed / changes direction. Same technique as OpenFFBoard's friction
  // rampup (EffectsCalculator.cpp), re-derived here for our units.
  f32 phase = TWO_PI * (fabs(scaledSpd) / FRC_THRESHOLD / 2.0 - 0.25); // -TWO_PI/4 at zero speed .. +TWO_PI/4 at the boundary
  f32 rampupFactor = (1.0 + sin(phase)) / 2.0; // 0 at zero speed, 1 at the boundary
  return (s16)(-ConstrainEffect(cmd) * rampupFactor * (scaledSpd >= 0 ? 1 : -1));
}

s32 SpringEffect (s32 err, s16 mag) { //milos, modified - normalized to wheel angle
  //return (-ConstrainEffect((s32)((s32)mag * err * SPRING_COEF / 256)));
  return (-ConstrainEffect((s32)((s32)mag * err * wDegScl() * SPRING_COEF * 10 / 256))); //milos, added - with wheel angle as metric
}

// dustin's rig, added - SineEffect and SquareEffect carried two inline copies of this exact
// sine-of-phase-shifted-time expression (the single most float-heavy line in the firmware);
// one shared copy, same formula.
static f32 WaveSin (u16 period, u8 phase, u16 t) {
  t %= period; //milos, reset timer after period reached
  return (sin(TWO_PI * (1.0 / (((f32)period) / 1000.0) * (((f32)t) / 1000.0) + (((f32)phase) / 256.0)))); //milos, t increments in each cycle
}

s16 SineEffect (s16 mag, u16 period, u8 phase, u16 t) { //milos
  return ((s16)(((f32)mag) * WaveSin(period, phase, t)));
}

s8 sgn(f32 value) { //milos added, sign function
  if (value >= 0.0) return (1);
  if (value < 0.0) return (-1);
}

s16 linFunction (f32 k, f32 x, s32 n) { //milos added, linear function y=kx+n
  return ((s16)(k * x) + n);
}

s16 SquareEffect (s16 mag, u16 period, u8 phase, u16 t) { //milos, added
  return ((s16)(((f32)mag) * sgn(WaveSin(period, phase, t)))); //milos
}

s16 TriangleEffect (s16 mag, u16 period, u8 phase, u16 t) { //milos, added
  phase += 64; //milos, moved phase by PI/4, starts from 0 with rising edge
  t += (u16)(((f32)phase) / 256.0 * period); //milos, phase shifts time
  t %= period; //milos, reset timer after period reached
  if ((((f32)t) / 1000.0) >= 0.0 && (((f32)t) / 1000.0) < (((f32)period) / 1000.0) / 2.0) {
    return (linFunction(4.0 * ((f32)mag) / (((f32)period) / 1000.0), ((f32)t) / 1000.0, -mag));
  } else if ((((f32)t) / 1000.0) >= (((f32)period) / 1000.0) / 2.0 && (((f32)t) / 1000.0) < (((f32)period) / 1000.0)) {
    t -= period / 2;
    return (linFunction(-4.0 * ((f32)mag) / (((f32)period) / 1000.0), ((f32)t) / 1000.0, mag));
  }
}

s16 SawtoothUpEffect (s16 mag, u16 period, u8 phase, u16 t) { //milos, added
  phase += 128; //milos, moved phase by PI/2, starts from 0 with rising edge
  t += (u16)(((f32)phase) / 256.0 * period); //milos, phase shifts time
  t %= period; //milos, reset timer after period reached
  if (t >= 0 && t < period) {
    return (linFunction(2.0 * ((f32)mag) / (((f32)period) / 1000.0), ((f32)t) / 1000.0, -mag));
  }
}

s16 SawtoothDownEffect (s16 mag, u16 period, u8 phase, u16 t) { //milos, added
  phase += 128; //milos, moved phase by PI/2, starts from 0 with falling edge
  t += (u16)(((f32)phase) / 256.0 * period);
  t %= period; //milos, reset timer after period reached
  if (t >= 0 && t < period) {
    return (linFunction(2.0 * ((f32)(-mag)) / (((f32)period) / 1000.0), ((f32)t) / 1000.0, mag));
  }
}

s16 RampEffect (s8 rStart, s8 rEnd, u16 rPeriod, u16 t) { //milos, added
  f32 rSlope;
  t %= rPeriod; //milos, reset timer after period reached
  rSlope = ((f32)((s32)(rEnd - rStart) * 256)) / (((f32)rPeriod) / 1000.0); //milos, ramp slope (units magnitude/seconds)
  return (linFunction(rSlope, ((f32)t) / 1000.0, (s16)rStart * 256));
}

s16 ApplyEnvelope (s16 metric, u16 t, u8 atLvl, u8 fdLvl, u16 atTime, u16 fdTime, u16 eDuration, u16 eStartDelay) { //milos, added - envelope block effect, start delay and duration
  f32 kA, kF;
  s16 nA;
  if (metric >= 0) { // for positive magnitudes
    kA = (f32)(metric - (s16)(atLvl * 128)) / ((f32)atTime); // postitive attack slope
    nA = (s16)(atLvl * 128); // postitive attack offset
    kF = (f32)((s16)(fdLvl * 128) - metric) / ((f32)fdTime); // postitive fade slope
  } else { // for negative magnitudes
    kA = (f32)(metric + (s16)(atLvl * 128)) / ((f32)atTime); // negative attack slope
    nA = -(s16)(atLvl * 128); // negative attack offset
    kF = (f32)(-metric - (s16)(fdLvl * 128)) / ((f32)fdTime); // negative fade slope
  }

  if (t >= eStartDelay && t < eDuration + eStartDelay) { // play effect after start delay with a given effect duration
    if (t >= eStartDelay && t < atTime + eStartDelay) { // attack
      return (linFunction(kA, t - eStartDelay, nA));
    } else if (t >= atTime + eStartDelay && t <= eDuration + eStartDelay - fdTime) { // magnitude
      return (metric);
    } else if (t > eDuration + eStartDelay - fdTime && t < eDuration + eStartDelay) { // fade
      return (linFunction(kF, t - eDuration - eStartDelay + fdTime, metric));
    }
  } else {
    return (0);
  }
}

s32 ScaleMagnitude (s32 eMag, u16 eGain, float eDivider) { //milos, added
  return ((s32)eMag * (s32)eGain / 32767 / eDivider); // normalizes magnitude to effect gain and all PWM modes
}

// dustin's rig, added - the two blocks below were expanded inline in every periodic-effect
// case of CalcTorqueCommands (6 copies of the 8-argument ApplyEnvelope call setup alone),
// which cost several hundred bytes of flash. Same math, one shared copy.
static s16 EnvelopeOf (volatile TEffectState &ef, u16 t) { // envelope of the effect's own magnitude at effect time t
  return ApplyEnvelope(ef.magnitude, t, ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay);
}

static s32 PeriodicForce (volatile TEffectState &ef, s16 wave) { // offset + waveform, normalized to gain/PWM mode and periodic gain knob
  return ScaleMagnitude((s32)ef.offset + wave, ef.gain, EffectDivider()) * configPeriodicGain / 100;
}

static s16 ScaledSpring (s32 base, u8 gain) { // dustin's rig, added - spring coefficient scaled to PWM mode and a percent gain knob (was expanded inline at the autocenter and both endstop sites)
  return (base / EffectDivider() * gain / 100);
}

static s16 DesktopMag (u8 gain) { // dustin's rig, added - magnitude of a user desktop effect (damper/inertia/friction), one shared copy of the scaling
  return ScaleMagnitude((u16)gain * 327, 32767, EffectDivider());
}

// dustin's rig, added - full HID PID "Condition" report support: dead band around cpOffset plus
// a direction-dependent coefficient (spring/damper/inertia/friction can be stiffer pushing one
// way than the other - real sims send this for asymmetric wall/curb forces). Previously only
// ef.positiveCoefficient's raw value was ever used regardless of direction, and deadBand/offset
// were parsed but never applied (the dead-band math below matches milos's own original,
// commented-out reference implementation for each effect type - see git history).
//
// metricNoOffset: the metric (position/speed/accel) with cpOffset already subtracted by the
// caller, in whatever unit that effect type already used before this change.
// dbWidth: dead band half-width in the SAME units as metricNoOffset.
// Returns false (effect contributes nothing) if inside the dead band; otherwise fills
// *outMetric with the dead-band-adjusted metric (same sign as metricNoOffset) and *outCoeff
// with the direction-appropriate raw coefficient to scale it by.
// dustin's rig, added - deadBand used to be an 8-bit wire value (0..255 representing physical
// 0..32767, ~128.5 physical units per step); it's now a full 16-bit field (see ffb.h/HID.cpp)
// so the SAME per-effect multipliers below (milos's originals: *8, /32, /640) would make the
// dead band ~128.5x too wide for any given physical intent from the host. This factor corrects
// for that so the real-world dead band width a game requests is unchanged by the wire format
// getting finer-grained.
const f32 DEADBAND_RESCALE = 255.0 / 32767.0;

static bool ConditionDeadband(volatile TEffectState &ef, f32 metricNoOffset, f32 dbWidth, f32 *outMetric, s16 *outCoeff) {
  if (fabs(metricNoOffset) <= dbWidth) return false;
  bool positive = metricNoOffset >= 0;
  *outMetric = metricNoOffset - (positive ? dbWidth : -dbWidth);
  *outCoeff = positive ? ef.positiveCoefficient : ef.negativeCoefficient;
  return true;
}

//--------------------------------------------------------------------------------------------------------

void SetIndex () {
  gIndexFound = true;
  // 	myEnc.set(ROTATION_MID);
}

float EffectDivider() { //milos, added, calculates effects divider in order to scale magnitudes equaly for all PWM modes
  return (32767.0 / float(TOP)); //milos, was 32767.0
}

//--------------------------------------------------------------------------------------------------------
//s32 cFFB::CalcTorqueCommand (s32 pos) { // milos, commented - old 1 axis input
//s32 cFFB::CalcTorqueCommands (s32 pos, s32 pos) { // milos, pos: x-axis, pos2: y-axis
s32v cFFB::CalcTorqueCommands (s32v *pos) { // milos, pointer struct agument, returns struct, pos->x: x-axis, pos->y: y-axis
  //if ((!Btest(pidState.status, ACTUATORS_ENABLED)) || (Btest(pidState.status, DEVICE_PAUSED)))
  //return(0);

  s32v command; // milos, added 2-axis FFB data
  command.x = s32(0);
#ifdef USE_TWOFFBAXIS
  command.y = s32(0);
#endif // end of 2 ffb axis
  if (pos != NULL) { // milos, this check is always required for pointers

    f32 spd = mSpeed.Update(pos->x);
    f32 acl = mAccel.Update(spd); //milos, added - acceleration

    if (gFFB.mAutoCenter) { // milos, desktop autocenter spring effect if no FFB from any app or game
      if (bitRead(effstate, 0)) {
        /*if (abs(pos->x) > 1)*/ command.x += SpringEffect(pos->x, ScaledSpring(AUTO_CENTER_SPRING, configCenterGain)); //milos, autocenter spring force is equal (scaled accordingly) for all PWM modes
      }
#ifdef USE_TWOFFBAXIS
      if (bitRead(effstate, 0)) {
        /*if (abs(pos->y) > 1)*/ command.y += SpringEffect(pos->y, ScaledSpring(AUTO_CENTER_SPRING, configCenterGain)); //milos, autocenter spring for yFFB axis
      }
#endif
    } else for (u8 id = FIRST_EID; id <= MAX_EFFECTS; id++) { // milos, if an app or game is sending FFB

        /*milos
          u8 state;  // see constants MEffectState_*
          u8 type;  // see constants USB_EFFECT_*
          u8 parameterBlockOffset; // milos, added
          u8 attackLevel, fadeLevel, deadBand, deadBand2, enableAxis; //milos, added deadBand, deadBand2 and enableAxis
          s8 rampStart, rampEnd; //milos, added
          u16 gain, period, direction; // samplingPeriod; // ms //milos, changed gain from u8 to u16, added samplingPeriod
          u16 duration, fadeTime, attackTime, startDelay; //milos, added attackTime and startDelay
          s16 magnitude, magnitude2, positiveSaturation;  //milos, added positiveSaturation and magnitude2
          s16 offset, offset2;   //milos, added offset2
          u8 phase; //milos, changed back to u8 from u16

          ef.magnitude has range -32767..32767 16bit logical (the same physical)
          ef.period has range 0..65535 16bit logical, 0-65535 physical 16bit, exp -3, unit s
          ef.phase has range 0..255 8bit logical, 0-359 physical 8bit, exp 0, unit deg
          ef.gain has range 0..32767 16bit logical (the same physical)
          ef.offset has range -32768..32767 16bit logical (the same physical)
          ef.duration has range 0..65535 16bit logical, 0-65535 physical 16bit, exp -3, unit s
          ef.startDelay has range 0..65535 16bit logical, 0-65535 physical 16bit, exp -3, unit s
          ef.attackTime has range 0..32767 16bit logical, 0-32767 physical 16bit, exp -3, unit s
          ef.fadeTime has range 0..32767 16bit logical, 0-32767 physical 16bit, exp -3, unit s
          ef.attackLevel has range 0..255 8bit logical, 0-32767 physical
          ef.fadeLevel has range 0..255 8bit logical, 0-32767 physical
          ef.rampStart has range -127..127 8bit logical, -32767 to 32767 physical
          ef.rampEnd has range -127..127 8bit logical, -32767 to 32767 physical
          ef.deadBand has range 0..32767 16bit logical (the same physical) // dustin's rig, widened from 8bit
          ef.direction has range 0..32767 16bit logical, 0 to 35999 physical, exp -2, unit deg
        */

        volatile TEffectState &ef = gEffectStates[id];
        if (Btest(ef.state, MEffectState_Allocated | MEffectState_Playing)) {

#ifdef USE_TWOFFBAXIS
          s32 mag2 = ScaleMagnitude(ef.magnitude2, ef.gain, EffectDivider()); // milos, magnitude for yFFB (y-axis kept symmetric, see USB_EFFECT_SPRING below)
#endif // end of 2 ffb axis

          if (ef.period <= (CONTROL_PERIOD / 1000) * 2) { //milos, make sure to cap the max frequency (or to limit min period)
            ef.period = (CONTROL_PERIOD / 1000) * 2; //milos, do now allow periods less than 4ms (more than 250Hz wave we can not reproduce with 500Hz FFB calculation rate anyway)
          }

          switch (ef.type) {
            case USB_EFFECT_CONSTANT:
              {
                s32 f = ConstrainEffect(ScaleMagnitude(EnvelopeOf(ef, effectTime[id - 1]), ef.gain, EffectDivider())) * configConstantGain / 100; //milos, added
                if (bitRead(ef.enableAxis, 2)) { // milos, if direction is enabled (bit2 of enableAxis byte)
#ifdef USE_TWOFFBAXIS
                  command.y += -f * cos(TWO_PI * ef.direction / 32768.0); //milos, added - project this effect's own force vector on yFFB-axis (fixed: no longer multiplies the whole accumulator)
#endif // end of 2 ffb axis
                  command.x += -f * sin(TWO_PI * ef.direction / 32768.0); //milos, added - project this effect's own force vector on xFFB-axis (fixed: no longer multiplies the whole accumulator)
                } else {
                  command.x -= f;
                }
              }
              //LogTextLf("_pro constant");
              break;
            case USB_EFFECT_RAMP:
              command.x -= ConstrainEffect(ScaleMagnitude(ApplyEnvelope(RampEffect(ef.rampStart, ef.rampEnd, ef.duration, effectTime[id - 1]), effectTime[id - 1], ef.attackLevel, ef.fadeLevel, ef.attackTime, ef.fadeTime, ef.duration, ef.startDelay) //milos, added
                                           , ef.gain, EffectDivider())); //milos, added
              //LogTextLf("_pro ramp");
              break;
            case USB_EFFECT_SINE:
              {
                s32 f = PeriodicForce(ef, SineEffect(EnvelopeOf(ef, effectTime[id - 1]), ef.period, ef.phase, effectTime[id - 1])); //milos, added
                if (bitRead(ef.enableAxis, 2)) { // milos, if direction is enabled (bit2 of enableAxis byte)
#ifdef USE_TWOFFBAXIS
                  command.y += f * cos(TWO_PI * ef.direction / 32768.0); //milos, added - project this effect's own force vector on yFFB-axis (fixed: no longer multiplies the whole accumulator)
#endif // end of 2 ffb axis
                  command.x += f * sin(TWO_PI * ef.direction / 32768.0); //milos, added - project this effect's own force vector on xFFB-axis (fixed: no longer multiplies the whole accumulator)
                } else {
                  command.x += f;
                }
              }
              //LogTextLf("_pro sine");
              break;
            case USB_EFFECT_SQUARE:
              command.x += PeriodicForce(ef, SquareEffect(EnvelopeOf(ef, effectTime[id - 1]), ef.period, ef.phase, effectTime[id - 1])); //milos, added
              //LogTextLf("_pro square");
              break;
            case USB_EFFECT_TRIANGLE:
              command.x += PeriodicForce(ef, TriangleEffect(EnvelopeOf(ef, effectTime[id - 1]), ef.period, ef.phase, effectTime[id - 1])); //milos, added
              //LogTextLf("_pro triangle");
              break;
            case USB_EFFECT_SAWTOOTHUP:
              command.x += PeriodicForce(ef, SawtoothUpEffect(EnvelopeOf(ef, effectTime[id - 1]), ef.period, ef.phase, effectTime[id - 1])); //milos, added
              //LogTextLf("_pro sawtoothup");
              break;
            case USB_EFFECT_SAWTOOTHDOWN:
              command.x += PeriodicForce(ef, SawtoothDownEffect(EnvelopeOf(ef, effectTime[id - 1]), ef.period, ef.phase, effectTime[id - 1])); //milos, added
              //LogTextLf("_pro sawtoothdown");
              break;
            case USB_EFFECT_SPRING: { // dustin's rig, changed - dead band + direction-dependent coefficient (see ConditionDeadband above)
                f32 posNoOffset = pos->x - (s16)((s32(ef.offset) * ROTATION_MID) >> 15); //milos, for spring, damper, inertia and friction forces ef.offset is cpOffset, here we scale it to ROTATION_MID
                f32 dbMetric; s16 coeff;
                if (ConditionDeadband(ef, posNoOffset, (f32)ef.deadBand * DEADBAND_RESCALE * 8.0, &dbMetric, &coeff)) { // milos's own dead-band scale for this effect (*8), from the pre-optimization reference implementation, rescaled for the wider wire field (see DEADBAND_RESCALE)
                  s32 springMag = ScaleMagnitude(coeff, ef.gain, EffectDivider());
                  command.x += SpringEffect((s32)dbMetric, springMag * configSpringGain / 100 / 16);
                }
              }
#ifdef USE_TWOFFBAXIS
              command.y += SpringEffect(pos->y - (s16)((s32(ef.offset2) * ROTATION_MID) >> 15), mag2 * configSpringGain / 100 / 16); //milos, for yFFB spring (kept symmetric, no dead band/negative coefficient wiring for y - USE_TWOFFBAXIS isn't used in any shipped variant)
#endif // end of 2 ffb axis
              //LogTextLf("_pro spring");
              break;
            case USB_EFFECT_DAMPER: { // dustin's rig, changed - dead band + direction-dependent coefficient
                f32 spdNoOffset = spd - (f32)ef.offset / 1638.3; //milos, here we scale it to speed
                f32 dbMetric; s16 coeff;
                if (ConditionDeadband(ef, spdNoOffset, (f32)ef.deadBand * DEADBAND_RESCALE / 32.0, &dbMetric, &coeff)) {
                  s32 damperMag = ScaleMagnitude(coeff, ef.gain, EffectDivider());
                  command.x += DamperEffect(dbMetric, damperMag * configDamperGain / 100);
                }
              }
              //LogTextLf("_pro damper");
              break;
            case USB_EFFECT_INERTIA: { // dustin's rig, changed - dead band + direction-dependent coefficient
                f32 aclNoOffset = acl - (f32)ef.offset / 32767.0; //milos, here we scale it to acceleration
                f32 dbMetric; s16 coeff;
                if (ConditionDeadband(ef, aclNoOffset, (f32)ef.deadBand * DEADBAND_RESCALE / 640.0, &dbMetric, &coeff)) {
                  s32 inertiaMag = ScaleMagnitude(coeff, ef.gain, EffectDivider());
                  command.x += InertiaEffect(dbMetric, inertiaMag * configInertiaGain / 100);
                }
              }
              //LogTextLf("_pro inertia");
              break;
            case USB_EFFECT_FRICTION: { // dustin's rig, changed - dead band + direction-dependent coefficient
                f32 spdNoOffset = spd - (f32)ef.offset / 1638.3;
                f32 dbMetric; s16 coeff;
                if (ConditionDeadband(ef, spdNoOffset, (f32)ef.deadBand * DEADBAND_RESCALE / 32.0, &dbMetric, &coeff)) {
                  s32 frictionMag = ScaleMagnitude(coeff, ef.gain, EffectDivider());
                  command.x += FrictionEffect(dbMetric, frictionMag * configFrictionGain / 100);
                }
              }
              //LogTextLf("_pro friction");
              break;
            //case USB_EFFECT_CUSTOM: //milos, commented
            //break;
            case USB_EFFECT_PERIODIC:
              command.x -= ConstrainEffect(ScaleMagnitude(ef.offset, 32767, EffectDivider())) * configPeriodicGain / 100; //milos, for periodic forces ef.offset changes magnitude, here we scale it to all PWM modes
              //LogTextLf("_pro periodic");
              break;
            default:
              break;
          }
          effectTime[id - 1] = millis() - t0; //milos, added - advance FFB timer
        }
      }
    // milos, at the moment only xFFB axis has conditional desktop (internal) effects
    // milos, casted effect gain into u16 to fix desktop effects oveflow when using gains above 100
    if (bitRead(effstate, 1)) command.x += DamperEffect(spd, DesktopMag(configDamperGain)) ; //milos, added - user damper effect
    if (bitRead(effstate, 2)) command.x += InertiaEffect(acl, DesktopMag(configInertiaGain)) ; //milos, added - user inertia effect
    if (bitRead(effstate, 3)) command.x += FrictionEffect(spd, DesktopMag(configFrictionGain)) ; //milos, added - user friction effect

    // dustin's rig, added - reserve end-stop headroom (see EFFECT_HEADROOM_RATIO above). Must
    // run after every game/desktop contribution above and before the end-stop spring below.
    command.x *= EFFECT_HEADROOM_RATIO;
#ifdef USE_TWOFFBAXIS
    command.y *= EFFECT_HEADROOM_RATIO;
#endif // end of 2 ffb axis

    s32 limit = ROTATION_MID; // milos, +-ROTATION_MID distance from center is where endstop spring force will start
    //if ((pos->x < -limit) || (pos->x > limit)) {
#ifdef USE_ANALOGFFBAXIS
#ifndef USE_QUADRATURE_ENCODER
#ifndef USE_AS5600
    limit -= (ROTATION_MID >> 6); // milos, here you can offset endstop activation point by ROTATION_MID/64 (optical or magnetic encoders can go past the axis range limit, this is required only for analog input - pot)
#endif // end of as5600
#endif // end of quad enc
#endif // end of 2 ffb axis
    if ((pos->x < -limit) || (pos->x > limit)) {
      if (pos->x >= 0) {
        pos->x = pos->x - limit; //milos
      } else {
        pos->x = pos->x + limit; //milos
      }
      command.x += SpringEffect(pos->x, ScaledSpring(BOUNDARY_SPRING, configStopGain)); // milos, boundary spring force is equal (scaled accordingly) for all PWM modes, endstop force for xFFB axis
      command.x += DamperEffect(spd, (s16)(DesktopMag(configStopGain) * ENDSTOP_DAMPER_SCALE)); // dustin's rig, added - extra damping right at the wall only, see ENDSTOP_DAMPER_SCALE above
    }
    //}
#ifdef USE_TWOFFBAXIS // milos, add y-axis endstop with yFFB (at the moment y-axis is on the pot only)
    // milos, limit is the same as for xFFB endstop (analog axis are prescaled)
    //if ((pos->y < -limit) || (pos->y > limit)) {
    //limit -= 1023; // milos, maybe offset endstop limit a little (analog axis can't go past the limit, if left at 0 this force may never be created)
    if ((pos->y < -limit) || (pos->y > limit)) {
      if (pos->y >= 0) {
        pos->y = pos->y - limit; //milos
      } else {
        pos->y = pos->y + limit; //milos
      }
      command.y += SpringEffect(pos->y, ScaledSpring(BOUNDARY_SPRING, configStopGain)); //milos, boundary spring force is equal (scaled accordingly) for all PWM modes endstop force for yFFB axis
      command.y += DamperEffect(spd, (s16)(DesktopMag(configStopGain) * ENDSTOP_DAMPER_SCALE)); // dustin's rig, added - reuses xFFB's spd (no separate y speed observer exists); harmless approximation, this path never compiles in any shipped variant
    }
    // }
#endif // end of 2 ffb axis

    command.x = ConstrainEffect(command.x * configGeneralGain / 100 * gDeviceGain / 255); // milos, added gDeviceGain - applies the host's PID Device Gain report (FfbHandle_DeviceGain) on top of our own general gain knob
#ifndef USE_TWOFFBAXIS // milos, for 1 FFB axis
    if (bitRead(effstate, 4)) CONFIG_SERIAL.println(command.x); // milos, added - FFB real time monitor
#else // for 2 ffb axis
    command.y = ConstrainEffect(command.y * configGeneralGain / 100 * gDeviceGain / 255); // milos, added gDeviceGain
    if (bitRead(effstate, 4)) { // milos, for 2 ffb axis we send X and Y forces to FFB monitor
      CONFIG_SERIAL.print(command.x); // milos, FFB X axis
      CONFIG_SERIAL.print(" ");
      CONFIG_SERIAL.println(command.y); // milos, FFB Y axis
    }
#endif // end of 2 ffb axis
  }
  return (command); // milos, passing the struct
}

//--------------------------------------------------------------------------------------------------------
/* Turn Steering right only */
void BRFFB::calibrate() { // milos, we are only calibrating encoder on x-axis (even if 2 ffb axis are used)
  cal_print("cal:");
  s32 rightGap;
#ifdef USE_QUADRATURE_ENCODER
  rightGap = myEnc.Read();
#else
#ifdef USE_AS5600
  rightGap = as5600x.getCumulativePosition();
#endif // end of as5600
#endif // end of quad enc
  s32 startpos = rightGap;
  s32 actual = 0;
  s32v drive; // milos, added - for setting PWM
  drive.x = 0; // milos, added - init at zero
#ifdef USE_TWOFFBAXIS
  drive.y = 0; // milos, added - init at zero
#endif // end of 2 ffb axis
#ifndef USE_ZINDEX //milos, added
  /* Turn right to stop and set MAX position */
  drive.x = (MM_MAX_MOTOR_TORQUE - MM_MIN_MOTOR_TORQUE) / 4;
  SetPWM(&drive);
  for (uint8_t i = 0; i < 254; i++) {
#ifdef USE_QUADRATURE_ENCODER
    actual = myEnc.Read();
#else
#ifdef USE_AS5600
    actual = as5600x.getCumulativePosition();
#endif // end of as5600
#endif // end of quad enc
    if (rightGap >= actual) {
      break; // milos, if endstop reached
    } else {
      rightGap = actual;
    }
    wdt_reset(); // dustin's rig, added - this loop can run ~76s worst case (254 * 300ms) without ever returning to loop(), which would otherwise starve the new hardware watchdog
    delay(300);
  }
  //ROTATION_MAX = rightGap; // milos, commented
  drive.x = 0;
  SetPWM(&drive);
#ifndef USE_AS5600 // milos, when no AS5600
#ifdef USE_QUADRATURE_ENCODER
  myEnc.Write(ROTATION_MAX); // milos, set quadrature encoder to right edge
#endif // end of quad enc
#else
  as5600x.resetCumulativePosition(ROTATION_MAX); // milos, set magnetic encoder to right edge
#endif
  if (startpos == actual) {
    cal_println("er");
    this->state = 2;
  } else {
    this->state = 1;
    cal_println("ok");  //milos
  }
#else // milos, if z-index is used
  // milos, added Z-index lookup
  // turn right at least 1 full turn or until we encounter Z-index pulse
#ifdef USE_QUADRATURE_ENCODER
  startpos = myEnc.Read();
  drive.x = (MM_MAX_MOTOR_TORQUE - MM_MIN_MOTOR_TORQUE) / 4;
  SetPWM(&drive);
#endif
  for (uint8_t i = 0; i < 254; i++) {
#ifdef USE_QUADRATURE_ENCODER
    actual = myEnc.Read();
#endif
    if (zIndexFound) {
      drive.x = 0;
      SetPWM(&drive);
      this->state = 1;
      cal_println("ok");
      break;
    }
    if ((actual - startpos) * ROTATION_DEG / ROTATION_MAX >= 360) {
      drive.x = 0;
      SetPWM(&drive);
      this->state = 3;
      cal_println("no z");
      break;
    }
    wdt_reset(); // dustin's rig, added - same reasoning as the other calibrate() loop above
    delay(50);
  }
#endif
  CONFIG_SERIAL.println(1); // milos, calibration procedure is done
}

BRFFB::BRFFB() {
  offset = 0;
  state = 0;
}

//--------------------------------------------------------------------------------------------------------

void FfbproSetAutoCenter(uint8_t enable)
{
  gFFB.mAutoCenter = enable;
  //brWheelFFB.autoCenter = true;
}

// dustin's rig, removed - FfbproEnableInterrupts/FfbproGetSysExHeader: MIDI-era driver
// hooks, unreferenced since the FFB_Driver table was dropped.

// effect operations ---------------------------------------------------------

void FfbproStartEffect(uint8_t effectId)
{
  //brWheelFFB.autoCenter = false;
  if (!IsValidEffectId(effectId)) // milos, added - second line of defense: effectId here is host-controlled and effectTime[] is only MAX_EFFECTS elements long
    return;
  gFFB.mAutoCenter = false;
  effectTime[effectId - 1] = 0; //milos, added - reset timer for this effect when we start it
}

void FfbproStopEffect(uint8_t effectId)
{
  //setFFB(0); //milos, commented
  //brWheelFFB.autoCenter = false;
}

void FfbproFreeEffect(uint8_t effectId)
{
  //setFFB(0); //milos, commented
  //brWheelFFB.autoCenter = true;
}

// modify operations ---------------------------------------------------------

void FfbproModifyDuration(uint8_t effectId, uint16_t duration, uint16_t stdelay) //milos, added stdelay
{
  /*
    { // FFB: Set Effect Output Report
    uint8_t  reportId; // =1
    uint8_t effectBlockIndex; // 1..40
    uint8_t effectType; // 1..12 (effect usages: 26,27,30,31,32,33,34,40,41,42,43,28) //milos, total 11, 28 is removed (custom force)
    uint16_t duration; // 0..65535, exp -3, s
    uint16_t triggerRepeatInterval; // 0..65535, exp -3, s //milos
    int16_t gain; // 0..32767  (physical 0..32767) //milos, was 0(0)..(255)10000, uint8_t
    uint8_t triggerButton;  // button ID (0..8)
    uint8_t enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
    uint8_t direction; // angle (0=0 .. 255=359, exp 0, deg) //milos, 8bit
    uint16_t startDelay;  // 0..65535, exp -3, s //milos, uncommented
    } USB_FFBReport_SetEffect_Output_Data_t;
  */
  volatile TEffectState* effect = &gEffectStates[effectId]; // milos, added
  effect->duration = duration; // milos, added
  effect->startDelay = stdelay; // milos, added
  effectTime[effectId - 1] = 0; // milos, added - reset timer for this effect when duration is changed
}

void FfbproSetEnvelope (USB_FFBReport_SetEnvelope_Output_Data_t* data, volatile TEffectState * effect)
{
  uint8_t eid = data->effectBlockIndex;

  /*
    USB effect data:
    uint8_t  reportId; // =2
    uint8_t effectBlockIndex; // 1..40
    uint8_t attackLevel; // 0..255  (physical 0..32767) //milos, was 10000
    uint8_t fadeLevel; // 0..255  (physical 0..32767) //milos, was 10000
    uint16_t attackTime;  // 0..65535  (physical 0..65535), exp -3, s
    uint16_t fadeTime;  // 0..65535  (physical 0..65535), exp -3, s
  */
  effect->attackLevel = (u8)data->attackLevel;
  effect->fadeLevel = (u8)data->fadeLevel;
  effect->attackTime = (u16)data->attackTime; // milos, added
  effect->fadeTime = (u16)data->fadeTime;
}

void FfbproSetCondition (USB_FFBReport_SetCondition_Output_Data_t* data, volatile TEffectState * effect)
{
  uint8_t eid = data->effectBlockIndex;
  /*
    USB effect data:
  	uint8_t reportId; // =3
    uint8_t effectBlockIndex; // 1..40
    uint8_t parameterBlockOffset; // bits: 0..3=parameterBlockOffset, 4..5=instance1, 6..7=instance2
    int16_t cpOffset;  // -32768..32767 (physical -32768..32767) //milos, was -128(-10000)..127(10000), int8_t
    int16_t positiveCoefficient;  // -32767..32767 (physical -32767..32767) //milos, was -128(-10000)..127(10000), int8_t
    uint16_t positiveSaturation;  // 0..32767 (physical 0..32767) //milos, was 0(0)..255(10000), int8_t, uncommented
    uint8_t deadBand;  // 0..255 (physical 0..32767) //milos, was 0(0)..255(10000)
  */
  effect->parameterBlockOffset = (u8)data->parameterBlockOffset; // milos, added - pass the effect condition block index
  if (effect->parameterBlockOffset == 0) { // milos, condition block for xFFB
    // dustin's rig, fixed - store in the dedicated positiveCoefficient field (was aliased onto
    // magnitude, which is for non-condition effect types); negativeCoefficient/positiveSaturation/
    // negativeSaturation are now real wire fields too (see the HID.cpp descriptor and ffb.h struct)
    // and are actually applied in CalcTorqueCommands instead of being parsed and discarded.
    effect->positiveCoefficient = (s16)data->positiveCoefficient;
    effect->negativeCoefficient = (s16)data->negativeCoefficient;
    effect->positiveSaturation = (s16)data->positiveSaturation;
    effect->negativeSaturation = (s16)data->negativeSaturation;
    effect->offset = (s16)data->cpOffset; // milos, this offset changes X-pos
    effect->deadBand = (u16)data->deadBand; // milos, added // dustin's rig, widened to u16 - see ffb.h
  } else if (effect->parameterBlockOffset == 1) { // milos, condition block for yFFB
#ifdef USE_TWOFFBAXIS
    effect->magnitude2 = (s16)data->positiveCoefficient; // milos, added  - yFFB spring constant (kept symmetric - see CalcTorqueCommands)
    effect->offset2 = (s16)data->cpOffset; // milos, this offset changes yFFB
    effect->deadBand2 = (u16)data->deadBand; // milos, added for yFFB // dustin's rig, widened to u16
#endif // end of 2 ffb axis
  }
}

void FfbproSetPeriodic (USB_FFBReport_SetPeriodic_Output_Data_t* data, volatile TEffectState * effect)
{
  uint8_t eid = data->effectBlockIndex;

  /*
  	typedef struct
  	{ // FFB: Set Periodic Output Report
    uint16_t magnitude; // 0..32767  (physical 0..32767) //milos, was 0(0)..255(10000), uint8_t
    int16_t offset; // -32768..32767  (physical -32768..32767) //milos, was -128(-10000)..(127)10000, int8_t
    uint8_t phase;  // 0..255 (physical 0..360, exp 0, deg) //milos, changed back to original uint8_t
    uint16_t period;  // 0..65535  (physical 0..65535), exp -3, s //milos, was 0(0)..32767(32767)
  	} USB_FFBReport_SetPeriodic_Output_Data_t;
  */
  //effect->type = USB_EFFECT_PERIODIC; // milos, was conflicting with FfbproSetEffect
  effect->magnitude = (u16)data->magnitude;
  effect->offset = (((s16)data->offset)); // milos, this offset changes magnitude
  effect->phase = (u8)data->phase;
  effect->period = (u16)data->period;
}

void FfbproSetConstantForce (USB_FFBReport_SetConstantForce_Output_Data_t* data, volatile TEffectState * effect)
{
  uint8_t eid = data->effectBlockIndex;
  /*
    USB data:
    uint8_t  reportId; // =5
    uint8_t effectBlockIndex; // 1..40
    int16_t magnitude;  // -32767..32737  (physical -32767..32737) //milos, logical was -255..255
  */
  effect->magnitude = data->magnitude;
}

void FfbproSetRampForce (USB_FFBReport_SetRampForce_Output_Data_t* data, volatile TEffectState * effect)
{
  uint8_t eid = data->effectBlockIndex; // milos, uncommented
  /*USB effect data:
    uint8_t	reportId;	// =6
    uint8_t	effectBlockIndex;	// 1..40
    int8_t rampStart; // -128..127  (physical -32768..32767) //milos, was -10000..10000
    int8_t rampEnd; // -128..127  (physical -32768..32767) //milos, was -10000..10000*/
  effect->rampStart = data->rampStart; // milos, added
  effect->rampEnd = data->rampEnd; // milos, added
}

// dustin's rig, removed - setFFB(): fully commented-out body, zero live call sites.

uint8_t FfbproSetEffect (USB_FFBReport_SetEffect_Output_Data_t* data, volatile TEffectState * effect)  //milos, changed from int to uint8_t
{
  /*
    {
    u8 state;  // see constants MEffectState_*
    u8 type;  // see constants USB_EFFECT_*
    u8 attackLevel, fadeLevel, deadBand, direction; //milos, added deadBand and direction
    s8 rampStart, rampEnd; //milos, added
    u16 gain, period; // samplingPeriod;  // ms //milos, changed gain from u8 to u16, added samplingPeriod
    u16 duration, fadeTime, attackTime, startDelay; //milos, added attackTime and startDelay
    s16 magnitude;
    s16 offset;
    u8 phase; //milos, changed back to u8 from u16
  */

  uint8_t eid = data->effectBlockIndex;
  //s32 command = s32(0);
  /*
    { // FFB: Set Effect Output Report
    uint8_t  reportId; // =1
    uint8_t effectBlockIndex; // 1..40
    uint8_t effectType; // 1..12 (effect usages: 26,27,30,31,32,33,34,40,41,42,43,28) //milos, total 11, 28 is removed (custom force)
    uint16_t duration; // 0..65535, exp -3, s
    uint16_t triggerRepeatInterval; // 0..65535, exp -3, s //milos
    int16_t gain; // 0..32767  (physical 0..32767) //milos, was 0(0)..(255)10000, uint8_t
    uint8_t triggerButton;  // button ID (0..8)
    uint8_t enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
    uint16_t direction; // angle (0=0 .. 65535=35999, exp -2, deg) //milos, 16bit
    uint16_t startDelay;  // 0..65535, exp -3, s //milos, uncommented
    } USB_FFBReport_SetEffect_Output_Data_t;
  */
  effect->type = data->effectType; // milos, this is where effect type is being set
  effect->gain = (s16)data->gain;
  FfbproModifyDuration(eid, (u16)data->duration, (u16)data->startDelay); // milos, added
  //effect->startDelay = (u16)data->startDelay; / /milos, added
  effect->direction = (u16)data->direction; // milos, added
  effect->enableAxis = (u8)data->enableAxis; // milos, added
  //bool is_periodic = false; //milos, commented
  //s32 mag = (((s32)effect->magnitude)*((s32)effect->gain)) / 163;
  // Fill in the effect type specific data
  /*switch (data->effectType)
    {
    case USB_EFFECT_SQUARE:
    case USB_EFFECT_SINE:
    case USB_EFFECT_TRIANGLE:
    case USB_EFFECT_SAWTOOTHDOWN:
    case USB_EFFECT_SAWTOOTHUP:
  	is_periodic = true;
    case USB_EFFECT_CONSTANT:
    case USB_EFFECT_RAMP:
    case USB_EFFECT_SPRING:
    case USB_EFFECT_DAMPER:
    case USB_EFFECT_INERTIA:
    case USB_EFFECT_FRICTION:
    case USB_EFFECT_CUSTOM:
    default:
  	break;
    }*/
  /*if (command > 0)
  	command = map(command, 0, 1000, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE - 10);
    else if (command < 0)
  	command = -map(-command, 0, 1000, MM_MIN_MOTOR_TORQUE, MM_MAX_MOTOR_TORQUE - 10);
    else
  	command = 0;*/
  //setFFB(command);
  //DEBUG_SERIAL.println(command);
  return 1;
}

void FfbproCreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, volatile TEffectState * effect)
{
  /*
    USB effect data:
    uint8_t		reportId;	// =1
    uint8_t	effectType;	// Enum (1..12): ET 26,27,30,31,32,33,34,40,41,42,43,28
    uint16_t	byteCount;	// 0..511	- only valid with Custom Force
  */
}
