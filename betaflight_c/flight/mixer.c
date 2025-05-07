/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <stdio.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"

#include "fc/rc_controls.h"

#include "flight/mixer_init.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "mixer.h"

#define DYN_LPF_THROTTLE_STEPS             100
#define DYN_LPF_THROTTLE_UPDATE_DELAY_US  5000 // minimum of 5ms between updates

#define CRASHFLIP_MOTOR_DEADBAND         0.02f // 2%; send 'disarm' value to motors below this drive value
#define CRASHFLIP_STICK_DEADBAND         0.15f // 15%

static FAST_DATA_ZERO_INIT float motorMixRange;

float FAST_DATA_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

static FAST_DATA_ZERO_INIT int throttleAngleCorrection;

float getMotorMixRange(void)
{
    return motorMixRange;
}

static FAST_DATA_ZERO_INIT float throttle = 0;
static FAST_DATA_ZERO_INIT float rcThrottle = 0;
static FAST_DATA_ZERO_INIT float mixerThrottle = 0;
static FAST_DATA_ZERO_INIT float motorOutputMin;
static FAST_DATA_ZERO_INIT float motorRangeMin;
static FAST_DATA_ZERO_INIT float motorRangeMax;
static FAST_DATA_ZERO_INIT float motorOutputRange;
static FAST_DATA_ZERO_INIT int8_t motorOutputMixSign;

static void calculateThrottleAndCurrentMotorEndpoints(timeUs_t currentTimeUs)
{
    static uint16_t rcThrottlePrevious = 0;   // Store the last throttle direction for deadband transitions
    static timeUs_t reversalTimeUs = 0; // time when motors last reversed in 3D mode
    static float motorRangeMinIncrease = 0;

    float currentThrottleInputRange = 0;
#ifndef ZL_EDIT
    if (mixerRuntime.feature3dEnabled) {
        uint16_t rcCommand3dDeadBandLow;
        uint16_t rcCommand3dDeadBandHigh;

        if (!ARMING_FLAG(ARMED)) {
            rcThrottlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.
        }

        if (IS_RC_MODE_ACTIVE(BOX3D) || flight3DConfig()->switched_mode3d) {
            // The min_check range is halved because the output throttle is scaled to 500us.
            // So by using half of min_check we maintain the same low-throttle deadband
            // stick travel as normal non-3D mode.
            const int mincheckOffset = (rxConfig()->mincheck - PWM_RANGE_MIN) / 2;
            rcCommand3dDeadBandLow = rxConfig()->midrc - mincheckOffset;
            rcCommand3dDeadBandHigh = rxConfig()->midrc + mincheckOffset;
        } else {
            rcCommand3dDeadBandLow = rxConfig()->midrc - flight3DConfig()->deadband3d_throttle;
            rcCommand3dDeadBandHigh = rxConfig()->midrc + flight3DConfig()->deadband3d_throttle;
        }

        const float rcCommandThrottleRange3dLow = rcCommand3dDeadBandLow - PWM_RANGE_MIN;
        const float rcCommandThrottleRange3dHigh = PWM_RANGE_MAX - rcCommand3dDeadBandHigh;

        if (rcCommand[THROTTLE] <= rcCommand3dDeadBandLow || isCrashFlipModeActive()) {
            // INVERTED
            motorRangeMin = mixerRuntime.motorOutputLow;
            motorRangeMax = mixerRuntime.deadbandMotor3dLow;
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutputMin = mixerRuntime.motorOutputLow;
                motorOutputRange = mixerRuntime.deadbandMotor3dLow - mixerRuntime.motorOutputLow;
            } else
#endif
            {
                motorOutputMin = mixerRuntime.deadbandMotor3dLow;
                motorOutputRange = mixerRuntime.motorOutputLow - mixerRuntime.deadbandMotor3dLow;
            }

            if (motorOutputMixSign != -1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = -1;

            rcThrottlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand3dDeadBandLow - rcCommand[THROTTLE];
            currentThrottleInputRange = rcCommandThrottleRange3dLow;
        } else if (rcCommand[THROTTLE] >= rcCommand3dDeadBandHigh) {
            // NORMAL
            motorRangeMin = mixerRuntime.deadbandMotor3dHigh;
            motorRangeMax = mixerRuntime.motorOutputHigh;
            motorOutputMin = mixerRuntime.deadbandMotor3dHigh;
            motorOutputRange = mixerRuntime.motorOutputHigh - mixerRuntime.deadbandMotor3dHigh;
            if (motorOutputMixSign != 1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = 1;
            rcThrottlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand[THROTTLE] - rcCommand3dDeadBandHigh;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh;
        } else if ((rcThrottlePrevious <= rcCommand3dDeadBandLow &&
                !flight3DConfigMutable()->switched_mode3d) ||
                isMotorsReversed()) {
            // INVERTED_TO_DEADBAND
            motorRangeMin = mixerRuntime.motorOutputLow;
            motorRangeMax = mixerRuntime.deadbandMotor3dLow;

#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutputMin = mixerRuntime.motorOutputLow;
                motorOutputRange = mixerRuntime.deadbandMotor3dLow - mixerRuntime.motorOutputLow;
            } else
#endif
            {
                motorOutputMin = mixerRuntime.deadbandMotor3dLow;
                motorOutputRange = mixerRuntime.motorOutputLow - mixerRuntime.deadbandMotor3dLow;
            }

            if (motorOutputMixSign != -1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = -1;

            throttle = 0;
            currentThrottleInputRange = rcCommandThrottleRange3dLow;
        } else {
            // NORMAL_TO_DEADBAND
            motorRangeMin = mixerRuntime.deadbandMotor3dHigh;
            motorRangeMax = mixerRuntime.motorOutputHigh;
            motorOutputMin = mixerRuntime.deadbandMotor3dHigh;
            motorOutputRange = mixerRuntime.motorOutputHigh - mixerRuntime.deadbandMotor3dHigh;
            if (motorOutputMixSign != 1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = 1;
            throttle = 0;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh;
        }
        if (currentTimeUs - reversalTimeUs < 250000) {
            // keep iterm zero for 250ms after motor reversal
            pidResetIterm();
        }
    } else {
#endif
        throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN + throttleAngleCorrection;
        currentThrottleInputRange = PWM_RANGE;

        motorRangeMax = mixerRuntime.motorOutputHigh;

        motorRangeMin = mixerRuntime.motorOutputLow + motorRangeMinIncrease * (mixerRuntime.motorOutputHigh - mixerRuntime.motorOutputLow);
        motorOutputMin = motorRangeMin;
        motorOutputRange = motorRangeMax - motorRangeMin;
        motorOutputMixSign = 1;
#ifndef ZL_EDIT
    }
#endif

    throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
    rcThrottle = throttle;
}


static void applyMixToMotors(const float motorMix[MAX_SUPPORTED_MOTORS], motorMixer_t *activeMixer)
{
    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        float motorOutput = motorOutputMixSign * motorMix[i] + throttle * activeMixer[i].throttle;
#ifdef USE_THRUST_LINEARIZATION
        motorOutput = pidApplyThrustLinearization(motorOutput);
#endif
        motorOutput = motorOutputMin + motorOutputRange * motorOutput;
        
#ifndef ZL_EDIT
        if (failsafeIsActive()) {
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < motorRangeMin) ? mixerRuntime.disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
#endif
            motorOutput = constrainf(motorOutput, mixerRuntime.disarmMotorOutput, motorRangeMax);
        } else {
#endif
            motorOutput = constrainf(motorOutput, motorRangeMin, motorRangeMax);
#ifndef ZL_EDIT
        }
#endif
        motor[i] = motorOutput;
    }

#ifndef ZL_EDIT
    // Disarmed mode
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
    DEBUG_SET(DEBUG_EZLANDING, 1, throttle * 10000U);
    // DEBUG_EZLANDING 0 is the ezLanding factor 2 is the throttle limit
#endif
}


static void applyMixerAdjustmentLinear(float *motorMix, const bool airmodeEnabled)
{
    float airmodeTransitionPercent = 1.0f;
    float motorDeltaScale = 0.5f;

    if (!airmodeEnabled && throttle < 0.5f) {
        // this scales the motor mix authority to be 0.5 at 0 throttle, and 1.0 at 0.5 throttle as airmode off intended for things to work.
        // also lays the groundwork for how an airmode percent would work.
        airmodeTransitionPercent = scaleRangef(throttle, 0.0f, 0.5f, 0.5f, 1.0f); // 0.5 throttle is full transition, and 0.0 throttle is 50% airmodeTransitionPercent
        motorDeltaScale *= airmodeTransitionPercent; // this should be half of the motor authority allowed
    }

    const float motorMixNormalizationFactor = motorMixRange > 1.0f ? airmodeTransitionPercent / motorMixRange : airmodeTransitionPercent;

    const float motorMixDelta = motorDeltaScale * motorMixRange;

    float minMotor = FLT_MAX;
    float maxMotor = FLT_MIN;

    for (int i = 0; i < mixerRuntime.motorCount; ++i) {
        if (mixerConfig()->mixer_type == MIXER_LINEAR) {
            motorMix[i] = scaleRangef(throttle, 0.0f, 1.0f, motorMix[i] + motorMixDelta, motorMix[i] - motorMixDelta);
        } else {
            motorMix[i] = scaleRangef(throttle, 0.0f, 1.0f, motorMix[i] + fabsf(motorMix[i]), motorMix[i] - fabsf(motorMix[i]));
        }
        motorMix[i] *= motorMixNormalizationFactor;

        maxMotor = MAX(motorMix[i], maxMotor);
        minMotor = MIN(motorMix[i], minMotor);
    }

    // constrain throttle so it won't clip any outputs
    throttle = constrainf(throttle, -minMotor, 1.0f - maxMotor);
}

static float calcEzLandLimit(float maxDeflection, float speed)
{
    // calculate limit to where the mixer can raise the throttle based on RPY stick deflection
    // 0.0 = no increas allowed, 1.0 = 100% increase allowed
    const float deflectionLimit = mixerRuntime.ezLandingThreshold > 0.0f ? fminf(1.0f, maxDeflection / mixerRuntime.ezLandingThreshold) : 0.0f;
    DEBUG_SET(DEBUG_EZLANDING, 4, lrintf(deflectionLimit * 10000.0f));

    // calculate limit to where the mixer can raise the throttle based on speed
    // TODO sanity checks like number of sats, dop, accuracy?
    const float speedLimit = mixerRuntime.ezLandingSpeed > 0.0f ? fminf(1.0f, speed / mixerRuntime.ezLandingSpeed) : 0.0f;
    DEBUG_SET(DEBUG_EZLANDING, 5, lrintf(speedLimit * 10000.0f));

    // get the highest of the limits from deflection, speed, and the base ez_landing_limit
    const float deflectionAndSpeedLimit = fmaxf(deflectionLimit, speedLimit);
    return fmaxf(mixerRuntime.ezLandingLimit, deflectionAndSpeedLimit);
}

static void applyMixerAdjustmentEzLand(float *motorMix, const float motorMixMin, const float motorMixMax)
{
    // Calculate factor for normalizing motor mix range to <= 1.0
    const float baseNormalizationFactor = motorMixRange > 1.0f ? 1.0f / motorMixRange : 1.0f;
    const float normalizedMotorMixMin = motorMixMin * baseNormalizationFactor;
    const float normalizedMotorMixMax = motorMixMax * baseNormalizationFactor;

#ifdef USE_GPS
    const float speed = STATE(GPS_FIX) ? gpsSol.speed3d / 100.0f : 0.0f;  // m/s
#else
    const float speed = 0.0f;
#endif

    const float ezLandLimit = calcEzLandLimit(getMaxRcDeflectionAbs(), speed);
    // use the largest of throttle and limit calculated from RPY stick positions
    float upperLimit = fmaxf(ezLandLimit, throttle);
    // limit throttle to avoid clipping the highest motor output
    upperLimit = fminf(upperLimit, 1.0f - normalizedMotorMixMax);

    // Lower throttle Limit
    const float epsilon = 1.0e-6f;  // add small value to avoid divisions by zero
    const float absMotorMixMin = fabsf(normalizedMotorMixMin) + epsilon;
    const float lowerLimit = fminf(upperLimit, absMotorMixMin);

    // represents how much motor values have to be scaled to avoid clipping
    const float ezLandFactor = upperLimit / absMotorMixMin;

    // scale motor values
    const float normalizationFactor = baseNormalizationFactor * fminf(1.0f, ezLandFactor);
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motorMix[i] *= normalizationFactor;
    }
    motorMixRange *= baseNormalizationFactor;
    // Make anti windup recognize reduced authority range
    motorMixRange = fmaxf(motorMixRange, 1.0f / ezLandFactor);

    // Constrain throttle
    throttle = constrainf(throttle, lowerLimit, upperLimit);

    // Log ezLandFactor, upper throttle limit, and ezLandFactor if throttle was zero
    DEBUG_SET(DEBUG_EZLANDING, 0, fminf(1.0f, ezLandFactor) * 10000U);
    // DEBUG_EZLANDING 1 is the adjusted throttle
    DEBUG_SET(DEBUG_EZLANDING, 2, upperLimit * 10000U);
    DEBUG_SET(DEBUG_EZLANDING, 3, fminf(1.0f, ezLandLimit / absMotorMixMin) * 10000U);
    // DEBUG_EZLANDING 4 and 5 is the upper limits based on stick input and speed respectively
}

static void applyMixerAdjustment(float *motorMix, const float motorMixMin, const float motorMixMax, const bool airmodeEnabled)
{
#ifdef USE_AIRMODE_LPF
    const float unadjustedThrottle = throttle;
    throttle += pidGetAirmodeThrottleOffset();
    float airmodeThrottleChange = 0.0f;
#endif
    float airmodeTransitionPercent = 1.0f;

    if (!airmodeEnabled && throttle < 0.5f) {
        // this scales the motor mix authority to be 0.5 at 0 throttle, and 1.0 at 0.5 throttle as airmode off intended for things to work.
        // also lays the groundwork for how an airmode percent would work.
        airmodeTransitionPercent = scaleRangef(throttle, 0.0f, 0.5f, 0.5f, 1.0f); // 0.5 throttle is full transition, and 0.0 throttle is 50% airmodeTransitionPercent
    }

    const float motorMixNormalizationFactor = motorMixRange > 1.0f ? airmodeTransitionPercent / motorMixRange : airmodeTransitionPercent;

    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motorMix[i] *= motorMixNormalizationFactor;
    }

    const float normalizedMotorMixMin = motorMixMin * motorMixNormalizationFactor;
    const float normalizedMotorMixMax = motorMixMax * motorMixNormalizationFactor;
    throttle = constrainf(throttle, -normalizedMotorMixMin, 1.0f - normalizedMotorMixMax);

#ifdef USE_AIRMODE_LPF
    airmodeThrottleChange = constrainf(unadjustedThrottle, -normalizedMotorMixMin, 1.0f - normalizedMotorMixMax) - unadjustedThrottle;
    pidUpdateAirmodeLpf(airmodeThrottleChange);
#endif
}

FAST_CODE_NOINLINE void mixTable(timeUs_t currentTimeUs)
{
    // Find min and max throttle based on conditions. Throttle has to be known before mixing
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);

#ifndef ZL_EDIT
    if (applyCrashFlipModeToMotors()) {
        return; // if crash flip mode has been applied to the motors, mixing is done
    }
#endif

    const bool launchControlActive = isLaunchControlActive();

    motorMixer_t * activeMixer = &mixerRuntime.currentMixer[0];

    // Calculate and Limit the PID sum
    const float scaledAxisPidRoll =
        constrainf(pidData[FD_ROLL].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    const float scaledAxisPidPitch =
        constrainf(pidData[FD_PITCH].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;

    uint16_t yawPidSumLimit = currentPidProfile->pidSumLimitYaw;

    float scaledAxisPidYaw =
        constrainf(pidData[FD_YAW].Sum, -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;

    if (!mixerConfig()->yaw_motors_reversed) {
        scaledAxisPidYaw = -scaledAxisPidYaw;
    }

#ifndef ZL_EDIT
    // Apply the throttle_limit_percent to scale or limit the throttle based on throttle_limit_type
    if (currentControlRateProfile->throttle_limit_type != THROTTLE_LIMIT_TYPE_OFF) {
        throttle = applyThrottleLimit(throttle);
    }
#endif

    // use scaled throttle, without dynamic idle throttle offset, as the input to antigravity
    pidUpdateAntiGravityThrottleFilter(throttle);

    // and for TPA
    pidUpdateTpaFactor(throttle);

    // apply throttle boost when throttle moves quickly
#if defined(USE_THROTTLE_BOOST)
    if (throttleBoost > 0.0f) {
        const float throttleHpf = throttle - pt1FilterApply(&throttleLpf, throttle);
        throttle = constrainf(throttle + throttleBoost * throttleHpf, 0.0f, 1.0f);
    }
#endif

    // send throttle value to blackbox, including scaling and throttle boost, but not TL compensation, dyn idle or airmode
    mixerThrottle = throttle;

    // Find roll/pitch/yaw desired output
    // ??? Where is the optimal location for this code?
    float motorMix[MAX_SUPPORTED_MOTORS];
    float motorMixMax = 0, motorMixMin = 0;
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        float mix =
            scaledAxisPidRoll  * activeMixer[i].roll +
            scaledAxisPidPitch * activeMixer[i].pitch +
            scaledAxisPidYaw   * activeMixer[i].yaw;

        if (mix > motorMixMax) {
            motorMixMax = mix;
        } else if (mix < motorMixMin) {
            motorMixMin = mix;
        }
        motorMix[i] = mix;
    }

    //  The following fixed throttle values will not be shown in the blackbox log
    // ?? Should they be influenced by airmode?  If not, should go after the apply airmode code.
#ifndef ZL_EDIT
    const bool airmodeEnabled = airmodeIsEnabled() || launchControlActive;
#else
    const bool airmodeEnabled = true;
#endif


    motorMixRange = motorMixMax - motorMixMin;

    switch (mixerConfig()->mixer_type) {
    case MIXER_LEGACY:
        applyMixerAdjustment(motorMix, motorMixMin, motorMixMax, airmodeEnabled);
        break;
    case MIXER_LINEAR:
    case MIXER_DYNAMIC:
        applyMixerAdjustmentLinear(motorMix, airmodeEnabled);
        break;
    case MIXER_EZLANDING:
        applyMixerAdjustmentEzLand(motorMix, motorMixMin, motorMixMax);
        break;
    default:
        applyMixerAdjustment(motorMix, motorMixMin, motorMixMax, airmodeEnabled);
        break;
    }

#ifndef ZL_EDIT
    if (featureIsEnabled(FEATURE_MOTOR_STOP)
        && ARMING_FLAG(ARMED)
        && !mixerRuntime.feature3dEnabled
        && !airmodeEnabled
        && !FLIGHT_MODE(GPS_RESCUE_MODE | ALT_HOLD_MODE)   // disable motor_stop while GPS Rescue / Altitude Hold is active
        && (rcData[THROTTLE] < rxConfig()->mincheck)) {
        // motor_stop handling
        applyMotorStop();
    } else {
        // Apply the mix to motor endpoints
        applyMixToMotors(motorMix, activeMixer);
    }
#else
    applyMixToMotors(motorMix, activeMixer);
#endif

}

float mixerGetRcThrottle(void)
{
    return rcThrottle;
}
