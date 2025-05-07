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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "core.h"


enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

enum {
    ARMING_DELAYED_DISARMED = 0,
    ARMING_DELAYED_NORMAL = 1,
    ARMING_DELAYED_CRASHFLIP = 2,
    ARMING_DELAYED_LAUNCH_CONTROL = 3,
};

#define GYRO_WATCHDOG_DELAY 80 //  delay for gyro sync


#if defined(USE_GPS) || defined(USE_MAG)
int16_t magHold;
#endif

static FAST_DATA_ZERO_INIT uint8_t pidUpdateCounter;

static bool crashFlipModeActive = false;

#ifndef ZL_EDIT
    static timeUs_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero
#else
    static uint32_t disarmAt;
#endif

static int lastArmingDisabledReason = 0;
#ifndef ZL_EDIT
    static timeUs_t lastDisarmTimeUs;     
#else
    static uint32_t lastDisarmTimeUs;
#endif
static int tryingToArm = ARMING_DELAYED_DISARMED;

static bool airmodeIsActivated;

bool isAirmodeActivated(void)
{
    return airmodeIsActivated;
}


bool isLaunchControlActive(void)
{
    return false;
}
