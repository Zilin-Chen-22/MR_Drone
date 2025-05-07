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
#include <math.h>

#include "platform.h"

#include "common/axis.h"

#include "rc.h"

// #define RX_INTERVAL_MIN_US     950 // 0.950ms to fit 1kHz without an issue
// #define RX_INTERVAL_MAX_US   65500 // 65.5ms or 15.26hz

// typedef float (applyRatesFn)(const int axis, float rcCommandf, const float rcCommandfAbs);
// note that rcCommand[] is an external float

static float rawSetpoint[XYZ_AXIS_COUNT];

static float setpointRate[3], rcDeflection[3], rcDeflectionAbs[3]; // deflection range -1 to 1
static float maxRcDeflectionAbs;


#ifdef USE_FEEDFORWARD

static float feedforwardRaw[3];


float getFeedforward(int axis)
{
    return feedforwardRaw[axis];
}
#endif // USE_FEEDFORWARD


float getSetpointRate(int axis)
{
    return rawSetpoint[axis];
}

static float maxRcRate[3];
float getMaxRcRate(int axis)
{
    return maxRcRate[axis];
}

float getRcDeflection(int axis)
{
    return rcDeflection[axis];
}


float getMaxRcDeflectionAbs(void)
{
    return maxRcDeflectionAbs;
}


// Zilin Chen Added this function to input the data
void setSetpointRate(float SetpointRate, int axis)
{
    rawSetpoint[axis] = SetpointRate;
}
