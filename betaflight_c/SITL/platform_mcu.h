/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#undef USE_DYN_LPF
#undef USE_THRUST_LINEARIZATION
#undef USE_LAUNCH_CONTROL
#undef USE_RC_SMOOTHING_FILTER
#undef USE_INTEGRATED_YAW_CONTROL
#undef USE_ITERM_RELAX
#undef USE_YAW_SPIN_RECOVERY
#undef USE_GYRO_OVERFLOW_CHECK
#undef USE_BLACKBOX
#undef USE_RTC_TIME
#undef USE_SIMPLIFIED_TUNING
#undef USE_BATTERY_VOLTAGE_SAG_COMPENSATION
#undef USE_SERVOS
#undef USE_RUNAWAY_TAKEOFF

#define ZL_EDIT

#include "common/maths.h"