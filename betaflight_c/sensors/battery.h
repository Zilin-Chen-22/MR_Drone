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

#pragma once

#include "pg/pg.h"


//TODO: Make the 'cell full' voltage user adjustble
#define CELL_VOLTAGE_FULL_CV 420

#define VBAT_CELL_VOTAGE_RANGE_MIN 100
#define VBAT_CELL_VOTAGE_RANGE_MAX 500
#define VBAT_CELL_VOLTAGE_DEFAULT_MIN 330
#define VBAT_CELL_VOLTAGE_DEFAULT_MAX 430

#define MAX_AUTO_DETECT_CELL_COUNT 8

#define GET_BATTERY_LPF_FREQUENCY(period) (1 / (period / 10.0f))

enum {
    AUTO_PROFILE_CELL_COUNT_STAY = 0, // Stay on this profile irrespective of the detected cell count. Use this profile if no other profile matches (default, i.e. auto profile switching is off)
    AUTO_PROFILE_CELL_COUNT_CHANGE = -1, // Always switch to a profile with matching cell count if there is one
};
