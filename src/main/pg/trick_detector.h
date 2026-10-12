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

#include <stdint.h>

#include "pg/pg.h"

typedef struct trickDetectorConfig_s {
    uint8_t hover_throttle;     // percent; 0 = learn in flight
    uint8_t loop_share;         // percent: back pitch Thrust share above this is a Power Loop
    uint16_t loop_min_ms;       // back pitch longer than this is a Power Loop; 0 = off
    uint16_t recover_ms;        // Recovery window
    uint16_t announce_ms;       // how long the OSD announcement stays on screen
} trickDetectorConfig_t;

PG_DECLARE(trickDetectorConfig_t, trickDetectorConfig);
