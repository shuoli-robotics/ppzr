/*
 * Copyright (C) Shuo Li
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/time/time.c"
 * @author Shuo Li
 * used for time in flight
 */

#include "modules/time/time.h"

struct time_st time;

void time_run() {
    time.autopilot_counter++;
    time.guidance_mode_counter++;
    time.motion_counter++;
    time.autopilot_time = time.autopilot_counter/20.0;
    time.guidance_mode_time = time.guidance_mode_counter/20.0;
    time.motion_time =  time.motion_counter/20.0;
}

