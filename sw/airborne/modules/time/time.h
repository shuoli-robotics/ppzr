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
 * @file "modules/time/time.h"
 * @author Shuo Li
 * used for time in flight
 */

#ifndef TIME_H
#define TIME_H

struct time_st
{
    int autopilot_counter;
    int guidance_mode_counter;
    int motion_counter;
    float autopilot_time;
    float guidance_mode_time;
    float motion_time;

};

extern void time_run(void);
extern struct time_st time;

#endif

