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
 * @file "modules/scheduleloop/scheduleloop.h"
 * @author Shuo Li
 * This module is the highest loop of control. 
 */

#ifndef SCHEDULELOOP_H
#define SCHEDULELOOP_H

#include "firmwares/rotorcraft/autopilot.h"
#include "modules/nn_controller/nn_controller.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"

struct AutopilotMode
{
    uint8_t previousMode;
    uint8_t currentMode;
};

struct Clock
{
    int counterTime1;
    int counterTime2;
    int counterTime3;
    float time1;
    float time2;
    float time3;
};

enum HIGH_LEVEL_GUIDANCE_STATE {FIRST_HIGH_LEVEL,SECOND_HIGH_LEVEL};
enum LOW_LEVEL_GUIDANCE_STATE {TEMP};


struct Pos
{
    float x; float y; float z;
};

struct Vel
{
    float vx; float vy; float vz;
};

struct Euler
{
    float phi; float theta; float psi;
};

struct AngularRate
{
    float p;float q;float r;
};


struct DroneState
{
    struct Pos pos;
    struct Vel vel;
    struct Euler euler;
    struct AngularRate angularRate;
};

 extern void schedule_init(void);
 extern void schedule_run(void);
 extern void clock_run(void);
 extern void clearClock(int clockNum);
 extern float getTime(int clockNum);

#endif

