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
 * @file "modules/scheduleloop/scheduleloop.c"
 * @author Shuo Li
 * This module is the highest loop of control. 
 */

#include "modules/scheduleloop/scheduleloop.h"
struct Clock clockSchedule;
struct AutopilotMode autopilotMode;
struct DroneState currentDroneState,lastStepDroneState, droneStateTemp;

void readDroneState(struct DroneState * droneState);

void schedule_init(){
     clearClock(1);
     clearClock(2);
     clearClock(3);
     autopilotMode.previousMode = autopilot_mode;
     autopilotMode.currentMode = autopilot_mode;
 }


void schedule_run() {
    autopilotMode.currentMode = autopilot_mode;
    if (autopilot_mode != AP_MODE_GUIDED) return;


    if(autopilotMode.currentMode != autopilotMode.previousMode)
    {
        clearClock(1);
        readDroneState(&currentDroneState);
    }

    if(getTime(1)<5.0)
    {
        if(guidance_h.mode != GUIDANCE_H_MODE_HOVER) guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
        if(guidance_v_mode != GUIDANCE_V_MODE_HOVER) guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
    }
    else
    {
        if(guidance_h.mode != GUIDANCE_H_MODE_GUIDED)
        {
            guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
            set_nn_run();
        }
        if(guidance_v_mode != GUIDANCE_V_MODE_GUIDED) guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);

    }


    autopilotMode.previousMode = autopilot_mode;
    readDroneState(&lastStepDroneState);
}
 
void clock_run(){
     clockSchedule.counterTime1++;
     clockSchedule.time1 = clockSchedule.counterTime1 / 100.0;
     clockSchedule.counterTime2++;
     clockSchedule.time2 = clockSchedule.counterTime2 / 100.0;
     clockSchedule.counterTime3++;
     clockSchedule.time3 = clockSchedule.counterTime3 / 100.0;
 }

void clearClock(int clockNum)
{
    switch(clockNum)
    {
        case 1:
            clockSchedule.counterTime1 = 0;
            clockSchedule.time1 = 0;
            break;
        case 2:
            clockSchedule.counterTime2= 0;
            clockSchedule.time2 = 0;
            break;
        case 3:
            clockSchedule.counterTime3= 0;
            clockSchedule.time3 = 0;
            break;
    }
}

float getTime(int clockNum)
{

    switch(clockNum)
    {
        case 1:
            return clockSchedule.time1;
        case 2:
            return clockSchedule.time2;
        case 3:
            return clockSchedule.time3;
        default:
            return 0;
    }
}

void readDroneState(struct DroneState * droneState)
{
    droneState->pos.x = stateGetPositionNed_f()->x;
    droneState->pos.y = stateGetPositionNed_f()->y;
    droneState->pos.z = stateGetPositionNed_f()->z;
    droneState->vel.vx = stateGetSpeedNed_f()->x;
    droneState->vel.vy = stateGetSpeedNed_f()->y;
    droneState->vel.vz = stateGetSpeedNed_f()->z;
    droneState->euler.phi = stateGetNedToBodyEulers_f()->phi;
    droneState->euler.theta= stateGetNedToBodyEulers_f()->theta;
    droneState->euler.psi = stateGetNedToBodyEulers_f()->psi;
    droneState->angularRate.p = stateGetBodyRates_f()->p;
    droneState->angularRate.q = stateGetBodyRates_f()->q;
    droneState->angularRate.r = stateGetBodyRates_f()->r;
}


