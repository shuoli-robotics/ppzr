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
 * @file "modules/state_autonomous_race/state_autonomous_race.c"
 * @author Shuo Li
 * The module is used to store all states in the competition
 */

#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

//void state_autonomous_race_init();
//void display_lower_state();



struct state_autonomous_race states_race;

void display_upper_state(void);
void display_lower_state(void);
void display_guidance_mode(void);


void state_autonomous_race_init() {
    states_race.gate_counter = 0;
    states_race.ready_pass_through = FALSE;
    states_race.turning = FALSE;
    states_race.altitude_is_achieved = FALSE;
    states_race.land_is_finished =FALSE;
    states_race.gate_counter_in_second_part = 0;
    states_race.gate_counter_in_third_part = 0;
}

void display_states()
{
    printf("Thrust is %d\n",stabilization_cmd[COMMAND_THRUST]);
    if (autopilot_mode != AP_MODE_MODULE)
        return;
   // printf("gate_counter is %d \n",states_race.gate_counter);

    display_upper_state();
    display_lower_state();
    printf("\n");
    printf("\n");
    printf("\n");
    display_guidance_mode();
    printf("\n");
    printf("\n");
    printf("\n");
    printf("Time in primitve is %f\n",time_primitive);


    printf("\n");
    printf("\n");
    printf("\n");

}

void display_lower_state()
{
    switch(state_lower_level){
        case WAIT_FOR_DETECTION_CM:
            printf("It is in WAIT_FOR_DETECTION\n");
            break;
        case ADJUST_POSITION_CM:
            printf("It is in ADJUST_POSITION\n");
            break;
        case GO_THROUGH_CM:
            printf("It is in GO_THROUGH\n");
            break;
        case HOVER_CM:
            printf("It is in HOVER\n");
            break;
        case TURN_CM:
            printf("It is in TURN\n");
            break;
        case SEARCH_GATE_CM:
            printf("It is in SEARCH_GATE\n");
            break;
        case TAKE_OFF_OPEN_LOOP_CM:
            printf("It is in TAKE_OFF_OPEN_LOOP\n");
            break;
        case TAKE_OFF_CLOSE_LOOP_CM:
            printf("It is in TAKE_OFF_CLOSE_LOOP\n");
            break;
        case LAND_CM:
            printf("It is in LAND\n");
            break;
        case GO_STRAIGHT_CM:
            printf("It is in GO_STRAIGHT\n");
            break;
        case ADJUST_HEIGHT_CM:
            printf("It is in ADJUST_HEIGHT\n");
            break;
        case PREPARE_CM:
            printf("It is in PREPARE\n");
            break;
        case REPLAY_CM:
            printf("It is in REPLAY\n");
            break;
        case APPROACH_GATE_CM:
            printf("It is in APPROACH_GATE\n");
            break;

    }
}

void display_upper_state()
{
    switch(state_upper_level)
    {
        case FIRST_PART:
            printf("It is in FIRST_PART\n");
            break;
        case SECOND_PART:
            printf("It is in SECOND_PART\n");
            break;
        case THIRD_PART:
            printf("It is in THIRD_PART\n");
            break;
        case FOURTH_PART:
            printf("It is in FOURTH_PART\n");
            break;
    }
};

void display_guidance_mode()
{
    switch(guidance_h.mode)
    {
        case GUIDANCE_H_MODE_MODULE:
            printf("Horizontal mode is　MODULE mode\n");
            break;
        case GUIDANCE_H_MODE_GUIDED:
            printf("Horizontal mode is　GUIDED mode\n");
            break;
        default:
            break;
    }


    switch(guidance_v_mode)
    {
        case GUIDANCE_V_MODE_MODULE:
            printf("Vertial mode is　MODULE mode\n");
            break;
        case GUIDANCE_V_MODE_GUIDED:
            printf("Vertial mode is　GUIDED mode\n");
            break;

        default:
            break;
    }
}