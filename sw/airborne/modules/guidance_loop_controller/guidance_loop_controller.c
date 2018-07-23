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
 * @file "modules/guidance_loop_controller/guidance_loop_controller.c"
 * @author Shuo Li
 * This module includes all controllers that calculate low level command to attitude loop or even lower level loop.
 */

#include "modules/guidance_loop_controller/guidance_loop_controller.h"
#include "modules/nn/nn.h"
#include "stdio.h"

enum ControllerInUse controllerInUse;
bool flagNN;

bool hover_with_optitrack(float hoverTime)
{
    if(controllerInUse!= CONTROLLER_HOVER_WITH_OPTITRACK)
    {
            controllerInUse= CONTROLLER_HOVER_WITH_OPTITRACK;
            clearClock(2);
            guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
            guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
            flagNN = false;
    }

    if(getTime(2)>hoverTime)
        return true;
    else
        return false;

}


void nn_controller(void)
{
    if(controllerInUse!= CONTROLLER_NN_CONTROLLER)
    {
            controllerInUse = CONTROLLER_NN_CONTROLLER;
            clearClock(2);
            guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
            guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
            flagNN = true;
    }

    double state[NUM_STATE_VARS] = {
        -4.2232016635363578e-02, -3.0225037024451664e+00, -6.1490007593689278e-01,
        5.1089365659897990e-01, 2.9941452020833115e-01
    };
    double control[NUM_CONTROL_VARS];
    nn(state, control);
}

