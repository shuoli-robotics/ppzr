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
 * @file "modules/guidance_loop_controller/guidance_loop_controller.h"
 * @author Shuo Li
 * This module includes all controllers that calculate low level command to attitude loop or even lower level loop.
 */




#ifndef GUIDANCE_LOOP_CONTROLLER_H
#define GUIDANCE_LOOP_CONTROLLER_H

#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "modules/scheduleloop/scheduleloop.h"
#include "modules/guidance_loop_controller/guidance_loop_controller.h"
#include "modules/guidance_loop_controller/guidance_loop_controller.h"


enum ControllerInUse {NO_CONTROLLER,CONTROLLER_HOVER_WITH_OPTITRACK,CONTROLLER_NN_CONTROLLER,CONTROLLER_GO_TO_POINT} ;

extern bool hover_with_optitrack(float hoverTime);
extern void nn_controller(void);
extern bool go_to_point(float desired_x,float desired_y,float desired_z,float desired_heading);

extern bool flagNN;

#endif

