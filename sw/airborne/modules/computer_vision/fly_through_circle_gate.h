/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/computer_vision/cv_opencvdemo.h"
 * @author C. De Wagter
 * opencv
 */

#ifndef CV_OPENCVDEMO_H
#define CV_OPENCVDEMO_H

extern enum DRONE_STATE{DETECT_WINDOW,GO_THROUGH_WINDOW,GO_SAFETY};
extern enum DRONE_STATE dronerace_drone_state;
extern uint8_t navigate_through_blue_window(void);
extern uint8_t navigate_through_red_window(void);
extern uint8_t make_turn_right_radians(float);
extern void fly_through_gate_init(void);
extern uint8_t start_fly_through(void);
extern uint8_t guided_stay_wp(uint8_t);

extern uint8_t should_go_safety(void);
extern float getPosErrorMeters(uint8_t);


#endif


