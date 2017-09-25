/*
 * Copyright (C) Freek van Tienen
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
 * @file "modules/computer_vision/cv_me_mwb.c"
 * @author Freek van Tienen
 * Manual exposure and Manual white balancing for the Bebop 1 and 2
 */

#include "modules/computer_vision/cv_me_mwb.h"
#include "lib/isp/libisp.h"

#include "boards/bebop/mt9f002.h"
extern struct mt9f002_t mt9f002;

float cv_me_mwb_exposure;
float cv_me_mwb_red;
float cv_me_mwb_blue;
float cv_me_mwb_green1;
float cv_me_mwb_green2;


void cv_me_mwb_init(void) {
  cv_me_mwb_exposure = mt9f002.real_exposure;
  cv_me_mwb_red = mt9f002.gain_red;
  cv_me_mwb_blue = mt9f002.gain_blue;
  cv_me_mwb_green1 = mt9f002.gain_green1;
  cv_me_mwb_green2 = mt9f002.gain_green2;
}


void cv_me_mwb_periodic(void) {

    mt9f002.target_exposure = cv_me_mwb_exposure;
    mt9f002_set_exposure(&mt9f002);


    mt9f002.gain_green1 = cv_me_mwb_green1;
    mt9f002.gain_green2 = cv_me_mwb_green2;
    mt9f002.gain_blue = cv_me_mwb_blue;
    mt9f002.gain_red = cv_me_mwb_red;
    Bound(mt9f002.gain_blue, 2, 50);
    Bound(mt9f002.gain_red, 2, 50);
    mt9f002_set_gains(&mt9f002);
}


