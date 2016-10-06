/*
 * Copyright (C) Roland
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
 * @file "modules/stereocam/state2camera/state2camera.h"
 * @author Roland
 * Sends rotation using the stereoboard protocol over the UART.
 */

#ifndef UARTROTATION_H
#define UARTROTATION_H

#include <inttypes.h>

struct stereocam_edgeflow_t{
  uint8_t window_size;
  uint8_t search_distance;
  uint8_t derotation;
  uint8_t adaptive_time_horizon;
  uint8_t snapshot;
  uint8_t kalman;
};
extern struct stereocam_edgeflow_t edgeflow;

void init_state2camera(void);
extern void write_serial_rot(void);

#endif

