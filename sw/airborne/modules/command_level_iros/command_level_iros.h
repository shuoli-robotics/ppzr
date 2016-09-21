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
 * @file "modules/command_level_iros/command_level_iros.h"
 * @author Shuo Li
 * This module is the highest level
 */

#ifndef COMMAND_LEVEL_IROS_H
#define COMMAND_LEVEL_IROS_H

extern void command_run(void);  // 20HZ
extern void command_init(void);

enum states_lower_level{WAIT_FOR_DETECTION,ADJUST_POSITION,GO_THROUGH,HOVER};
enum states_upper_level{HALF_GATE,SQUARE};

#endif

