/*
 * Copyright (C) Isabelle
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
 * @file "modules/replay_commands/replay_commands.h"
 * @author Isabelle
 * will replay the commands recorded by the logger
 */

#ifndef REPLAY_COMMANDS_H
#define REPLAY_COMMANDS_H

extern void replay_commands_init();
extern void replay_commands_periodic();
extern void replay_commands_start();
extern void replay_commands_stop();

#endif

extern int n; //maximum number of characters in the line
extern char str[105];


extern int counter;
extern double x;
extern double y;
extern double z;
extern double vx;
extern double vy;
extern double vz;
extern double phi;
extern double theta;
extern double psi;
extern int phi_i;
extern int theta_i;
extern int psi_i;
extern int replay;

