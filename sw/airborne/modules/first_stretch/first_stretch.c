/*
 * Copyright (C) CDW
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
 * @file "modules/first_stretch/first_stretch.c"
 * @author CDW
 * first stretch
 */

#include "state.h"
#include "modules/first_stretch/first_stretch.h"

#include <stdio.h>

#define MAX_FIRST_STRETCH  200
#define DELTA_T (1.0f/512.0f)

#define MODEL_DRAG  0.53f

typedef struct
{
  float t;
  float x;
  float psi;
  float certainty;
} first_stretch_data_struct;


typedef struct
{
  int active;
  int counter;

  float vx;
  float x;
  float t;

  first_stretch_data_struct data[MAX_FIRST_STRETCH];
} first_stretch_struct;

first_stretch_struct first_stretch;

void first_stretch_init(void) {
  first_stretch.counter = 0;
  first_stretch.active = 0;
  first_stretch.vx = 0;
  first_stretch.x = 0;
  first_stretch.t = 0;
}

void first_stretch_debug_store(void);
void first_stretch_debug_store(void) {
  FILE* fp = fopen("/data/ftp/internal_000/first_stretch.csv","w");
  for (int i=0; i<MAX_FIRST_STRETCH; i++ ) {
    fprintf(fp,"%f,%f,%f,%f\n",first_stretch.data[i].t,first_stretch.data[i].x,first_stretch.data[i].psi,first_stretch.data[i].certainty);
  }
  fclose(fp);
}

void first_stretch_periodic(void) {
  if (first_stretch.active) {
    // Propagate the model
    float theta = stateGetNedToBodyEulers_f()->theta;
    float dvx = 9.81f * sin(-theta) - first_stretch.vx * MODEL_DRAG;
    first_stretch.vx += dvx * (DELTA_T);
    first_stretch.x += first_stretch.vx * (DELTA_T);
    first_stretch.t += (DELTA_T);
  }
}

void first_stretch_start(void) {
  first_stretch_init();
  first_stretch.active = 1;
}

void first_stretch_add( float psi, float certainty) {
  if (first_stretch.active > 0) {
    if (first_stretch.counter < MAX_FIRST_STRETCH) {
      // Store set of data
      first_stretch.data[first_stretch.counter].t = first_stretch.t;
      first_stretch.data[first_stretch.counter].x = first_stretch.x;
      first_stretch.data[first_stretch.counter].psi = psi;
      first_stretch.data[first_stretch.counter].certainty = certainty;
      first_stretch.counter++;

      // Stop when buffer full
      if (first_stretch.counter >= MAX_FIRST_STRETCH) {
        first_stretch.active = 0;
        first_stretch_debug_store();
      }
    }
  }
}



