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
 * @file "modules/computer_vision/cv_opencvdemo.c"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_opencvdemo.h"
#include "modules/computer_vision/opencv_example.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "modules/computer_vision/lib/vision/image.h"

#include "std.h"

#include <stdlib.h>
#include <string.h>
// Function
struct image_t* opencv_func(struct image_t* img);
struct image_t* opencv_func(struct image_t* img)
{

  if (img->type == IMAGE_YUV422)
  {
    // Call OpenCV (C++ from paparazzi C function)
    opencv_example((char*) img->buf, img->w, img->h);
  }
  float yaw = stateGetNedToBodyEulers_f()->psi;
  float viewingAngle=0.45;//radians
  float diff = loc_y-(img->h/2);
  double pixelsPerDegree = viewingAngle/img->h;
  yaw += pixelsPerDegree * diff;

  guidance_h_set_guided_heading(yaw);

  guidance_h_set_guided_body_vel(0.5,0.0);

//  guidance_h_set_guided_body_vel(0.15,0.0);

  return img;
}

void opencvdemo_init(void)
{
  cv_add_to_device(&OPENCVDEMO_CAMERA, opencv_func);
  opencv_init_rects();
}

