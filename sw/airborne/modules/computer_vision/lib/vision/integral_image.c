/*
 * Copyright (C) 2016 G. de Croon <guido.de.croon@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/lib/vision/integral_image.c
 * @brief Generic integral image functions.
 *
 */

#include "integral_image.h"

// get an integral image
void get_integral_image(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t *integral_image)
{
  uint16_t x, y;
  for (x = 0; x < image_width; x++) {
    for (y = 0; y < image_height; y++) {
      if (x >= 1 && y >= 1) {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width] + integral_image[x - 1 + y * image_width] +
                                              integral_image[x + (y - 1) * image_width] - integral_image[x - 1 + (y - 1) * image_width];
      } else if (x >= 1) {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width] + integral_image[x - 1 + y * image_width];
      } else if (y >= 1) {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width] + integral_image[x + (y - 1) * image_width];
      } else {
        integral_image[x + y * image_width] = (uint32_t) in[x + y * image_width];
      }
    }
  }
}

// get the integral image from a Region Of Interest
void get_integral_image_ROI(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t *integral_image, uint32_t x_min, uint32_t y_min, uint32_t x_max, uint32_t y_max)
{
  uint16_t x, y, ix, iy;

  // width and height of the integral image:
  uint32_t width;
  width = x_max - x_min;

  // create an integral image for a region of interest:
  for (x = x_min; x < x_max; x++) {

    // x index in the integral image:
    ix = x - x_min;

    for (y = y_min; y < y_max; y++) {
      
      // y index in the integral image:
      iy = y - y_min;      
    
      if (x >= 1 && y >= 1) {
        integral_image[ix + iy * width] = (uint32_t) in[x + y * image_width] + integral_image[ix - 1 + iy * width] +
                                              integral_image[ix + (iy - 1) * width] - integral_image[ix - 1 + (iy - 1) * width];
      } else if (x >= 1) {
        integral_image[ix + iy * width] = (uint32_t) in[x + y * image_width] + integral_image[ix - 1 + iy * width];
      } else if (y >= 1) {
        integral_image[ix + iy * width] = (uint32_t) in[x + y * image_width] + integral_image[ix + (iy - 1) * width];
      } else {
        integral_image[ix + iy * width] = (uint32_t) in[x + y * image_width];
      }
    }
  }
}

// determine the sum of pixels in a given ROI
uint32_t get_sum_ii(uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y, uint32_t *integral_image,
                             uint32_t image_width, uint32_t image_height)
{
  uint32_t sum;
  // If variables are not unsigned, then check for negative inputs
  // if (min_x + min_y * image_width < 0) { return 0; }
  if (max_x + max_y * image_width >= image_width * image_height) { return 0; }
  sum = integral_image[min_x + min_y * image_width] + integral_image[max_x + max_y * image_width] -
        integral_image[max_x + min_y * image_width] - integral_image[min_x + max_y * image_width];
  return sum;
}

// determine the average of pixels in a given ROI
uint32_t get_avg_ii(uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y, uint32_t *integral_image,
                           uint32_t image_width, uint32_t image_height __attribute__((unused)))
{
  uint16_t w, h;
  uint32_t sum, avg, n_pix;

  // width and height of the window
  w = max_x - min_x + 1;
  h = max_y - min_y + 1;
  n_pix = w * h;
  // sum over the area:
  sum = integral_image[min_x + min_y * image_width] + integral_image[max_x + max_y * image_width] -
        integral_image[max_x + min_y * image_width] - integral_image[min_x + max_y * image_width];
  // take the average, scaled by RES:
  avg = (sum * RES) / n_pix;
  return avg;
}

