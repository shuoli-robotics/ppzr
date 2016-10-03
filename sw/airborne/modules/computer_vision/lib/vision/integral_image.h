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
 * @file modules/computer_vision/lib/vision/integral_image.h
 * @brief Generic integral image functions.
 *
 */

#ifndef INTEGRAL_IMAGE_H
#define INTEGRAL_IMAGE_H

#define RES 100

#include "inttypes.h"

// get the integral image of the whole image:
void get_integral_image(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t *integral_image);
// get the integral image of a part of the image:
void get_integral_image_ROI(uint8_t *in, uint32_t image_width, uint32_t image_height, uint32_t *integral_image, uint32_t x_min, uint32_t y_min, uint32_t x_max, uint32_t y_max);
// get the sum of the illuminance values in an area: 
uint32_t get_sum_ii(uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y, uint32_t *integral_image,
                             uint32_t image_width, uint32_t image_height);
// get the average of the illuminance values in an area: 
uint32_t get_avg_ii(uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y, uint32_t *integral_image,
                           uint32_t image_width, uint32_t image_height);

#endif /* INTEGRAL_IMAGE_H */
