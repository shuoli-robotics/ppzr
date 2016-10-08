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
 * @file modules/computer_vision/lib/vision/qr_code_recognition.h
 * @brief Efficient recognition of QR-codes. Note: only applicable to a very limited number of codes! Not generic like z-lib.
 *
 */

#ifndef QR_CODE_RECOGNITION_H
#define QR_CODE_RECOGNITION_H

#include "image.h"


int get_QR_class(struct image_t *img, float* uncertainty);
int get_QR_class_ROI(struct image_t *img, uint32_t x_min, uint32_t y_min, uint32_t x_max, uint32_t y_max, float* uncertainty);
void get_QR_integral_features(uint32_t* integral_image, uint32_t width, uint32_t height, float* features);
void QR_J48(float* attributes, int* class, float* uncertainty);
void get_closest_template(struct image_t *img, uint32_t x_min, uint32_t y_min, uint32_t w, uint32_t h, int* QR_template, float* cost_ratio);
uint8_t get_sub_pixel(struct image_t *img, float x, float y, uint8_t templ);

#endif /* QR_CODE_RECOGNITION_H */
