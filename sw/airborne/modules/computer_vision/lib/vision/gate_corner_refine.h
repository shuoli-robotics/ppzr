

#include <stdint.h>
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"


void gate_corner_ref(struct image_t* color_image,int *x_points, int *y_points, int* x_center, int* y_center, int* radius, uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y);
void draw_shape(struct image_t* im, int c1_x,int c1_y,int c2_x,int c2_y,int c3_x,int c3_y,int c4_x,int c4_y);
void refine_corner(struct image_t* im, int corner_x, int corner_y, int size, float size_factor);
void gate_corner_ref_2(struct image_t* color_image,int *x_points, int *y_points, int size);