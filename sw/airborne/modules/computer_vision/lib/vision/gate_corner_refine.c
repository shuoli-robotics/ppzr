


#include <stdint.h>
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/snake_gate_detection.h"


uint8_t g_color[4] = {255,128,255,128};
int bound_value_int(int input, int min, int max);

//AXIS system
//(0,0)   (160
//       Y
// -|----->
//  |
//  |
//  |
//  |
//  \/
// X
//320

void gate_corner_ref(struct image_t* color_image,int *x_points, int *y_points, int* x_center, int* y_center, int* radius, uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y)
{
  //printf("enter gate_corner_ref\n");
  //corner point coordinates of snake gate square
  int rad = (*radius);
  int C_1_x = (*x_center) - (*radius);
  int C_1_y = (*y_center) + (*radius);
  
  int C_2_x = (*x_center) + (*radius);
  int C_2_y = (*y_center) + (*radius);
  
  int C_3_x = (*x_center) + (*radius);
  int C_3_y = (*y_center) - (*radius);
  
  int C_4_x = (*x_center) - (*radius);
  int C_4_y = (*y_center) - (*radius);
  
  
  float corner_area = 0.4f;
  refine_corner(color_image,&C_1_x,&C_1_y,rad,corner_area);
  refine_corner(color_image,&C_2_x,&C_2_y,rad,corner_area);
  refine_corner(color_image,&C_3_x,&C_3_y,rad,corner_area);
  refine_corner(color_image,&C_4_x,&C_4_y,rad,corner_area);
  
  //draw_shape(color_image,C_1_x,C_1_y,C_2_x,C_2_y,C_3_x,C_3_y,C_4_x,C_4_y);
  
  x_points[0] = C_1_x;
  y_points[0] = C_1_y;
  x_points[1] = C_2_x;
  y_points[1] = C_2_y;
  x_points[2] = C_3_x;
  y_points[2] = C_3_y;
  x_points[3] = C_4_x;
  y_points[3] = C_4_y;
  //printf("exit gate_corner_ref\n");
  
}

void refine_corner(struct image_t* im, int *corner_x, int *corner_y, int size, float size_factor)
{
  //printf("enter refine_corner\n");
  float x_corner_f = (float)(*corner_x);
  float y_corner_f = (float)(*corner_y);
  float size_f     = (float)size;
  
  //printf("size_factor:%f\n",size_factor);//not working???
  size_factor = 0.4;
  
  
  int x_l = (int)(x_corner_f-size_f*size_factor);
  x_l = bound_value_int(x_l,0,320);
  int x_r = (int)(x_corner_f+size_f*size_factor);
  x_r = bound_value_int(x_r,0,320);
  int y_h = (int)(y_corner_f+size_f*size_factor);
  y_h = bound_value_int(y_h,0,160);
  int y_l = (int)(y_corner_f-size_f*size_factor);
  y_l = bound_value_int(y_l,0,160);
  
  
  int c1_x = x_l;
  int c1_y = y_h;
  
  int c2_x = x_r;
  int c2_y = y_h;
  
  int c3_x = x_r;
  int c3_y = y_l;
  
  int c4_x = x_l;
  int c4_y = y_l;
  
    int x_size = x_r-x_l+1;
    int y_size = y_h-y_l+1;
    
    int x_hist[x_size];
    int y_hist[y_size];
    memset(x_hist, 0, sizeof(int)*x_size);
    memset(y_hist, 0, sizeof(int)*y_size);
    
    
    int best_x = 0;
    int best_x_loc = x_l;
    int x_best_start = x_l;
    int best_y = 0;
    int best_y_loc = y_l;
    int y_best_start = y_l;
    
    
    for(int y_pix = y_l; y_pix < y_h; y_pix++){
        for(int x_pix = x_l; x_pix < x_r; x_pix++){
            if(check_color(im,x_pix,y_pix)>0){
                
                int cur_x = x_hist[x_pix-x_l];
                int cur_y = y_hist[y_pix-y_l];
		
		x_hist[x_pix-x_l] = cur_x+1;
                y_hist[y_pix-y_l] = cur_y+1;
                
                if(x_hist[x_pix-x_l] > best_x){
                    best_x = x_hist[x_pix-x_l];
                    best_x_loc = x_pix;
                    x_best_start = x_pix;
		}
                else if(cur_x == best_x){
                    best_x_loc = (x_pix+x_best_start)/2;
		}
                if(y_hist[y_pix-y_l] > best_y){
                    best_y = y_hist[y_pix-y_l];
                    best_y_loc = y_pix;
                    y_best_start = y_pix;
		}
                else if(cur_y == best_y){
                    best_y_loc = (y_pix+y_best_start)/2;
		}
                
	}
      }
    }
    
    //refined_corner = [best_x_loc,best_y_loc];
//   struct point_t from, to;
//   from.x = x_l;
//   from.y = best_y_loc;
//   to.x = x_r;
//   to.y = best_y_loc;
//   image_draw_line(im, &from, &to);
//   
//   from.x = best_x_loc;
//   from.y = y_l;
//   to.x = best_x_loc;
//   to.y = y_h;
//   image_draw_line(im, &from, &to);
//     
//   
//    draw_shape(im,c1_x,c1_y,c2_x,c2_y,c3_x,c3_y,c4_x,c4_y);
  
  *corner_x = best_x_loc;
  *corner_y = best_y_loc;
  
  //draw_cross(im,best_x_loc,best_y_loc,g_color);
  
  //printf("enter refine_corner\n");
}

void draw_shape(struct image_t* im, int c1_x,int c1_y,int c2_x,int c2_y,int c3_x,int c3_y,int c4_x,int c4_y)
{
  //printf("enter draw_shape\n");
   struct point_t from, to;
  from.x = c1_x;
  from.y = c1_y;
  to.x = c2_x;
  to.y = c2_y;
  image_draw_line(im, &from, &to);
  from.x = c2_x;
  from.y = c2_y;
  to.x = c3_x;
  to.y = c3_y;
  image_draw_line(im, &from, &to);
  from.x = c3_x;
  from.y = c3_y;
  to.x = c4_x;
  to.y = c4_y;
  image_draw_line(im, &from, &to);
  from.x = c4_x;
  from.y = c4_y;
  to.x = c1_x;
  to.y = c1_y;
  image_draw_line(im, &from, &to);
  //printf("exit draw_shape\n");
}

int bound_value_int(int input, int min, int max){
  if(input < min){
    return min;
  }
  else if(input > max){
    return max;
  }
  else{
    return input;
  }
}

