


#include <stdint.h>
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/snake_gate_detection.h"
#include "modules/computer_vision/lib/vision/open_gate_processing.h"


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

void gate_corner_ref_2(struct image_t* color_image,int *x_points, int *y_points, int* radius)
{
  //printf("enter gate_corner_ref\n");
  //corner point coordinates of snake gate square
  int rad = (*radius);
  int C_1_x = x_points[0];//(*x_center) - (*radius);
  int C_1_y = y_points[0];//(*y_center) + (*radius);
  
  int C_2_x = x_points[1];//(*x_center) + (*radius);
  int C_2_y = y_points[1];//(*y_center) + (*radius);
  
  int C_3_x = x_points[2];//(*x_center) + (*radius);
  int C_3_y = y_points[2];//(*y_center) - (*radius);
  
  int C_4_x = x_points[3];//(*x_center) - (*radius);
  int C_4_y = y_points[3];//(*y_center) - (*radius);
  
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

int array_y[160]; //actual size not known yet.
int cmp_i_y(const void *a, const void *b){
    int ia = *(int *)a;
    int ib = *(int *)b;
    return array_y[ia] < array_y[ib] ? -1 : array_y[ia] > array_y[ib];
}


//without a neccessary gap, histogram might find two close peaks close to each other.
void find_thres_hist_peaks(int* hist_raw, int* hist_peek, int max_hist, int hist_size) {
	float thres_f = 0.4 * (float) max_hist;
	int thres = (int) thres_f;
	memset(hist_peek, 0, sizeof(int)*hist_size);
	//for (int i=0; i<hist_size; i++) {
	//	if (hist_raw[i] > thres) {
	//		hist_peek[i] = hist_raw[i];
	//		i+=15; // once found a peak, the next peak should be at least twenty? pixels away.
	//	}
	//}
	int window_size = 20;
	int outer_loop = hist_size / window_size;
	int residue = hist_size - outer_loop*window_size;
	int window_max=0;
	int window_max_index=0;
	for (int j=0; j<outer_loop; j++) {
		for (int k=0; k<window_size; k++) {
			if (hist_raw[j*window_size+k] > window_max) {
				window_max = hist_raw[j*window_size+k];
				window_max_index = j*window_size+k;
			}
		}
		if (hist_raw[window_max_index] > thres) {
			hist_peek[window_max_index] = hist_raw[window_max_index];
		}
		window_max = 0;
		window_max_index=0;
	}
	for (int m=0; m<residue; m++) {
		if (hist_raw[outer_loop*window_size+m] > window_max) {
			window_max = hist_raw[outer_loop*window_size+m];
			window_max_index = outer_loop*window_size+m;
		}
	}
	if (hist_raw[window_max_index] > thres) {
		hist_peek[window_max_index] = hist_raw[window_max_index];
	}
}

void find_pointy_hist_peaks(int* hist_raw, int* hist_peek, int hist_size) {
	int last_sign = 0;
	int last_idx = 0;
	memset(hist_peek, 0, hist_size*sizeof(int));
	for (int i=1;i<hist_size;i++) {
		int sign = hist_raw[i] - hist_raw[i-1];
		if (last_sign > 0 && sign < 0){
			int m_idx = (int) (i+last_idx)/2.0;
			hist_peek[m_idx] = hist_raw[m_idx];
		}
		if (sign!=0) {
			last_sign = sign;
			last_idx = i;
		}
	}
}

float sort_hist_peaks(int *hist_raw, int hist_size, int side_bars[]) {
	/*
	 * It will be always good if there are two peaks found in histogram of one side.
	 * It won't bring in false positive if no peaks are found since min_ratio_bar will rule them out.
	 * It is however tricky if one peak is found in one side histogram. In this case, if one bar is detected
	 * in initial snake detection, this bar might overlap with the found one.
	 * */
	// histogram_raw can be raw, smoothed or peek histogram.
	//int size = sizeof(hist_raw)/sizeof(*hist_raw);
	//int index[size];
	int index[hist_size];
	int half_hist_size = hist_size/2;

	float peak_thres_f;
	int peak_thres;
	for (int i=0; i<hist_size; i++) {
		index[i] = i;
	}
	memset(array_y, 0, 160*sizeof(int));
	memcpy(array_y, hist_raw, hist_size*sizeof(int));
	qsort(index, hist_size, sizeof(*index), cmp_i_y);

	peak_thres_f =  0.2* (float) hist_raw[index[hist_size-1]];
	peak_thres = (int) peak_thres_f;
	// if the second largest index corresponds to a histogram count that are lower than 0.2*largest
	// (should be at least 0.4 from find_thres_hist_peeks), then that means there is only one peak.
	// The second peak is probably zero. In this case, we cannot trust the comparision of index[hist_size-2]
	// and index[hist_size-1] to decide which one is the lower bar because there are a lot of zeros in hist_raw
	// (hist_left_peak, hist_right peak fed in ogate_histogram function).
	if (hist_raw[index[hist_size-2]] < peak_thres) {
		if (index[hist_size-1] > half_hist_size) {
			side_bars[0] = -1;
			side_bars[1] = index[hist_size-1];
		} else {
			side_bars[0] = index[hist_size-1];
			side_bars[1] = -1;
		}
	} else {
		if (index[hist_size-2] < index[hist_size-1]) {
			side_bars[0] = index[hist_size-2]; //lower index assigned to side_bars[0];
			side_bars[1] = index[hist_size-1];
		} else {
			side_bars[0] = index[hist_size-1];
			side_bars[1] = index[hist_size-2];
		}
	}

	float peek_value = (hist_raw[index[hist_size-2]] + hist_raw[index[hist_size-1]])/2;
	return peek_value;
}

void ogate_histogram(struct image_t* im, struct opengate_img* opengate, float size_factor, int best_bars[]) {
	int best_loc_left=0;
	int best_loc_right=0;
	int best_hist_left = 0;
	int best_hist_right = 0;

	int x = opengate->x;
	int x_range = (int) (size_factor * (float) opengate->size_bar);

	int y_center = (int) ((opengate->y_h + opengate->y_l)/2);
    float y_half = (float) ((opengate->y_h - opengate->y_l)/2);
	int y_range = (int) (size_factor * y_half);
	int y_range_extend = (int) (3 * size_factor * y_half);

	int x_l = (x - x_range < 0)? 0 : x - x_range;
	int x_h = (x + x_range >= im->h) ? im->h-1 : x + x_range;
	//int y_l = (y_center - y_range < 0) ? 0 : y_center - y_range;
	//int y_h = (y_center + y_range > im->w) ? im->w : y_center + y_range;
	int y_l, y_h;
	if ( (opengate->found[0]||opengate->found[3]) && (opengate->found[1]||opengate->found[2]) ) {
		y_l = (y_center - y_range < 0) ? 0 : y_center - y_range;
		y_h = (y_center + y_range >= im->w) ? im->w-1 : y_center + y_range;
	} else if ( (opengate->found[0]||opengate->found[3]) && ~(opengate->found[1]||opengate->found[2]) ) {
		y_l = (y_center - y_range < 0) ? 0 : y_center - y_range;
		y_h = (y_center + y_range_extend >= im->w) ? im->w-1 : y_center + y_range_extend;
	} else if ( ~(opengate->found[0]||opengate->found[3]) && (opengate->found[1]||opengate->found[2]) ) {
		y_l = (y_center - y_range_extend < 0) ? 0 : y_center - y_range_extend;
		y_h = (y_center + y_range >= im->w) ? im->w-1 : y_center + y_range;
	} else {}

	int histogram_size = y_h - y_l + 1;
	int histogram_left[histogram_size];
	memset(histogram_left, 0, histogram_size*sizeof(int));
	int histogram_right[histogram_size];
	memset(histogram_right, 0, histogram_size*sizeof(int));

	for (int y_pix=y_l; y_pix<=y_h; y_pix++) {
		for (int x_pix=x_l; x_pix<x; x_pix++) {
		    if (check_color(im, x_pix, y_pix)) {
				histogram_left[y_pix-y_l]++; //pixel at y_l a.k.a histogram at 0.
			}
		}
	}
	for (int y_pix=y_l; y_pix<=y_h;y_pix++) {
		for (int x_pix=x; x_pix<x_h; x_pix++) {
			if (check_color(im, x_pix, y_pix)) {
				histogram_right[y_pix-y_l]++;
			}
		}
	}
	for (int i=0; i<histogram_size; i++) {
		if (histogram_left[i] > best_hist_left) {
			best_hist_left = histogram_left[i];
			best_loc_left = i;
		}
		if (histogram_right[i] > best_hist_right) {
			best_hist_right = histogram_right[i];
			best_loc_right = i;
		}
	}

	best_loc_left += y_l; // histogram[0] corresponds to lowest y position y_l
	best_loc_right += y_l;

	best_bars[0] = best_loc_left;
	best_bars[1] = best_loc_right;

	int side_bars[2];
	int bars[4];
	int hist_left_peak[histogram_size], hist_right_peak[histogram_size]; //hist_peek set to zero in the next function.

	find_thres_hist_peaks(histogram_left, hist_left_peak, best_hist_left, histogram_size);
	//find_pointy_hist_peaks(hist_left_peak, hist_l_peak_p, histogram_size);
	float peak_value_left = sort_hist_peaks(hist_left_peak, histogram_size, side_bars);
	if (side_bars[0] >= 0) {bars[0] = side_bars[0]+y_l;} else {bars[0] = -1;}
	if (side_bars[1] >= 0) {bars[1] = side_bars[1]+y_l;} else {bars[1] = -1;}


	find_thres_hist_peaks(histogram_right, hist_right_peak, best_hist_right, histogram_size);
	//find_pointy_hist_peaks(hist_right_peak, hist_r_peak_p, histogram_size);
	float peak_value_right = sort_hist_peaks(hist_right_peak, histogram_size, side_bars);
	//bars[2] = side_bars[1]+y_l;
	//bars[3] = side_bars[0]+y_l;
	if (side_bars[1] >= 0) {bars[2] = side_bars[1]+y_l;} else {bars[2] = -1;}
	if (side_bars[0] >= 0) {bars[3] = side_bars[0]+y_l;} else {bars[3] = -1;}

	float min_ratio_bar = 0.3;
	int np, nc;
    struct point_t from, to;

	if (opengate->found[0]==0 && bars[0] >= 0) {
		from.x = x;
		from.y = bars[0];
		to.x = ((x - opengate->size_bar) < 0) ? 0 : x - opengate->size_bar;
		to.y = bars[0];
		if ( (opengate->found[1]>0 && abs(bars[0]-opengate->y_corners[1])>10) || opengate->found[1]==0 ) {
		    check_line(im, from, to, &np, &nc);
			if ((float) nc/ (float) np >= min_ratio_bar) {
				opengate->y_corners[0] = bars[0];
				opengate->x_corners[0] = ((x - opengate->size_bar) < 0) ? 0 : x - opengate->size_bar;
				opengate->found[0] = 2;
				//printf("bar 0 refined\n");
			}
		}

	}
	if (opengate->found[1]==0 && bars[1] >= 0) {
		from.x = x;
		from.y = bars[1];
		to.x = ((x - opengate->size_bar) < 0) ? 0 : x - opengate->size_bar;
		to.y = bars[1];
		if ( (opengate->found[0]>0 && abs(bars[1]-opengate->y_corners[0])>10) || opengate->found[0]==0) {
			check_line(im, from, to, &np, &nc);
			if ((float) nc/ (float) np >= min_ratio_bar) {
				opengate->y_corners[1] = bars[1];
				opengate->x_corners[1] = ((x - opengate->size_bar) < 0) ? 0 : x - opengate->size_bar;
				opengate->found[1] = 2;
				//printf("bar 1 refined\n");
			}
		}
	}
	if (opengate->found[2]==0 && bars[2] >= 0) {
		from.x = x;
		from.y = bars[2];
		to.x = ((x + opengate->size_bar) > im->h) ? im->h : x + opengate->size_bar;
		to.y = bars[2];
		if ( (opengate->found[3]>0 && abs(bars[2]-opengate->y_corners[3])>10) || opengate->found[3]==0 ) {
			check_line(im, from, to, &np, &nc);
			if ((float) nc/ (float) np >= min_ratio_bar) {
				opengate->y_corners[2] = bars[2];
				opengate->x_corners[2] = ((x + opengate->size_bar) > im->h) ? im->h : x + opengate->size_bar;
				opengate->found[2] = 2;
				//printf("bar 2 refined\n");
			}
		}
    }
	if (opengate->found[3]==0 && bars[3] >= 0) {
		from.x = x;
		from.y = bars[3];
		to.x = ((x + opengate->size_bar) > im->h) ? im->h : x + opengate->size_bar;
		to.y = bars[3];
		if ( (opengate->found[2]>0 && abs(bars[3]-opengate->y_corners[2])>10) || opengate->found[2]==0) {
			check_line(im, from, to, &np, &nc);
			if ((float) nc/ (float) np >= min_ratio_bar) {
				opengate->y_corners[3] = bars[3];
				opengate->x_corners[3] = ((x + opengate->size_bar) > im->h) ? im->h : x + opengate->size_bar;
				opengate->found[3] = 2;
				//printf("bar 3 refined\n");
			}
		}
	}
	/* //for debugging
	struct point_t from, to;
	//from.x = x-5;
	//from.y = best_loc_left;
	//to.x = 0;
	//to.y = best_loc_left;
	//image_draw_line(im, &from, &to);
	//from.x = x+5;
	//from.y = best_loc_right;
	//to.x = 315;
	//to.y = best_loc_right;
	//image_draw_line(im, &from, &to);
	//from.x = x;
	//from.y = y_h;
	//to.x = x;
	//to.y = y_l;
	//image_draw_line(im, &from, &to);
	if (opengate->found[0]==0) {
		//from.x = x_l;
		from.x = x - opengate->size_bar;
		from.y = bars[0];
		to.x = x;
		to.y = bars[0];
		image_draw_line(im, &from, &to);
		printf("bar 0 drawn\n");
	}
	if (opengate->found[1]==0) {
		//from.x = x_l;
		from.x = x - opengate->size_bar;
		from.y = bars[1];
		to.x = x;
		to.y = bars[1];
		image_draw_line(im, &from, &to);
		printf("bar 1 drawn\n");
	}
	if (opengate->found[2]==0) {
		from.x = x;
		from.y = bars[2];
		//to.x = x_h;
		to.x = x + opengate->size_bar;
		to.y = bars[2];
		image_draw_line(im, &from, &to);
		printf("bar 2 drawn\n");
	}
	if (opengate->found[3]==0) {
		from.x = x;
		from.y = bars[3];
		//to.x = x_h;
		to.x = x + opengate->size_bar;
		to.y = bars[3];
		image_draw_line(im, &from, &to);
		printf("bar 3 drawn\n");
	}
	printf("bars location A%d, B%d,..., C%d, D%d\n", bars[0], bars[1], bars[2], bars[3]);
	printf("bars found A%d B%d ,..., C%d D%d y_l %d\n\n", opengate->found[0], opengate->found[1], opengate->found[2], opengate->found[3], y_l);*/
}

static void smooth_hist_y(int *smooth, int *raw_hist, int window) {
	for (int i=0; i< window; i++) {
		smooth[i] = 0;
	}
	for (int i=160-window; i<160; i++) {
		smooth[i] = 0;
	}
	for (int i = window; i<160-window; i++) {
		float sum = 0;
		for (int c = -window; c<window; c++) {
			sum += raw_hist[i+c];
		}
		sum/=(window*2);
		smooth[i] = (int) sum;
	}
}

static int find_max_hist_y(int *hist) {
	int max = 0;
	for (int i=0; i<160; i++) {
		if (hist[i] > max) max = hist[i];
	}
	return max;
}

void print_bars(struct image_t *im, int bars[]) {
	struct point_t from, to;
	from.x = 0;
	from.y = bars[0];
	to.x = bars[5];
	to.y = bars[0];
	image_draw_line(im, &from, &to);
	from.x = 0;
	from.y = bars[1];
	to.x = bars[5];
	to.y = bars[1];
	image_draw_line(im, &from, &to);
	from.x = bars[5];
	from.y = bars[2];
	to.x = 315;
	to.y = bars[2];
	image_draw_line(im, &from, &to);
	from.x = bars[5];
	from.y = bars[3];
	to.x = 315;
	to.y = bars[3];
	image_draw_line(im, &from, &to);
}

void print_hist_y(struct image_t* img, int *hist_l, int *hist_r, int gap) {
	int max_hist = (find_max_hist_y(hist_l) > find_max_hist_y(hist_r)) ? find_max_hist_y(hist_l) : find_max_hist_y(hist_r);
	int bound = max_hist/500;
	if (bound<=0) bound = 1;
	for (int i = 0; i < img->w; i++) {
		image_yuv422_set_color(img, img, gap+hist_l[i]/bound, i);
		image_yuv422_set_color(img, img, 315-gap-hist_r[i]/bound, i);
	}
}

