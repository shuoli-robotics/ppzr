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
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#include "opencv_example.h"

#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "modules/computer_vision/lib/vision/image.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;
#include "opencv_image_functions.h"
#include <ctime>
int loc_y;
float stddev_colors = 7.0;
float prev_stddev_colors;
int mean_u=75;
int mean_v=171;
int prev_mean_u;
int prev_mean_v;
int too_close=0;
clock_t begin;
int super_roll=0;
RNG rng(12345);
Mat image, mask;
Mat image_color, yuvimage, Z;
int16_t distance_pixels;
int16_t center_pixels;
 int16_t left_height;
 int16_t right_height;

uint8_t only_uv_u_lookup[256];
uint8_t only_uv_v_lookup[256];
uint8_t h_lookup[256];

inline void start_clock() {
	begin = clock();
}
inline void end_clock(char *name_part) {

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / 1000.0;
	printf("%s took %f seconds\n", name_part, elapsed_secs);
}

static void init_lookup_table() {

	int max=256/2;
	int meanu2=1255;
	int meanv2=1255;
	int meanh1=0;
	int meanh2=180; // H between 0 and 180 (360/2)
	for (int x = 0; x < 255; x++) {
		only_uv_u_lookup[x] = max
					* exp(-0.5 * pow((x - mean_u) / stddev_colors, 2.0));

		only_uv_u_lookup[x] += max
					* exp(-0.5 * pow((x - meanu2) / stddev_colors, 2.0));

		only_uv_v_lookup[x] = max
					* exp(-0.5 * pow((x - mean_v) / stddev_colors, 2.0));
		only_uv_v_lookup[x] += max
					* exp(-0.5 * pow((x - meanv2) / stddev_colors, 2.0));

		h_lookup[x] = 256* exp(-0.5 * pow((x - meanh1) / stddev_colors, 2.0));
		h_lookup[x] += 256* exp(-0.5 * pow((x - meanh2) / stddev_colors, 2.0));

		}

}
void opencv_init_rects() {
	prev_stddev_colors = stddev_colors;
	init_lookup_table();
}

static void yuv422_set_color_intensity(Mat colorIntensity,char* img) {
	int n_rows = colorIntensity.rows;
	int n_cols = colorIntensity.cols;

	// If the image is one block in memory we can iterate over it all at once!
	if (colorIntensity.isContinuous()) {
		n_cols *= n_rows;
		n_rows = 1;
	}

	// Iterate over the image,
	int i, j;
	uchar *p;
	int index_img = 0;
	for (i = 0; i < n_rows; ++i) {
		p = colorIntensity.ptr<uchar>(i);
		for (j = 0; j < n_cols; j += 2) {
			// U Y V Y
			if(only_uv_u_lookup[img[index_img+1]]<170){ // Max Y value
				p[j] = only_uv_u_lookup[img[index_img]]
					+ only_uv_v_lookup[img[index_img + 2]];
			}
			else{
				p[j]=0;
			}
			if(only_uv_u_lookup[img[index_img+3]]<170){
				p[j + 1] = only_uv_u_lookup[img[index_img]]
					+ only_uv_v_lookup[img[index_img + 2]];
			}
			else{
				p[j+1]=0;
			}
			index_img += 4; // TODO p+=2
		}
	}

}


static void hsv_set_color_intensity(Mat colorIntensity,Mat hsvImage) {
	int n_rows = colorIntensity.rows;
	int n_cols = colorIntensity.cols;

	// If the image is one block in memory we can iterate over it all at once!
	if (colorIntensity.isContinuous()) {
		n_cols *= n_rows;
		n_rows = 1;
	}

	// Iterate over the image,
	int i, j;
	uchar *p;
	uchar *hsvRow;
	int index_img = 0;
	for (i = 0; i < n_rows; ++i) {
		p = colorIntensity.ptr<uchar>(i);
		hsvRow = hsvImage.ptr<uchar>(i);
		for (j = 0; j < n_cols; j ++) {
			p[j]=h_lookup[hsvRow[0]];
			hsvRow+=3;
		}
	}

}

static void smooth_array(float array[],int length_array, int smooth_dist ){
		for (int j = length_array-1;j>smooth_dist; j--) {
			int sum=0;
			for(int x=0;x<smooth_dist;x++){
				sum+=array[j-x];
			}
			array[j]=sum/float(smooth_dist);
		}
}

bool changedParameters(){
	if(prev_mean_u!=mean_u || prev_mean_v != mean_v || prev_stddev_colors != stddev_colors){
		prev_mean_u = mean_u;
		prev_mean_v = mean_v;
		prev_stddev_colors = stddev_colors;
		return true;

	}
	return false;
}


float getAverageValueArea(int min_x, int min_y,int max_x,int max_y, Mat integral_image){
//% function dd = getAvgDisparity(min_x, min_y, max_x, max_y, II)
		int w = max_x - min_x + 1;
	int h = max_y - min_y + 1;
	float n_pix = w * h;
	int sum_disparities = integral_image.at<int>(min_y, min_x) +  integral_image.at<int>(max_y, max_x) - integral_image.at<int>(min_y, max_x) - integral_image.at<int>(max_y, min_x);
	float dd = sum_disparities / n_pix;
	return dd;
}

struct ValueAndPos{
	float value;
	int position;
};

int compare_doubles (const void *a, const void *b)
{
	struct ValueAndPos *da = (struct ValueAndPos *) a;
	struct ValueAndPos *db = (struct ValueAndPos *) b;
	int val1 = da->value*1000.0;
	int val2 = db->value*1000.0;
  return val2-val1;
}
void guidoMethod(Mat probImage){
	uint32_t totalResponse=0;
	int n_rows = probImage.rows;
	int n_cols = probImage.cols;
	printf("%d %d \n",n_rows,n_cols);
	if(countNonZero(probImage)>(n_rows*n_cols)/4){
		too_close=1;
		return;
	}
	else{
		too_close=0;
	}
	Mat my_int(n_rows,n_cols,CV_32S);

	integral(probImage,my_int,CV_32S);
	int border_size=8;
	int border_y=0;
	int border_x=0;
	int guidoLength=n_rows-3*border_size;
	float Ver [guidoLength];
	for(int i=0;i<guidoLength;i++){
		Ver[i]=0.0;
	}
	//% Center surround ding
	for(int y =0;y< guidoLength;y++){
		   Ver[y] = - getAverageValueArea(border_x, y, n_cols-border_x,y+3*border_size, my_int) +  2 * getAverageValueArea(border_x,y+border_size, n_cols-border_x,y+2*border_size-1, my_int);
	}

	smooth_array(Ver,guidoLength, 4 );
	struct ValueAndPos values[guidoLength];
	int amountPeaks=0;
	// find peaks

	for(int i=1;i<guidoLength-1;i++){
		if(Ver[i]>Ver[i-1]&&Ver[i]>Ver[i+1]){
			values[amountPeaks].position=i;
			values[amountPeaks].value=Ver[i];
			amountPeaks++;
		}
	}

	// sort on size
	if(amountPeaks>=2){
	  qsort (values, amountPeaks, sizeof (struct ValueAndPos), compare_doubles);
	  //printf("values largest: %f %f %d %d\n",values[0].value,values[1].value,values[0].position,values[1].position);
	  float meanVals = (values[0].value+values[1].value)/2;

	     if(meanVals > 1.0 && abs(1-values[1].value/values[0].value) < 0.5){
////		  // draw
		  for(int i=0;i<2;i++){
		//	  printf("%f at position %d\n",values[i].value,values[i].position);
			  line(probImage,Point(0,values[i].position),Point(n_cols,values[i].position),Scalar(255,255,255),5);
		  }

		  	  int dist =values[0].position-values[1].position;
		  	  if(dist < 0){
		  		  dist *=-1;
		  	  }
			distance_pixels = dist ;

		     loc_y=(values[0].position+values[1].position)/2;
		     center_pixels=loc_y;
		     int idx_l=0;
		     int idx_r=1;
		     left_height = values[0].value;
		     right_height = values[1].value;
		     if(values[0].position>values[1].position){
		    	 idx_l=1;
		    	 idx_r=0;

			     left_height = values[1].value;
			     right_height = values[0].value;
		     }
		     printf("Left %f Right %f\n",values[idx_l].value,values[idx_r].value);
		     float toal = values[idx_l].value+values[idx_r].value;
		     if(values[idx_l].value>toal*.66){
		    	 super_roll=1;
		     }
		     else if(values[idx_r].value>toal*.66){
		    	 super_roll=-1;
		     }
		     else{
		    	 super_roll=0;
		     }
	     }
	     else{
	    	 loc_y=n_rows/2;
	     }
	}else{
		loc_y=n_rows/2;
	}

}
int opencv_example(char *img, int width, int height) {
	if(changedParameters()){
		init_lookup_table();
	}
	Mat M(height, width, CV_8UC2, img); // original
	Mat hsvImage;
	Mat rgbImage;
	cvtColor(M,rgbImage,CV_YUV2BGR_Y422);

	cvtColor(rgbImage,hsvImage,CV_BGR2HSV);


	Mat probImage(height,width,CV_8UC1); // prob projected
	hsv_set_color_intensity(probImage,hsvImage);

	//	yuv422_set_color_intensity(probImage,img);
	guidoMethod(probImage);
	grayscale_opencv_to_yuv422(probImage, img, width, height);
	return 0;
}
