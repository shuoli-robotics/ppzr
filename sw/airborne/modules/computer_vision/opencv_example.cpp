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
float stddev_colors = 20.0;
float prev_stddev_colors;
int mean_u=75;
int mean_v=171;
int prev_mean_u;
int prev_mean_v;

clock_t begin;

RNG rng(12345);
Mat image, mask;
Mat image_color, yuvimage, Z;

uint8_t only_uv_u_lookup[256];
uint8_t only_uv_v_lookup[256];

static unsigned int get_area_in_border(Mat image, int x, int y,
		int width_heigth) {
	Point right_under(y + width_heigth, x + width_heigth);
	Point right_up(y, x + width_heigth);
	Point left_under(y + width_heigth, x);
	Point left_up(y, x);
	return image.at<int>(right_under) - image.at<int>(right_up)
			- image.at<int>(left_under) + image.at<int>(left_up);
}

inline void start_clock() {
	begin = clock();
}
inline void end_clock(char *name_part) {

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / 1000.0;
	printf("%s took %f seconds\n", name_part, elapsed_secs);
}

void init_lookup_table() {

	int max=256/2;
	int meanu2=1255;
	int meanv2=1255;
	for (int x = 0; x < 255; x++) {
		only_uv_u_lookup[x] = max
					* exp(-0.5 * pow((x - mean_u) / stddev_colors, 2.0));

		only_uv_u_lookup[x] += max
					* exp(-0.5 * pow((x - meanu2) / stddev_colors, 2.0));

		only_uv_v_lookup[x] = max
					* exp(-0.5 * pow((x - mean_v) / stddev_colors, 2.0));
		only_uv_v_lookup[x] += max
					* exp(-0.5 * pow((x - meanv2) / stddev_colors, 2.0));
		}

}
void opencv_init_rects() {
	prev_stddev_colors = stddev_colors;
	init_lookup_table();
}

void yuv422_set_color_intensity(Mat colorIntensity,char* img) {
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

void grayscale_hor_sum(Mat grayImage, uint32_t hor_sum[],uint32_t vert_sum[]){
	int n_rows = grayImage.rows;
	int n_cols = grayImage.cols;

	// Iterate over the image,
	int i, j;
	uchar *p;
	int index_img = 0;
	int bin_divider=4;

	// TODO Memset
	for (j = 0; j < n_cols/bin_divider; j ++) {
		hor_sum[j]=0;
	}

	for (j = 0; j < n_rows/bin_divider; j ++) {
		vert_sum[j]=0;
	}

	// set each sum
	for (i = 0; i < n_rows; ++i) {
		p = grayImage.ptr<uchar>(i);
		for (j = 0; j < n_cols; j++) {
			hor_sum[j/bin_divider]+=p[j];
			vert_sum[i/bin_divider]+=p[j];
		}
	}

	// Now smooth!
	int smooth_dist=5;
	for (j = n_cols/bin_divider;j>smooth_dist; j--) {
		int sum=0;
		for(int x=0;x<smooth_dist;x++){
			sum+=hor_sum[j-x];
		}
		hor_sum[j]=sum;
	}

	for (j = n_rows/bin_divider;j>smooth_dist; j--) {
		int sum=0;
		for(int x=0;x<smooth_dist;x++){
			sum+=vert_sum[j-x];
		}
		vert_sum[j]=sum;
	}

	int maxLocation2=0;
	uint32_t maxValue2=0;
	int maxLocation_vert=0;
	int max_value_vert=0;
	for (j = 0; j < n_rows/bin_divider; j ++) {
		if(vert_sum[j]>max_value_vert){
			max_value_vert=vert_sum[j];
			maxLocation_vert = j;
		}
	}
	for (j = 0; j < n_cols/bin_divider; j ++) {
		if(hor_sum[j]>maxValue2){
			maxValue2=hor_sum[j];
			maxLocation2 = j;
		}

	}

//	line(grayImage,Point(0,maxLocation*bin_divider),Point(n_rows,maxLocation*bin_divider),Scalar(255,255,255),5);

	//line(grayImage,Point(maxLocation2*bin_divider,0),Point(maxLocation2*bin_divider,n_rows),Scalar(255,255,255),5);
	loc_y=maxLocation_vert*bin_divider;
	line(grayImage,Point(0,maxLocation_vert*bin_divider),Point(n_cols,maxLocation_vert*bin_divider),Scalar(255,255,255),5);

	//	C++: void line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)

	printf("%d %d\n",maxLocation2,maxValue2);
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

int opencv_example(char *img, int width, int height) {
	if(changedParameters()){
		init_lookup_table();
	}
	Mat M(height, width, CV_8UC2, img); // original
	Mat probImage(height,width,CV_8UC1); // prob projected
	yuv422_set_color_intensity(probImage,img);

	uint32_t hor_sum_image[width];
	uint32_t vert_sum_image[height];

	grayscale_hor_sum( probImage,hor_sum_image,vert_sum_image);
	grayscale_opencv_to_yuv422(probImage, img, width, height);
	return 0;
}
