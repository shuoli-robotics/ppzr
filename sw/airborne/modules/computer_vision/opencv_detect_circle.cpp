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

#include "opencv_detect_gate.h"

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
int mean_u = 75;
int mean_v = 171;
int prev_mean_u;
int prev_mean_v;
int too_close = 0;
clock_t begin;
int meanh1 = 0;
int meanh2 = 180; // H between 0 and 180 (360/2)
bool changed_parameter;
int super_roll = 0;
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

uint8_t h_color_u = 20;
uint8_t s_color_u = 255;
uint8_t v_color_u  = 255;
uint8_t h_color_l = 0;
uint8_t s_color_l = 30;
uint8_t v_color_l  = 10;

double dp = 1;
double param1 = 10;
double param2 = 100;

/** Functions for measuring some time */
inline void start_clock() {
	begin = clock();
}
inline void end_clock(char *name_part) {
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / 1000.0;
	printf("%s took %f seconds\n", name_part, elapsed_secs);
}

struct ValueAndPos {
	float value;
	int position;
};

/**
 * Gaussian distribution
 */
static inline uint8_t gaus(int amp, int mean, float stdev, int pos) {
	return amp * exp(-0.5 * pow((pos - mean) / stdev, 2.0));
}
void set_blue_window(){
	meanh1=98;
	meanh2=98;
	changed_parameter=true;
}
void set_red_window(){
	meanh1=0;
	meanh2=180;

	changed_parameter=true;
}
/** Initialise the lookup table used in the color seperation
 */
static void init_lookup_table() {

	int max = 256 / 2;
	int meanu2 = 1255;
	int meanv2 = 1255;

	for (int x = 0; x < 255; x++) {
		only_uv_u_lookup[x] = gaus(max, mean_u, stddev_colors, x);
		only_uv_u_lookup[x] += gaus(max, meanu2, stddev_colors, x);

		only_uv_v_lookup[x] = gaus(max, mean_v, stddev_colors, x);
		only_uv_v_lookup[x] += gaus(max, meanv2, stddev_colors, x);

		h_lookup[x] = gaus(256, meanh1, stddev_colors, x);
		h_lookup[x] += gaus(256, meanh2, stddev_colors, x);
	}
}
void opencv_init_rects() {
	prev_stddev_colors = stddev_colors;
	init_lookup_table();
}

/* Sets the color intensity in a single chanel image
 * based on the passed UYVY image img.
 * Note that the color_intensity image should have a specified amount of rows
 * and columns.
 */
static void yuv422_set_color_intensity(Mat color_intensity, char* img) {
	int n_rows = color_intensity.rows;
	int n_cols = color_intensity.cols;

	// If the image is one block in memory we can iterate over it all at once!
	if (color_intensity.isContinuous()) {
		n_cols *= n_rows;
		n_rows = 1;
	}

	// Iterate over the image,
	int i, j;
	uchar *p;
	int index_img = 0;
	for (i = 0; i < n_rows; ++i) {
		p = color_intensity.ptr<uchar>(i);
		for (j = 0; j < n_cols; j += 2) {
			// U Y V Y
			if (only_uv_u_lookup[img[index_img + 1]] < 170) { // Max Y value
				p[j] = only_uv_u_lookup[img[index_img]]
						+ only_uv_v_lookup[img[index_img + 2]];
			} else {
				p[j] = 0;
			}
			if (only_uv_u_lookup[img[index_img + 3]] < 170) {
				p[j + 1] = only_uv_u_lookup[img[index_img]]
						+ only_uv_v_lookup[img[index_img + 2]];
			} else {
				p[j + 1] = 0;
			}
			index_img += 4; // TODO p+=2
		}
	}

}

static void hsv_set_color_intensity(Mat color_intensity, Mat hsvImage) {
	int n_rows = color_intensity.rows;
	int n_cols = color_intensity.cols;

	// If the image is one block in memory we can iterate over it all at once!
	if (color_intensity.isContinuous()) {
		n_cols *= n_rows;
		n_rows = 1;
	}

	// Iterate over the image,
	int i, j;
	uchar *p;
	uchar *hsvRow;
	int index_img = 0;
	for (i = 0; i < n_rows; ++i) {
		p = color_intensity.ptr<uchar>(i);
		hsvRow = hsvImage.ptr<uchar>(i);
		for (j = 0; j < n_cols; j++) {
			p[j] = h_lookup[hsvRow[0]];
			hsvRow += 3;
		}
	}

}

/**
 * Smooths array in place by taking the average of smooth_dist values.
 * Single pass.
 * First smooth_dist elements are not smoothed
 */
static void smooth_array(float array[], int length_array, int smooth_dist) {
	for (int j = length_array - 1; j > smooth_dist; j--) {
		int sum = 0;
		for (int x = 0; x < smooth_dist; x++) {
			sum += array[j - x];
		}
		array[j] = sum / float(smooth_dist);
	}
}

/**
 * Checks if any of the global parameters changed.
 */
bool changed_parameters() {
	if (changed_parameter || prev_mean_u != mean_u || prev_mean_v != mean_v
			|| prev_stddev_colors != stddev_colors ) {
		prev_mean_u = mean_u;
		prev_mean_v = mean_v;
		prev_stddev_colors = stddev_colors;

		return true;

	}
	return false;
}

/**
 * Given an integral image, returns the average value in a certain area.
 */
float get_average_value_area(int min_x, int min_y, int max_x, int max_y,
		Mat integral_image) {
	int w = max_x - min_x + 1;
	int h = max_y - min_y + 1;
	float n_pix = w * h;
	int sum_disparities = integral_image.at<int>(min_y, min_x)
			+ integral_image.at<int>(max_y, max_x)
			- integral_image.at<int>(min_y, max_x)
			- integral_image.at<int>(max_y, min_x);
	float dd = sum_disparities / n_pix;
	return dd;
}

/** Compare function for two ValueAndPos structs
 *  Looks at the value (not the pos) of the two structs.
 */
int compare_value_and_pos(const void *a, const void *b) {
	struct ValueAndPos *da = (struct ValueAndPos *) a;
	struct ValueAndPos *db = (struct ValueAndPos *) b;
	int val1 = da->value * 1000.0;
	int val2 = db->value * 1000.0;
	return val2 - val1;
}
void guidoMethod(Mat probImage) {
	uint32_t totalResponse = 0;
	int n_rows = probImage.rows;
	int n_cols = probImage.cols;

	// Check if we are too close to the obstacle,
	// by counting the amount of activated pixels.
	// At the time of writing this comment too close means
	// More than 25 percent of the pixels activated.
	if (countNonZero(probImage) > (n_rows * n_cols) * 0.25) {
		too_close = 1;
	} else {
		too_close = 0;
	}

	// Get the integral image of the activated pixels
	Mat my_int(n_rows, n_cols, CV_32S);
	integral(probImage, my_int, CV_32S);

	// Create an activation array.
	// Perform a center surround (but not really surround) square feature search over the whole image
	// And store the values in Ver
	int border_size = 8;
	int guidoLength = n_rows - 3 * border_size;
	float vertical_activation[guidoLength];
	int border_y = 0; // borders can be used to filter out parts of the image that are irrelevant
	int border_x = 0; // examples: floor, ceiling, sides of the drone.
	for (int y = 0; y < guidoLength; y++) {
		vertical_activation[y] = -get_average_value_area(border_x, y, n_cols - border_x,
				y + 3 * border_size, my_int)
				+ 2
						* get_average_value_area(border_x, y + border_size,
								n_cols - border_x, y + 2 * border_size - 1,
								my_int);
	}

	// Smooth the array
	smooth_array(vertical_activation, guidoLength, 4);

	// find peaks by searching for places that are higher than their neighbours
	struct ValueAndPos values[guidoLength];
	int amountPeaks = 0;
	for (int i = 1; i < guidoLength - 1; i++) {
		if (vertical_activation[i] > vertical_activation[i - 1] && vertical_activation[i] > vertical_activation[i + 1]) {
			values[amountPeaks].position = i;
			values[amountPeaks].value = vertical_activation[i];
			amountPeaks++;
		}
	}

	if (amountPeaks >= 2) {
		// sort the peaks based on the height of the peak.
		qsort(values, amountPeaks, sizeof(struct ValueAndPos),
				compare_value_and_pos);
		float mean_vals = (values[0].value + values[1].value) / 2;

		// Check if this is indeed a window
		// Correcsponds to the cross in Guido's MATLAB code.
		// The treshold for
		float tresholdActuallyGate = 0.4;
		float some_sim = 1.0 - values[1].value / values[0].value;
		if (mean_vals > 1.0
				&& fabs(some_sim) < tresholdActuallyGate) {
			for (int i = 0; i < 2; i++) {
				line(probImage, Point(0, values[i].position),
						Point(n_cols, values[i].position),
						Scalar(255, 255, 255), 5);
			}

			int dist = values[0].position - values[1].position;
			if (dist < 0) {
				dist *= -1;
			}
			distance_pixels = dist;

			loc_y = (values[0].position + values[1].position) / 2;
			center_pixels = loc_y;
			left_height = values[0].value;
			right_height = values[1].value;
			if (values[0].position > values[1].position) {
				left_height = values[1].value;
				right_height = values[0].value;
			}
		} else {
			loc_y = n_rows / 2;
		}
	} else {
		// Less than 2 peaks? set the loc_y to the center of the screen
		loc_y = n_rows / 2;
	}

}
int opencv_gate_detect(char *img, int width, int height) {
	/*meanh1++;
		meanh2=meanh1;
		changed_parameter=true;
		printf("meanh1 %d\n",meanh1);
		if(meanh1>max_h){
			meanh1=min_h;
		}*/
	if (changed_parameters()) {
		init_lookup_table();
		changed_parameter=false;
	}
	Mat M(height, width, CV_8UC2, img); // original
	Mat hsvImage;
	Mat rgbImage;
	Mat enhancedImage;
	Mat grayImage;
	Mat red_1;
	//Mat red_2;
	cvtColor(M, rgbImage, CV_YUV2BGR_Y422);
	//cvtColor(rgbImage, grayImage, CV_BGR2GRAY);
	cvtColor(rgbImage, hsvImage, CV_BGR2HSV);
	
	//HERE CIRCLE DETECTION
	
	/// Reduce the noise so we avoid false circle detection
	//GaussianBlur( grayImage, grayImage, Size(9, 9), 2, 2 );

	//rgbImage.convertTo(enhancedImage,-1,1,0);
	
	vector<Vec3f> circles;
	
	//inRange(hsvImage,Scalar(0,30,10),Scalar(20,255,255),red_1);
	inRange(hsvImage,Scalar(h_color_l,s_color_l,v_color_l),Scalar(h_color_u,s_color_u,v_color_u),red_1);
	
	//inRange(hsvImage,Scalar(230,50,50),Scalar(255,255,255),red_2);
	
	//GaussianBlur( enhancedImage, enhancedImage, Size(9, 9), 2, 2 );

	/// Apply the Hough Transform to find the circles //200 100
	//HoughCircles( grayImage, circles, CV_HOUGH_GRADIENT, 0.5, grayImage.rows/8, 10, 100, 0, 0 );
	
	//HoughCircles( red_1, circles, CV_HOUGH_GRADIENT, 2, grayImage.rows/8, 10, 100, 0, 0 );
	HoughCircles( red_1, circles, CV_HOUGH_GRADIENT, dp, grayImage.rows/8, param1, param2, 0, 0 );
	
	
	

	/// Draw the circles detected
	for( size_t i = 0; i < circles.size(); i++ )
	{
	    Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	    int radius = cvRound(circles[i][2]);
	    // circle center
	    //circle( grayImage, center, 3, 255, -1, 8, 0 );
	    // circle outline
	    circle( red_1, center, radius, 200, 4, 8, 0 );
	}
	
	Point center(20, 20);
	//circle test      center radius 
	circle( red_1, center, 30, 200, 4, 8, 0 );
	
	//Mat probImage(height, width, CV_8UC1); // prob projected
	//hsv_set_color_intensity(probImage, hsvImage);
	//guidoMethod(probImage);
	//grayscale_opencv_to_yuv422(probImage, img, width, height);
	//grayscale_opencv_to_yuv422(grayImage, img, width, height);
	grayscale_opencv_to_yuv422(red_1, img, width, height);
	//colorrgb_opencv_to_yuv422(enhancedImage, img, width, height);
	return 0;
}
