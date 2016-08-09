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
Point origin;
Rect selection;
RotatedRect trackBox;
int loc_y;
float stddev_colors = 25.0;
float prev_stddev_colors;

struct point_t rect_top;
struct point_t rect_size;
/** my desk **/
/*
 int cvcolor_lum_min=0,cvcolor_lum_max=53,cvcolor_cb_min=121;
 int cvcolor_cb_max=135,cvcolor_cr_min=80,cvcolor_cr_max=103;*/
/** Cyber zoo **/
int cvcolor_lum_min = 0, cvcolor_lum_max = 178, cvcolor_cb_min = 128;
int cvcolor_cb_max = 169, cvcolor_cr_min = 109, cvcolor_cr_max = 136;
int function_to_send_opencv = 4;
clock_t begin;

RNG rng(12345);

float mean_red = 100.0;
float mean_blue = -100.0;
float mean_green = 140.0;
Mat image, mask;
Mat image_color, yuvimage, Z;
uint8_t red_lookup[256];
uint8_t green_lookup[256];
uint8_t blue_lookup[256];

float y_lookup[256];
float u_lookup[256];
float v_lookup[256];

uint8_t only_uv_u_lookup[256];
uint8_t only_uv_v_lookup[256];

/*
// YUV in opencv convert to YUV on Bebop
void yuv_opencv_to_yuv422(Mat image, char *img, int width, int height) {
//Turn the opencv RGB colored image back in a YUV colored image for the drone
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			// Extract pixel color from image
			cv::Vec3b &c = image.at<cv::Vec3b>(row, col);

			// Set image buffer values
			int i = row * width + col;
			img[2 * i + 1] = c[0]; // y;
			img[2 * i] = col % 2 ? c[1] : c[2]; // u or v
		}
	}
}

void uyvy_opencv_to_yuv_opencv(Mat image, Mat image_in, int width, int height) {
//Turn the opencv RGB colored image back in a YUV colored image for the drone
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			// Extract pixel color from image
			cv::Vec3b c = image_in.at<cv::Vec3b>(row, col);
			cv::Vec3b c_m1 = image_in.at<cv::Vec3b>(row, col);
			cv::Vec3b c_p1 = image_in.at<cv::Vec3b>(row, col);
			if (col > 0) {
				c_m1 = image_in.at<cv::Vec3b>(row, col - 1);
			}
			if (col < width) {
				c_p1 = image_in.at<cv::Vec3b>(row, col + 1);
			}
			image.at<cv::Vec3b>(row, col)[0] = c[1];
			image.at<cv::Vec3b>(row, col)[1] = col % 2 ? c[0] : c_m1[0];
			image.at<cv::Vec3b>(row, col)[2] = col % 2 ? c_p1[0] : c[0];

		}
	}
}*/

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
	//40
	int meanu1=40;
	int meanu2=255;

//200
	int meanv1=175;
	int meanv2=255;
	for (int x = 0; x < 255; x++) {
		only_uv_u_lookup[x] = max
					* exp(-0.5 * pow((x - meanu1) / stddev_colors, 2.0));

		only_uv_u_lookup[x] += max
					* exp(-0.5 * pow((x - meanu2) / stddev_colors, 2.0));

		only_uv_v_lookup[x] = max
					* exp(-0.5 * pow((x - meanv1) / stddev_colors, 2.0));
		only_uv_v_lookup[x] += max
					* exp(-0.5 * pow((x - meanv2) / stddev_colors, 2.0));
		}

}
void opencv_init_rects() {
	selection = Rect(Point(0, 0), Point(400, 400));
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

	line(grayImage,Point(maxLocation2*bin_divider,0),Point(maxLocation2*bin_divider,n_rows),Scalar(255,255,255),5);

	line(grayImage,Point(0,maxLocation_vert*bin_divider),Point(n_cols,maxLocation_vert*bin_divider),Scalar(255,255,255),5);

	//	C++: void line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)

	printf("%d %d\n",maxLocation2,maxValue2);
}

Mat colorIntensityYUV(830, 512, CV_8U);

void opencv_distance_shape(Mat imageGrayscale, char *img, int width,
		int height) {
	Canny(imageGrayscale, imageGrayscale, 35, 35 * 3);
	bitwise_not(imageGrayscale, imageGrayscale);
	distanceTransform(imageGrayscale, imageGrayscale, CV_DIST_L2, 3);
}

void opencv_back_projection(Mat imageHSV, int width, int height) {
	/// Get Backprojection
	Mat backproj;
	MatND hist;
	int bins=6;
	int histSize = MAX(bins, 2);
	float hue_range[] = { 0, 180 };
	const float* ranges = { hue_range };

	calcBackProject(&imageHSV, 1, 0, hist, backproj, &ranges, 1, true);

}


void draw_line_example(Mat imggray, char* img, int width, int height){
	Rect bounding_rect;
	bounding_rect.x=100;
	bounding_rect.y=100;
	bounding_rect.width=100;
	bounding_rect.height=100;

	rectangle(imggray, bounding_rect, Scalar(140,200,10), 2, 8, 0);
	grayscale_opencv_to_yuv422(imggray, img, width, height);
}
int opencv_example(char *img, int width, int height) {

	Mat M(height, width, CV_8UC2, img); // original
	Mat probImage(height,width,CV_8UC1); // prob projected
	yuv422_set_color_intensity(probImage,img);

	uint32_t hor_sum_image[width];
	uint32_t vert_sum_image[height];

	grayscale_hor_sum( probImage,hor_sum_image,vert_sum_image);

	grayscale_opencv_to_yuv422(probImage, img, width, height);

/*


		  	const int channels[] = {0, 1, 2};
	    const int histSize[] = {32, 32, 32};
	    const float rgbRange[] = {0, 256};
	    const float* ranges[] = {rgbRange, rgbRange, rgbRange};
	    MatND hist;

	    // compute histogram, scale it to 0-255 range and backproject
	    calcHist( &hsv_roi, 1, channels, Mat(), hist, 1, histSize, ranges, true, false);
	    // find min and max values of histogram bins
	    double minval, maxval;
	    cv::minMaxIdx(hist, &minval, &maxval);
	    printf("Min %d max %d \n",minval , maxval );


/*


	   18 cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)
	   19
	   20 # Setup the termination criteria, either 10 iteration or move by atleast 1 pt
	   21 term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
	   22
	   23 while(1):
	   24     ret ,frame = cap.read()
	   25
	   26     if ret == True:
	   27         hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	   28         dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)
	   29
	   30         # apply meanshift to get the new location
	   31         ret, track_window = cv2.meanShift(dst, track_window, term_crit)
	   32
	   33         # Draw it on image
	   34         x,y,w,h = track_window
	   35         img2 = cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
	   36         cv2.imshow('img2',img2)
	   37
	   38         k = cv2.waitKey(60) & 0xff
	   39         if k == 27:
	   40             break
	   41         else:
	   42             cv2.imwrite(chr(k)+".jpg",img2)
	   43
	   44     else:
	   45         break
	   46
	   47 cv2.destroyAllWindows()
	   48 cap.release()

	/** distance to edge
	Mat M(height, width, CV_8UC2, img); // original
	Mat grayImage;
	//cvtColor(M, grayImage, CV_YUV2GRAY_Y422);
	//draw_line_example(grayImage,img,width,height);

	start_clock();
	Mat image, edge_image;

	// convert UYVY in paparazzi to YUV in opencv
	cvtColor(M, M, CV_YUV2RGB_Y422);
	cvtColor(M, M, CV_RGB2YUV);

	// Threshold all values within the indicted YUV values.
	Mat thresh_image(width, height, CV_8U);
	inRange(M, Scalar(0, 0, 0), Scalar(255, 135, 255), thresh_image);

	Mat output;
	distanceTransform(thresh_image, output, CV_DIST_L2, 3,CV_8U);
	output.convertTo(thresh_image,CV_8U);
	bitwise_not(thresh_image, thresh_image);*/
//	thresh_image=output;

/*
	/// Find contours
	vector < vector<Point> > contours;
	vector < Vec4i > hierarchy;

	thresh_image.copyTo(edge_image);
	int edgeThresh = 35;
	Canny(edge_image, edge_image, edgeThresh, edgeThresh * 3);
	findContours(edge_image, contours, hierarchy, CV_RETR_EXTERNAL,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// Find Largest Contour
	int largest_contour_index = 0;
	int largest_area = 0;
	Rect bounding_rect;

	// iterate through each contour.
	for (unsigned int i = 0; i < contours.size(); i++) {

		//  Find the area of contour
		double a = contourArea(contours[i], false);

		if (a > largest_area) {
			largest_area = a;
			// Store the index of largest contour
			largest_contour_index = i;
			// Find the bounding rectangle for biggest contour
			bounding_rect = boundingRect(contours[i]);
			rect_top.x = bounding_rect.x;
			rect_top.y = bounding_rect.y;
			rect_size.x = bounding_rect.width;

			rect_size.y = bounding_rect.height;
		}
	}


	Scalar color(255, 255, 255);
	// Draw the contour and rectangle
//	  drawContours(M, contours, largest_contour_index, color, CV_FILLED, 8, hierarchy);

	rectangle(thresh_image, bounding_rect, Scalar(140), 2, 8, 0);
	printf("My dims: %d, rows %d, cols %d\n", thresh_image.dims,
			thresh_image.rows, thresh_image.cols);
*/
	//line(thresh_image,Point(0,0),Point(100,100),Scalar(180),15);
//
//	  // some figure can cause there are no largest circles, in this case, do not draw circle
//	  circle(M, mc[largest_contour_index], 4, Scalar(0, 255, 0), -1, 8, 0);
//	  Point2f rect_center(bounding_rect.x + bounding_rect.width / 2 , bounding_rect.y + bounding_rect.height / 2);
//	  circle(image, rect_center, 4, Scalar(0, 0, 255), -1, 8, 0);

	//grayscale_opencv_to_yuv422(thresh_image, img, width, height);

	//  grayscale_opencv_to_yuv422(thresh_image, img, width, height);
	//yuv_opencv_to_yuv422(M, img, width, height);
//	 Mat lookUpTable(1, 256, CV_8U);
//	    uchar* p = lookUpTable.data;
//	    int m = 240;
//
//	    if(prev_stddev_colors!=stddev_colors){
//	    		init_lookup_table();
//	    		prev_stddev_colors=stddev_colors;
//	    }
	// Create a new image, using the original bebop image.
//	start_clock();
//  Mat M(height, width, CV_8UC2, img);
//	printf("%d %d\n",width,height);
//  yuv422_set_color_intensity( colorIntensityYUV,width, height, img);
	// grayscale_opencv_to_yuv422(colorIntensityYUV, img, width, height);
	//end_clock("end of everything");

	/*

	 Mat colorIntensityImage= Mat::zeros(height, width, CV_8U);
	 // If you want a color image, uncomment this line
	 cvtColor(M, image, CV_YUV2BGR_Y422);

	 int channels = image.channels();

	 int nRows = image.rows;
	 int nCols = image.cols * channels;

	 if (image.isContinuous())
	 {
	 nCols *= nRows;
	 nRows = 1;
	 }

	 int i,j;
	 //      uchar* p;
	 end_clock("start of setting");
	 start_clock();
	 uchar* colorIntensityPointer=colorIntensityImage.data;
	 int indexColorIntensityImage=0;
	 for( i = 0; i < nRows; ++i)
	 {
	 p = image.ptr<uchar>(i);
	 for ( j = 0; j < nCols; )
	 {
	 colorIntensityPointer[indexColorIntensityImage++]=blue_lookup[p[j]]+green_lookup[p[j+1]]+red_lookup[p[j+1]];
	 p[j++] = blue_lookup[p[j]];
	 p[j++] = green_lookup[p[j]];
	 p[j++] = red_lookup[p[j]];

	 }
	 }

	 end_clock("end of setting");
	 start_clock();

	 //
	 //         for( int y = 0; y < image.rows; y++ )
	 //         {
	 //       	  for( int x = 0; x < image.cols; x++ )
	 //       	  { for( int c = 0; c < 3; c++ ) {
	 //       //		  image.at<Vec3b>(y,x)[c] = saturate_cast<uchar>( alphas[c]*( image.at<Vec3b>(y,x)[c] ) ); }
	 //       	 // image.at<Vec3b>(y,x)[c] = saturate_cast<uchar>( 1+( image.at<Vec3b>(y,x)[c] ) );
	 //       	  image.at<Vec3b>(y,x)[c] = lookup_table[image.at<Vec3b>(y,x)[c]];
	 //
	 //       	  }
	 //
	 //       	  }
	 //         }
	 end_clock("end of setting slowly");
	 start_clock();
	 LUT(image, lookUpTable, image);

	 end_clock("end of setting very very fast");
	 start_clock();
	 // For a grayscale image, use this one
	 //cvtColor(M, image, CV_YUV2GRAY_Y422);
	 cvtColor(image, yuvimage, CV_BGR2YUV);
	 end_clock("initialiseation");
	 start_clock();
	 // Threshold all values within the indicted YUV values.
	 inRange(yuvimage, Scalar(cvcolor_lum_min, cvcolor_cb_min, cvcolor_cr_min),
	 Scalar(cvcolor_lum_max,cvcolor_cb_max, cvcolor_cr_max), thresh_image);
	 end_clock("in range");
	 start_clock();
	 Mat integral_image;
	 integral(thresh_image, integral_image,CV_32S);
	 printf("normal image %d %d integral image %d %d\n",height,width,integral_image.rows,integral_image.cols);
	 end_clock("integral");
	 start_clock();
	 int stupid_square_width=10;//pixels
	 int window_width=50;
	 int border_width=10;

	 Z = Mat::zeros(integral_image.rows,integral_image.cols, CV_8U);

	 for(int ypos=1;ypos<integral_image.cols-2*window_width-100;ypos++){
	 for(int xpos=1;xpos<integral_image.rows-2*window_width;xpos++){
	 //    for(int xpos=0;xpos<width-window_width;xpos++){
	 //    	for(int ypos=0;ypos<height-window_width;ypos++){

	 unsigned int area_out = get_area_in_border(integral_image,xpos,ypos,window_width);
	 unsigned int area_in = get_area_in_border(integral_image,xpos+border_width,ypos+border_width,window_width-2*border_width);
	 volatile int to_fill =(area_out-area_in)/1024;

	 if(to_fill>=250){
	 Z.at<uint8_t>({ypos,xpos})=250;
	 }
	 else{
	 Z.at<uint8_t>({ypos,xpos})=to_fill;
	 }
	 //    		Z.at<uint8_t>({ypos,xpos})=250;
	 }
	 }

	 end_clock("fancy stuff");
	 start_clock();
	 trackBox = CamShift(Z, selection,
	 TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 ));
	 if( selection.area() <= 1 )
	 {
	 int cols = Z.cols, rows = Z.rows, r = (MIN(cols, rows) + 5)/6;
	 selection = Rect(selection.x - r, selection.y - r,
	 selection.x + r, selection.y + r) &
	 Rect(0, 0, cols, rows);
	 }


	 ellipse( Z, trackBox, Scalar(255,0,255), 3, LINE_AA );
	 loc_y=trackBox.center.y;

	 // meanShift(Z, selection,TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 ));
	 //circle(Z, Point(selection.x,selection.y), 20, Scalar(255),20);
	 end_clock("camshift");
	 start_clock();
	 //  thresh_image=thresh_image*10;
	 //
	 // cvcolor_lum_min,cvcolor_lum_max,cvcolor_cb_min;
	 // extern int cvcolor_cb_max,cvcolor_cr_min,cvcolor_cr_max;
	 //  // Blur it, because we can
	 //  blur(image, image, Size(5, 5));
	 /* inRange(image, Scalar(minb,ming,minr), Scalar(maxb,maxg,maxr), thresh_image);
	 Scalar blue = Scalar(255, 0, 0);  //BGR VALUE
	 Scalar red = Scalar(0, 255, 0);  //BGR VALUE
	 // image.setTo(blue, thresh_image);
	 mask = cv::Mat::ones(thresh_image.size(), thresh_image.type()) * 255 - thresh_image;
	 image.setTo(red, mask);

	 // Mat positive= cv::Mat::ones(thresh_image.size(), thresh_image.type()) * 255;
	 // Canny edges, only works with grayscale image
	 //  int edgeThresh = 35;
	 //  Canny(image, image, edgeThresh, edgeThresh * 3);
	 */
	// Convert back to YUV422, and put it in place of the original image 8*/
	//grayscale_opencv_to_yuv422(image, img, width, height);
	/*
	 switch(function_to_send_opencv){
	 case 0: break; // do nothing. original image
	 case 1:
	 // thresholded image in range
	 //	   cvtColor(thresh_image, image, CV_GRAY2BGR);
	 grayscale_opencv_to_yuv422(thresh_image, img, width, height);

	 break;
	 case 2:
	 // something with the integral image
	 //	  cvtColor(Z, image, CV_GRAY2BGR);
	 grayscale_opencv_to_yuv422(Z, img, width, height);
	 break;
	 case 3:
	 // thresholded image in range
	 // 	   cvtColor(yuvimage, image, CV_YUV2BGR);
	 end_clock("camshift");
	 start_clock();
	 coloryuv_opencv_to_yuv422(yuvimage, img, width, height);
	 end_clock("YUV TO YUV");
	 start_clock();
	 break;
	 case 4:
	 // thresholded image in range
	 //   cvtColor(colorIntensityImage, image, CV_GRAY2BGR);
	 end_clock("camshift");
	 start_clock();
	 grayscale_opencv_to_yuv422(colorIntensityImage, img, width, height);
	 end_clock("grayscale funcn now");
	 start_clock();
	 break;

	 default: break; // do nothing. original image
	 }
	 end_clock("visualisation");
	 start_clock();*/
	return 0;
}
