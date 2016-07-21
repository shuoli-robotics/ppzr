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
/** my desk **/
/*
int cvcolor_lum_min=0,cvcolor_lum_max=53,cvcolor_cb_min=121;
 int cvcolor_cb_max=135,cvcolor_cr_min=80,cvcolor_cr_max=103;*/
/** Cyber zoo **/
int cvcolor_lum_min=0,cvcolor_lum_max=178,cvcolor_cb_min=128;
 int cvcolor_cb_max=169,cvcolor_cr_min=109,cvcolor_cr_max=136;
 int function_to_send_opencv=2;
clock_t begin;
Mat image,thresh_image,mask;
  Mat image_color,yuvimage;
 static unsigned int get_area_in_border(Mat image, int x,int y,int width_heigth){
	 Point right_under(y+width_heigth,x+width_heigth);
	 Point right_up(y,x+width_heigth);
	 Point left_under(y+width_heigth,x);
	 Point left_up(y,x);
	 return image.at<int>(right_under)-image.at<int>(right_up)-image.at<int>(left_under)+image.at<int>(left_up);
 }

 inline void start_clock(){
	 begin = clock();
 }
 inline void end_clock(char *name_part){

	  clock_t end = clock();
	  double elapsed_secs = double(end - begin)/1000.0;
	 printf("%s took %f seconds\n",name_part,elapsed_secs);
 }

 void opencv_init_rects(){
		selection = Rect(Point(0,0),Point(400,400));

 }
int opencv_example(char *img, int width, int height)
{
  // Create a new image, using the original bebop image.
	start_clock();
  Mat M(height, width, CV_8UC2, img);

  // If you want a color image, uncomment this line
  cvtColor(M, image, CV_YUV2BGR_Y422);
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

    Mat Z = Mat::zeros(integral_image.rows,integral_image.cols, CV_8U);

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
  switch(function_to_send_opencv){
  case 0: break; // do nothing. original image
  case 1:
	  // thresholded image in range
	   cvtColor(thresh_image, image, CV_GRAY2BGR);
	  color_opencv_to_yuv422(image, img, width, height);

	  break;
  case 2:
	  // something with the integral image
	  cvtColor(Z, image, CV_GRAY2BGR);
		  color_opencv_to_yuv422(image, img, width, height);
		  break;
  default: break; // do nothing. original image
  }
  end_clock("visualisation");
     start_clock();
  return 0;
}
