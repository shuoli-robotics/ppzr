/*
 * gate_detection.c
 *
 *  Created on: Sep 5, 2016
 *      Author: Guido de Croon
 */

#include "gate_detection.h"
#include <math.h>
// one would expect this to be part of math.h, but...
#define PI 3.14159265359
#include "main_parameters.h"
#include <stdlib.h>

// variables that have to be remembered in between function calls:

// since MAX_POINTS means that the algorithm will stop gathering points after MAX_POINTS, we sample according to a "moving" grid
// these starting points are made for a grid with step size 3
#define GRID_STEP 3
int Y0[9] = {0,1,2,0,1,2,0,1,2};
int X0[9] = {0,0,0,1,1,1,2,2,2};

struct point_f points[MAX_POINTS];
uint16_t n_points;

// Settings for the evolution:
// 10 individuals 30 generations is a normal setting
#define N_INDIVIDUALS 10
#define N_GENES 3
uint16_t n_generations = 10; // could be reduced for instance when there are many points
float Population[N_INDIVIDUALS][N_GENES];

// watch out: inliers fit does not work so well...
#define DISTANCE_FIT 0
#define INLIERS_FIT 1
#define FF DISTANCE_FIT

// Settings for the fitting:
float weights[MAX_POINTS];
int min_points = 5;
int WEIGHTED = 1; 
int STICK = 1;
#define CIRCLE 0
#define SQUARE 1
int SHAPE = CIRCLE;
// Now a parameter:
// int min_disparity = 2;
float outlier_threshold = 20.0f;


// whether to draw on the image:
int GRAPHICS = 0;

/**
 * Function takes a disparity image and fits a circular gate to the close-by points.
 * - x0, y0, size0 contain an initial guess and indicate where the closest object probably is
 * - x_center, y_center, and radius are also used for initializing the optimization. The optimized results are put back in these parameters.
 * - fitness: how good is the hypothesis of x_center, etc. 
 * - min_x, min_y, ... The region of interest in which to sample the points.
 * @author Guido
 */

void gate_detection(struct image_t* color_image, float* x_center, float* y_center, float* radius, float* fitness, float* x0, float* y0, float* size0, uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y)
{
  // 1) convert the disparity map to a vector of points:
	convert_disparitymap_to_points(color_image);

  // if there are enough colored points:
	if (n_points > min_points)
	{
		// 2) fit a window to the points

		// determine initial guess (we always do):
		// initialize with average position and size
		float mean_x = 0;
		float mean_y = 0;
    
    uint16_t i;
		for (i = 0; i < n_points; i++)
		{
			mean_x += points[i].x; // could still do a weighted average
			mean_y += points[i].y;
		}
		mean_x /= n_points;
		mean_y /= n_points;
		(*x0) = mean_x;
		(*y0) = mean_y;
    // TODO: make a better initial size estimation - this is actually ridiculous:
    // For instance, take min, max x, min, max y
		(*size0) = 40.0f;//sqrtf(mean_x*mean_x + mean_y*mean_y);

		// run the fit procedure:
		fit_window_to_points(x0, y0, size0, x_center, y_center, radius, fitness);

    if(GRAPHICS)
    {
      // draw a circle on the disparity image:
      // a grey color:
      uint8_t color[3];
      color[0] = 128; // Y
      color[1] = 128; // U
      color[2] = 128; // V

      if(SHAPE == CIRCLE)
      {
		    draw_circle(color_image, (*x_center), (*y_center), (*radius), color);
      }
      else
      {
        // square:
        int x = (*x_center);
        int y = (*y_center);
        int r = (*radius);
        struct point_f square_top_left;
        struct point_f square_top_right;
        struct point_f square_bottom_right;
        struct point_f square_bottom_left;
        square_top_left.x = x-r;
        square_top_left.y = y-r; // positive y direction is down
        square_top_right.x = x+r;
        square_top_right.y = y-r; 
        square_bottom_left.x = x-r;
        square_bottom_left.y = y+r; 
        square_bottom_right.x = x+r;
        square_bottom_right.y = y+r;
        // draw lines:
        draw_line_segment(color_image, square_top_left, square_top_right, color);
        draw_line_segment(color_image, square_top_left, square_bottom_left, color);
        draw_line_segment(color_image, square_bottom_left, square_bottom_right, color);
        draw_line_segment(color_image, square_bottom_right, square_top_right, color);
      }
      if(STICK)
        draw_stick(color_image, (*x_center), (*y_center), (*radius), color);

      // draw initial guess as a crosshair:
      struct point_f top;
      struct point_f bottom;
      struct point_f left;
      struct point_f right;
      top.x = (*x0); top.y = (*y0) - 5;
      bottom.x = (*x0); bottom.y = (*y0) + 5;
      left.x = (*x0) - 5; left.y = (*y0);
      right.x = (*x0) + 5; right.y = (*y0);
      draw_line_segment(color_image, top, bottom, color);
      draw_line_segment(color_image, left, right, color);
    }
	}
	else
	{
		(*fitness) = BAD_FIT;
	}

}

void fit_window_to_points(float* x0, float* y0, float* size0, float* x_center, float* y_center, float* radius, float* fitness)
{
  // a) initialize Population, seeding it with the initial guess and the previous best individuals:
  // The idea behind this is that evolution can extend over multiple images, while a wrong tracking will not
  // influence the tracking excessively long.
  uint16_t i, g, ge;
	for (i = 0; i < N_INDIVIDUALS/2; i++)
	{
    Population[i][0] = (*x0) + 5 * get_random_number() - 2.5f;
    Population[i][1] = (*y0) + 5 * get_random_number() - 2.5f;
		Population[i][2] = (*size0) + 5 * get_random_number() - 2.5f;
	}
	for (i = N_INDIVIDUALS/2; i < N_INDIVIDUALS; i++)
	{
    Population[i][0] = (*x_center) + 5 * get_random_number() - 2.5f;
    Population[i][1] = (*y_center) + 5 * get_random_number() - 2.5f;
		Population[i][2] = (*radius) + 5 * get_random_number() - 2.5f;
	}

  float total_sum_weights = get_sum(weights, n_points);

	// large number, since we will minimize it:
	(*fitness) = 1000000;
	float fits[N_INDIVIDUALS];
	float best_genome[N_GENES];
  best_genome[0] = (*x0);
  best_genome[1] = (*y0);
  best_genome[2] = (*size0);
	for (g = 0; g < n_generations; g++)
	{
		for (i = 0; i < N_INDIVIDUALS; i++)
		{
			if (SHAPE == CIRCLE)
			{
        if(FF == DISTANCE_FIT)
        {
          // optimize mean distance to circle (and possibly stick) 
				  fits[i] = mean_distance_to_circle(Population[i]);
        }
        else
        {
          // optimize the number of inliers
          fits[i] = get_outlier_ratio(Population[i], total_sum_weights);
        }
			}
      else
      {
          // optimize mean distance to square (and possibly stick) 
				  fits[i] = mean_distance_to_square(Population[i]);
      }
		}

		// get the best individual and store it in min_genome:
		int index;
		float min_fit = get_minimum(fits, N_INDIVIDUALS, &index);
    float min_genome[N_GENES];
    for(ge = 0; ge < N_GENES; ge++)
    {
		  min_genome[ge] = Population[index][ge];
    }

		// if better than any previous individual, remember it:
		if (min_fit < (*fitness))
		{
			for (ge = 0; ge < 3; ge++)
			{
				best_genome[ge] = min_genome[ge];
			}
			(*fitness) = min_fit;
		}

		// fill the new population with mutated copies of this generation's best:
		if (g < n_generations - 1)
		{
			// super elitist evolution:
			for (i = 0; i < N_INDIVIDUALS; i++)
			{
				Population[i][0] = min_genome[0] + 5 * get_random_number() - 2.5f;
				Population[i][1] = min_genome[1] + 5 * get_random_number() - 2.5f;
				Population[i][2] = min_genome[2] + 5 * get_random_number() - 2.5f;
			}
		}

	}

  // put the final values back in the parameters:
  if(FF == DISTANCE_FIT) (*fitness) /= total_sum_weights;
	(*x_center) = best_genome[0];
	(*y_center) = best_genome[1];
	(*radius) = best_genome[2];

  return;
}

float get_random_number()
{
	int rand_num = rand() % 1000;
	float r = (float)rand_num / 1000.0f;
	return r;
}

float get_minimum(float* nums, int n_elements, int *index)
{
  (*index) = 0;
	float min = nums[0];
  uint16_t i;
	for (i = 1; i < n_elements; i++)
	{
		if (nums[i] < min)
		{
			(*index) = i;
			min = nums[i];
		}
	}
	return min;
}

float get_sum(float* nums, int n_elements)
{
  float sum = nums[0];
  uint16_t i;
	for (i = 1; i < n_elements; i++)
	{
		sum += nums[i];
	}
	return sum;
}

void convert_image_to_points(struct image_t* color_image, uint16_t min_x, uint16_t min_y, uint16_t max_x, uint16_t max_y)
{
  int y, x, sp;
  int disp = 0;
	uint16_t p = 0;
  
  // We stop sampling at MAX_POINTS, but do not want our samples to be biased toward a certain
  // part of the image. 
  // We have different grid starting points (GRID_STEP*GRID_STEP) so that for every different 
  // starting point sp, we will sample different positions in the image, finally covering the
  // whole image.
  for(sp = 0; sp < GRID_STEP*GRID_STEP; sp++)
  {
	  for (y = min_y + Y0[sp]; y < max_y; y+=GRID_STEP)
	  {
		  for (x = min_x + X0[sp]; x < max_x; x+=GRID_STEP)
		  {
        // get the disparity from the image:
			  disp = check_color(color_image, x, y);

			  if(disp)
			  {
          // add the points to the array, and use disparity as the weight:
				  points[p].x = (float) x;
				  points[p].y = (float) y;
				  weights[p] = 1.0f; // TODO: we could make this depend on how close it is to the ideal color

          // count the number of points:
          p++;

          // if the maximum number of points is reached, return:
          if(p == MAX_POINTS)
          {
            n_points = p;
            return;
          }
			  }
		  }
	  }
  }

  // set the global variable n_points to the right value:
  n_points = p;
}

float mean_distance_to_circle(float* genome)
{
	float x = genome[0];
	float y = genome[1];
	float r = genome[2];

	float mean_distance = 0.0f;
	struct point_f point;
	float dx, dy;
	float dist_center, error, error_stick;
  uint16_t p;
	for (p = 0; p < n_points; p++)
	{
		point = points[p];
		dx = point.x - x;
		dy = point.y - y;
		dist_center = sqrtf(dx*dx + dy*dy);
		error = fabs(dist_center - r);

		if (STICK)
		{
			// determine distance to the stick:
			struct point_f stick1;
      stick1.x = x;
      stick1.y = y + r;
			struct point_f stick2;
      stick2.x = x;
      stick2.y = y + 2*r;
			error_stick = distance_to_vertical_segment(stick1, stick2, point);

			// take the smallest error:
			if (error_stick < error) error = error_stick;
		}

		// apply outlier threshold before applying weights:
		if (error > outlier_threshold) error = outlier_threshold;

		if (WEIGHTED)
		{
			mean_distance += error * weights[p];
		}
		else
		{
			mean_distance += error;
		}
	}
	mean_distance /= n_points;
	return mean_distance;
}

float mean_distance_to_square(float* genome)
{
	float x = genome[0];
	float y = genome[1];
	float r = genome[2];

	float mean_distance = 0.0f;
	struct point_f point;
	float error, error_stick;
  uint16_t p;
  int index;

  // determine corner points:
  struct point_f square_top_left;
  struct point_f square_top_right;
  struct point_f square_bottom_right;
  struct point_f square_bottom_left;
  square_top_left.x = x-r;
  square_top_left.y = y-r; // positive y direction is down
  square_top_right.x = x+r;
  square_top_right.y = y-r; 
  square_bottom_left.x = x-r;
  square_bottom_left.y = y+r; 
  square_bottom_right.x = x+r;
  square_bottom_right.y = y+r;
  float side_distances[2];

	for (p = 0; p < n_points; p++)
	{
    // get the current point:
    point = points[p];
    // determine the distance to the four sides of the square and select the smallest one:
    side_distances[0] = distance_to_vertical_segment(square_top_left, square_bottom_left, point);
    side_distances[1] = distance_to_vertical_segment(square_top_right, square_bottom_right, point);
    //side_distances[2] = distance_to_horizontal_segment(square_top_left, square_top_right, point);
    //side_distances[3] = distance_to_horizontal_segment(square_bottom_left, square_bottom_right, point);
    error = get_minimum(side_distances, 2, &index);

		if (STICK)
		{
			// determine distance to the stick:
			struct point_f stick1;
      stick1.x = x;
      stick1.y = y + r;
			struct point_f stick2;
      stick2.x = x;
      stick2.y = y + 2*r;
			error_stick = distance_to_vertical_segment(stick1, stick2, point);

			// take the smallest error:
			if (error_stick < error) error = error_stick;
		}

		// apply outlier threshold before applying weights:
		if (error > outlier_threshold) error = outlier_threshold;

		if (WEIGHTED)
		{
			mean_distance += error * weights[p];
		}
		else
		{
			mean_distance += error;
		}
	}
	mean_distance /= n_points;
	return mean_distance;
}

float distance_to_line(struct point_f Q1, struct point_f Q2, struct point_f P)
{
  // see e.g., http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
	float norm_Q2_Q1 = sqrtf((Q1.x - Q2.x)*(Q1.x - Q2.x) + (Q1.y - Q2.y)*(Q1.y - Q2.y));
	float det = (Q2.x - Q1.x)*(P.y - Q1.y) - (Q2.y - Q1.y)*(P.x - Q1.x);
	float dist_line = fabs(det) / norm_Q2_Q1;
	return dist_line;
}

float distance_to_segment(struct point_f Q1, struct point_f Q2, struct point_f P)
{
  float dist_line = distance_to_line(Q1, Q2, P);

	// calculate intersection point:
	float rx = Q2.y - Q1.y; // always negative, -r
	float ry = -(Q2.x - Q1.x);
	float norm_r = sqrtf(rx*rx+ry*ry);
  rx = (rx / norm_r) * dist_line;
  ry = (ry / norm_r) * dist_line;

  // rx < 0, so:
  // if P.x > Q1.x, it should be P.x + rx
  // else it should be P.x - rx
	float i_x;
	float i_y;
  if(P.x > Q1.x)
  {
    i_x = P.x + rx;
    i_y = P.y + ry;
  }
  else
  {
    i_x = P.x - rx;
    i_y = P.y - ry;
  }
	struct point_f I;
  I.x = i_x;
  I.y = i_y;

  /*
  Slow code:
	float dI = distance_to_line(Q1, Q2, I);
	if (dI > 1e-10)
	{
		I.x = P.x - rx;
		I.y = P.y - ry;
	}
  */
		
  /*
  // Slow code:
	// check if it is on the segment:
	float d1 = sqrtf((Q1.x-I.x)*(Q1.x - I.x) + (Q1.y - I.y)*(Q1.y - I.y));
	float d2 = sqrtf((Q2.x - I.x)*(Q2.x - I.x) + (Q2.y - I.y)*(Q2.y - I.y));
	float d_12 = sqrtf((Q2.x - Q1.x)*(Q2.x - Q1.x) + (Q2.y - Q1.y)*(Q2.y - Q1.y));
	if (d1 > d_12 || d2 > d_12)
	{
		// not on segment, determine minimum distance to one of the two extremities:
		dist_line = sqrtf((Q1.x - P.x)*(Q1.x - P.x) + (Q1.y - P.y)*(Q1.y - P.y));
		d2 = sqrtf((Q2.x - P.x)*(Q2.x - P.x) + (Q2.y - P.y)*(Q2.y - P.y));
		if (d2 < dist_line) dist_line = d2;
	}
  */
		 
  // leave out superfluous sqrtf - for comparisons it does not matter (monotonously increasing functions)
  // we can still precalculate (Q1.x - I.x) etc. but I don't know if it is actually calculated twice (optimized by compiler?)
  float d1 = (Q1.x - I.x)*(Q1.x - I.x) + (Q1.y - I.y)*(Q1.y - I.y);
	float d2 = (Q2.x - I.x)*(Q2.x - I.x) + (Q2.y - I.y)*(Q2.y - I.y);
	float d_12 = (Q2.x - Q1.x)*(Q2.x - Q1.x) + (Q2.y - Q1.y)*(Q2.y - Q1.y);
	if (d1 > d_12 || d2 > d_12)
	{
		// not on segment, determine minimum distance to one of the two extremities:
		dist_line = (Q1.x - P.x)*(Q1.x - P.x) + (Q1.y - P.y)*(Q1.y - P.y);
		d2 = (Q2.x - P.x)*(Q2.x - P.x) + (Q2.y - P.y)*(Q2.y - P.y);
		if (d2 < dist_line) dist_line = sqrtf(d2);
	}

	return dist_line;
}

float distance_to_vertical_segment(struct point_f Q1, struct point_f Q2, struct point_f P)
{
  // Q1.y should be smaller than Q2.y, y positive down
  // so first top then bottom

  // Calculating the distance to a vertical segment is actually quite simple:
  // If the y coordinate of P is in between Q1.y and Q2.y, the shortest distance is orthogonal to the line
  // If P.y < Q1.y (which is < Q2.y), then the distance to Q1 should be taken
  // If P.y > Q2.y, then the distance to Q2 should be taken:
  float dist_line;

  if(P.y < Q1.y)
  {
  	dist_line = sqrtf((Q1.x - P.x)*(Q1.x - P.x) + (Q1.y - P.y)*(Q1.y - P.y));
  }
  else if(P.y <= Q2.y)
  {
    dist_line = fabs(P.x - Q1.x); // straight line to the vertical line segment
  }
  else
  {
    dist_line = sqrtf((Q2.x - P.x)*(Q2.x - P.x) + (Q2.y - P.y)*(Q2.y - P.y));
  }

  return dist_line;
}

float distance_to_horizontal_segment(struct point_f Q1, struct point_f Q2, struct point_f P)
{
  // Q1.x should be smaller than Q2.x

  // Calculating the distance to a horizontal segment is actually quite simple:
  // If the x coordinate of P is in between Q1.x and Q2.x, the shortest distance is orthogonal to the line
  // If P.x < Q1.x (which is < Q2.x), then the distance to Q1 should be taken
  // If P.x > Q2.x, then the distance to Q2 should be taken:
  float dist_line;

  if(P.x > Q2.x)
  {
  	dist_line = sqrtf((Q2.x - P.x)*(Q2.x - P.x) + (Q2.y - P.y)*(Q2.y - P.y));
  }
  else if(P.x >= Q1.x)
  {
    dist_line = fabs(P.y - Q1.y); // straight line to the horizontal line segment
  }
  else
  {
    dist_line = sqrtf((Q1.x - P.x)*(Q1.x - P.x) + (Q1.y - P.y)*(Q1.y - P.y));
  }

  return dist_line;
}


void draw_circle(struct image_t* Im, float x_center, float y_center, float radius, uint8_t* color)
{
  float t_step = 0.05; // should depend on radius, but hey...
	int x, y;
  float t;
	for (t = 0.0f; t < (float)(2 * PI); t += t_step)
	{
		x = (int)x_center + (int)(cosf(t)*radius);
		y = (int)y_center + (int)(sinf(t)*radius);
		if (x >= 0 && x < Im->w && y >= 0 && y < Im->h)
		{
      Im->image[y*Im->w*2+x*4] = color[1];
      Im->image[y*Im->w*2+x*4+1] = color[0];
      Im->image[y*Im->w*2+x*4+2] = color[2];
      Im->image[y*Im->w*2+x*4+3] = color[0];
		}
	}
  return;
}

void draw_stick(struct image_t* Im, float x_center, float y_center, float radius, uint8_t* color)
{
  // Potential TODO: use image_draw_line(struct image_t *img, struct point_t *from, struct point_t *to);

  int x, y;
  x = (int) x_center;
  for(y = (int)(y_center + radius); y <  (int)(y_center + 2*radius); y++)
  {
    if (x >= 0 && x < Im->w && y >= 0 && y < Im->h)
		{
      Im->image[y*Im->w*2+x*4] = color[1];
      Im->image[y*Im->w*2+x*4+1] = color[0];
      Im->image[y*Im->w*2+x*4+2] = color[2];
      Im->image[y*Im->w*2+x*4+3] = color[0];
		} 
  }
}

void draw_line_segment(struct image_t* Im, struct point_f Q1, struct point_f Q2, uint8_t* color)
{

  // Potential TODO: use image_draw_line(struct image_t *img, struct point_t *from, struct point_t *to); 

  float t_step = 0.05; // should depend on distance, but hey...
	int x, y;
  float t;
	for (t = 0.0f; t < 1.0f; t += t_step)
	{
		x = (int)(t * Q1.x + (1.0f - t) * Q2.x);
		y = (int)(t * Q1.y + (1.0f - t) * Q2.y);
		if (x >= 0 && x < Im->w && y >= 0 && y < Im->h)
		{
      Im->image[y*Im->w*2+x*4] = color[1];
      Im->image[y*Im->w*2+x*4+1] = color[0];
      Im->image[y*Im->w*2+x*4+2] = color[2];
      Im->image[y*Im->w*2+x*4+3] = color[0];
		}
	}
  return;
}

