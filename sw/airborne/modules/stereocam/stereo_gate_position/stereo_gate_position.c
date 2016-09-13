/*
 * Copyright (C) Michaël Ozo
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/stereocam/stereo_gate_position/stereo_gate_position.c"
 * @author Michaël Ozo
 * Stereo board gate detection is fused with optic flow velocity.
 */

#include <math.h>
#include "subsystems/datalink/telemetry.h"
#include "modules/stereocam/stereo_gate_position/stereo_gate_position.h"


#define PI 3.1415926

#define GOOD_FIT 8


void stereocam_to_state(void);

char x_center = 0;
char y_center = 0;
char radius   = 0;
char fitness  = 0;
char fps      = 0;

float measured_x_gate = 0;
float measured_y_gate = 0;
float measured_z_gate = 0;

// Settings:
float FOV_width = 57.4f;
float FOV_height = 44.5f;
float gate_size_meters = 1.0f;

float deg2rad(float deg)
{
	return ((deg * PI) / 180.0f);
}

static void stereo_gate_send(struct transport_tx *trans, struct link_device *dev)
    {
    pprz_msg_send_STEREO_GATE_INFO(trans, dev, AC_ID,&x_center, &y_center,&radius,&fitness,&fps,&measured_x_gate,&measured_y_gate,&measured_z_gate);
    }  

 void stereo_gate_position_init(void)
 {
   register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STEREO_GATE_INFO, stereo_gate_send);
 }
 
 void get_stereo_data_periodic(void) 
 {
   if (stereocam_data.fresh) {
	  stereocam_to_state();
	  y_center+=1;
    stereocam_data.fresh = 0;
  }
 }
 


void stereocam_to_state(void)
{
  //message of length 5 with the x_center (image coord), y_center, radius, fitness (<4 is good, > 10 bad),
  //and frame rate of the calculations.
  
	x_center = stereocam_data.data[0];
	y_center = stereocam_data.data[1];
	radius   = stereocam_data.data[2];
	fitness  = stereocam_data.data[3];
	fps      = stereocam_data.data[4];
  
  // Determine the measurement:
	float alpha = (radius / 128.0f) * FOV_width;
	float measured_distance_gate = (0.5f * gate_size_meters) / tan(deg2rad(alpha));
	float measured_angle_gate = ((x_center - 64.0f) / (128.0f)) * FOV_width;
	float measured_angle_vert = ((y_center - 48.0f) / (96.0f)) * FOV_height;
	
	measured_x_gate = measured_distance_gate * sin(deg2rad(measured_angle_gate));
	measured_y_gate = measured_distance_gate * cos(deg2rad(measured_angle_gate));
	measured_z_gate = measured_distance_gate * sin(deg2rad(measured_angle_vert));
	
  /*
  //State filter 
	

	
    // predict the new location:
	float gate_turn_rate = -(turn_rate); 
	printf("Gate turn rate = %f\n", gate_turn_rate);
	float current_distance = sqrtf(current_x_gate*current_x_gate + current_y_gate*current_y_gate);
	float dx_gate = dt * (cos(current_angle_gate) * gate_turn_rate * current_distance);
	float dy_gate = dt * (velocity_gate - sin(current_angle_gate) * gate_turn_rate * current_distance);
	predicted_x_gate = previous_x_gate + dx_gate;
	predicted_y_gate = previous_y_gate + dy_gate;
	
  if (fitness < GOOD_FIT)
	{
	
		// Mix the measurement with the prediction:
		float weight_measurement;
		if (uncertainty_gate > 30)
			weight_measurement = 1.0f;
		else
			weight_measurement = (*color_fitness);

		current_x_gate = weight_measurement * measured_x_gate + (1.0f - weight_measurement) * predicted_x_gate;
		current_y_gate = weight_measurement * measured_y_gate + (1.0f - weight_measurement) * predicted_y_gate;
		current_angle_gate = atan2f(current_x_gate, current_y_gate);

		// reset uncertainty:
		uncertainty_gate = 0;
	}
	else
	{
		// just the prediction
		current_x_gate = predicted_x_gate;
		current_y_gate = predicted_y_gate;
		current_angle_gate = atan2f(current_x_gate, current_y_gate);

		// increase uncertainty
		uncertainty_gate++;
	}
	// set the previous state for the next time:
	previous_x_gate = current_x_gate;
	previous_y_gate = current_y_gate;
  */
}

