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
#include <sys/time.h>
#include "subsystems/datalink/telemetry.h"
#include "modules/stereocam/stereo_gate_position/stereo_gate_position.h"
#include "state.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_clock.h"

#define PI 3.1415926

#define GOOD_FIT 7//8

//initial position after gate pass
#define INITIAL_X 0
#define INITIAL_Y 1.5
#define INITIAL_Z 0

//initial position and speed safety margins
#define X_POS_MARGIN 0.15//m
#define Y_POS_MARGIN 0.3//m
#define Z_POS_MARGIN 0.2//m
#define X_SPEED_MARGIN 0.15//m/s
#define Y_SPEED_MARGIN 0.15//m/s

void stereocam_to_state(void);
void read_stereo_board(void);

char x_center = 0;
char y_center = 0;
char radius   = 0;
char fitness  = 0;
char fps      = 0;
char x_center_p = 0;
char y_center_p = 0;

float measured_x_gate = 0;
float measured_y_gate = 0;
float measured_z_gate = 0;

float body_v_x = 0;
float body_v_y = 0;

float body_filter_x = 0;
float body_filter_y = 0;

float predicted_x_gate = 0;
float predicted_y_gate = 0;
float predicted_z_gate = 0;

float current_x_gate = 0;
float current_y_gate = 0;
float current_z_gate = 0;
float delta_z_gate   = 0;

float previous_x_gate = 0;
float previous_y_gate = 0;
float previous_z_gate = 0;

// Settings:
float FOV_width = 57.4f;
float FOV_height = 44.5f;
float gate_size_meters = 1.0f;

//SAFETY AND RESET FLAGS
int uncertainty_gate = 0;
int gate_detected = 0;
int init_pos_filter = 0;
int safe_pass_counter = 0;

float fps_filter = 0;


struct timeval stop, start;




float deg2rad(float deg)
{
	return ((deg * PI) / 180.0f);
}

static void stereo_gate_send(struct transport_tx *trans, struct link_device *dev)
    {
    pprz_msg_send_STEREO_GATE_INFO(trans, dev, AC_ID,&x_center, &y_center,&radius,&fitness,&fps,
				   &measured_x_gate,&measured_y_gate,&measured_z_gate,
				   &current_x_gate,&current_y_gate,&delta_z_gate,&fps_filter,
				   &body_v_x,&body_v_y,&uncertainty_gate,
				   &predicted_x_gate,&predicted_y_gate,&gate_detected,&states_race.ready_pass_through);
    }  

 void stereo_gate_position_init(void)
 {
   register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STEREO_GATE_INFO, stereo_gate_send);
   gettimeofday(&start, NULL);
 }
 
 void get_stereo_data_periodic(void) 
 {
   if (stereocam_data.fresh) {
	  read_stereo_board();
	  y_center+=1;
    stereocam_data.fresh = 0;
  }
  stereocam_to_state();
 }
 
void read_stereo_board(void)
{
  x_center = stereocam_data.data[0];
  y_center = stereocam_data.data[1];
  radius   = stereocam_data.data[2];
  fitness  = stereocam_data.data[3];
  fps      = stereocam_data.data[4];
    x_center_p = stereocam_data.data[5];
    y_center_p = stereocam_data.data[6];
}


void stereocam_to_state(void)
{
	//message of length 5 with the x_center (image coord), y_center, radius, fitness (<4 is good, > 10 bad),
	//and frame rate of the calculations.
	
	
	// Determine the measurement:
  
      
	float alpha = (radius / 128.0f) * FOV_width;
	if(alpha < 0.001) alpha = 0.001;
	float measured_distance_gate = (0.5f * gate_size_meters) / tanf(deg2rad(alpha));
	float measured_angle_gate = ((x_center - 64.0f) / (128.0f)) * FOV_width;
	float measured_angle_vert = ((y_center - 48.0f) / (96.0f)) * FOV_height;
	
	measured_x_gate = measured_distance_gate * sin(deg2rad(measured_angle_gate));
	measured_y_gate = measured_distance_gate * cos(deg2rad(measured_angle_gate));
	measured_z_gate = measured_distance_gate * sin(deg2rad(measured_angle_vert));
	
	//SAFETY  gate_detected
	if(measured_y_gate > 1.0 && measured_y_gate < 3.5 && fitness < GOOD_FIT){
	  gate_detected = 1;
        counter_gate_detected = 0;
        time_gate_detected = 0;
	}
	else{
	  gate_detected = 0;
        counter_gate_detected = 0;
        time_gate_detected = 0;
	}
	
	//SAFETY ready_pass_trough
	if(gate_detected == 1 && fabs(measured_x_gate-INITIAL_X) < X_POS_MARGIN && fabs(measured_y_gate-INITIAL_Y) < Y_POS_MARGIN
	  && fabs(measured_z_gate-INITIAL_Z) < Z_POS_MARGIN && fabs(opt_body_v_x)<Y_SPEED_MARGIN && fabs(opt_body_v_x)<Y_SPEED_MARGIN ){
        safe_pass_counter += 1;
	}
	else{
        safe_pass_counter = 0;
	 states_race.ready_pass_through = 0;
	}

    if(safe_pass_counter > 20)
    {
        safe_pass_counter = 0;
        states_race.ready_pass_through = 1;
    }
	  
	
	// Reinitialization after gate is cleared and turn is made(called from velocity guidance module)
	if(init_pos_filter == 1)
	{
	  init_pos_filter = 0;
	  //assumed initial position at other end of the gate
	  predicted_x_gate = INITIAL_X;//0;
	  predicted_y_gate = INITIAL_Y;//1.5;
	  
	}
	
        //State filter 
	

	//convert earth velocity to body x y velocity
	float v_x_earth =stateGetSpeedNed_f()->x;
	float v_y_earth = stateGetSpeedNed_f()->y;
	float psi = stateGetNedToBodyEulers_f()->psi;
	//When using optitrack
	//body_v_x = cosf(psi)*v_x_earth + sinf(psi)*v_y_earth;
	//body_v_y = -sinf(psi)*v_x_earth+cosf(psi)*v_y_earth;
	
	body_v_x = opt_body_v_x;
	body_v_y = opt_body_v_y;
	
	//body velocity in filter frame
	body_filter_x = -body_v_y;
	body_filter_y = -body_v_x;
	
	gettimeofday(&stop, 0);
	double curr_time = (double)(stop.tv_sec + stop.tv_usec / 1000000.0);
	double elapsed = curr_time - (double)(start.tv_sec + start.tv_usec / 1000000.0);
	gettimeofday(&start, 0);
	float dt = elapsed;
	
	fps_filter = (float)1.0/dt;
	
        // predict the new location:
	float dx_gate = dt * body_filter_x;//(cos(current_angle_gate) * gate_turn_rate * current_distance);
	float dy_gate = dt * body_filter_y; //(velocity_gate - sin(current_angle_gate) * gate_turn_rate * current_distance);
	predicted_x_gate = previous_x_gate + dx_gate;
	predicted_y_gate = previous_y_gate + dy_gate;
	predicted_z_gate = previous_z_gate;
	
	float sonar_alt = stateGetPositionNed_f()->z;
	
        if (gate_detected == 1)
	{
	
		// Mix the measurement with the prediction:
		float weight_measurement;
		if (uncertainty_gate > 150)
		{
			weight_measurement = 1.0f;
			uncertainty_gate = 151;//max
		}
		else
			weight_measurement = (GOOD_FIT-(float)fitness)/GOOD_FIT;//check constant weight 

		current_x_gate = weight_measurement * measured_x_gate + (1.0f - weight_measurement) * predicted_x_gate;
		current_y_gate = weight_measurement * measured_y_gate + (1.0f - weight_measurement) * predicted_y_gate;
		current_z_gate = weight_measurement * (measured_z_gate + sonar_alt) + (1.0f - weight_measurement) * predicted_z_gate;
		

		// reset uncertainty:
		uncertainty_gate = 0;
	}
	else
	{
		// just the prediction
		current_x_gate = predicted_x_gate;
		current_y_gate = predicted_y_gate;
		current_z_gate = predicted_z_gate;

		// increase uncertainty
		uncertainty_gate++;
	}
	// set the previous state for the next time:
	previous_x_gate = current_x_gate;
	previous_y_gate = current_y_gate;
	previous_z_gate = current_z_gate;
	delta_z_gate = current_z_gate - sonar_alt;

}

