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

#include "subsystems/datalink/telemetry.h"
#include "modules/stereocam/stereo_gate_position/stereo_gate_position.h"

void stereocam_to_state(void);

char x_center = 1;
char y_center = 0;
char radius   = 0;
char fitness  = 0;
char fps      = 1;

static void stereo_gate_send(struct transport_tx *trans, struct link_device *dev)
    {
    pprz_msg_send_STEREO_GATE_INFO(trans, dev, AC_ID,&x_center, &y_center,&radius,&fitness,&fps);
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
  
}

