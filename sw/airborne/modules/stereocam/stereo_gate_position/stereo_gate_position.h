/*
 * Copyright (C) Michaël Ozo
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/stereocam/stereo_gate_position/stereo_gate_position.h"
 * @author Michaël Ozo
 * based on stereo vision module data  gate detection is fused with optic flow velocity.
 */

#ifndef STEREO_GATE_POSITION_H
#define STEREO_GATE_POSITION_H

#include <std.h>
#include "modules/stereocam/stereocam.h"

extern void stereo_gate_position_init(void);
extern void get_stereo_data_periodic(void);

extern float measured_x_gate;
extern float measured_y_gate;
extern float measured_z_gate;
extern float delta_z_gate;

extern float current_x_gate;
extern float current_y_gate;
extern char fitness;
extern int gate_detected;
extern int ready_pass_trough;
extern int init_pos_filter;

#endif

