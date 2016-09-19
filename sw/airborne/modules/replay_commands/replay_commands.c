/*
 * Copyright (C) Isabelle
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
 * @file "modules/replay_commands/replay_commands.c"
 * @author Isabelle
 * will replay the commands recorded by the logger
 */

#include "modules/replay_commands/replay_commands.h"
#include "modules/guidance_loop_velocity_autonomous_race/guidance_loop_velocity_autonomous_race.h"
#include<stdlib.h>
#include<string.h>
#include<stdio.h>
//#include<conio.h>
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.h"
#include "modules/command_level_iros/command_level_iros.h"
#include "modules/loggers/file_logger.h"
#include "math/pprz_algebra_int.h"
#include "firmwares/rotorcraft/autopilot.h"



#define NULL 0

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

int replay = 0;
struct Int32Eulers guidance_replay;
FILE * file;

void replay_commands_start(void){
	//command_run();
	replay = 1;

	guidance_replay.phi = 0;
	guidance_replay.theta = 0;
	guidance_replay.psi = BFP_OF_REAL(stateGetNedToBodyEulers_f()->psi, INT32_ANGLE_FRAC);
	int primitive_number = 0; //set the primitive number manually for now.

	char filename[512];
	sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), primitive_number);

	FILE * file;
	file = fopen( filename , "r");

	if(file == NULL)
	{
	  printf("\n file opening failed ");

	}
}

void replay_commands_init(void){




}

void replay_commands_periodic(void){
	int n = 105; //maximum number of characters in the line
	char str[105];

	int counter,phi_i,theta_i,psi_i, cmd_phi, cmd_theta, cmd_psi;
	double x,y,z,vx,vy,vz,phi,theta,psi;

	int file_end = feof(file);

	if(file_end == 1)
		{
		 replay = 0;
		 fclose(file);

		}

	if(replay == 1 && autopilot_mode == AP_MODE_MODULE){ //while(fgets(str,n,file)!=NULL){
			printf("Here is the start\n");

			fgets(str,n,file);
			for (int i=0 ; i<strlen(str) ; i++){
				if (str[i]==',')
					str[i]=' ';
			}
			printf("%s",str);
			sscanf(str,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %d %d %d %d",&counter,&x,&y,&z,&vx,&vy,&vz,&phi,&theta,&psi,&phi_i,&theta_i,&psi_i, &cmd_phi, &cmd_theta, &cmd_psi);
			//printf("counter: %d x: %lf, y:%lf, z:%lf, vx:%lf, vy:%lf, vz:%lf, phi:%lf, theta:%lf, psi:%lf, phi_i %d, theta_i %d, psi_d %d \n",counter, x,y,z,vx,vy,vz,phi,theta,psi,phi_i, theta_i, psi_i);


	//give the commands here
			
			guidance_replay.phi = cmd_phi;
			guidance_replay.theta = cmd_theta;
			guidance_replay.psi = cmd_psi;



			//stabilization_attitude_set_rpy_setpoint_i(&guidance_module.cmd);

			//stabilization_attitude_set_earth_cmd_i(cmd,heading);

	}

}

void replay_commands_stop(void){
	//if(autopilot_mode != AP_MODE_MODULE)
	replay = 0;
	fclose(file);

}







