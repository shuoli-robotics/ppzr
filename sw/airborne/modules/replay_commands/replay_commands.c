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
#include<stdlib.h>
#include<string.h>
#include<stdio.h>

#include"std.h"
//#include<conio.>
#include "modules/replay_commands/replay_commands.h"
#include "modules/guidance_loop_velocity_autonomous_race/guidance_loop_velocity_autonomous_race.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.h"
#include "modules/command_level_iros/command_level_iros.h"
#include "modules/loggers/file_logger.h"
#include "math/pprz_algebra_int.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/datalink/telemetry.h"
#include "state.h"

struct guidance_module_st guidance_module;
struct Int32Eulers guidance_replay;


int replay = 0;
int counter, phi_i, theta_i, psi_i, cmd_phi, cmd_theta, cmd_psi,cmd_thrust;
double x, y, z, vx, vy, vz, phi, theta, psi;
int32_t phi_euler = 0;
int32_t theta_euler = 0;
int32_t psi_euler = 0;


/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif
struct Int32Eulers guidance_replay;
FILE * file;





static void replay_cmd_send(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_REPLAY_CMD_INFO(trans, dev, AC_ID, &cmd_thrust, &replay, &psi_i, &cmd_phi, &cmd_theta, &cmd_psi, &phi_euler, &theta_euler, &psi_euler);
}


void replay_commands_start(void)
{
  //command_run();
  replay = 1;
   // guidance_v_mode_changed(GUIDANCE_V_MODE_MODULE);
  guidance_replay.phi = 0;
  guidance_replay.theta = 0;
  guidance_replay.psi = BFP_OF_REAL(stateGetNedToBodyEulers_f()->psi, INT32_ANGLE_FRAC);
  int primitive_number = 6; //set the primitive number manually for now.
  //30 straight 3m turn 90 deg left and straith 3m while going up.

  char filename[512];
  sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), primitive_number);
  file = fopen(filename , "r");
  
  if (file == NULL) {
    printf("\n file opening failed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
  }
}



void replay_commands_init(void)
{
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_REPLAY_CMD_INFO, replay_cmd_send);
  file = NULL;
}

void replay_commands_periodic(void)
{
  int n = 400; //maximum number of characters in the line
  char str[400];

  if (replay == 1 && file != NULL){// && autopilot_mode == AP_MODE_MODULE) { //while(fgets(str,n,file)!=NULL){
   // printf("Here is the start\n");

    if( fgets(str, n, file) != NULL ){
      for (int i = 0 ; i < strlen(str) ; i++) {
        if (str[i] == ',') {
          str[i] = ' ';
        }
      }
      //printf("%s", str);
      sscanf(str, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %d %d %d %d %d", &counter, &x, &y, &z, &vx, &vy, &vz, &phi, &theta, &psi, &phi_i, &theta_i, &psi_i, &cmd_phi, &cmd_theta, &cmd_psi,&cmd_thrust);
      //printf("counter: %d x: %lf, y:%lf, z:%lf, vx:%lf, vy:%lf, vz:%lf, phi:%lf, theta:%lf, psi:%lf, phi_i %d, theta_i %d, psi_d %d \n",counter, x,y,z,vx,vy,vz,phi,theta,psi,phi_i, theta_i, psi_i);


      //give the commands here (copied to guidance_loop_velocity_autonomous_race)
      //struct Int32Eulers guidance_replay;
      guidance_replay.phi = cmd_phi;
      guidance_replay.theta = cmd_theta;
      guidance_replay.psi = cmd_psi;

      //stabilization_attitude_set_rpy_setpoint_i(&guidance_module.cmd);

      //stabilization_attitude_set_earth_cmd_i(cmd,heading);
    } else {
      /*replay = 0;
      fclose(file);
      file = NULL;*/
        replay_commands_stop();
    }
  }

//  phi_euler   = stateGetNedToBodyEulers_i()->phi;
//  theta_euler = stateGetNedToBodyEulers_i()->theta;
//  psi_euler   = stateGetNedToBodyEulers_i()->psi;

}

void replay_commands_stop(void)
{
  //if(autopilot_mode != AP_MODE_MODULE)
  replay = 0;
    guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
  if (file != NULL) {
    fclose(file);
    file = NULL;
  }

}







