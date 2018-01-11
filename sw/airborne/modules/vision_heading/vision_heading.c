/*
 * Copyright (C) Michaël Ozo
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
 * @file "modules/vision_heading/vision_heading.c"
 * @author Michaël Ozo
 * Use gate detection data for the drones heading with respect to this gate. This using back-projection optimization.
 */

#include "modules/vision_heading/vision_heading.h"
#include "modules/computer_vision/lib/vision/closed_gate_processing.h"

#define RING_BUFFER_SIZE 50

int buf_pointer = 0;
int read_pointer = 0;

void vision_heading_init(void) {}
void vision_heading_periodic(void) {
  
  //Accept new valid gate corner points and put in array for processing, including time stamp, initial position etc for received.
  //If processing array is not empty, process new point pair from array.
    //For each point pair initialize the problem with initial pos etc
    //gradient descent for error minimization using attitude and heading angles and physical gate point locations
    //using back_proj_points(struct FloatVect3 *gate_point, struct FloatVect3 *cam_pos, struct FloatMat33 *R_mat, float *x_res, float *y_res)
    //repeat on each gate bearing angle
  //put end result in result array, together with time stamp and possible metadata
  //this result array is then later used by a filter module which fuses the heading data with gyro data
  
  
  
  //Arrays for corner points should be a ring buffer
  float x_1[RING_BUFFER_SIZE];
  float y_1[RING_BUFFER_SIZE];
  float x_2[RING_BUFFER_SIZE];
  float y_2[RING_BUFFER_SIZE];
  float x_3[RING_BUFFER_SIZE];
  float y_3[RING_BUFFER_SIZE];
  float x_4[RING_BUFFER_SIZE];
  float y_4[RING_BUFFER_SIZE];
  float altitude[RING_BUFFER_SIZE];
  struct FloatVect3 gate_vectors[RING_BUFFER_SIZE];
  double detection_time[RING_BUFFER_SIZE];
  
  //if new data avaiable ring buffer pointer increment from zero to max length
  //fill buffer arrays with data at pointer location
  //check in case of write trying to pass read, stop processing and pass to next location, is this needed??
  if(new_detection){
    new_detection = 0;
    buf_pointer++;
    if(buf_pointer >= RING_BUFFER_SIZE)buf_pointer = 0;
    
    
    x_1[buf_pointer] = gate_img_point_x_1;
    y_1[buf_pointer] = gate_img_point_y_1;
    x_2[buf_pointer] = gate_img_point_x_2;
    y_2[buf_pointer] = gate_img_point_y_2;
    x_3[buf_pointer] = gate_img_point_x_3;
    y_3[buf_pointer] = gate_img_point_y_3;
    x_4[buf_pointer] = gate_img_point_x_4;
    y_4[buf_pointer] = gate_img_point_y_4;
    altitude[buf_pointer];
    gate_vectors[buf_pointer];
    detection_time[buf_pointer];
  }
  
  //read buffer until read_pointer is at same location as buf_pointer
  //this way read cant pass write pointer
  //also do something when lagging to far behind???!!!
  if(read_pointer != buf_pointer){
    read_pointer++;
    if(read_pointer >= RING_BUFFER_SIZE)read_pointer = 0;
    
    //init calculation
    /*
    cam_gd = sqrt((gate_pos(1)-cam_pos(1))^2+(gate_pos(2)-cam_pos(2))^2);
    cam_a = atan((gate_pos(2)-cam_pos(2))/(gate_pos(1)-cam_pos(1)));
    phi_g =  phi_gate;%positive rotation left ccw
    theta_g = theta_gate;%positive down
    psi_g = psi_gate;%positive to the left*/
    //xy plane distance
    //float cam_gd = 
    
//  for cam_a_i = (cam_a-deg2rad(35)):deg2rad(2):(cam_a+deg2rad(35))
//   angle_iter = rad2deg(cam_a_i);
//   cam_a = cam_a_i;
//   [fit, pos_a] = get_pix_error(sim_points_x,sim_points_y,gate_position,cam_pos,cam_gd,cam_a_i,phi_g,theta_g,psi_g);
//   for i = 1:n_iter
//     
//       %explore area around previous fit
//       %[fit_a, ~] = get_pix_error(sim_points_x,sim_points_y,gate_position,cam_pos,cam_gd,cam_a_i+g_step,phi_g,theta_g,psi_g);
//       [fit_phi, ~] = get_pix_error(sim_points_x,sim_points_y,gate_position,cam_pos,cam_gd,cam_a_i,phi_g+g_step,theta_g,psi_g);
//       [fit_theta, ~] = get_pix_error(sim_points_x,sim_points_y,gate_position,cam_pos,cam_gd,cam_a_i,phi_g,theta_g+g_step,psi_g);
//       [fit_psi, ~] = get_pix_error(sim_points_x,sim_points_y,gate_position,cam_pos,cam_gd,cam_a_i,phi_g,theta_g,psi_g+g_step);
//       
//       %gradient at current solution
//       gradient = fit-[0 fit_phi fit_theta fit_psi];
//       
//       %new fit
//       %cam_a = cam_a+gradient(1)*g_factor;
//       phi_g = phi_g+gradient(2)*g_factor;
//       theta_g = theta_g+gradient(3)*g_factor;
//       psi_g = psi_g+gradient(4)*g_factor;
//       [fit, pos_a] = get_pix_error(sim_points_x,sim_points_y,gate_position,cam_pos,cam_gd,cam_a_i,phi_g,theta_g,psi_g);
//     
// 	  
//   %      waitforbuttonpress
// 
//       if(fit < best_fit)
// 	  best_fit = fit;
// 	  rad2deg(cam_a);
// 	  
// 	  %save best results
// 	  phi_est = phi_g;
// 	  theta_est = theta_g;
// 	  psi_est = psi_g;
// 	  rad2deg(psi_est)
// 	  pos_a(3) = -pos_a(3);
// 	  angle_iter = rad2deg(cam_a_i)
// 	  %waitforbuttonpress
//       end
//       
//   %     %%%%PLOTTING DEBUGGING
//   %     %waitforbuttonpress
//   %      best_fit
// 
//   end
//   %waitforbuttonpress
    
    
  }
  
  
  
}

// function [error, p_a] = get_pix_error(sim_p_x,sim_p_y,gate_p,cam_p,cam_gate_dist,c_a,phi_,theta_,psi_)
//      [search_points_x, search_points_y, p_a] = Gate_angle_search_ng(gate_p,cam_p(3),cam_gate_dist,c_a,phi_,theta_,psi_);
//     
// %     plot(search_points_x,search_points_y,'+')
// %     axis([0 320, 0 160])
// %     hold on
//      
//     pix_error = zeros(4,1);
//     for n = 1:4
// %      pix_error(n) = sqrt((sim_p_x(n)-search_points_x(n)+153.2)^2+(sim_p_y(n)-search_points_y(n)+32)^2);
//        pix_error(n) = sqrt((sim_p_x(n)-search_points_x(n)+150.3280)^2+(sim_p_y(n)-search_points_y(n)+34.8997)^2);
//     end
// 
// %     plot(search_points_x,search_points_y,'+')
// %     axis([0 320, 0 160])
// %     hold on
// %     plot(sim_points_x+153.2,sim_points_y+32,'+')
// %     axis([0 320, 0 160])
//     
//     error = sum(pix_error);%sum mean etc
// end


//Back_projection function, based on cam_pos from gradient variables
// function [sim_points_x, sim_points_y, cam_pos] = Gate_angle_search_ng(gate_pos,cam_h,cam_gd,cam_a,phi_g,theta_g,psi_g)
// 
// f = 169;
// % intr = [f 0  153.2;
// %         0  f 32 ;
// %         0   0  0  ];
// %matlab undistort
// intr = [f 0  150.3280;
//         0  f 34.8997 ;
//         0   0  0  ];
//     
// 
// R_20 = C_b_n(0, deg2rad(-20), 0);
// R = C_b_n( phi_g, theta_g, psi_g );
// 
// %[ gate_points ] = Calc_gate_points_order( gate_pos );
// [ gate_points ] = Calc_gate_points_neu( gate_pos );
// 
// sim_points_x = zeros(4,1);
// sim_points_y = zeros(4,1);
// 
// cam_pos = [gate_pos(1)-cam_gd*cos(cam_a) gate_pos(2)-cam_gd*sin(cam_a) cam_h];
// %cam_pos = [1.0 0 1.4]
// for n = 1:4
//     gate_points(:,n);
//      [ u_b, v_b ] = proj_point( gate_points(:,n),intr, (R_20*R'),cam_pos);
//      sim_points_x(n) = u_b;
//      sim_points_y(n) = v_b;
// end

