
#include <stdint.h>
#include "subsystems/imu.h"
#include "state.h"
#include "modules/computer_vision/snake_gate_detection.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_simple_matrix.h"

#include "filters/low_pass_filter.h"

#include "modules/computer_vision/lib/vision/ekf_race.h"

// float Jac_F[7][7] = {0};

float eye_7[7][7] = {{0}};
float temp_m_1[7][7] = {{0}};
float temp_m_2[7][7] = {{0}};
float temp_m_3[7][7] = {{0}};
float DFx[7][7] = {{0}};
float Phi_d[7][7] = {{0}};
float P_k_1_d[7][7] = {{0}};
float P_k_1_k_1_d[7][7] = {{0}};
float Q[7][7] = {{0}};

float DHx[3][7] = {   
   { 1, 0, 0, 0, 0, 0, 0} ,
   { 0, 1, 0, 0, 0, 0, 0} ,
   { 0, 0, 1, 0, 0, 0, 0}
};

float R_k_d[3][3] = {   
   { 0.2, 0, 0, } ,
   { 0, 0.2, 0, } ,
   { 0, 0, 0.2, }
};

float K_d[7][3] = {{0}};

float temp_7_3_1[7][3] = {{0}};

float EKF_inn[3][1] = {{0}};

float temp_7_1_1[7][1] = {{0}};

float temp_3_7_1[3][7] = {{0}};

float temp_3_3_1[3][3] = {{0}};

float temp_3_3_2[3][3] = {{0}};

void EKF_init(void){
  
  init_eye_7(eye_7);
  
  float P_k_1_diag[7] = {1,1,1,1,1,1,1};
  EKF_init_diag(P_k_1_k_1_d,P_k_1_diag);
  
  //check  process noise of biases---------------------------------
  //float Q_diag[7] = {0.2,0.2,0.2,0.2,0,0,0};
  float Q_diag[7] = {0.2,0.2,0.2,0.2,0.01,0.01,0.01};
  EKF_init_diag(Q,Q_diag);
  
  //Trail solution matlab
//   float X_[7][1] = {{0}};
//   float z_k_d[3];
//   float EKF_m_dt = 0.067;
//   z_k_d[0] = 0.1;
//   z_k_d[1] = 0.1;
//   z_k_d[2] = 0.1;
//   EKF_update_state(X_,X_,z_k_d,EKF_m_dt);
//   
//   while(1){
//   }
  
}

void init_eye_7(float MAT[7][7]){
  for(int i = 0; i<7;i++){
    MAT[i][i] = 1;
  }
}

void EKF_init_diag(float A[7][7], float diag[7]){
  for(int i = 0; i<7;i++){
    A[i][i] = diag[i];
  }
}

void EKF_propagate_state(float x_prev[7][1], float new_state[7][1], float dt, float u_k[8][1]){
  
//    dt = time_stamp(n)-time_stamp(n-1);
//    omega = [gyro_p(n) gyro_q(n)];
//    
//    body_speed(n,:) = (C_b_n(phi(n),theta(n),psi(n))'*[vel_x(n) vel_y(n) vel_z(n)]')';
//  
//    U_k = [acc_x_filter(n) acc_y_filter(n) acc_z(n) omega phi(n) theta(n) psi(n)];
// 
//    X_int = [X_int; X_int_prev + d_kf_calc_f_nl(dt,X_int_prev,U_k)'*dt];

  const float D_x = -1/0.5;
  const float D_y = -1/0.43;
  
  const float g = 9.81;
  
  float x[7];
  float u[8];
  float xdot[7][1];
  
  //copy x into 1d array
  for(int i = 0;i < 7; i++){
    x[i] = x_prev[i][0];
  }
  
  //copy x into 1d array
  for(int i = 0;i < 8; i++){
    u[i] = u_k[i][0];
  }
  
  // %xdot
  xdot[0][0] = (D_x*(u[0]-x[4])*cos(u[6])+( D_y*(u[1]-x[5])*sin(u[5])+x[3]*cos(u[5]))*sin(u[6]) )*cos(u[7])-( D_y*(u[1]-x[5])*cos(u[5])-x[3]*sin(u[5]))*sin(u[7]);
  //%ydot
  xdot[1][0] = (D_x*(u[0]-x[4])*cos(u[6])+( D_y*(u[1]-x[5])*sin(u[5])+x[3]*cos(u[5]))*sin(u[6]) )*sin(u[7])+( D_y*(u[1]-x[5])*cos(u[5])-x[3]*sin(u[5]))*cos(u[7]);
  //%zdot
  xdot[2][0] = -D_x*(u[0]-x[4])*sin(u[6])+( D_y*(u[1]-x[5])*sin(u[5])+x[3]*cos(u[5]))*cos(u[6]); 
  //%wdot 6
  xdot[3][0] = (u[2]-x[6])+g*cos(u[6])*cos(u[5])+(u[4])*D_x*(u[0]-x[4])-(u[3])*D_y*(u[1]-x[5]);
  //%x10 = lambda_x
  xdot[4][0] = 0;//%constant
  //%x11 = lambda_y
  xdot[5][0] = 0;
  //%x12 = lambda_z
  xdot[6][0] = 0;

  //new_state = curr_state + xdot*dt
  MAT_SUM_c(7,1, new_state, x_prev, xdot,dt);
  
}

void EKF_update_state(float x_state[7][1],float x_opt[7][1], float z_k_d[3], float EKF_delta){
//         x_kk_1 = X_int(n,:);
//         %G = d_kf_calc_G_nl(x_kk_1);
//         G = eye(7);
//         DFx = d_kf_calc_Fx_nl(Jac_mat_f, x_kk_1, U_k);
//         sum_norm = sum_norm + norm(DFx^2);
//         [Phi, Gamma] = c2d(DFx, G,EKF_dt);
//         Phi = c2d_A(DFx,EKF_dt);
//         
//         % P(k+1|k) (prediction covariance matrix) with additive noise model
//         P_k_1 = Phi*P_k_1_k_1*Phi' + Q;%Gamma*Q*Gamma';
// 
//         DHx = d_kf_calc_Hx_nl(Jac_mat_h, x_kk_1, U_k);
// 
//         K = P_k_1 * DHx' / (DHx*P_k_1 * DHx' + R_k);
// 
//         %z_k = [pos_x(n) pos_y(n) pos_z(n)];
//          %z_k = [pos_vision_x(detect_count_m) pos_vision_y(detect_count_m) pos_z(n)];% -sonar_alt(n)];
//         z_k = [pos_vision_x(detect_count_m) pos_vision_y(detect_count_m) -sonar_alt(n)];
//        
//         X_opt = x_kk_1' + K * (z_k - X_int(n,1:3))';
//         X_int(n,:) = X_opt';
// 
//         P_k_1_k_1 = (eye(7) - K*DHx) * P_k_1 * (eye(7) - K*DHx)' + K*R_k*K';

  float phi_s = stateGetNedToBodyEulers_f()->phi;
  float theta_s = stateGetNedToBodyEulers_f()->theta;
  float psi_s = stateGetNedToBodyEulers_f()->psi;
  
  float p_s = stateGetBodyRates_f()->p;
  float q_s = stateGetBodyRates_f()->q;
  
//   //Trail solution matlab
//   float phi_s = 0.1;
//   float theta_s = 0.1;
//   float psi_s = 0.1;
//   
//   float p_s = 0.1;
//   float q_s = 0.1;
  
  //jacobian
  EKF_evaluate_jacobian(DFx,phi_s,theta_s,psi_s,q_s,p_s);

  //MAT_PRINT(7, 7,DFx);
  //discretize the system
  c_2_d(Phi_d, DFx,EKF_delta);

//  MAT_PRINT(7, 7,Phi_d);
  
  //P_k_1 = Phi*P_k_1_k_1*Phi' + Q
  //temp_m_1=P_k_1_k_1*Phi'
   MAT_MUL_T(7,7,7, temp_m_1, P_k_1_k_1_d, Phi_d);
  //temp_m_2=Phi*temp_m_1
   MAT_MUL(7,7,7, temp_m_2, Phi_d,temp_m_1);
  //P_k_1_d=temp_m_2 + Q
   MAT_SUM(7, 7, P_k_1_d,temp_m_2, Q);
  
  // MAT_PRINT(7, 7,P_k_1_d);
  
  //K = P_k_1 * DHx' / (DHx*P_k_1 * DHx' + R_k);
  //temp_7_3_1=P_k_1 * DHx' 
  MAT_MUL_T(7,7,3, temp_7_3_1, P_k_1_d, DHx);
  //temp_3_3_1=DHx*temp_7_3_1
  MAT_MUL(3,7,3, temp_3_3_1, DHx, temp_7_3_1);
  //temp_3_3_2=temp_3_3_1 + R_k
  MAT_SUM(3, 3, temp_3_3_2, temp_3_3_1, R_k_d);
  //temp_3_3_1=inv(temp_3_3_2)
  MAT_INV33(temp_3_3_1, temp_3_3_2);
  //K_d=temp_7_3_1*temp_3_3_1
  MAT_MUL(7,3,3, K_d, temp_7_3_1, temp_3_3_1);
  
  //MAT_PRINT(7, 3,K_d);
  
  //X_opt = x_kk_1' + K * (z_k - X_int(n,1:3))';
  //EKF_inn=z_k - X_int(n,1:3)
  EKF_inn[0][0] =  z_k_d[0] - x_state[0][0];
  EKF_inn[1][0] =  z_k_d[1] - x_state[1][0];
  EKF_inn[2][0] =  z_k_d[2] - x_state[2][0];
  //temp_7_1_1=K_d*EKF_inn
  MAT_MUL(7,3,1, temp_7_1_1, K_d, EKF_inn);
  //x_opt=x_state + temp_7_1_1
  MAT_SUM(7, 1, x_opt, x_state, temp_7_1_1);
  
  //MAT_PRINT(3, 1,EKF_inn);
  //MAT_PRINT(7, 1,x_opt);
  
  //P_k_1_k_1 = (eye(7) - K*DHx) * P_k_1 * (eye(7) - K*DHx)' + K*R_k*K';
  MAT_MUL(7, 3, 7, temp_m_1, K_d,DHx);
  //MAT_PRINT(7, 7,temp_m_1);
    
  MAT_SUB(7, 7, temp_m_2, eye_7, temp_m_1);
  //MAT_PRINT(7, 7,temp_m_2);
  //error free until here?
  MAT_MUL_T(7, 7, 7, temp_m_1, P_k_1_d, temp_m_2);
  // MAT_PRINT(7, 7,temp_m_1);
  //temp_4_4_a reuse
  MAT_MUL(7, 7, 7, temp_m_3, temp_m_2,temp_m_1);
  // MAT_PRINT(7, 7,temp_m_3);
  MAT_MUL_T(3, 3, 7, temp_3_7_1, R_k_d, K_d);
  //  MAT_PRINT(3, 7,temp_3_7_1);
  MAT_MUL(7, 3, 7, temp_m_1, K_d,temp_3_7_1);
  // MAT_PRINT(7, 7,temp_m_1);
  MAT_SUM(7, 7, P_k_1_k_1_d, temp_m_3, temp_m_1);
  
  //MAT_PRINT(7, 7,P_k_1_k_1_d);
  
}

void EKF_evaluate_jacobian(float Jac_F[7][7], float phi_s, float theta_s, float psi_s, float q_s, float p_s){
//   Jacobian F
// 
// [ 0, 0, 0, sin(phi_s)*sin(psi_s) + cos(phi_s)*cos(psi_s)*sin(theta_s), 2*cos(psi_s)*cos(theta_s), (100*cos(psi_s)*sin(phi_s)*sin(theta_s))/43 - (100*cos(phi_s)*sin(psi_s))/43,  0]
// [ 0, 0, 0, cos(phi_s)*sin(psi_s)*sin(theta_s) - cos(psi_s)*sin(phi_s), 2*cos(theta_s)*sin(psi_s), (100*cos(phi_s)*cos(psi_s))/43 + (100*sin(phi_s)*sin(psi_s)*sin(theta_s))/43,  0]
// [ 0, 0, 0,                                    cos(phi_s)*cos(theta_s),           -2*sin(theta_s),                                             (100*cos(theta_s)*sin(phi_s))/43,  0]
// [ 0, 0, 0,                                                          0,                     2*q_s,                                                                -(100*p_s)/43, -1]
// [ 0, 0, 0,                                                          0,                         0,                                                                            0,  0]
// [ 0, 0, 0,                                                          0,                         0,                                                                            0,  0]
// [ 0, 0, 0,                                                          0,                         0,                                                                            0,  0]
//  
// 
// Jac_mat_h =
//  
// [ 1, 0, 0, 0, 0, 0, 0]
// [ 0, 1, 0, 0, 0, 0, 0]
// [ 0, 0, 1, 0, 0, 0, 0]
  
  // Jacobian F
  //Jac_F[0][0] - Jac_F[0][6]
  // [ 0, 0, 0, sin(phi_s)*sin(psi_s) + cos(phi_s)*cos(psi_s)*sin(theta_s), 2*cos(psi_s)*cos(theta_s), (100*cos(psi_s)*sin(phi_s)*sin(theta_s))/43 - (100*cos(phi_s)*sin(psi_s))/43,  0]
  Jac_F[0][3] = sin(phi_s)*sin(psi_s) + cos(phi_s)*cos(psi_s)*sin(theta_s);
  Jac_F[0][4] = 2*cos(psi_s)*cos(theta_s);
  Jac_F[0][5] = (100*cos(psi_s)*sin(phi_s)*sin(theta_s))/43 - (100*cos(phi_s)*sin(psi_s))/43;
  
  //Jac_F[1][0] - Jac_F[1][6]
  // [ 0, 0, 0, cos(phi_s)*sin(psi_s)*sin(theta_s) - cos(psi_s)*sin(phi_s), 2*cos(theta_s)*sin(psi_s), (100*cos(phi_s)*cos(psi_s))/43 + (100*sin(phi_s)*sin(psi_s)*sin(theta_s))/43,  0]
  Jac_F[1][3] = cos(phi_s)*sin(psi_s)*sin(theta_s) - cos(psi_s)*sin(phi_s);
  Jac_F[1][4] = 2*cos(theta_s)*sin(psi_s);
  Jac_F[1][5] = (100*cos(phi_s)*cos(psi_s))/43 + (100*sin(phi_s)*sin(psi_s)*sin(theta_s))/43;
  
  //Jac_F[2][0] - Jac_F[2][6]
  // [ 0, 0, 0,                                    cos(phi_s)*cos(theta_s),           -2*sin(theta_s),                                             (100*cos(theta_s)*sin(phi_s))/43,  0]
  Jac_F[2][3] = cos(phi_s)*cos(theta_s);
  Jac_F[2][4] = -2*sin(theta_s);
  Jac_F[2][5] = (100*cos(theta_s)*sin(phi_s))/43;
  
  //Jac_F[3][0] - Jac_F[3][6]
  // [ 0, 0, 0,                                                          0,                     2*q_s,                                                                -(100*p_s)/43, -1]
  Jac_F[3][4] = 2*q_s;
  Jac_F[3][5] = -(100*p_s)/43;
  Jac_F[3][6] = -1;
  
// [ 0, 0, 0,                                                          0,                         0,                                                                            0,  0]
// [ 0, 0, 0,                                                          0,                         0,                                                                            0,  0]
// [ 0, 0, 0,                                                          0,                         0,                                                                            0,  0]
//  
// 
// Jac_mat_h =
//  
// [ 1, 0, 0, 0, 0, 0, 0]
// [ 0, 1, 0, 0, 0, 0, 0]
// [ 0, 0, 1, 0, 0, 0, 0]

}

//discretizing 7x7 Jacobian using the power series for exp(A). until A^2 (zero from A^3, since upper triangular mat)
//A_d = I + A + (1/2)*A*A*dt
void c_2_d(float A_d[7][7], float A[7][7],float dt){
  
  //I + A*dt
  MAT_SUM_c(7, 7, temp_m_1,eye_7, A,dt);
  
  //(1/2)*A*A*c
  float time_c = 0.5*dt;
  MAT_MUL_c(7, 7, 7, temp_m_2, A, A,time_c);
  
  MAT_SUM(7, 7, A_d,temp_m_1, temp_m_2);
  
  //MAT_PRINT(7, 7,A_d);
  
}




