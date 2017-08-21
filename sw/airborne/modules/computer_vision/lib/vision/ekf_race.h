
#include <stdint.h>

void EKF_init(void); 
void init_eye_7(float MAT[7][7]);
void EKF_init_diag(float A[7][7], float diag[7]);
void EKF_propagate_state(float x_prev[7][1], float new_state[7][1], float dt, float xdot[7][1], float x[7], float u[8]);
void EKF_update_state(float x_state[7][1],float x_opt[7][1], float z_k_d[3], float EKF_delta);
void EKF_evaluate_jacobian(float Jac_F[7][7], float phi_s, float theta_s, float psi_s, float q_s, float p_s);
void c_2_d(float A_d[7][7], float A[7][7],float dt);