#ifndef _NN_H
#define _NN_H
#include "nn_params.h"

void compute_control(double **ptr_arr_1, double **ptr_arr_2);
extern void nn(double state[NUM_STATE_VARS], double control[NUM_CONTROL_VARS]);
#endif
