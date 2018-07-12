#ifndef _NN_H
#define _NN_H

#include "nn_params.h"
#include <time.h>

extern int nn_counter;
extern clock_t nn_time;

void compute_control(double **ptr_arr_1, double **ptr_arr_2);
void nn_run(void);
void test(double state[NUM_STATE_VARS]) ;

#endif
