#include <stdio.h>
#include <string.h>
#include "nn.h"
#include "nn_params.h"


/* This function is for testing purposes only and should not be used for */
/* embedded implementation since it (unnecessarily) allocates */
/* two arrays every time */
void nn(double state[NUM_STATE_VARS], double control[NUM_CONTROL_VARS]) {
    /* allocate memory for two arrays required by compute_control() */
    /* number of array elements must be at least no. of units in widest layer */
    double arr_1_tmp[MAX_LAYER_DIMS];
    double arr_2_tmp[MAX_LAYER_DIMS];
    double *arr_1 = (double *) arr_1_tmp; /* replace with malloc if necessary */
    double *arr_2 = (double *) arr_2_tmp;
    /* note the block of memory pointed to by the above may swap as a result */
    /* of calling compute_control() */

    /* on function call arr_1 contains state values */
    /* on completion, arr_1 contains control values */
    memcpy(arr_1, state, NUM_STATE_VARS * sizeof(double));
    compute_control(&arr_1, &arr_2);
    memcpy(control, arr_1, NUM_CONTROL_VARS * sizeof(double));
}


int main() {
    double state[NUM_STATE_VARS] = {
        -4.2232016635363578e-02, -3.0225037024451664e+00, -6.1490007593689278e-01,
            5.1089365659897990e-01, 2.9941452020833115e-01
    };
    double control[NUM_CONTROL_VARS];
    int i;

    nn(state, control);
    for (i = 0; i < NUM_CONTROL_VARS; i++) {
        printf("%.16f\n", control[i]);
    }

    return 0;
}
