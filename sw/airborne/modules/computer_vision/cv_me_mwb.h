/*
 * cv_ae_mwb.c
 *
 *  Created on: Sep 24, 2017
 *      Author: mavlab
 */

#ifndef CV_AE_MWB_C_
#define CV_AE_MWB_C_


extern float cv_me_mwb_exposure;
extern float cv_me_mwb_red;
extern float cv_me_mwb_blue;
extern float cv_me_mwb_green1;
extern float cv_me_mwb_green2;


void cv_me_mwb_init(void);
void cv_me_mwb_periodic(void);

#endif /* CV_AE_MWB_C_ */
