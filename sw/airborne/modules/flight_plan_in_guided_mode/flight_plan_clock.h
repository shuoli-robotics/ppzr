//
// Created by shuoli on 2016/8/4.
//

#ifndef PPRZ_ON_DESKTOP_FLIGHT_PLAN_CLOCK_H
#define PPRZ_ON_DESKTOP_FLIGHT_PLAN_CLOCK_H

#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"

extern bool clock_mask[5];

extern uint32_t counter_autopilot_mode; // start when autopilot mode is changed
extern uint32_t counter_primitive;
extern float time_global;
extern float time_autopilot_mode;
extern float time_primitive;
extern float time_temp1;
extern float time_temp2;
extern float time_temp3;
extern uint32_t counter_temp1;
extern uint32_t counter_temp2;
extern float time_gate_detected;
extern uint32_t counter_gate_detected;

extern void flight_plan_clock_init(void);
extern void flight_plan_clock_run(void);


#endif //PPRZ_ON_DESKTOP_FLIGHT_PLAN_CLOCK_H
