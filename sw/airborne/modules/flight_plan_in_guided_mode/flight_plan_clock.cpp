//
// Created by shuoli on 2016/8/4.
//

#include<std.h>
#include "flight_plan_clock.h"

uint32_t counter_global;  // start when codes are uploaded
uint32_t counter_autopilot_mode; // start when autopilot mode is changed
uint32_t counter_primitive;
uint32_t counter_temp1; // counter for temporarily use
uint32_t counter_temp2; // counter for temporarily use
uint32_t counter_temp3; // counter for temporarily use


uint8_t clock_mask;

float time_global;
float time_autopilot_mode;
float time_primitive;
float time_temp1;
float time_temp2;
float time_temp3;

void flight_plan_clock_init(){
    clock_mask = 0;
    SetBit(clock_mask,1);
    counter_global = 0;
    counter_autopilot_mode = 0;
    uint32_t counter_primitive = 0;
    counter_temp1 = 0;
    counter_temp2 = 0;
    counter_temp3 = 0;
}

void flight_plan_clock_run() {
    if (bit_is_set(clock_mask,1))
    {
        counter_global++;
    }
    else
    counter_global = 0;

    if (bit_is_set(clock_mask,2))
    counter_autopilot_mode++;
    else
    counter_autopilot_mode = 0;

    if (bit_is_set(clock_mask,3))
        counter_primitive++;
    else
        counter_primitive = 0;

    if (bit_is_set(clock_mask,4))
        counter_temp1++;
    else
        counter_temp1 = 0;

    if (bit_is_set(clock_mask,5))
        counter_temp2++;
    else
        counter_temp2 = 0;

    if (bit_is_set(clock_mask,6))
        counter_temp3++;
    else
        counter_temp3 = 0;

    time_global = counter_global/20.0;
    time_autopilot_mode = counter_autopilot_mode/20.0;
    time_primitive = counter_primitive/20.0;
    time_temp1 = counter_temp1/20.0;
    time_temp2 = counter_temp2/20.0;
    time_temp3 = counter_temp3/20.0;
}