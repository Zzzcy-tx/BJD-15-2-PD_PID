#ifndef __RINGJUDGE_H
#define __RINGJUDGE_H

#include "headfile.h"
#include "data.h"
#include "element.h"
#include "pid.h"

extern uint8 Ring_forecast;
extern int8 annulus_flag;
extern uint8 Ring_in;
extern uint8 Ring_running;
extern uint8 Ring_out;
extern uint8 Left_Annulus;
extern uint8 Ring_finish;
extern uint8 first_flag;
extern uint8 angle_integral_flag1;
extern uint8 straight_flag;
extern int16 k_add;
extern int8 many_ring;
extern uint8 annulus_flag_k_index;

extern uint16 gyro_x_count;
extern uint8 dodge_flag; //±‹’œ±Í÷æª∑

void Ring_control(void);
void all_reset(void);
void obstacle_control(void);
#endif