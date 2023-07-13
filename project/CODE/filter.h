#ifndef __FILTER_H
#define __FILTER_H

#include "headfile.h"
#include "math.h"
#include "data.h"
#include "eeprom.h"

#define MAF_LENGTH 5 // 滑动平均滤波器长度

extern uint16 Max_Ad[10];
extern uint16 Min_AD[10];

extern int32 adc_sum[10];
extern uint8 time_count1;
void read_adc(void);
void adc_select(void);
float MovingAverageFilter0(float x0);
float MovingAverageFilter1(float x1);
float MovingAverageFilter2(float x2);
float MovingAverageFilter3(float x3);
float MovingAverageFilter4(float x4);
float MovingAverageFilter5(float x5);
float MovingAverageFilter6(float x6);
float MovingAverageFilter7(float x7);
float MovingAverageFilter8(float x7);
float MovingAverageFilter9(float x7);
float MovingAverageFilter_0(float x_0);
float MovingAverageFilter_1(float x_1);
float MovingAverageFilter_2(float x_2);
float MovingAverageFilter_3(float x_3);
float MovingAverageFilter_4(float x_4);
float MovingAverageFilter_5(float x_5);
float MovingAverageFilter_6(float x_6);
float MovingAverageFilter_7(float x_7);
float MovingAverageFilter_8(float x_8);
float MovingAverageFilter_9(float x_9);
float Filter10(float x10);

#endif