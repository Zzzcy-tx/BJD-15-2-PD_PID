#ifndef __DATA_H7
#define __DATA_H
#define N 1 //待调节参数数量
#include "headfile.h"
#include "pid.h"
extern uint16 t;
extern float all_angle;
extern int16 PO_or_OBSTACLE[5];
extern int16 ring_set_k[5];
extern uint16 sp_ready;

extern int16 mag;
extern int16 tof_distance; //测量的距离

extern float gyro_add;
extern float gyro1;

extern int32 all_distance;

extern float P_S_MAX;

extern uint16 test;
extern int16 speedL;
extern int16 speedR;
extern float k;
extern int32 adc_data[10];
extern uint16 adc_d[10];
extern float Adc_gui[10];
extern uint16 AD_value[10];
extern int8 finish_flag;
extern uint16 Gui_flag;
extern char str_item[100]; //参数名
extern int str_num;        //参数值
extern char uart_str[100]; //串口字符串
extern int16 test1;
extern float test2;
extern int8 start_flag;

extern float P_LR;
extern float I_LR;
extern uint8 puodao;
extern float P_S;
extern float D_S;
extern float G_S;
extern float G_S_1;

extern float P_W;
extern float D_W;
extern float D_S_z;
extern float G_S_zhi;

extern float gyro;
extern int16 wSet;

extern int16 dis1;
extern int16 dis2;
extern int16 ring_angle;

extern int16 fanL;
extern int16 fanR;

extern float aimed_angle;

extern float gyro; //滤波后陀螺仪z轴角速度

extern int8 irs_count;
extern int16 sp_set;
extern int16 sp_set_max;

extern int16 aimed_into_speed;
extern float into_R;
extern int16 ku_distance;

extern uint8 start_diration;

extern int8 keys;
extern uint8 SINGLE_CLICK;
extern uint8 DOUBLE_CLICK;
extern int16 strm;
extern uint8 R_or_L;

extern uint32 time_count;
extern uint16 mode;

extern uint32 encoder_count;
extern int16 speed3;
#endif