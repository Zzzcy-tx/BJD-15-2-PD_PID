#include "data.h"
#include "math.h"

extern float accel_x;     // X轴加速度值
extern float accel_y;     // Y轴加速度值
extern float accel_z;     // Z轴加速度值
extern float gyro_x;      // X轴陀螺仪角速度数据
extern float gyro_y;      // Y轴陀螺仪角速度数据
extern float gyro_z;      // Z轴陀螺仪角速度数据
extern float accel_x_rea; // X轴加速度值
extern float accel_y_rea; // Y轴加速度值
extern float accel_z_rea; // Z轴加速度值
extern float gyro_x_rea;  // X轴陀螺仪角速度数据
extern float gyro_y_rea;  // Y轴陀螺仪角速度数据
extern float gyro_z_rea;  // Z轴陀螺仪角速度数据

extern float yaw_raw;      //航线角yaw原始数据
extern float yaw_kalman;   // yaw滤波后数据
extern float pitch_raw;    //俯仰角pitch原始数据
extern float pitch_kalman; // pitch滤波后数据
extern float roll_raw;     //横滚角roll原始数据
extern float roll_kalman;  // roll滤波后数据

double KalmanFilter(const double ResrcData);
void Angle_Cal(void);
void Kalman_Cal_Pitch(float acc, float gyro);
void Kalman_Cal_Roll(float acc, float gyro);
void Kalman_Cal_Yaw(float acc, float gyro);