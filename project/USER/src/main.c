/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC16F40K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "headfile.h"
#include "data.h"
#include "other.h"
#include "speed.h"
#include "pid.h"
#include "element.h"
#include "kaerman.h"
#include "keyboard.h"
#include "filter.h"
/*
 *关于内核频率的设定，可以查看board.h文件
 *在board_init中,已经将P54引脚设置为复位
 *如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可
 */
int i_count = 10;
#define HALL_PIN P26
extern float k1;
uint32 last_time_count = 0;
void main()
{

  // sys_clk可选值:33177600, 30000000, 27000000. 24000000, 22118400, 20000000, 18432000, 12000000, 11059200, 6000000, 5529600。
  //设置系统频率，此频率需要跟STC-ISP软件中的 <输入用户程序运行时的IRC频率>选项的频率一致。
  //如果频率设置不对，将会导致串口的数据不正常,PWM的工作不正常等等。
  sys_clk = 30000000; //设置系统频率为30MHz

  board_init();
  allInit();
  // ips200_init();
  // ips200_clear(WHITE);
  // delay_ms(1000);
  pit_timer_ms(TIM_4, 5); //启用中断
  exit_init(INT0_P32, FALLING_EDGE);
  setPidLR(20, 8, 0, 16000);
  setSpeedLR(110);
  setPidS(8, 0, 1);

  pidS.setpoint = 0;
  pidW.setpoint = 0;

  while (i_count)
  {
    delay_ms(10);
    i_count--;
    gyro_add += (float)KalmanFilter((double)imu963ra_gyro_z);
  }

  gyro_add /= 10;
  sp_set = 0;
  P77 = 0;
  while (1)
  {

    if (P50 || abs(speedL > 15) || abs(speedR > 15))
    {
      Keystroke_Scan();
      Keystroke_Menu();
    }

    pwm_duty(PWMA_CH3P_P64, 2000);
    if (finish_flag)
    {
      rx_analysis();
      finish_flag = 0;
    } //无线调参解析函数

    // if (last_time_count != time_count && sp_set != 0)
    // {
    //   printf("%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", time_count, adc_data[0], adc_data[1], adc_data[2], adc_data[3], adc_data[4], adc_data[5], adc_data[6], adc_data[7], adc_data[8], adc_data[9]);
    //   printf(",%d,%f,%d,%d,%f,%f,%f,%f,%f,%f", sp_set, k, speedL, speedR, gyro, pidL.setpoint, pidL.proportiongain, pidL.integralgain, pidR.proportiongain, pidR.integralgain);
    //   printf(",%f,%f,%f,%f,%f", pidL.result, pidR.result, pidS.proportiongain, pidS.derivativegain, pidS.result);
    //   printf(",%f,%f,%f,%f,%f,%f,%f\n", G_S * gyro, pidL.preerror, pidL.integral, pidR.preerror, pidR.integral, pidS.preerror, pidS.derivativegain);
    //   last_time_count = time_count;
    // }
    // printf("%d,%d,%f,%f,%f,%d,%f\n", speedL, speedR, pidL.result, pidR.result, k, speed3, gyro);
    setPidLR(P_LR, I_LR, 0, 15000); //设置速度环参数
                                    // setPidS(P_S, 0, D_S);           //设置方向环参数
    // setPidS(P_S, 0, D_S);           //设置方向环参数
    setPidW(P_W, 0, D_W); //设置角度环参数

    if (sp_set <= 2)
    {
      fanL = 0;
    }
    pwm_duty(PWMB_CH1_P74, fanL);

    sp_set_max = sp_set;
    //  printf("%d,%d,%d,%f,%f,%f,%d,%d,%d,%f,%f,%d\n", adc_data[0], adc_data[4], aimed_into_speed, into_R, angle, all_angle, adc_data[2], tof_distance, leave_flag, k, pidS.result, dodge_flag);
    //   printf("%d,%d,%d\n", imu963ra_mag_x, imu963ra_mag_y, imu963ra_mag_z);
    //   printf("%d,%d,%d,%d,%d,%d",imu963ra_acc_x,imu963ra_acc_y,imu963ra_acc_z,imu963ra_gyro_x,imu963ra_gyro_y,imu963ra_gyro_z);
    //  printf("%d,%d,%d,%d,%f,%d,%d,%f,%f,%f,%d,%d,%f,%f\n", adc_data[2], adc_data[0], adc_data[4], dodge_flag, angle1, distance1, sp_set, P_S, D_S, G_S, annulus_flag, k_add, gyro, pidS.result);
    //     printf("%f,%f,%f,%f,%f,%f,%f\n", angle, pidW.result / 90, pidL.setpoint, pidR.setpoint, pidL.result, pidR.result, k, aimed_angle);
    //   printf("%d\n", dl1a_distance_mm);
    //   printf("%f,%f\n",k,pidS.result);
    //  printf("%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,", adc_data[9] - adc_data[2], adc_data[9], adc_data[2], adc_data[0], adc_data[4], pidL.setpoint, pidR.setpoint, pidL.result, pidR.result, pidS.result, k);
    //  printf("%f,%d,%d,%f,%d\n", k1, speedL, speedR, ((float)speedL + (float)speedR) / 2 * 200 / 10100, into_flag);
    //  printf("%d,%d,%d,%d,%d,%d,%f,%f,%f,%f\n",adc_data[0],adc_data[1],adc_data[2],adc_data[3],adc_data[4],sp_set,P_LR,I_LR,P_S,D_S);
    //  printf("%d,%f,%f,%f,%d,%d,%f,%f,%d,%f,%d,%d,%d\n", adc_data[5], pidS.result, pidL.result, pidR.result, speedL, speedR, gyro, all_angle, distance1, k, dl1a_distance_mm, dodge_flag, annulus_flag,(adc_data[]));
    //    printf("%d,%d,%d,%f,%f,%f,%f,%f,%d,%d,%f,%f,%f\n",adc_data[0],adc_data[2],adc_data[3],pidL.proportiongain,pidL.integralgain,pidS.proportiongain,pidS.derivativegain,k,speedL,speedR,pidL.result,pidR.result);
    //    printf("%f\n",pidS.result);
    // printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", adc_sum[0], adc_sum[1], adc_sum[2], adc_sum[3], adc_sum[4], adc_sum[5], adc_sum[6], adc_sum[7], adc_sum[8], adc_sum[9], Gui_flag, t);
    printf("%d,%d,%f\n", adc_once(ADC_P00, ADC_10BIT), adc_sum[0], (float)adc_once(ADC_P00, ADC_10BIT) / adc_sum[0] * 600.0);
    // printf("%d,%d,%d,%d,%d,%d,%d\n",adc_data[0],adc_data[1],adc_d[2],adc_d[3],adc_d[4],Gui_flag,t);
    // printf("%d,%d,%f,%f,%f,%f,%f,%f,%f,%f\n",adc_data[0],Max_Ad[0],Adc_gui[0],Adc_gui[1],Adc_gui[2],Adc_gui[3],Adc_gui[4],Adc_gui[5],Adc_gui[6],Adc_gui[7]);
  }
}
