#include "ringjudge.h"
#include "math.h"

uint8 Ring_forecast = 0;
int8 annulus_flag = 0;
uint8 last_annulus_flag = 0;
uint8 annulus_flag_k_index = 0;

uint8 Ring_in = 0;
uint8 Ring_running = 0;
uint8 Ring_out = 0;
uint8 Left_Annulus = 0;
uint8 Ring_finish = 0;
uint8 first_flag = 0;
uint8 straight_flag = 0;
uint16 gyro_x_count = 0;

int8 many_ring = 0;

uint8 angle_integral_flag1 = 0;
uint8 distance_integral_flag1 = 0; //路程积分标志位
uint8 distance_integral_flag2 = 0; //路程积分标志位

uint8 if_first_dis1 = 0;
int16 k_add = 0; // k值累加

float angle1 = 0;
int16 distance1 = 0;
int16 distance2 = 0;

uint16 tof_count = 0;	  //记录十次中采集到TOF读数小于320的次数
uint8 tof_count_flag = 0; //检测到障碍物的次数
uint8 dodge_flag = 0;	  //避障标志位

void Ring_control(void)
{
	last_annulus_flag = annulus_flag;

	//编码器积分
	if (distance_integral_flag1)
	{
		distance1 += (abs(speedL) + abs(speedR)) / 2;
	}

	if (angle_integral_flag1)
	{
		angle1 += gyro * 0.01;
	}

	if (annulus_flag > 0)
	{
		P77 = 1;
		setSpeedLR(sp_set / 1.8);
	}
	else
	{
		P77 = 0;
	}

	Ring_forecast = 1;

	//----------圆环入----------//
	if (adc_data[7] > 600 && annulus_flag == 0 && adc_data[1] < 350 && adc_data[3] < 350)
	{
		Ring_in = 1;	  //圆环进入标志位
		annulus_flag = 1; //打开标志位1
		distance1 = 0;	  //距离积分
		distance_integral_flag1 = 1;
		k_add = 0;
		angle_integral_flag1 = 1; //启动陀螺仪积分
	}

	if (annulus_flag == 1 && annulus_flag != 0)
	{
		if (distance1 < dis1 && annulus_flag != 0)
		{
			if (adc_data[1] < adc_data[3])
			{
				k_add += 1;
			}
			else
			{
				k_add -= 1;
			}
		}

		if (distance1 > dis1 && distance1 < dis2 && annulus_flag != 0 /*&&Annulus_Distance>500*/) //控制固定打角的时间50---100   非常重要！！！！！！！！！！！！
		{
			if (first_flag == 0) //判断环向
			{
				if (adc_data[1] < adc_data[3])
				{
					Left_Annulus = 1;
				}
				first_flag = 1;
			}

			//如果可以识别到圆环，但是进不了的话修改下面这里打角幅度或者修改上面的时间
			if (Left_Annulus == 1) //左环入
			{
				// adc_data[0] = adc_data[3]+80;
				// k=-40;
				// if (adc_data[2] > 500)
				// {
				k = -ring_set_k[annulus_flag_k_index];
				// }

				// pidR.setpoint *= 1.8;
				//	pwm_duty(PWB_CH1_P74,angle_max);  //强行向左打角（右电机加速）
			}
			else if (!Left_Annulus) //右环入
			{
				// if (adc_data[2] > 500)
				// {
				k = ring_set_k[annulus_flag_k_index];
				// }
				// pidS.result = 2000;
				// pidL.setpoint *= 1.8;
				//  adc_data[3] = adc_data[0]+80;
				//	pwm_duty(PWMB_CH1_P74,640); //强行向右打角（左电机加速）
			}
		}
		else if (distance1 >= dis2 && annulus_flag != 0) //拐进去后正常循迹
		{
			if_first_dis1 = 0;
			annulus_flag = 2;
			distance1 = 0;
			distance_integral_flag1 = 1;
			k_add = 0;
		}
	}
	// else if (annulus_flag == 2)
	// {
	// 	if (distance1 > 5000)
	// 	{
	// 		annulus_flag = many_ring;
	// 		distance1 = 0;
	// 		distance_integral_flag1 = 0; //停止路程积分
	// 	}
	// }
	else if (annulus_flag == 2) //在环岛内
	{
		if (abs(angle1) > 9500)
		{
			distance_integral_flag1 = 1; //开启路程积分
			annulus_flag = -1;
			// annulus_flag = 0; //出环
			annulus_flag_k_index++;
		}
	}
	// else if (annulus_flag == 3) //打角出环
	// {
	// 	if (distance1 < 2600 /*&&Annulus_Distance>500*/) //控制固定打角的时间50---100   非常重要！！！！！！！！！！！！
	// 	{
	// 		if (Left_Annulus == 1) //左环出
	// 		{
	// 			pidS.result = 1200;
	// 		}
	// 		else if (!Left_Annulus) //右环出
	// 		{
	// 			pidS.result = -1200;
	// 		}
	// 	}
	// 	else if (/*distance > 2600*/ 1) //环岛结束
	// 	{
	// 		annulus_flag = 4; //转向结束
	// 		distance1 = 0;
	// 		distance_integral_flag1 = 1; //继续路程积分防止直接进入环岛
	// 		k_add = 0;
	// 	}
	// }
	// else if (annulus_flag == 4)
	// {
	// 	if (distance1 > 10000)
	// 	{
	// 		all_reset();
	// 	}
	// }

	//坡道防误判为环岛
	if (abs(imu963ra_gyro_x) > 1000)
	{
		gyro_x_count++;
	}
	else
	{
		gyro_x_count = 0;
	}
	if (gyro_x_count >= 3)
	{
		all_reset();
	}
}

void obstacle_control()
{
	//***********************避障*****************************
	if (dodge_flag == -1 && leave_flag >= 3)
	{
		//检测十次  tof
		if (tof_distance <= 0)
		{
			tof_count++;
		}
		tof_count_flag++; //物体检测次数++
		if (tof_count_flag == 10)
		{
			if (tof_count >= 8)
			{
				dodge_flag++;
				distance1 = 0;
				angle1 = 0;
			}
			tof_count_flag = 0;
		}
	}
	else if (dodge_flag == 1) //要转角
	{
		//左打角
		distance_integral_flag1 = 1;
		angle_integral_flag1 = 1;
		pidS.result = 500;
		if (angle1 > 25)
		{
			pidS.result = 0;
			// distance_integral_flag1 = 0;
			// angle_integral_flag1 = 0;	//关闭角度积分
			if (distance1 < 6500)
			{
				//不做处理
			}
			else
			{
				distance_integral_flag1 = 0;
				distance1 = 0;
				dodge_flag++;
			}
		}
	}
	else if (dodge_flag == 2)
	{
		//打回正向
		pidS.result = -600;
		if (straight_flag == 0 && angle1 < 5)
		{
			distance_integral_flag1 = 1;
			if (distance1 < 9000)
			{
				pidS.result = 0;
			}
			else
			{
				straight_flag = 1;
				distance_integral_flag1 = 0;
				distance1 = 0;
				dodge_flag++;
			}
		}
	}
	else if (dodge_flag == 3)
	{
		//右打角
		pidS.result = -600;
		if (angle1 < -25)
		{
			dodge_flag++;
			//			distance1 = 0;
			angle1 = 0;
			//			distance_integral_flag1 = 0;
			angle_integral_flag1 = 0;
		}
	}
}

void all_reset()
{
	angle = 0;
	distance1 = 0;
	Ring_forecast = 0;
	annulus_flag = 0;
	Ring_in = 0;
	Ring_running = 0;
	Ring_out = 0;
	Left_Annulus = 0;
	Ring_finish = 0;
	first_flag = 0;
	angle_integral_flag1 = 0;
	distance_integral_flag1 = 0; //路程积分标志位
	k_add = 0;					 // k值累加
}