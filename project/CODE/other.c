#include "headfile.h"
#include "stdlib.h"
#include "data.h"
#include "element.h"
#include "eeprom.h"

int str_i = 0;
int str_eq = 0;
int str_position = 0;
char str_num_str[100];

void allInit()
{
	//编码器计数初始化
	ctimer_count_init(CTIM0_P34);
	ctimer_count_init(CTIM3_P04);

	//电机pwm与方向初始化
	pwm_init(PWMA_CH1P_P60, 17000, 0);
	pwm_init(PWMA_CH2P_P62, 17000, 0);

	pwm_init(PWMB_CH1_P74, 50, 0);
	pwm_init(PWMB_CH2_P75, 50, 0);

	gpio_mode(P6_6, GPO_PP);
	gpio_mode(P6_4, GPO_PP);

	gpio_mode(P7_7, GPO_PP);
	P77 = 0;

	imu963ra_init(); //六轴陀螺仪初始化

	adc_init(ADC_P00, ADC_SYSclk_DIV_2);
	adc_init(ADC_P01, ADC_SYSclk_DIV_2);
	adc_init(ADC_P05, ADC_SYSclk_DIV_2);
	adc_init(ADC_P06, ADC_SYSclk_DIV_2);
	adc_init(ADC_P02, ADC_SYSclk_DIV_2);
	adc_init(ADC_P10, ADC_SYSclk_DIV_2);
	adc_init(ADC_P11, ADC_SYSclk_DIV_2);
	adc_init(ADC_P15, ADC_SYSclk_DIV_2);
	adc_init(ADC_P16, ADC_SYSclk_DIV_2);
	adc_init(ADC_P14, ADC_SYSclk_DIV_2);
	imu963ra_init(); //陀螺仪初始化
					 // while (dl1a_init())
					 // {
					 // 	delay_ms(500);
					 // 	printf("VL53L0X init try again.\r\n");
					 // }
	ips200_init();
	ips200_clear(IPS200_BGCOLOR);

	iap_init();
	my_iap_init();
}

//无线调参的参数在这里设置，加一个switch即可加一个调参参数(浮点数用%f,整数用%.0f)
void any_change(int16 id, char cp[])
{
	switch (id)
	{
	case 0:
		start_flag = atoi(cp);
		break;
	case 1:
		test1 = atoi(cp);
		break;
	case 2:
		P_LR = atof(cp);
		break;
	case 3:
		I_LR = atof(cp);
		break; //浮点数用atof
	case 4:
		wSet = atoi(cp);
		break; //整数数用atoi
	case 5:
		test1 = atoi(cp);
		break; //整数数用atoi
	case 6:
		P_S = atof(cp);
		break; //浮点数用atof
	case 7:
		D_S = atof(cp);
		break; //浮点数用atof
			   // case后面的是序号
	case 8:
		sp_set = atoi(cp);
		break;
	case 9:
		into_flag = atoi(cp);
		break; //浮点数用atof
	case 10:
		P_W = atoi(cp);
		break; //浮点数用atof
	case 11:
		D_W = atoi(cp);
		break; //浮点数用atof
	case 12:
		aimed_angle = atoi(cp);
		break; //浮点数用atof
	case 13:
		dis1 = atoi(cp);
		break; //浮点数用atof
	case 14:
		dis2 = atoi(cp);
		break; //浮点数用atof
	case 15:
		G_S_1 = atof(cp);
		break; //浮点数用atof
	case 16:
		fanL = atoi(cp);
		break; //浮点数用atof
	case 17:
		fanR = atoi(cp);
		break; //浮点数用atof
	case 18:
		aimed_into_speed = atoi(cp);
		break;
	case 19:
		into_R = atof(cp);
		break;
	case 20:
		sp_set_max = atoi(cp);
		break;
	case 21:
		P_S_MAX = atof(cp);
		break;
	case 22:
		ku_distance = atoi(cp);
		break;
	case 23:
		ring_angle = atoi(cp);
		break;
	case 24:
		G_S_zhi = atof(cp);
		break;
	case 25:
		D_S_z = atof(cp);
		break;
	}
}

//无线调参字符串解析函数
void rx_analysis()
{
	str_i = 0;
	str_eq = 0;
	str_position = 0;
	//字符串解析函数

	//解析变量名
	while (uart_str[str_position] != '=' && str_position < 100)
	{
		str_position++;
	}

	for (str_i = 0; str_i < str_position; str_i++)
	{
		str_item[str_i] = uart_str[str_i];
	}
	str_item[str_i] = '\0';
	str_eq = str_position;
	// printf("%d,%d\n",str_position,str_eq);
	//解析值
	while (uart_str[str_position] != '#' && str_position < 100)
	{
		str_position++;
	}
	// printf("%d,%d\n",str_position,str_eq);
	for (str_i = 0; str_i < str_position - 2; str_i++)
	{
		str_num_str[str_i] = uart_str[str_i + str_eq + 1];
	}
	str_num_str[str_i] = '\0';
	str_num = atoi(str_num_str);

	any_change(atoi(str_item), str_num_str);
	// printf("%s,%d\n",str_item,atoi(str_num_str)+1);
}

//将角度转换为陀螺仪输出值 例如90度即为angle_change(90)
float angle_change(float ang)
{
	return ang * 28.05;
}

void my_iap_init()
{
	P_S_MAX = iap_read_float(8, 0x00);
	D_S_z = iap_read_float(8, 0x10);
	D_S = iap_read_float(8, 0x20);
	G_S_1 = iap_read_float(8, 0x30);
	G_S_zhi = iap_read_float(8, 0x40);
	P_LR = iap_read_float(8, 0x50);
	I_LR = iap_read_float(8, 0x60);
	P_S = iap_read_float(8, 0x70);
	dis1 = iap_read_float(8, 0x80);
	dis2 = iap_read_float(8, 0x90);
	// adc_sum[0] = iap_read_float(8, 0xA0);
	// adc_sum[1] = iap_read_float(8, 0xB0);
	// adc_sum[2] = iap_read_float(8, 0xC0);
	// adc_sum[3] = iap_read_float(8, 0xD0);
	// adc_sum[4] = iap_read_float(8, 0xE0);
	// adc_sum[5] = iap_read_float(8, 0xF0);
	// adc_sum[6] = iap_read_float(8, 0x100);
	// adc_sum[7] = iap_read_float(8, 0x110);
	// adc_sum[8] = iap_read_float(8, 0x120);
	// adc_sum[9] = iap_read_float(8, 0x130);
	ring_set_k[0] = iap_read_float(8, 0x140);
	ring_set_k[1] = iap_read_float(8, 0x150);
	ring_set_k[2] = iap_read_float(8, 0x160);
	ring_set_k[3] = iap_read_float(8, 0x170);
	ring_set_k[4] = iap_read_float(8, 0x180);
}
