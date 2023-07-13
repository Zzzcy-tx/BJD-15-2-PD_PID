#include "filter.h"

uint16 Min_AD[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; //最小值设为1
uint16 Max_Ad[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; //电感最大值
int32 adc_sum[10] = {0};
int16 Range = 1000; //放大倍数
uint8 time_count1 = 0;

void read_adc()
{
	static float ad_sum[10];
	//获取电感值
	adc_data[0] = MovingAverageFilter0(adc_once(ADC_P00, ADC_10BIT));
	adc_data[1] = MovingAverageFilter1(adc_once(ADC_P01, ADC_10BIT));
	adc_data[2] = MovingAverageFilter2(adc_once(ADC_P05, ADC_10BIT));
	adc_data[3] = MovingAverageFilter3(adc_once(ADC_P06, ADC_10BIT));
	adc_data[4] = MovingAverageFilter4(adc_once(ADC_P02, ADC_10BIT));
	adc_data[5] = MovingAverageFilter5(adc_once(ADC_P10, ADC_10BIT));
	adc_data[6] = MovingAverageFilter6(adc_once(ADC_P11, ADC_10BIT));
	adc_data[7] = MovingAverageFilter7(adc_once(ADC_P15, ADC_10BIT));
	adc_data[8] = MovingAverageFilter8(adc_once(ADC_P16, ADC_10BIT));
	adc_data[9] = MovingAverageFilter9(adc_once(ADC_P14, ADC_10BIT));

	//进行归一化
	// for (i = 0; i < 10; i++)
	// {
	// 	Adc_gui[i] = Range * ((float)AD_value[i] / (Max_Ad[i] - Min_AD[i]));
	// }
	// adc_data[0] = MovingAverageFilter_0(adc_data[0] * 600 / adc_sum[0]);
	// adc_data[1] = MovingAverageFilter_1(AD_value[1] * 800 / adc_sum[1]);
	// adc_data[2] = MovingAverageFilter_2(AD_value[2] * 600 / adc_sum[2]);
	// adc_data[3] = MovingAverageFilter_3(AD_value[3] * 800 / adc_sum[3]);
	// adc_data[4] = MovingAverageFilter_4(AD_value[4] * 600 / adc_sum[4]);
	// adc_data[5] = MovingAverageFilter_5(AD_value[5] * 600 / adc_sum[5]);
	// adc_data[6] = MovingAverageFilter_6(AD_value[6] * 800 / adc_sum[6]);
	// adc_data[7] = MovingAverageFilter_7(AD_value[7] * 600 / adc_sum[7]);
	// adc_data[8] = MovingAverageFilter_8(AD_value[8] * 800 / adc_sum[8]);
	// adc_data[9] = MovingAverageFilter_9(AD_value[9] * 600 / adc_sum[9]);

	/*	//数据处理量太大
		//对归一化的数据再进行冒泡排序
		for(i = 0; i < 10; i++)
		{
			for(j = 0; j < 10; j++)
			{
				for(k = 0; k < 9 - j; k++)
				{
					if(AD_value[i][k] > AD_value[i][k + 1])
					{
						temp = AD_value[i][k + 1];
						AD_value[i][k + 1] = AD_value[i][k];
						AD_value[i][k] = temp;
					}
				}
			}
		}
		//取中间的值获取
		for (i = 0; i < 10; i++)
		{
			ad_sum[i] = (AD_value[i][4] + AD_value[i][5] + AD_value[i][6] + AD_value[i][7])/4;
			Adc_gui[i] = (uint16)ad_sum[i];
		//	if (GUI_AD[i] >= Range)
		//        GUI_AD[i] = Range;
		//   if (GUI_AD[i] <= Min_AD[i])
		//        GUI_AD[i] = Min_AD[i];
		}
	*/
}

void adc_select()
{
	AD_value[0] = MovingAverageFilter0(adc_once(ADC_P00, ADC_10BIT));
	AD_value[1] = MovingAverageFilter1(adc_once(ADC_P01, ADC_10BIT));
	AD_value[2] = MovingAverageFilter2(adc_once(ADC_P05, ADC_10BIT));
	AD_value[3] = MovingAverageFilter3(adc_once(ADC_P06, ADC_10BIT));
	AD_value[4] = MovingAverageFilter4(adc_once(ADC_P02, ADC_10BIT));
	AD_value[5] = MovingAverageFilter5(adc_once(ADC_P10, ADC_10BIT));
	AD_value[6] = MovingAverageFilter6(adc_once(ADC_P11, ADC_10BIT));
	AD_value[7] = MovingAverageFilter7(adc_once(ADC_P15, ADC_10BIT));
	AD_value[8] = MovingAverageFilter8(adc_once(ADC_P16, ADC_10BIT));
	AD_value[9] = MovingAverageFilter9(adc_once(ADC_P14, ADC_10BIT));
	if (Gui_flag == 0)
		read_adc();
	if (Gui_flag == 1)
	{
		//求十次取值的平均值，作为基准值
		if (time_count1 < 10)
		{
			//横电感
			time_count1++;
			adc_sum[0] += adc_once(ADC_P00, ADC_10BIT);
			adc_sum[2] += adc_once(ADC_P05, ADC_10BIT);
			adc_sum[4] += adc_once(ADC_P02, ADC_10BIT);

			adc_sum[5] += adc_once(ADC_P10, ADC_10BIT);
			adc_sum[7] += adc_once(ADC_P15, ADC_10BIT);
			adc_sum[9] += adc_once(ADC_P14, ADC_10BIT);
		}
		else if (time_count1 == 10)
		{
			adc_sum[0] /= 10;
			adc_sum[2] /= 10;
			adc_sum[4] /= 10;

			adc_sum[5] /= 10;
			adc_sum[7] /= 10;
			adc_sum[9] /= 10;
			time_count1++;
		}
	}
	else if (Gui_flag == 2)
	{
		if (time_count1 < 21)
		{
			time_count1++;
			adc_sum[1] += adc_once(ADC_P01, ADC_10BIT);
			adc_sum[3] += adc_once(ADC_P06, ADC_10BIT);
		}
		else if (time_count1 == 21)
		{
			adc_sum[1] /= 10;
			adc_sum[3] /= 10;
			time_count1++;
		}
	}
	else if (Gui_flag == 3)
	{
		if (time_count1 < 32)
		{
			time_count1++;
			adc_sum[6] += adc_once(ADC_P11, ADC_10BIT);
			adc_sum[8] += adc_once(ADC_P16, ADC_10BIT);
		}
		else if (time_count1 == 32)
		{
			adc_sum[6] /= 10;
			adc_sum[8] /= 10;
			time_count1++;
		}
	}
	else if (Gui_flag == 4)
	{
		// extern_iap_write_float((float)adc_sum[0], 5, 1, 0xA0);
		// extern_iap_write_float((float)adc_sum[1], 5, 1, 0xB0);
		// extern_iap_write_float((float)adc_sum[2], 5, 1, 0xC0);
		// extern_iap_write_float((float)adc_sum[3], 5, 1, 0xD0);
		// extern_iap_write_float((float)adc_sum[4], 5, 1, 0xE0);
		// extern_iap_write_float((float)adc_sum[5], 5, 1, 0xF0);
		// extern_iap_write_float((float)adc_sum[6], 5, 1, 0x100);
		// extern_iap_write_float((float)adc_sum[7], 5, 1, 0x110);
		// extern_iap_write_float((float)adc_sum[8], 5, 1, 0x120);
		// extern_iap_write_float((float)adc_sum[9], 5, 1, 0x130);
		Gui_flag = 0;
		time_count1 = 0;
	}
}

//平均滑动滤波算法
float MovingAverageFilter0(float x0)
{
	static int8 MAF_index0 = -1;
	static float buffer0[MAF_LENGTH];
	static float yk_0 = 0;
	float y0 = 0;
	uint8 i = 0;
	if (MAF_index0 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer0[i] = x0;
		}
		yk_0 = x0;
		MAF_index0 = 0;
		y0 = yk_0;
	}
	else
	{
		y0 = yk_0 + (x0 - buffer0[MAF_index0]) / MAF_LENGTH;
		buffer0[MAF_index0] = x0;
		MAF_index0++;
		if (MAF_index0 >= MAF_LENGTH)
		{
			MAF_index0 = 0;
		}
		yk_0 = y0;
	}
	return y0;
}
float MovingAverageFilter1(float x1)
{
	static int8 MAF_index1 = -1;
	static float buffer1[MAF_LENGTH];
	static float yk_1 = 0;
	float y1 = 0;
	uint8 i = 0;
	if (MAF_index1 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer1[i] = x1;
		}
		yk_1 = x1;
		MAF_index1 = 0;
		y1 = yk_1;
	}
	else
	{
		y1 = yk_1 + (x1 - buffer1[MAF_index1]) / MAF_LENGTH;
		buffer1[MAF_index1] = x1;
		MAF_index1++;
		if (MAF_index1 >= MAF_LENGTH)
		{
			MAF_index1 = 0;
		}
		yk_1 = y1;
	}
	return y1;
}

float MovingAverageFilter2(float x2)
{
	static int8 MAF_index2 = -1;
	static float buffer2[MAF_LENGTH];
	static float yk_2 = 0;
	float y2 = 0;
	uint8 i = 0;
	if (MAF_index2 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer2[i] = x2;
		}
		yk_2 = x2;
		MAF_index2 = 0;
		y2 = yk_2;
	}
	else
	{
		y2 = yk_2 + (x2 - buffer2[MAF_index2]) / MAF_LENGTH;
		buffer2[MAF_index2] = x2;
		MAF_index2++;
		if (MAF_index2 >= MAF_LENGTH)
		{
			MAF_index2 = 0;
		}
		yk_2 = y2;
	}
	return y2;
}

float MovingAverageFilter3(float x3)
{
	static int8 MAF_index3 = -1;
	static float buffer3[MAF_LENGTH];
	static float yk_3 = 0;
	float y3 = 0;
	uint8 i = 0;
	if (MAF_index3 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer3[i] = x3;
		}
		yk_3 = x3;
		MAF_index3 = 0;
		y3 = yk_3;
	}
	else
	{
		y3 = yk_3 + (x3 - buffer3[MAF_index3]) / MAF_LENGTH;
		buffer3[MAF_index3] = x3;
		MAF_index3++;
		if (MAF_index3 >= MAF_LENGTH)
		{
			MAF_index3 = 0;
		}
		yk_3 = y3;
	}
	return y3;
}

float MovingAverageFilter4(float x4)
{
	static int8 MAF_index4 = -1;
	static float buffer4[MAF_LENGTH];
	static float yk_4 = 0;
	float y4 = 0;
	uint8 i = 0;
	if (MAF_index4 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer4[i] = x4;
		}
		yk_4 = x4;
		MAF_index4 = 0;
		y4 = yk_4;
	}
	else
	{
		y4 = yk_4 + (x4 - buffer4[MAF_index4]) / MAF_LENGTH;
		buffer4[MAF_index4] = x4;
		MAF_index4++;
		if (MAF_index4 >= MAF_LENGTH)
		{
			MAF_index4 = 0;
		}
		yk_4 = y4;
	}
	return y4;
}

float MovingAverageFilter5(float x5)
{
	static int8 MAF_index5 = -1;
	static float buffer5[MAF_LENGTH];
	static float yk_5 = 0;
	float y5 = 0;
	uint8 i = 0;
	if (MAF_index5 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer5[i] = x5;
		}
		yk_5 = x5;
		MAF_index5 = 0;
		y5 = yk_5;
	}
	else
	{
		y5 = yk_5 + (x5 - buffer5[MAF_index5]) / MAF_LENGTH;
		buffer5[MAF_index5] = x5;
		MAF_index5++;
		if (MAF_index5 >= MAF_LENGTH)
		{
			MAF_index5 = 0;
		}
		yk_5 = y5;
	}
	return y5;
}

float MovingAverageFilter6(float x6)
{
	static int8 MAF_index6 = -1;
	static float buffer6[MAF_LENGTH];
	static float yk_6 = 0;
	float y6 = 0;
	uint8 i = 0;
	if (MAF_index6 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer6[i] = x6;
		}
		yk_6 = x6;
		MAF_index6 = 0;
		y6 = yk_6;
	}
	else
	{
		y6 = yk_6 + (x6 - buffer6[MAF_index6]) / MAF_LENGTH;
		buffer6[MAF_index6] = x6;
		MAF_index6++;
		if (MAF_index6 >= MAF_LENGTH)
		{
			MAF_index6 = 0;
		}
		yk_6 = y6;
	}
	return y6;
}

float MovingAverageFilter7(float x7)
{
	static int8 MAF_index7 = -1;
	static float buffer7[MAF_LENGTH];
	static float yk_7 = 0;
	float y7 = 0;
	uint8 i = 0;
	if (MAF_index7 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer7[i] = x7;
		}
		yk_7 = x7;
		MAF_index7 = 0;
		y7 = yk_7;
	}
	else
	{
		y7 = yk_7 + (x7 - buffer7[MAF_index7]) / MAF_LENGTH;
		buffer7[MAF_index7] = x7;
		MAF_index7++;
		if (MAF_index7 >= MAF_LENGTH)
		{
			MAF_index7 = 0;
		}
		yk_7 = y7;
	}
	return y7;
}
float MovingAverageFilter8(float x8)
{
	static int8 MAF_index8 = -1;
	static float buffer8[MAF_LENGTH];
	static float yk_8 = 0;
	float y8 = 0;
	uint8 i = 0;
	if (MAF_index8 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer8[i] = x8;
		}
		yk_8 = x8;
		MAF_index8 = 0;
		y8 = yk_8;
	}
	else
	{
		y8 = yk_8 + (x8 - buffer8[MAF_index8]) / MAF_LENGTH;
		buffer8[MAF_index8] = x8;
		MAF_index8++;
		if (MAF_index8 >= MAF_LENGTH)
		{
			MAF_index8 = 0;
		}
		yk_8 = y8;
	}
	return y8;
}
float MovingAverageFilter9(float x9)
{
	static int8 MAF_index9 = -1;
	static float buffer9[MAF_LENGTH];
	static float yk_9 = 0;
	float y9 = 0;
	uint8 i = 0;
	if (MAF_index9 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer9[i] = x9;
		}
		yk_9 = x9;
		MAF_index9 = 0;
		y9 = yk_9;
	}
	else
	{
		y9 = yk_9 + (x9 - buffer9[MAF_index9]) / MAF_LENGTH;
		buffer9[MAF_index9] = x9;
		MAF_index9++;
		if (MAF_index9 >= MAF_LENGTH)
		{
			MAF_index9 = 0;
		}
		yk_9 = y9;
	}
	return y9;
}

float Filter10(float x10)
{
	static int8 MAF_index10 = -1;
	static float buffer10[MAF_LENGTH];
	static float yk_10 = 0;
	float y10 = 0;
	uint8 i = 0;
	if (MAF_index10 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer10[i] = x10;
		}
		yk_10 = x10;
		MAF_index10 = 0;
		y10 = yk_10;
	}
	else
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer10[i] = buffer10[i + 1]; // 所有数据左移，低位仍掉
			y10 += buffer10[i];
		}
	}
	return y10;
}

float MovingAverageFilter_0(float x_0)
{
	static int8 MAF_index_0 = -1;
	static float buffer_0[MAF_LENGTH];
	static float yk__0 = 0;
	float y_0 = 0;
	uint8 i = 0;
	if (MAF_index_0 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer_0[i] = x_0;
		}
		yk__0 = x_0;
		MAF_index_0 = 0;
		y_0 = yk__0;
	}
	else
	{
		y_0 = yk__0 + (x_0 - buffer_0[MAF_index_0]) / MAF_LENGTH;
		buffer_0[MAF_index_0] = x_0;
		MAF_index_0++;
		if (MAF_index_0 >= MAF_LENGTH)
		{
			MAF_index_0 = 0;
		}
		yk__0 = y_0;
	}
	return y_0;
}
float MovingAverageFilter_1(float x_1)
{
	static int8 MAF_index_1 = -1;
	static float buffer_1[MAF_LENGTH];
	static float yk__1 = 0;
	float y_1 = 0;
	uint8 i = 0;
	if (MAF_index_1 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer_1[i] = x_1;
		}
		yk__1 = x_1;
		MAF_index_1 = 0;
		y_1 = yk__1;
	}
	else
	{
		y_1 = yk__1 + (x_1 - buffer_1[MAF_index_1]) / MAF_LENGTH;
		buffer_1[MAF_index_1] = x_1;
		MAF_index_1++;
		if (MAF_index_1 >= MAF_LENGTH)
		{
			MAF_index_1 = 0;
		}
		yk__1 = y_1;
	}
	return y_1;
}
float MovingAverageFilter_2(float x_2)
{
	static int8 MAF_index_2 = -1;
	static float buffer_2[MAF_LENGTH];
	static float yk__2 = 0;
	float y_2 = 0;
	uint8 i = 0;
	if (MAF_index_2 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer_2[i] = x_2;
		}
		yk__2 = x_2;
		MAF_index_2 = 0;
		y_2 = yk__2;
	}
	else
	{
		y_2 = yk__2 + (x_2 - buffer_2[MAF_index_2]) / MAF_LENGTH;
		buffer_2[MAF_index_2] = x_2;
		MAF_index_2++;
		if (MAF_index_2 >= MAF_LENGTH)
		{
			MAF_index_2 = 0;
		}
		yk__2 = y_2;
	}
	return y_2;
}

float MovingAverageFilter_3(float x_3)
{
	static int8 MAF_index_3 = -1;
	static float buffer_3[MAF_LENGTH];
	static float yk__3 = 0;
	float y_3 = 0;
	uint8 i = 0;
	if (MAF_index_3 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer_3[i] = x_3;
		}
		yk__3 = x_3;
		MAF_index_3 = 0;
		y_3 = yk__3;
	}
	else
	{
		y_3 = yk__3 + (x_3 - buffer_3[MAF_index_3]) / MAF_LENGTH;
		buffer_3[MAF_index_3] = x_3;
		MAF_index_3++;
		if (MAF_index_3 >= MAF_LENGTH)
		{
			MAF_index_3 = 0;
		}
		yk__3 = y_3;
	}
	return y_3;
}

float MovingAverageFilter_4(float x_4)
{
	static int8 MAF_index_4 = -1;
	static float buffer_4[MAF_LENGTH];
	static float yk__4 = 0;
	float y_4 = 0;
	uint8 i = 0;
	if (MAF_index_4 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer_4[i] = x_4;
		}
		yk__4 = x_4;
		MAF_index_4 = 0;
		y_4 = yk__4;
	}
	else
	{
		y_4 = yk__4 + (x_4 - buffer_4[MAF_index_4]) / MAF_LENGTH;
		buffer_4[MAF_index_4] = x_4;
		MAF_index_4++;
		if (MAF_index_4 >= MAF_LENGTH)
		{
			MAF_index_4 = 0;
		}
		yk__4 = y_4;
	}
	return y_4;
}
float MovingAverageFilter_5(float x_5)
{
	static int8 MAF_index_5 = -1;
	static float buffer_5[MAF_LENGTH];
	static float yk__5 = 0;
	float y_5 = 0;
	uint8 i = 0;
	if (MAF_index_5 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer_5[i] = x_5;
		}
		yk__5 = x_5;
		MAF_index_5 = 0;
		y_5 = yk__5;
	}
	else
	{
		y_5 = yk__5 + (x_5 - buffer_5[MAF_index_5]) / MAF_LENGTH;
		buffer_5[MAF_index_5] = x_5;
		MAF_index_5++;
		if (MAF_index_5 >= MAF_LENGTH)
		{
			MAF_index_5 = 0;
		}
		yk__5 = y_5;
	}
	return y_5;
}
float MovingAverageFilter_6(float x_6)
{
	static int8 MAF_index_6 = -1;
	static float buffer_6[MAF_LENGTH];
	static float yk__6 = 0;
	float y_6 = 0;
	uint8 i = 0;
	if (MAF_index_6 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer_6[i] = x_6;
		}
		yk__6 = x_6;
		MAF_index_6 = 0;
		y_6 = yk__6;
	}
	else
	{
		y_6 = yk__6 + (x_6 - buffer_6[MAF_index_6]) / MAF_LENGTH;
		buffer_6[MAF_index_6] = x_6;
		MAF_index_6++;
		if (MAF_index_6 >= MAF_LENGTH)
		{
			MAF_index_6 = 0;
		}
		yk__6 = y_6;
	}
	return y_6;
}
float MovingAverageFilter_7(float x_7)
{
	static int8 MAF_index_7 = -1;
	static float buffer_7[MAF_LENGTH];
	static float yk__7 = 0;
	float y_7 = 0;
	uint8 i = 0;
	if (MAF_index_7 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer_7[i] = x_7;
		}
		yk__7 = x_7;
		MAF_index_7 = 0;
		y_7 = yk__7;
	}
	else
	{
		y_7 = yk__7 + (x_7 - buffer_7[MAF_index_7]) / MAF_LENGTH;
		buffer_7[MAF_index_7] = x_7;
		MAF_index_7++;
		if (MAF_index_7 >= MAF_LENGTH)
		{
			MAF_index_7 = 0;
		}
		yk__7 = y_7;
	}
	return y_7;
}
float MovingAverageFilter_8(float x_8)
{
	static int8 MAF_index_8 = -1;
	static float buffer_8[MAF_LENGTH];
	static float yk__8 = 0;
	float y_8 = 0;
	uint8 i = 0;
	if (MAF_index_8 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer_8[i] = x_8;
		}
		yk__8 = x_8;
		MAF_index_8 = 0;
		y_8 = yk__8;
	}
	else
	{
		y_8 = yk__8 + (x_8 - buffer_8[MAF_index_8]) / MAF_LENGTH;
		buffer_8[MAF_index_8] = x_8;
		MAF_index_8++;
		if (MAF_index_8 >= MAF_LENGTH)
		{
			MAF_index_8 = 0;
		}
		yk__8 = y_8;
	}
	return y_8;
}
float MovingAverageFilter_9(float x_9)
{
	static int8 MAF_index_9 = -1;
	static float buffer_9[MAF_LENGTH];
	static float yk__9 = 0;
	float y_9 = 0;
	uint8 i = 0;
	if (MAF_index_9 == -1)
	{
		for (i = 0; i < MAF_LENGTH; i++)
		{
			buffer_9[i] = x_9;
		}
		yk__9 = x_9;
		MAF_index_9 = 0;
		y_9 = yk__9;
	}
	else
	{
		y_9 = yk__9 + (x_9 - buffer_9[MAF_index_9]) / MAF_LENGTH;
		buffer_9[MAF_index_9] = x_9;
		MAF_index_9++;
		if (MAF_index_9 >= MAF_LENGTH)
		{
			MAF_index_9 = 0;
		}
		yk__9 = y_9;
	}
	return y_9;
}