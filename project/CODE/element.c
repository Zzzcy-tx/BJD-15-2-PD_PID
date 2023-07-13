#include "element.h"
#define HALL_PIN P26
uint8 leave_flag = 0;             //出库阶段0:直行阶段1:右转阶段2:正常行驶
float angle = 0;                  //角速度积分量，即旋转角度，全局变量
uint8 angle_integral_flag = 0;    //角速度积分标志位，1为启用
int16 distance = 0;               //路程积分，1为启用，全局变量
uint8 distance_integral_flag = 0; //路程积分标志位
uint8 into_flag = 1;              //入库阶段1:选择2:倒车
uint8 hall_trigged = 0;

// control函数在renew函数中被循环调用
void control()
{
    //陀螺仪积分
    if (angle_integral_flag)
    {
        angle += gyro * 0.01;
    }

    //编码器积分
    if (distance_integral_flag)
    {
        distance += (speedL + speedR) / 2;
    }

    //出库
    if (leave_flag == 0 && sp_set != 0)
    {
        distance_integral_flag = 1; //启动路程积分
        pidL.setpoint = 100;
        pidR.setpoint = 100;
        hall_trigged = 0;
        //直线行驶一段距离
        if (distance >= 3600)
        {
            leave_flag++;
            distance = 0;
            distance_integral_flag = 0; //关闭路程积分
        }
    }
    else if (leave_flag == 1)
    {
        angle_integral_flag = 1; //启动陀螺仪积分
        if (start_diration == 0)
        {
            pidL.setpoint = 0; //转
            pidR.setpoint = 90;
        }
        else
        {
            pidL.setpoint = 90; //转
            pidR.setpoint = 0;
        }

        hall_trigged = 0;
        if (fabs(angle) >= 700)
        {
            angle_integral_flag = 0; //关闭陀螺仪积分
            angle = 0;               //角度复位
            leave_flag++;
            hall_trigged = 0;
        }
    }
    else if (leave_flag == 2)
    {
        distance_integral_flag = 1; //启动路程积分
        hall_trigged = 0;           //防止直接进入入库模式
        if (distance >= 15000)
        {
            leave_flag++;
            distance_integral_flag = 0; //关闭路程积分
            distance = 0;
        }
    }

    if (HALL_PIN == 0)
    {
        hall_trigged = 1;
    }

    //入库
    if (into_flag == 1 && hall_trigged == 1)
    {
        distance_integral_flag = 1;
        if (distance >= ku_distance)
        {
            sp_set = 0;
            if ((speedL + speedR) / 2 <= 7)
            {
                into_flag++;
                distance_integral_flag = 0;
                distance = 0;
            }
        }
    }
    else if (into_flag == 2) //旋转90°
    {
        angle_integral_flag = 1;
        if (start_diration == 0)
        {
            setSpeedL(-(int16)((2 * into_R - 0.18) / (2 * into_R + 0.18) * (float)aimed_into_speed));
            setSpeedR(-aimed_into_speed);
        }
        else
        {
            setSpeedL(-aimed_into_speed);
            setSpeedR(-(int16)((2 * into_R - 0.18) / (2 * into_R + 0.18) * (float)aimed_into_speed));
        }

        // setPidLR(27.4, 12, 0, 9000);
        // setPidW(3.68, 0, 9.86);
        // angle_integral_flag = 1;      //启动陀螺仪积分
        // pidW.setpoint = -2500;        //角度环目标值(即90度)
        // PIDRegulation1(&pidW, angle); //角度环计算
        // // setSpeedL(-pidW.result / 100 * 3 / 30); //设置目标速度
        // setSpeedL(0);
        // setSpeedR(-pidW.result / 100); //设置目标速度
        if (fabs(angle) >= 2100) //积分量达到目标(90度不到一点)
        {
            into_flag++;
            // sp_set = 0;
        }
    }
    else if (into_flag == 3) //直线倒车
    {
        // setPidW(6, 0, 11);                  //角度环参数，距离太短不太好调
        // pidW.setpoint = -2500;              //角度环目标值
        // PIDRegulation1(&pidW, angle);       // PID计算
        // setSpeedR(-pidW.result / 90 - 100); //速度设定
        // setSpeedL(pidW.result / 90 - 100);  //速度设定
        setSpeedLR(-aimed_into_speed);
        distance_integral_flag = 1; //启动路程积分

        if (distance <= -2000) //到达目标距离
        {
            into_flag++;                //入库完成
            distance = 0;               //距离复位
            distance_integral_flag = 0; //关闭路程积分
            angle_integral_flag = 0;    //关闭陀螺仪积分
            angle = 0;                  //角度复位
            sp_set = 0;                 //停车
        }
    }
}
