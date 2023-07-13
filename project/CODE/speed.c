#include "speed.h"

//���õ��pwm
//���õ��pwm
void setMotorL(int16 motorSetL)
{
  if (sp_set < 10)
  {
    motorSetL = 0;
  }
  pwm_duty(PWMA_CH1P_P60, abs(motorSetL) * 4 / 7);
  if (motorSetL > 0)
    P64 = 1;
  else
    P64 = 0;
}
void setMotorR(int16 motorSetR)
{
  if (sp_set < 10)
  {
    motorSetR = 0;
  }
  pwm_duty(PWMA_CH2P_P62, abs(motorSetR) * 4 / 7);
  if (motorSetR < 0)
    P66 = 0;
  else
    P66 = 1;
}

//���ټ���
float Diff_Factor = 0.006;            //����ϵ��
int16 Differential(float Diff_Factor) //������ԭ�����ݶ�����в��ٿ��ƣ�����������������ʵ��
{
  float Diff;
  Diff = Diff_Factor * (pidL.setpoint + pidR.setpoint) / 2 * pidS.result; // diff_factor�ǲ���ϵ��
  return (int16)Diff;
}

//�������ֵ��ٶ�Ŀ��ֵ
void diffSpeedSet()
{
  int16 Diff_Speed = Differential(Diff_Factor); //ͨ�������������ֵ
                                                // if (Diff_Speed > 2000)
                                                //	Diff_Speed =1000;
                                                // pidL.setpoint * Diff_Factor; //�����޷�

  if (k < 0)
  {
    setSpeedR(pidR.setpoint + Diff_Speed);
    setSpeedL(pidL.setpoint - Diff_Speed);
  }
  else
  {
    setSpeedR(pidR.setpoint + Diff_Speed);
    setSpeedL(pidL.setpoint - Diff_Speed);
  }

  // setSpeedLR(pidL.setpoint);
  // if (pidS.result < 0)
  //  setSpeedR(pidR.setpoint - abs(Diff_Speed));
  // else
  //  setSpeedL(pidR.setpoint - abs(Diff_Speed));
}

// //����ǶȻ����
// void diffSpeedSet1()
// {
//   setSpeedL(pidL.setpoint + pidW.result / 90);
//   setSpeedR(pidR.setpoint - pidW.result / 90);
// }

//�ж��б�ѭ������
void renew(void)
{
  if (adc_data[2] < 40)
  {
    sp_set = 0;
  }
  if (fabs(k) < 7)
  {
    setSpeedLR(sp_set_max); //�����ٶȳ�ʼֵΪĿ��ֵ
    pidS.proportiongain = P_S;
    pidS.derivativegain = D_S_z;
    G_S = G_S_zhi;
  }
  else
  {
    G_S = G_S_1;
    pidS.proportiongain = P_S_MAX;
    pidS.derivativegain = D_S;
    setSpeedLR(sp_set); //�����ٶȳ�ʼֵΪĿ��ֵ
  }
  if (abs(adc_data[1] - adc_data[3] < 50))
  {
    setSpeedLR(sp_set);
  }
  else
  {
    setSpeedLR(sp_set * 0.85);
  }
  if (sp_set < 3)
  {
    setSpeedLR(0);
  }
  if (puodao == 1)
  {
    setSpeedLR(100);
  }
  if (adc_data[2] > 820)
  {
    setSpeedLR(110);
  }

  Ring_control();
  //����ת��PID����110014
  setPidS(P_S + (fabs(k) / 50) * (P_S_MAX - P_S), 0, D_S_z + (fabs(k) / 50) * (D_S - D_S_z));
  PIDRegulation1(&pidS, k); //���򻷼���(�������ǻ���)
  pidS.result += G_S * gyro;
  // Ring_control();
  obstacle_control();
  diffSpeedSet(); //�������ã�������ΪpidS.result��pidL.result��
  // control();      //����Ԫ�ؿ���
  //          angle += gyro * 0.01;
  //          Jinyun baked cakeJinyun baked cakepidW.setpoint = angle_change(aimed_angle);
  //          PIDRegulation1(&pidW, angle);
  //          diffSpeedSet1();

  if (abs(speedL) > 800 || abs(speedR) > 800)
  {
    sp_set = 0;
  }
  if (pidL.setpoint < -900)
    pidL.setpoint = -900;
  if (pidR.setpoint < -1000)
    pidR.setpoint = -1000;
  if (pidL.setpoint > 1500)
    pidL.setpoint = 1500;
  if (pidR.setpoint > 1500)
    pidR.setpoint = 1500;
  PIDRegulation(&pidL, (float)speedL); //�ٶȻ�����
  PIDRegulation(&pidR, (float)speedR); //�ٶȻ�����

  if (pidL.result >= -pidL.deadband && pidL.result <= pidL.deadband)
  {
    setMotorL(-(int16)pidL.result);
  }

  if (pidR.result >= -pidR.deadband && pidR.result <= pidR.deadband)
  {
    setMotorR((int16)pidR.result);
  }
}