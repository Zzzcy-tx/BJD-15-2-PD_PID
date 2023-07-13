#include "kaerman.h"

float accel_x;      // X轴加速度值
float accel_y;      // Y轴加速度值
float accel_z;      // Z轴加速度值
float gyro_x;       // X轴陀螺仪角速度数据
float gyro_y;       // Y轴陀螺仪角速度数据
float gyro_z;       // Z轴陀螺仪角速度数据
float accel_x_rea;  // X轴加速度值
float accel_y_rea;  // Y轴加速度值
float accel_z_rea;  // Z轴加速度值
float gyro_x_rea;   // X轴陀螺仪角速度数据
float gyro_y_rea;   // Y轴陀螺仪角速度数据
float gyro_z_rea;   // Z轴陀螺仪角速度数据
float yaw_raw;      //航线角yaw原始数据
float yaw_kalman;   // yaw滤波后数据
float pitch_kalman; // pitch滤波后数据
float pitch_raw;    //俯仰角pitch原始数据
float roll_kalman;  // roll滤波后数据
float roll_raw;     //横滚角roll原始数据

//卡尔曼滤波（旧）
static double p_last = 0;
static double x_last = 0;
double R = 0.1; //滤波参数，越小越平滑
double Q = 30;  //滤波参数，越大越平滑
double KalmanFilter(const double ResrcData)
{

  double x_mid;
  double x_now;

  double p_mid;
  double p_now;

  double kg;

  //这里p_last 等于 kalmanFilter_A 的p直接取0
  x_mid = x_last;     // x_last=x(k-1|k-1),x_mid=x(k|k-1)
  p_mid = p_last + Q; // p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声

  /*
   *  卡尔曼滤波的五个重要公式
   */
  kg = p_mid / (p_mid + R);                 // kg为kalman filter，R 为噪声
  x_now = x_mid + kg * (ResrcData - x_mid); //估计出的最优值
  p_now = (1 - kg) * p_mid;                 //最优值对应的covariance
  p_last = p_now;                           //更新covariance 值
  x_last = x_now;                           //更新系统状态值

  return x_now;
}

void Angle_Cal()
{
  imu963ra_get_acc();
  imu963ra_get_gyro();
  // 1.获取加速度和角度数据
  accel_x = imu963ra_acc_x;
  accel_y = imu963ra_acc_y;
  accel_z = imu963ra_acc_z;
  gyro_x = imu963ra_gyro_x;
  gyro_y = imu963ra_gyro_y;
  gyro_z = imu963ra_gyro_z;

  // 2.角加速度原始值处理过程
  if (accel_x < 32764)
    accel_x_rea = accel_x / 4098.0;
  else
    accel_x_rea = 1 - (accel_x - 49152) / 4098.0;
  if (accel_y < 32764)
    accel_y_rea = accel_y / 4098.0;
  else
    accel_y_rea = 1 - (accel_y - 49152) / 4098.0;
  if (accel_z < 32764)
    accel_z_rea = accel_z / 4098.0;
  else
    accel_z_rea = 1 - (accel_z - 49152) / 4098.0;

  //加速度反正切公式计算三个轴和水平面坐标系之间的夹角
  pitch_raw = (atan(accel_y_rea / accel_z_rea)) * 180 / 3.14;
  roll_raw = (atan(accel_x_rea / accel_z_rea)) * 180 / 3.14;
  yaw_raw = (atan(accel_y_rea / accel_x_rea)) * 180 / 3.14;

  // 3.角速度原始值处理过程
  if (gyro_x_rea < 32768)
    gyro_x_rea = -(gyro_x_rea / 14.3);
  if (gyro_x_rea > 32768)
    gyro_x_rea = +(65535 - gyro_x_rea) / 14.3;
  if (gyro_y_rea < 32768)
    gyro_y_rea = -(gyro_y_rea / 14.3);
  if (gyro_y_rea > 32768)
    gyro_y_rea = +(65535 - gyro_y_rea) / 14.3;
  if (gyro_z_rea < 32768)
    gyro_z_rea = -(gyro_z_rea / 14.3);
  if (gyro_z_rea > 32768)
    gyro_z_rea = +(65535 - gyro_z_rea) / 14.3;

  // 4.调用卡尔曼函数
  Kalman_Cal_Pitch(pitch_raw, gyro_x_rea); //卡尔曼滤波计算俯仰角
                                           //	Kalman_Cal_Roll(roll_raw,gyro_y_rea);  //卡尔曼滤波计算倾斜角
                                           //	Kalman_Cal_Yaw(yaw_raw,gyro_z_rea);  //卡尔曼滤波计算偏航角
		if (pitch_kalman > 15)
		{
			puodao = 1;
		}
		else if(puodao == 1 && pitch_kalman < -10)
		{
			puodao = 2;
		}
	 else if(puodao == 2 && pitch_kalman <5)
		{
			puodao = 0;
		}
		else
		{
			puodao = 0;
		}
}

//卡尔曼参数
static float Q_angle = 0.0003; //角度数据置信度，角度噪声的协方差
static float Q_gyro = 0.0009;  //角速度数据置信度，角速度噪声的协方差
static float R_angle = 0.8;    //加速度计测量噪声的协方差
static float dt = 0.005;       //采样周期即计算任务周期5ms

void Kalman_Cal_Pitch(float acc, float gyro) //卡尔曼滤波pitch轴计算
{
  static float Q_bias0;                      // Q_bias:陀螺仪的偏差
  static float K_00, K_10;                   //卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差 t_0/1:中间变量
  static float PP0[2][2] = {{1, 0}, {0, 1}}; //过程协方差矩阵P，初始值为单位阵

  /*
    卡尔曼滤波的使用步骤
    (1) 选择状态量、观测量
    (2) 构建方程
    (3) 初始化参数
    (4) 带入公式迭代
    (5) 调节超参数P、Q
  */
  /*
  X(k)：k时刻系统状态				Z(k)：k时刻测量值
  U(k)：k时刻对系统控制量		H：测量系统参数
                                           方差
  A/F：状态转移矩阵					W(k)：过程噪声 ----> Q
                                           方差
  B：控制矩阵								V(k)：测量噪声 ----> R

                    离散控制系统
  系统描述：X(k|k-1) = AX(k-1|k-1) + BU(k) + (W(k))
  测量值：Z(k) = HX(k) + V(k)
  */
  /*
  1. 先验估计
* * *公式1：X(k|k-1) = AX(k-1|k-1) + BU(k) + (W(k))

    X = (Angle,Q_bias)
    A(1,1) = 1,A(1,2) = -dt
    A(2,1) = 0,A(2,2) = 1
    注：上下连“[”代表矩阵
    预测当前角度值：
    [ angle ] 	[1 -dt][ angle ]	 [dt]
    [ Q_bias] = [0  1 ][ Q_bias] + [ 0] * newGyro(加速度计测量值)
    故
    angle = angle - Q_bias*dt + newGyro * dt
    Q_bias = Q_bias
  */
  pitch_kalman += (gyro - Q_bias0) * dt; //状态方程,角度值等于上次最优角度加角速度减零漂后积分

  /*
  2. 预测协方差矩阵
* * *公式2：P(k|k-1)=AP(k-1|k-1)A^T + Q

    由先验估计有系统参数
        [1 -dt]
    A = [0  1 ]

    系统过程协方差噪声Q定义：
    | cov(angle,angle)  cov(Q_bias,angle) |
    |	cov(angle,Q_bias) cov(Q_bias,Q_bias)|
       角度噪声和角速度漂移噪声相互独立
    | D( angle )  			0 	 |
  = |			0 			D( Q_bias )|
    又Q_angle和Q_bias的方差为常数，
    可由经验或计算得出

    令D( angle )  = Q_angle
      D( Q_bias ) = Q_gyro

    设上一次预测协方差矩阵为P(k-1)
                      |a(k-1)  b(k-1)|
                      |c(k-1)  d(k-1)|
    本次预测协方差矩阵P(k)
                      |a(k)  b(k)|
                      |c(k)  d(k)|
    由公式2：P(k|k-1)=AP(k-1|k-1)A^T + Q 得
    |a(k)  b(k)|		|1 -dt|	|a(k-1) b(k-1)| |1 	 0|		| D( angle )  			0 	 |
    |c(k)  d(k)| =  |0  1 | |c(k-1) d(k-1)| |-dt 1| + |			0 			D( Q_bias )|

    进一步得
    |a(k)  b(k)|		|a(k-1) - [c(k-1) + b(k-1)]*dt + d(dt)^2		b(k-1) - d(k-1)*dt|		| D( angle )  			0 	 |
    |c(k)  d(k)| =  |				c(k-1) - d(k-1)*dt													d(k-1)		| + |			0 			D( Q_bias )|

    由于dt^2太小，故dt^2省略
  */

  PP0[0][0] = PP0[0][0] + Q_angle - (PP0[0][1] + PP0[1][0]) * dt;
  PP0[0][1] = PP0[0][1] - PP0[1][1] * dt;
  PP0[1][0] = PP0[1][0] - PP0[1][1] * dt;
  PP0[1][1] = PP0[1][1] + Q_gyro;

  /*
    3. 建立测量方程
      系统测量方程 Z(k) = HX(k) + V(k)
      系统测量系数 H = [1, 0]
      因为陀螺仪输出自带噪声
      所以
      measure = newAngle
  */

  /*
    4. 计算卡尔曼增益
* * *公式3：Kg(k)= P(k|k-1)H^T/(HP(k|k-1)H^T+R)
        Kg = (K_0,K_1) 对应angle,Q_bias增益
        H = (1,0)
  */
  K_00 = PP0[0][0] / (PP0[0][0] + R_angle);
  K_10 = PP0[1][0] / (PP0[0][0] + R_angle);

  /*
    5. 计算当前最优化估计值
* * *公式4：X(k|k) = X(k|k-1) + kg(k)[z(k) - HX(k|k-1)]
    angle = angle + K_0*(newAngle - angle)
    Q_bias = Q_bias + K_1*(newAngle - angle)
  */

  pitch_kalman = pitch_kalman + K_00 * (acc - pitch_kalman);
  Q_bias0 = Q_bias0 + K_10 * (acc - pitch_kalman);

  /*
    6. 更新协方差矩阵
* * *公式5：P(k|k)=[I-Kg(k)H]P(k|k-1)
  */
  PP0[0][0] = PP0[0][0] - K_00 * PP0[0][0];
  PP0[0][1] = PP0[0][1] - K_00 * PP0[0][1];
  PP0[1][0] = PP0[1][0] - K_10 * PP0[0][0];
  PP0[1][1] = PP0[1][1] - K_10 * PP0[00][1];
}

void Kalman_Cal_Roll(float acc, float gyro) //卡尔曼滤波roll轴计算
{
  static float Q_bias;                      // Q_bias:陀螺仪的偏差  Angle_err:角度偏量
  static float K_0, K_1;                    //卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差 t_0/1:中间变量
  static float PP[2][2] = {{1, 0}, {0, 1}}; //过程协方差矩阵P，初始值为单位阵
  roll_kalman += (gyro - Q_bias) * dt;      //状态方程,角度值等于上次最优角度加角速度减零漂后积分
  PP[0][0] = PP[0][0] + Q_angle - (PP[0][1] + PP[1][0]) * dt;
  PP[0][1] = PP[0][1] - PP[1][1] * dt;
  PP[1][0] = PP[1][0] - PP[1][1] * dt;
  PP[1][1] = PP[1][1] + Q_gyro;
  K_0 = PP[0][0] / (PP[0][0] + R_angle);
  K_1 = PP[1][0] / (PP[0][0] + R_angle);
  roll_kalman = roll_kalman + K_0 * (acc - roll_kalman);
  Q_bias = Q_bias + K_1 * (acc - roll_kalman);
  PP[0][0] = PP[0][0] - K_0 * PP[0][0];
  PP[0][1] = PP[0][1] - K_0 * PP[0][1];
  PP[1][0] = PP[1][0] - K_1 * PP[0][0];
  PP[1][1] = PP[1][1] - K_1 * PP[0][1];
}

void Kalman_Cal_Yaw(float acc, float gyro) //卡尔曼滤波roll轴计算
{
  static float Q_bias;                      // Q_bias:陀螺仪的偏差  Angle_err:角度偏量
  static float K_0, K_1;                    //卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差 t_0/1:中间变量
  static float PP[2][2] = {{1, 0}, {0, 1}}; //过程协方差矩阵P，初始值为单位阵
  yaw_kalman += (gyro - Q_bias) * dt;       //状态方程,角度值等于上次最优角度加角速度减零漂后积分
  PP[0][0] = PP[0][0] + Q_angle - (PP[0][1] + PP[1][0]) * dt;
  PP[0][1] = PP[0][1] - PP[1][1] * dt;
  PP[1][0] = PP[1][0] - PP[1][1] * dt;
  PP[1][1] = PP[1][1] + Q_gyro;
  K_0 = PP[0][0] / (PP[0][0] + R_angle);
  K_1 = PP[1][0] / (PP[0][0] + R_angle);
  yaw_kalman = yaw_kalman + K_0 * (acc - yaw_kalman);
  Q_bias = Q_bias + K_1 * (acc - yaw_kalman);
  PP[0][0] = PP[0][0] - K_0 * PP[0][0];
  PP[0][1] = PP[0][1] - K_0 * PP[0][1];
  PP[1][0] = PP[1][0] - K_1 * PP[0][0];
  PP[1][1] = PP[1][1] - K_1 * PP[0][1];
}