#include"micro_imu_lib.h"
#include<math.h>

//是否计算绝对坐标系加速度
#define EN_ACC_ABS 1
//是否使用重力加速度修正
#define EN_ACC_FIX 1
//重力系数
#define GRAV_FACT 9.8
//加速度计修正系数
#define ACC_FIX_FACT_MAX 0.8
//加速度计修正指数衰减系数
#define ATTEN_FACT 100

//初始化
void mil_handle_init(MIL_Handle_t*p);
//获取传感器值（rad/s）（m/s）（s）
void mil_get_imu(MIL_Handle_t*p, float gx, float gy, float gz, float ax, float ay, float az, float time);
//获取绝对坐标系加速度
void mil_get_aaccel(MIL_Handle_t*p, float* aax, float* aay, float* aaz);
//获取欧拉角
void mil_get_ang(MIL_Handle_t*p, float* roll, float* pitch, float* yaw);
//实时解算
void mil_while_run(MIL_Handle_t*p);
//加速度计姿态修正
static void acce_fix(float q0, float q1, float q2, float q3, float ax, float ay, float az, float* egx, float* egy, float* egz);
//陀螺仪姿态解算
static void gyro_solu(float iq0, float iq1, float iq2, float iq3, float gx, float gy, float gz, float time, float *oq0, float *oq1, float *oq2, float *oq3);
//四元数旋转矩阵计算
static void quat_to_rotmat(float q0, float q1, float q2, float q3, MIL_RotMat_t* R);
//绝对坐标系加速度
static void acce_to_abs(float ax, float ay, float az, MIL_RotMat_t* R, float *aax, float *aay, float *aaz);
//四元数转欧拉角
static void quat_to_attu(MIL_RotMat_t* R, float* roll, float* pitch, float* yaw);

//初始化
void mil_handle_init(MIL_Handle_t*p)
{
    if(!p)
    {
        return;
    }
    //传感器原始数据归零
    p->imuData.ax = 0.0f;
    p->imuData.ay = 0.0f;
    p->imuData.az = 0.0f;
    p->imuData.gx = 0.0f;
    p->imuData.gy = 0.0f;
    p->imuData.gz = 0.0f;
    p->imuData.time = 0.0f;
    //四元数初始化为单位四元数（初始时z轴指向地面）
    p->quat.w = 1.0f;
    p->quat.x = 0.0f;
    p->quat.y = 0.0f;
    p->quat.z = 0.0f;
    //旋转矩阵归零（初始正交矩阵）
    p->rotMat.R11 = 1.0f; p->rotMat.R12 = 0.0f; p->rotMat.R13 = 0.0f;
    p->rotMat.R21 = 0.0f; p->rotMat.R22 = 1.0f; p->rotMat.R23 = 0.0f;
    p->rotMat.R31 = 0.0f; p->rotMat.R32 = 0.0f; p->rotMat.R33 = 1.0f;
    //姿态角归零
    p->attiAng.roll  = 0.0f;
    p->attiAng.pitch = 0.0f;
    p->attiAng.yaw   = 0.0f;
    //绝对加速度归零
    p->acceAbs.x = 0.0f;
    p->acceAbs.y = 0.0f;
    p->acceAbs.z = 0.0f;
}

//获取传感器值
void mil_get_imu(MIL_Handle_t*p, float gx, float gy, float gz, float ax, float ay, float az, float time)
{
    if(!p)
    {
        return;
    }
    p->imuData.gx = gx;
    p->imuData.gy = gy;
    p->imuData.gz = gz;
    p->imuData.ax = ax;
    p->imuData.ay = ay;
    p->imuData.az = az;
    p->imuData.time = time;
}

//获取绝对坐标系加速度
void mil_get_aaccel(MIL_Handle_t*p, float* aax, float* aay, float* aaz)
{
    if(!p)
    {
        return;
    }
    if(aax)
    {
        *aax = p->acceAbs.x;
    }
    if(aay)
    {
        *aay = p->acceAbs.y;
    }
    if(aaz)
    {
        *aaz = p->acceAbs.z;
    }
}

//获取欧拉角
void mil_get_ang(MIL_Handle_t*p, float* roll, float* pitch, float* yaw)
{
    if(!p)
    {
        return;
    }
    if(roll)
    {
        *roll = p->attiAng.roll;
    }
    if(pitch)
    {
        *pitch = p->attiAng.pitch;
    }
    if(yaw)
    {
        *yaw = p->attiAng.yaw;
    }
}

//实时解算
void mil_while_run(MIL_Handle_t*p)
{
    if(!p)
    {
        return;
    }
    //计算修正系数
    float fix_fact=0;
    float egx=0,egy=0,egz=0;
    if(EN_ACC_FIX)
    {
        //加速度计姿态修正
        acce_fix(p->quat.w, p->quat.x, p->quat.y, p->quat.z, p->imuData.ax, p->imuData.ay, p->imuData.az, &egx, &egy, &egz);
        //计算加速度模长
        float acc_norm = sqrtf(p->imuData.ax * p->imuData.ax + p->imuData.ay * p->imuData.ay + p->imuData.az * p->imuData.az);
        //和重力常数比较
        float grav_diff = fabsf(acc_norm - GRAV_FACT);
        fix_fact=ACC_FIX_FACT_MAX*expf(-ATTEN_FACT*grav_diff);
    }
    //计算时间间隔
    static float last_time=0;
    if(!last_time)
    {
        last_time = p->imuData.time;
        return;
    }
    float curr_time = p->imuData.time;
    float err_time = curr_time-last_time;
    last_time = curr_time;
    //陀螺仪姿态解算
    gyro_solu(p->quat.w, p->quat.x, p->quat.y, p->quat.z, p->imuData.gx + egx*fix_fact, p->imuData.gy + egy*fix_fact, p->imuData.gz + egz*fix_fact, err_time, &p->quat.w, &p->quat.x, &p->quat.y, &p->quat.z);
    //四元数旋转矩阵计算
    quat_to_rotmat(p->quat.w, p->quat.x, p->quat.y, p->quat.z, &p->rotMat);
    if(EN_ACC_ABS)
    {
        //绝对坐标系加速度
        acce_to_abs(p->imuData.ax, p->imuData.ay, p->imuData.az, &p->rotMat, &p->acceAbs.x, &p->acceAbs.y, &p->acceAbs.z);
    }
    //四元数转欧拉角
    quat_to_attu(&p->rotMat, &p->attiAng.roll, &p->attiAng.pitch, &p->attiAng.yaw);
}

//加速度计姿态修正
static void acce_fix(float q0, float q1, float q2, float q3, float ax, float ay, float az, float* egx, float* egy, float* egz)
{
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q1q1 = q1*q1;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;
    //理论重力方向
    float qgx = 2*(q1q3 - q0q2);
    float qgy = 2*(q0q1 + q2q3);
    float qgz = q0q0 - q1q1 - q2q2 + q3q3;
    //与测量重力方向的误差
    *egx = (ay*qgz - az*qgy);
    *egy = (az*qgx - ax*qgz);
    *egz = (ax*qgy - ay*qgx);
}

//陀螺仪姿态解算
static void gyro_solu(float iq0, float iq1, float iq2, float iq3, float gx, float gy, float gz, float time, float *oq0, float *oq1, float *oq2, float *oq3)
{
    //四元数积分
    float temq0 = iq0 + (-iq1*gx - iq2*gy - iq3*gz) * time / 2;
    float temq1 = iq1 + ( iq0*gx + iq2*gz - iq3*gy) * time / 2;
    float temq2 = iq2 + ( iq0*gy - iq1*gz + iq3*gx) * time / 2;
    float temq3 = iq3 + ( iq0*gz + iq1*gy - iq2*gx) * time / 2;
    //四元数归一化
    float norm = sqrtf(temq0*temq0 + temq1*temq1 + temq2*temq2 + temq3*temq3);
    if (norm > 1e-6f)
    {
        temq0 /= norm;
        temq1 /= norm;
        temq2 /= norm;
        temq3 /= norm;
    }
    *oq0 = temq0;
    *oq1 = temq1;
    *oq2 = temq2;
    *oq3 = temq3;
}

//四元数旋转矩阵计算
static void quat_to_rotmat(float q0, float q1, float q2, float q3, MIL_RotMat_t* R)
{
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;
    R->R11 = 1 - 2*(q2q2 + q3q3);
    R->R12 = 2*(q1q2 - q0q3);
    R->R13 = 2*(q1q3 + q0q2);
    R->R21 = 2*(q1q2 + q0q3);
    R->R22 = 1 - 2*(q1q1 + q3q3);
    R->R23 = 2*(q2q3 - q0q1);
    R->R31 = 2*(q1q3 - q0q2);
    R->R32 = 2*(q2q3 + q0q1);
    R->R33 = 1 - 2*(q1q1 + q2q2);
}

//绝对坐标系加速度
static void acce_to_abs(float ax, float ay, float az, MIL_RotMat_t* R, float *aax, float *aay, float *aaz)
{
    *aax = R->R11*ax + R->R12*ay + R->R13*az;
    *aay = R->R21*ax + R->R22*ay + R->R23*az;
    *aaz = R->R31*ax + R->R32*ay + R->R33*az;
}

//四元数转欧拉角
static void quat_to_attu(MIL_RotMat_t* R, float* roll, float* pitch, float* yaw)
{
    *roll  = atan2(R->R32, R->R33) * 57.3f;    // roll = atan2(R32, R33)
    *pitch = -asin(R->R13) * 57.3f;            // pitch = -asin(R13)
    *yaw   = atan2(R->R12, R->R11) * 57.3f;    // yaw = atan2(R12, R11)
}
