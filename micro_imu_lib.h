#pragma once

//传感器数据结构体
typedef struct {
    // 加速度计原始数据
    float ax;    // X轴加速度 (m/s²)
    float ay;    // Y轴加速度 (m/s²)
    float az;    // Z轴加速度 (m/s²)
    // 陀螺仪角速度
    float gx;    // X轴角速度 (rad/s)
    float gy;    // Y轴角速度 (rad/s)
    float gz;    // Z轴角速度 (rad/s)
    // 时间戳
    float time;  // 时间戳 (s)
} MIL_IMU_t;

//四元数结构体
typedef struct {
    float w;   /**< 四元数实部，也叫 q0 */
    float x;   /**< 四元数虚部 x，也叫 q1 */
    float y;   /**< 四元数虚部 y，也叫 q2 */
    float z;   /**< 四元数虚部 z，也叫 q3 */
} MIL_Quat_t;

//旋转矩阵结构体
typedef struct {
    float R11, R12, R13;
    float R21, R22, R23;
    float R31, R32, R33;
} MIL_RotMat_t;

//姿态角结构体
typedef struct {
    float roll;           /**< 欧拉角 Roll (°) */
    float pitch;          /**< 欧拉角 Pitch (°) */
    float yaw;            /**< 欧拉角 Yaw (°) */
} MIL_AttiAng_t;

//绝对坐标系加速度结构体
typedef struct {
    float x;   // 绝对X轴加速度 (m/s²)
    float y;   // 绝对Y轴加速度 (m/s²)
    float z;   // 绝对Z轴加速度 (m/s²)
} MIL_AcceAbs_t;

//IMU库结构体
typedef struct {
    MIL_IMU_t imuData;  // 传感器数据
    MIL_Quat_t quat;          // 四元数
    MIL_RotMat_t rotMat;      // 旋转矩阵
    MIL_AttiAng_t attiAng;    // 姿态角
    MIL_AcceAbs_t acceAbs;    // 绝对坐标系加速度
} MIL_Handle_t;

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
