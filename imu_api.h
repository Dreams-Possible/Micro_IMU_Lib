#pragma once

//初始化
void imu_api_init();
//解算数据
void imu_api_run();
//获取姿态角
void imu_api_get_attu(float*roll,float*pitch,float*yaw);
//获取绝对加速度
void imu_api_get_accel(float*aax,float*aay,float*aaz);
