#include"imu_api.h"
#include"icm_api.h"
#include"micro_imu_lib.h"

//初始化
void imu_api_init();
//解算数据
void imu_api_run();
//获取姿态角
void imu_api_get_attu(float*roll,float*pitch,float*yaw);
//获取绝对加速度
void imu_api_get_accel(float*aax,float*aay,float*aaz);

//解算句柄
MIL_Handle_t imu_handle={0};

//初始化
void imu_api_init()
{
    icm_init();
    mil_handle_init(&imu_handle);
}

//解算数据
void imu_api_run()
{
    float ax=0,ay=0,az=0;
    float gx=0,gy=0,gz=0;
    float curr_time=0;
    //读加速度
    if(icm_read_accel(&ax,&ay,&az))
    {
        return;
    }
    //读陀螺仪
    if(icm_read_gyro(&gx,&gy,&gz))
    {
        return;
    }
    //换算单位
    ax*=9.8;
    ay*=9.8;
    az*=9.8;
    gx*=0.01745;
    gy*=0.01745;
    gz*=0.01745;
    //读时间
    curr_time=sp_timer_get()/1000000.0;
    //解算数据
    mil_get_imu(&imu_handle,gx,gy,gz,ax,ay,az,curr_time);
    mil_while_run(&imu_handle);
}

//获取姿态角
void imu_api_get_attu(float*roll,float*pitch,float*yaw)
{
    mil_get_ang(&imu_handle,roll,pitch,yaw);
}

//获取绝对加速度
void imu_api_get_accel(float*aax,float*aay,float*aaz)
{
    mil_get_aaccel(&imu_handle,aax,aay,aaz);
}
