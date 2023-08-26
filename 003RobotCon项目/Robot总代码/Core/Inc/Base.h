#ifndef __Base_H
#define __Base_H

#include "can.h"
#include "main.h"
#include "string.h"
#include "stdint.h"
#include "stm32f4xx_hal_can.h"
//#include "RM3508.h"
//

////右前轮
//void wheel_1(int duty,int dir);

////左前轮
//void wheel_2(int duty,int dir);

////右后轮
//void wheel_3(int duty,int dir);

////左后轮
//void wheel_4(int duty,int dir);

//Can初始化
void CAN_Init(CAN_HandleTypeDef *hcan);

//Can中断回调
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
//定义视觉数据的结构体
typedef struct{
    float angle;
    float distance;
}VISION_DATA;

//定义陀螺仪数据的结构体
typedef struct{
    float Z_Pos;
}GYRO_DATA;

//定义四个电机速度
extern int SPEED[4];

//定义目标速度
extern int speed; 

//定义修正速度
extern int fix_speed;

//计算循环次数，debug用
extern int flag;

//实例化接收陀螺仪数据的结构体
extern GYRO_DATA gyro_data;

//USART收到的数据
extern uint8_t Rx_USART[8];

//实例化接收视觉数据的结构体
extern VISION_DATA vision_data;

extern float aim_angle;
extern int Kv;

extern int flag_v;
extern int Rocker_Vx;
extern int Rocker_Vy;
extern int Rotate_Vx;
extern int Rotate_Vy;

//接受视觉数据
void get_VisionData(void);

//接受陀螺仪数据
void get_GyroData(uint8_t* CAN_DATA);

//小车向前移动
void run_forward(int speed, int fix_speed);

//小车向后移动
void run_back(int speed, int fix_speed);

//小车向左移动
void run_left(int speed, int fix_speed);

//小车向右移动
void run_right(int speed, int fix_speed);

//小车顺时针旋转
void run_rotate_clock(int speed);

//小车逆时针旋转
void run_rotate_inclock(int speed);

//运行到指定位置
void run_to_aim_1(float distance, float angle, float goal_distance,int speed, int fix_speed);

//调速度Kv
void HandleStringSpeed(char *inputdata, int len);

//
void HandleString_Rocker(char *inputdata, char chr, int len);
void HandleString_Rocker_2(char *inputdata, char chr, int len);	
//抓取环
void aircylinder_catch(void);

//发射环
void aircylinder_shoot(void);


#endif
