#include "string.h"
#include "Base.h"
#include "math.h"
#include "usart.h"
#include "RM3508.h"
#define PI 3.1415926
//#include "tim.h"
//#include "cmsis_os.h"
GYRO_DATA gyro_data;
VISION_DATA vision_data;


//*************************参数定义***************************//
int SPEED[4];//四个电机的速度  测试小车
int speed; //目标速度
int flag ;//全局变量，用来debug的
int fix_speed;//修正速度
uint8_t Rx_USART[8];//WIFI传递的信息
extern int Angle;
float aim_angle;
#define BUFFERSIZE 255           //定义缓冲区长度
extern uint8_t ReceiveBuff[BUFFERSIZE];
extern int flag_circle;
/***************************************************************/


////车轮的转动  duty占空比  dir=1正转  or 反转填0吧（）//
///*************************************************************/
////右前方
//void wheel_1(int duty,int dir)  
//{
//	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,duty);
//	if(dir==1)
//	{

//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,1);
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,0);
//	}
//	else
//	{

//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,0);
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,1);
//	}
//}
////左前方
//void wheel_2(int duty,int dir)
//{
//__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,duty);
//		if(dir==1)
//	{

//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,1);
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,0);
//	}
//	else
//	{

//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,0);
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,1);
//	}
//}
////右后方
//void wheel_3(int duty,int dir)
//{
//__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,duty);
//		if(dir==1)
//	{

//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,1);
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,0);
//	}
//	else
//	{

//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,0);
//	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,1);
//	}
//}
////左后方
//void wheel_4(int duty,int dir)
//{
//__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,duty);
//		if(dir==1)
//	{

//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,1);
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
//	}
//	else
//	{

//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);
//	}
//}
////***************************************************************//



//********************Can初始化*****************************//
void CAN_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;
    HAL_StatusTypeDef HAL_Status;
    /*这里默认CAN1和CAN2使用不同的FIFO*/
    if (hcan->Instance == hcan1.Instance)
    {
        sFilterConfig.FilterBank = 0;                     //过滤器组
        sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //CAN_FILTERMODE_IDLIST  CAN_FILTERMODE_IDMASK
        sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
        sFilterConfig.FilterIdHigh = 0x0000; //filter id
        sFilterConfig.FilterIdLow = 0x0000;
        sFilterConfig.FilterMaskIdHigh = 0x0000;
        sFilterConfig.FilterMaskIdLow = 0x0000;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1; //用FIFO接收
        sFilterConfig.FilterActivation = ENABLE;
        sFilterConfig.SlaveStartFilterBank = 1;

        HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
        HAL_CAN_Start(hcan); //开启CAN
        HAL_Status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    }
    else if (hcan->Instance == hcan2.Instance)
    {
        sFilterConfig.FilterBank = 15;                    //过滤器组
        sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //CAN_FILTERMODE_IDLIST  CAN_FILTERMODE_ISK
        sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
        sFilterConfig.FilterIdHigh = 0x0000; //filter id
        sFilterConfig.FilterIdLow = 0x0000;
        sFilterConfig.FilterMaskIdHigh = 0x0000;
        sFilterConfig.FilterMaskIdLow = 0x0000;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; //用FIFO接收
        sFilterConfig.FilterActivation = ENABLE;
        sFilterConfig.SlaveStartFilterBank = 15;

        HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
        HAL_CAN_Start(hcan); //开启CAN
        HAL_Status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
			
    }
    if (HAL_Status != HAL_OK)
    {
        Error_Handler();
    }
}

//********************Can中断回调函数*****************************//
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
  if (hcan->Instance == hcan2.Instance)
  {
    CAN_RxHeaderTypeDef RxMessage;
    uint8_t RxData[8] = {0};
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, RxData);
			if(RxMessage.IDE == CAN_ID_STD)
		{
			if(RxMessage.StdId >= 0x201 && RxMessage.StdId <= 0x208)
			{
				
			RM3508_Get_Feedback (RxMessage.StdId,RxData);
			}
		}
  }
	
	
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
  if (hcan->Instance == hcan1.Instance)
  {
    CAN_RxHeaderTypeDef RxMessage;
    uint8_t RxData[8] = {0};
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxMessage, RxData);
			if(RxMessage.IDE == CAN_ID_STD)
		{
			
			if(RxMessage.StdId == 0x351
				
			)
			{
				get_GyroData(RxData);
			//	printf("%f",gyro_data.Z_Pos);
				//HAL_Delay(1);
			}
		}
  }
}
//********************USART中断回调函数*****************************//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart4.Instance)
	{
		HAL_UART_Receive(huart, Rx_USART, sizeof(Rx_USART), 10000);		
	}
}


//*****************************************************************//

//将视觉数据传递给vision_data结构体
void get_VisionData(void) 
{
    //用**方式得到角度与距离
    //vision_data.angle, vision_data.distance = f();
};


//将陀螺仪数据传递给gyro_data结构体
void get_GyroData(uint8_t* CAN_DATA)
{ 
	uint8_t tmp[4] = {0};
    /*得到角度 , 右旋为正*/
	memcpy(tmp,&CAN_DATA[0], 4);
	gyro_data.Z_Pos = -*(float *)tmp-0.01/100;
};

//方案一：边跑边修正，此函数将在run_forward()等函数中调用，不干扰操作手的操作
//方案二：跑歪到一定角度修正，此函数将在外部调用，修正时会强制打断操作手的操作直至修正完成

//小车向前移动
void run_forward(int speed, int fix_speed)
{
	if(gyro_data.Z_Pos >= 3)
	{	
		SPEED[0] = speed + fix_speed;
		SPEED[1] = speed;
		SPEED[2] = speed + fix_speed;
		SPEED[3] = speed;
	}
	else if(gyro_data.Z_Pos <= -3)
	{
		SPEED[0] = speed;
		SPEED[1] = speed + fix_speed;
		SPEED[2] = speed;
		SPEED[3] = speed + fix_speed;
	}
	else
	{
		SPEED[0] = speed;
		SPEED[1] = speed;
		SPEED[2] = speed;
		SPEED[3] = speed;
	}
	
//	wheel_1(SPEED[0],1);
//  wheel_2(SPEED[1],1);
//	wheel_3(SPEED[2],1);
//	wheel_4(SPEED[3],1);
	HAL_Delay(0);
	RM3508_Set_Speed(SPEED[0],1);
	RM3508_Set_Speed(SPEED[1],2);
	RM3508_Set_Speed(SPEED[2],3);
	RM3508_Set_Speed(SPEED[3],4);
	//调用RM3508_Set_Speed（）
};

//小车向后移动
void run_back(int speed, int fix_speed)
{
	if(gyro_data.Z_Pos >= 3)
	{	
		SPEED[0] = speed;
		SPEED[1] = speed + fix_speed ;
		SPEED[2] = speed;
		SPEED[3] = speed + fix_speed ;
	}
	else if(gyro_data.Z_Pos <= -3)
	{
		SPEED[0] = speed + fix_speed;
		SPEED[1] = speed;
		SPEED[2] = speed + fix_speed;
		SPEED[3] = speed;
	}
	else
	{
		SPEED[0] = speed;
		SPEED[1] = speed;
		SPEED[2] = speed;
		SPEED[3] = speed;
	}
	
//	wheel_1(SPEED[0],0);
//  wheel_2(SPEED[1],0);
//	wheel_3(SPEED[2],0);
//	wheel_4(SPEED[3],0);
	RM3508_Set_Speed(-SPEED[0],1);
	RM3508_Set_Speed(-SPEED[1],2);
	RM3508_Set_Speed(-SPEED[2],3);
	RM3508_Set_Speed(-SPEED[3],4);
	HAL_Delay(0);
	//调用RM3508_Set_Speed（）
};

//小车向左移动
void run_left(int speed, int fix_speed)
{
	if(gyro_data.Z_Pos >= 3)
	{	
		SPEED[0] = speed + fix_speed;
		SPEED[1] = speed;
		SPEED[2] = speed + fix_speed;
		SPEED[3] = speed;
	}
	else if(gyro_data.Z_Pos <= -3)
	{
		SPEED[0] = speed;
		SPEED[1] = speed + fix_speed;
		SPEED[2] = speed;
		SPEED[3] = speed + fix_speed;
	}
	else
	{
		SPEED[0] = speed;
		SPEED[1] = speed;
		SPEED[2] = speed;
		SPEED[3] = speed;
	}
	
//	wheel_1(SPEED[0],0);
//  wheel_2(SPEED[1],1);
//	wheel_3(SPEED[2],1);
//	wheel_4(SPEED[3],0);
	RM3508_Set_Speed(SPEED[0],1);
	RM3508_Set_Speed(-SPEED[1],2);
	RM3508_Set_Speed(-SPEED[2],3);
	RM3508_Set_Speed(SPEED[3],4);
	HAL_Delay(0);
	//调用RM3508_Set_Speed（）
};

//小车向右移动
void run_right(int speed, int fix_speed)
{
	if(gyro_data.Z_Pos >= 3)
	{	
		SPEED[0] = speed;
		SPEED[1] = speed + fix_speed;
		SPEED[2] = speed;
		SPEED[3] = speed + fix_speed;
	}
	else if(gyro_data.Z_Pos <= -3)
	{
		SPEED[0] = speed + fix_speed;
		SPEED[1] = speed;
		SPEED[2] = speed + fix_speed;
		SPEED[3] = speed;
	}
	else
	{
		SPEED[0] = speed;
		SPEED[1] = speed;
		SPEED[2] = speed;
		SPEED[3] = speed;
	}
	
//	wheel_1(SPEED[0],1);
//  wheel_2(SPEED[1],0);
//	wheel_3(SPEED[2],0);
//	wheel_4(SPEED[3],1);
	RM3508_Set_Speed(-SPEED[0],1);
	RM3508_Set_Speed(SPEED[1],2);
	RM3508_Set_Speed(SPEED[2],3);
	RM3508_Set_Speed(-SPEED[3],4);
	HAL_Delay(0);
	//调用RM3508_Set_Speed（）
};

//小车旋转
void run_rotate_clock(int speed)
	{
//		wheel_1(speed,0);
//		wheel_2(speed,1);
//		wheel_3(speed,0);
//		wheel_4(speed,1);
	//调用RM3508_Set_Pos（）
};
void run_rotate_inclock(int speed)
	{
//		wheel_1(speed,1);
//		wheel_2(speed,0);
//		wheel_3(speed,1);
//		wheel_4(speed,0);
	//调用RM3508_Set_Pos（）
};

//作用：选择最近的柱子（未在Base.h中声明）
//如果视觉组能反馈最近的柱子，此函数无需调用
//void run_aim(struct VISION_DATA vision_data[num])
//{
//    float distance_used = 10;           //需要更改 
//    float angle_use = 10;               //需要更改                                              
//    for(int i = 0; i < num; i++)
//    {
//        if(vision_data[i].angle < set_angle)
//        {
//           if(vision_data[i].distance <  distance_used) 
//           {
//                distance_used = vision_data[i].distance;
//                angle_use = vision_data[i].angle;
//           }
//        }
//    }                                   //确认目标柱    
//}

//跑向目标柱并修正姿态模式一
void run_to_aim_1(float distance, float angle, float goal_distance, int speed, int fix_speed)
{
    while(distance * cos(angle * PI /180) < goal_distance)
    {
        run_back(speed, fix_speed);
    }
    while(angle != 0)
    {
			if(angle > 0 )//假设中轴线往右为正
        {
            run_left(speed, fix_speed);
        }
        else{
            run_right(speed, fix_speed);
        }
    }
    while(distance != goal_distance)
    {
        run_forward(speed, fix_speed);
    }

};
//跑向目标柱并修正姿态模式二
void run_to_aim_2(float distance, float angle, float goal_distance, int speed, int fix_speed)
{
    while(distance * cos(angle * PI /180) < goal_distance)
    {
        run_back(speed, fix_speed);
    }
    while(angle != 0)
    {
			if(angle > 0 )//假设中轴线往右为正
        {
            run_left(speed, fix_speed);
        }
        else{
            run_right(speed, fix_speed);
        }
    }
    while(distance != goal_distance)
    {
        run_forward(speed, fix_speed);
    }

};

//调kv

void HandleStringSpeed(char *inputdata, int len)
{
	
  char targetChar = 'B';
  char targetChar2 = 'C'; 
  char targetChar3 = 'Z'; 
  char targetChar4 = 'l'; 	//无
	char targetChar5 = 'T'; 
	char targetChar6 = 'r';
	char targetChar7 = 'L';
	char targetChar8 = 't';
	char targetChar9 = 'W';
	char targetChar10 = 'S';
	char targetChar11 = 'A';
	char targetChar12 = 'D';
	char targetChar13 = 'H';
	if (inputdata[0] == targetChar9) {   //W
		Rocker_Vx=0;
		Rocker_Vy=50;
	Rotate_Vx=0;
		Kz=0;
	}
	if (inputdata[0] == targetChar10) {   //S
		Rocker_Vx=0;
		Rocker_Vy=-50;
		Rotate_Vx=0;
		Kz=0;
	}
	if (inputdata[0] == targetChar11) {   //A
    Rocker_Vx=50;
		Rocker_Vy=0;
		Rotate_Vx=0;
		Kz = 0;
//		Kz=600;
	}
	if (inputdata[0] == targetChar12) {    //D
		Rocker_Vx=-50;
		Rocker_Vy=0;
		Rotate_Vx=0;
		Kz = 0;
	
//		Kz=600;
		
	}
	if (inputdata[0] == targetChar13) {   //H

				flag_circle=flag_circle+1;
ReceiveBuff[0]=0;	
		if(flag%2==0)
		{
		  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
		}
	}
	if (inputdata[0] == targetChar) {   //B
		Kv=25;
		speed_level = 0;
	}

	else if (inputdata[0] == targetChar2) {  //C
		Kv=60;
		speed_level = 1;
		
	}
	else if (inputdata[0] == targetChar3) {   //Z
		Kv=145;
		speed_level = 1;
	
	}
	else if (inputdata[0] == targetChar4) {  //无
		Kv=25;  
		
	}
	else if (inputdata[0] == targetChar5) {  //全停
		Kv=0;
		Rotate_Vx=0;
		Rocker_Vx=0;
		Rocker_Vy=0;
	
		Kz=0;
	}
	else if (inputdata[0] == targetChar6) {   //r

		Rotate_Vx=30;
		Kz=0;
//		Rocker_Vx=0;
//		Rocker_Vy=0;
//		HAL_Delay(1000);
//		Rotate_Vx=0;
//		inputdata[0] =0;
//			Rotate_Vx=Rotate_Vx+5;
//		if(Rotate_Vx>40)
//		{Rotate_Vx=40;}//speed
//		Angle=90;
	}
	else if (inputdata[0] == targetChar7) {//l

		Rotate_Vx=-30;
	//	Kz=0;
//		Rocker_Vx=0;
//		Rocker_Vy=0;
//		HAL_Delay(1000);
//		Rotate_Vx=0;
//		inputdata[0] =0;
//		Rotate_Vx=Rotate_Vx-5;
//		if(Rotate_Vx<-40)//speed
//		{Rotate_Vx=-40;}
//	Angle=-90;
	}
else if (inputdata[0] == targetChar8) {//t

Rotate_Vx=0;
	//Angle=0;
	
	}
}
//void HandleString_Rocker(char *inputdata, char chr, int len)
//	{
//	int i = 0;
//	int pos;
//	char SpeedString[8] = {0};
//	//HAL_UART_Transmit(&huart2,ReceiveBuff,Rx_len,100);
//	for(i = len -2; i> 0; i--){
//	if(inputdata[i] == chr){
//	pos = i;
//		break;
//	}
//	}
//  for(i = pos; i < len - 2; i++){
//	
//		SpeedString[i - pos] = inputdata[i+1];	
//	}
//	if (inputdata[0] == 'R') {
//	sscanf(SpeedString, "%d;%d", &Rocker_Vx, &Rocker_Vy);

//	}
//	
// if (inputdata[0] == 'Z') {
//	sscanf(SpeedString, "%d;%d", &Rotate_Vx, &Rotate_Vy);
//			
//}
////flag_v=0;
//}
//	void HandleString_Rocker_2(char *inputdata, char chr, int len)
//	{
//	
//	int i = 0;
//	int pos;
//	char SpeedString[8] = {0};
//	//HAL_UART_Transmit(&huart2,ReceiveBuff,Rx_len,100);
//	for(i = len -2; i> 0; i--){
//	if(inputdata[i] == chr){
//	pos = i;
//		break;
//	}
//	}
//  for(i = pos; i < len - 2; i++){
//	
//		SpeedString[i - pos] = inputdata[i+1];	
//	}
//	sscanf(SpeedString, "%d;%d", &Rotate_Vx, &Rotate_Vy);

//	flag_v=0;
//}

//抓取环
void aircylinder_catch();

//发射环
void aircylinder_shoot();

