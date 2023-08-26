#include "delay.h"
void HAL_TimDelay_us(uint32_t us)//延时us函数,不能超过65535
{
  __HAL_TIM_SetCounter(&US_TIM,0);//清零
  HAL_TIM_Base_Start(&US_TIM);//开启定时器
  while(__HAL_TIM_GetCounter(&US_TIM)<us){};
  HAL_TIM_Base_Stop(&US_TIM);//关闭定时器
}
void HAL_TimDelay_ms(uint32_t ms)//延时ms函数
{
	for(uint32_t Delay_Cnt=0;Delay_Cnt<ms;Delay_Cnt++)
	HAL_TimDelay_us(997);//考虑实际代码运行损耗，不写1000，差一点点补上
}
