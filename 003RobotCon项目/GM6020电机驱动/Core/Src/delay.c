#include "delay.h"
void HAL_TimDelay_us(uint32_t us)//��ʱus����,���ܳ���65535
{
  __HAL_TIM_SetCounter(&US_TIM,0);//����
  HAL_TIM_Base_Start(&US_TIM);//������ʱ��
  while(__HAL_TIM_GetCounter(&US_TIM)<us){};
  HAL_TIM_Base_Stop(&US_TIM);//�رն�ʱ��
}
void HAL_TimDelay_ms(uint32_t ms)//��ʱms����
{
	for(uint32_t Delay_Cnt=0;Delay_Cnt<ms;Delay_Cnt++)
	HAL_TimDelay_us(997);//����ʵ�ʴ���������ģ���д1000����һ��㲹��
}
