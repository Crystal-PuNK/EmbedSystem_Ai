#include "tim.h"
#include "stdint.h"
void HAL_TimDelay_us(uint32_t us);//��ʱus����
void HAL_TimDelay_ms(uint32_t ms);//��ʱms����
#define US_TIM htim6//�޸Ķ�ʱ����ע�����㣬��һ��Ҫ��ʱ����Ƶ��1M���ڶ��ǵð���tim.h�������Ҳ���htimx
