#include "tim.h"
#include "stdint.h"
void HAL_TimDelay_us(uint32_t us);//延时us函数
void HAL_TimDelay_ms(uint32_t ms);//延时ms函数
#define US_TIM htim6//修改定时器，注意两点，第一需要定时器分频到1M，第二记得包含tim.h，否则找不到htimx
