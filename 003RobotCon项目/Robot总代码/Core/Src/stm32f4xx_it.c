/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
#define BUFFERSIZE 255	//
extern uint8_t Rx_len,bootfirst;
extern uint8_t ReceiveBuff[BUFFERSIZE]; //
extern uint8_t Rx_len_2;
extern uint8_t ReceiveBuff_2[BUFFERSIZE]; //
extern int Rocker_Vx;
extern int Rocker_Vy;
extern int flag_v;
extern int flag;
extern uint8_t VisionData[10];
extern float angle_Vision, distance_Vision;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles CAN1 SCE interrupt.
  */
void CAN1_SCE_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_SCE_IRQn 0 */

  /* USER CODE END CAN1_SCE_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_SCE_IRQn 1 */

  /* USER CODE END CAN1_SCE_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */


/**
  * @brief This function handles UART4 global interrupt.
  */


/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN2 SCE interrupt.
  */
void CAN2_SCE_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_SCE_IRQn 0 */

  /* USER CODE END CAN2_SCE_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_SCE_IRQn 1 */

  /* USER CODE END CAN2_SCE_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
 
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN USART1_IRQn 1 */
	uint32_t temp;
	if(UART4 == huart4.Instance)//
	{
		if(RESET != __HAL_UART_GET_FLAG(&huart4,UART_FLAG_IDLE))//判断是否发生空闲中断 如果flag置一就是数据接收完成
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart4);//清楚标志位
			
		//	printf("callback");//我的测试代码 不用管
			HAL_UART_DMAStop(&huart4);// 停止dma接收
			temp  = __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);// 数空闲位
			Rx_len =  BUFFERSIZE - temp; //获取数据长度
//			
	printf("temp:%d Rx_len:%d flag:%d",temp,Rx_len,flag);//我的测试代码 不用管
//			if (flag==1)
//			{
//					
//		HAL_UART_Transmit_DMA(&huart1,ReceiveBuff,Rx_len);//发送接收的数据
		
		//	printf("-----------");
//			//	measure(ReceiveBuff,Rx_len);
					//HandleString_Rocker(ReceiveBuff, 'R', Rx_len);
			//	printf("        \n");
//			}
		//	Rx_len=0;// 清零
			

		//		flag_v=1;// 置1
					
		
	
		
		}
		HAL_UART_Receive_DMA(&huart4,ReceiveBuff,BUFFERSIZE);//	
			
	}
		
  /* USER CODE END USART1_IRQn 1 */
}
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
 
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART1_IRQn 1 */
	uint32_t temp;
	if(USART2 == huart2.Instance)//
	{
		if(RESET != __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE))//判断是否发生空闲中断 如果flag置一就是数据接收完成
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);//清楚标志位
			
		printf("callback");//我的测试代码 不用管
			HAL_UART_DMAStop(&huart2);// 停止dma接收
			temp  = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);// 数空闲位
			Rx_len_2 =  BUFFERSIZE - temp; //获取数据长度
		int	flag_2=1;
		//	printf("temp:%d Rx_len:%d flag:%d",temp,Rx_len,flag);//我的测试代码 不用管
			if (flag_2==1)
			{
					
	//	HAL_UART_Transmit_DMA(&huart2,ReceiveBuff_2,Rx_len_2);//发送接收的数据
				
				if(VisionData[1] == '!'&& VisionData[9] == '!')
		{
			uint8_t x[2],y[2];
			uint16_t X,Y;
			x[0] = VisionData[4];
			x[1] = VisionData[5];
			y[0] = VisionData[6];
			y[1] = VisionData[7];
			memcpy(&X, x, sizeof(uint16_t));
			memcpy(&Y, y, sizeof(uint16_t));
			if(VisionData[8] == 2)
			{
				angle_Vision = -(float)X/100;//距离
				distance_Vision = (float)Y/100;//角度
			}
			else
			{
				angle_Vision = (float)X/100;//距离
				distance_Vision = (float)Y/100;//角度
			}
			
		}
				
				
				
				
	flag_2=0;
			}
	
		
	}
	
		HAL_UART_Receive_DMA(&huart2,ReceiveBuff_2,BUFFERSIZE);//	
			
	}
		
  /* USER CODE END USART1_IRQn 1 */
}
/* USER CODE END 1 */
