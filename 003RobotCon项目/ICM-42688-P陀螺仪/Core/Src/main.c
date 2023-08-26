/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t ICM_Feedback_Buf[8];


struct{
	float X_Pos;
		float X_Spd;
		
		float X_SpdCounter;
		float X_Spd_Temp;
		float Z_Pos;
		float Z_Spd;
		float time_out_counter;
	}Cur_GYRO;

	
	
	
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
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; //用FIFO接收
        sFilterConfig.FilterActivation = ENABLE;
        sFilterConfig.SlaveStartFilterBank = 1;

        HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
        HAL_CAN_Start(hcan); //开启CAN
        HAL_Status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else if (hcan->Instance == hcan2.Instance)
    {
        sFilterConfig.FilterBank = 15;                    //过滤器组
        sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //CAN_FILTERMODE_IDLIST  CAN_FILTERMODE_IDMASK
        sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
        sFilterConfig.FilterIdHigh = 0x0000; //filter id
        sFilterConfig.FilterIdLow = 0x0000;
        sFilterConfig.FilterMaskIdHigh = 0x0000;
        sFilterConfig.FilterMaskIdLow = 0x0000;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1; //用FIFO接收
        sFilterConfig.FilterActivation = ENABLE;
        sFilterConfig.SlaveStartFilterBank = 15;

        HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
        HAL_CAN_Start(hcan); //开启CAN
        HAL_Status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
			
    }
    if (HAL_Status != HAL_OK)
    {
        Error_Handler();
    }
}


void ICM_Rec(uint8_t motor_id)
{
	   
	uint8_t tmp[4] = {0};
    	/*得到角度 , 右旋为正*/
	    memcpy(tmp,&ICM_Feedback_Buf[0], 4);
			Cur_GYRO.Z_Pos = -*(float *)tmp;

			/*得到角速度 , 右旋为正*/
			memcpy(tmp, &ICM_Feedback_Buf[4], 4);
			Cur_GYRO.Z_Spd = -*(float *)tmp;


}

void ICM_Get_Feedback(uint32_t std_id, uint8_t* data_p)
{
	
	int i;
	for(i = 0; i < 2; i++)  
	{
			
		if(std_id == 0x351 + i)
		{
			memcpy(ICM_Feedback_Buf, data_p, 8);
			ICM_Rec(i);
			return;
		}
	}
}


void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)

{
	
  if (hcan->Instance == hcan2.Instance)
  {
    CAN_RxHeaderTypeDef RxMessage;
    uint8_t RxData[8] = {0};
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxMessage, RxData);

    if (RxMessage.IDE == CAN_ID_STD) // 标准帧
    {
		
      if (RxMessage.StdId >= 0x351 && RxMessage.StdId <= 0x352)
			{ 
			  ICM_Get_Feedback(RxMessage.StdId, RxData);
			}
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
	CAN_Init(&hcan2);
  /* USER CODE BEGIN 2 */
printf("begin");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
   printf("Angle:%f  Angle_acc:%f\n",Cur_GYRO.Z_Pos,Cur_GYRO.Z_Spd);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
