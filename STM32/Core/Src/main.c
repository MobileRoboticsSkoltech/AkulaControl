/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Akula.h"
#include "usbd_cdc_if.h"
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
        if (htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            if (gFirstCapturedLeft == false) {
                gFirstValLeft = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
                gFirstCapturedLeft = true;  // set the first captured as true
            } else {
                gSecondValLeft = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value

                if (gSecondValLeft > gFirstValLeft) {
                    gSignalDiffLeft = gSecondValLeft - gFirstValLeft;
                } else if (gFirstValLeft > gSecondValLeft) {
                    gSignalDiffLeft = (0xffffffff - gFirstValLeft) + gSecondValLeft;
                }

                double ReferenceClock = (double)ENCODER_FREQ / ENCODER_PRESCALER;

                gFrequencyLeft = ReferenceClock / (double)gSignalDiffLeft;

                __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
                gFirstCapturedLeft = false; // set it back to false
            }
        }
    } else if (htim == &htim5) {
        if (htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            if (gFirstCapturedRight == false) {
                gFirstValRight = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
                gFirstCapturedRight = true;  // set the first captured as true
            } else {
                gSecondValRight = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value

                if (gSecondValRight > gFirstValRight) {
                    gSignalDiffRight = gSecondValRight - gFirstValRight;
                } else if (gFirstValRight > gSecondValRight) {
                    gSignalDiffRight = (0xffffffff - gFirstValRight) + gSecondValRight;
                }

                double ReferenceClock = (double) ENCODER_FREQ / ENCODER_PRESCALER;

                gFrequencyRight = ReferenceClock / (double)gSignalDiffRight;

                __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
                gFirstCapturedRight = false; // set it back to false
            }
        }
    }

    gSendEncoderCounter++;

    if (gSendEncoderCounter % 100 == 0) {
        uint32_t WriteTag = ENCODER;
        uint8_t SendResult;

        memcpy(WriteBuffer, &WriteTag, 4);
        memcpy(WriteBuffer + 4, &gFrequencyLeft, 8);
        memcpy(WriteBuffer + 12, &gFrequencyRight, 8);

        do {
            SendResult = CDC_Transmit_FS(WriteBuffer, PACKET_SIZE);
        } while (SendResult == USBD_BUSY);
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

//    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
//    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

    uint32_t WriteTag           = INVALID;
    uint32_t ReadTag            = INVALID;

    int32_t LeftPWM;
    int32_t RightPWM;

    int LeftMinus = 1;
    int RightMinus = 1;

    uint32_t CurrentTime;
    uint8_t SendResult;
    uint32_t EncoderSendCounter = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (gRunning) {
        if (!gConnected) {
            WriteTag = REQUEST_CONN;
            memcpy(WriteBuffer, &WriteTag, 4);
            CDC_Transmit_FS(WriteBuffer, PACKET_SIZE);
            HAL_Delay(2000);

            if (readSerial(ReadBuffer, PACKET_SIZE)) {
                memcpy(&ReadTag, ReadBuffer, 4);

                if (ReadTag == PING) {
                    WriteTag = PING;
                    memcpy(WriteBuffer, &WriteTag, 4);
                    CDC_Transmit_FS(WriteBuffer, PACKET_SIZE);
                    WriteTag = INVALID;

                    gConnected = 1;
                } else {
                    WriteTag = INVALID;
                    memcpy(WriteBuffer, &WriteTag, 4);
                    memcpy(WriteBuffer + 4, &ReadTag, 4);
                    CDC_Transmit_FS(WriteBuffer, PACKET_SIZE);

                    clearBuffer();
                }

                ReadTag = INVALID;
            }
        } else {
            if (readSerial(ReadBuffer, PACKET_SIZE)) {
                memcpy(&ReadTag, ReadBuffer, 4);

                switch (ReadTag) {
                    case PING:
                        CurrentTime = HAL_GetTick();
                        WriteTag = PING;
                        memcpy(WriteBuffer, &WriteTag, 4);

                        do {
                            SendResult = CDC_Transmit_FS(WriteBuffer, PACKET_SIZE);
                        } while (SendResult == USBD_BUSY && (HAL_GetTick() - CurrentTime) < TimeoutMs);

                        WriteTag = INVALID;

                        break;
                    case JOYSTICK_COORDS:
                        WriteTag = JOYSTICK_COORDS;
                        memcpy(WriteBuffer, &WriteTag, 4);

                        memcpy(&LeftPWM, ReadBuffer + 4, 4);
                        memcpy(&RightPWM, ReadBuffer + 8, 4);

                        if (LeftPWM > MAX_PWM) {
                            LeftPWM = MAX_PWM;
                        }

                        if (LeftPWM < -MAX_PWM) {
                            LeftPWM = -MAX_PWM;
                        }

                        if (RightPWM > MAX_PWM) {
                            RightPWM = MAX_PWM;
                        }

                        if (RightPWM < -MAX_PWM) {
                            RightPWM = -MAX_PWM;
                        }

                        //----------//

                        if (LeftPWM < 0) {
                            if (LeftMinus == 0) {
                                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
                                LeftMinus = 1;
                            }

                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, -LeftPWM);
                        }

                        if (LeftPWM > 0) {
                            if (LeftMinus == 1) {
                                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
                                LeftMinus = 0;
                            }

                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, LeftPWM);
                        }

                        if (RightPWM < 0) {
                            if (RightMinus == 0) {
                                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
                                RightMinus = 1;
                            }

                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -RightPWM);
                        }

                        if (RightPWM > 0) {
                            if (RightMinus == 1) {
                                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
                                RightMinus = 0;
                            }

                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, RightPWM);
                        }

                        //----------//

                        if (LeftPWM == 0) {
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                        }

                        if (RightPWM == 0) {
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
                        }

                        //----------//

                        do {
                            SendResult = CDC_Transmit_FS(WriteBuffer, PACKET_SIZE);
                        } while (SendResult == USBD_BUSY && (HAL_GetTick() - CurrentTime) < TimeoutMs);

                        WriteTag = INVALID;

                        break;
                    case LATENCY:
                        WriteTag = LATENCY;
                        memcpy(WriteBuffer, &WriteTag, 4);

                        do {
                            SendResult = CDC_Transmit_FS(WriteBuffer, PACKET_SIZE);
                        } while (SendResult == USBD_BUSY && (HAL_GetTick() - CurrentTime) < TimeoutMs);

                        WriteTag = INVALID;

                        break;
                    case STOP:
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

                        break;
                    case SHUTDOWN:
                        gRunning = 0;
                        break;
                    default:
                        break;
                }

                ReadTag = INVALID;
            }

            if (HAL_GetTick() - CurrentTime > TimeoutMs) {
                gConnected = 0;
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

                continue;
            }

            uint32_t WriteTag = ENCODER;
            uint8_t SendResult;

            gFrequencyLeft = ((TIM2->CNT)>>2);
            gFrequencyRight = ((TIM5->CNT)>>2);

            memcpy(WriteBuffer, &WriteTag, 4);
            memcpy(WriteBuffer + 4, &gFrequencyLeft, 8);
            memcpy(WriteBuffer + 12, &gFrequencyRight, 8);

            if (EncoderSendCounter % 100 == 0) {
                do {
                    SendResult = CDC_Transmit_FS(WriteBuffer, PACKET_SIZE);
                } while (SendResult == USBD_BUSY && (HAL_GetTick() - CurrentTime) < TimeoutMs);
            }

            EncoderSendCounter++;
        }
    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 280-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 280-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD4 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
