/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
extern UART_HandleTypeDef huart1;

osThreadId Task01Handle;
osThreadId Task02Handle;
osThreadId Task03Handle;
osThreadId Task04Handle;


void StartTask01(void const * argument);
void StartTask02(void const * argument);


void MX_FREERTOS_Init(void); 

void MX_FREERTOS_Init(void) 
{

 
  osThreadDef(Task01, StartTask01, osPriorityNormal, 0, 2048);
  Task01Handle = osThreadCreate(osThread(Task01), NULL);
  osThreadDef(Task02, StartTask02, osPriorityNormal, 0, 1024);
  Task02Handle = osThreadCreate(osThread(Task02), NULL);

 
}

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void const * argument)
{

  /* USER CODE BEGIN StartTask01 */
  /* Infinite loop */
  while(1)
  {
    sendCanard();
    receiveCanard();
    spinCanard();
    publishCanard();
    //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
   // HAL_UART_Transmit(&huart1,"go",2,0xffff);
    //showRcpwmonUart();
    //osDelay(100);
  }
  /* USER CODE END StartTask01 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  while(1)
  {
    uint8_t num[5];
    uint8_t num1=12;
    uint16_t value[10];
    float batteyVolt=15.7;
    value[1]=getAdcVaule(1);
    //batteyVolt=value[1]/4069*3.3;
    num1=batteyVolt;
    //gcvt(batteyVolt,num,10);
   // itoa(value[1],num,10);
   // HAL_UART_Transmit(&huart1,num,5,0xffff);
   // HAL_UART_Transmit(&huart1,"-----",5,0xffff);
  //  itoa(value[2],num,10);
   // HAL_UART_Transmit(&huart1,num,5,0xffff);
   // HAL_UART_Transmit(&huart1,"-----",5,0xffff);
    HAL_UART_Transmit(&huart1,&num1,1,0xffff);
    osDelay(100);
    //HAL_UART_Transmit(&huart1,"\n",1,0xffff);
    //printf("gogogo");
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
