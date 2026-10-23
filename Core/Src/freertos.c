/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "balance_control.h"
#include "encoder.h"
#include "DRV4950.h"
#include "ICM42688.h"
#include "MahonyAHRS.h"

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
/* USER CODE BEGIN Variables */

extern imu_t imu ;
extern MahonyAHRS_t mahony ;
extern Encoder_t encoder ;
extern Motor_t motorL , motorR ;
extern Balance_t balance ;

/* USER CODE END Variables */
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for UITask */
osThreadId_t UITaskHandle;
const osThreadAttr_t UITask_attributes = {
  .name = "UITask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ControlTaskStart(void *argument);
void UITaskStart(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(ControlTaskStart, NULL, &ControlTask_attributes);

  /* creation of UITask */
  UITaskHandle = osThreadNew(UITaskStart, NULL, &UITask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ControlTaskStart */
/**
  * @brief  Function implementing the ControlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ControlTaskStart */
void ControlTaskStart(void *argument)
{
  /* USER CODE BEGIN ControlTaskStart */
  /* Infinite loop */
    uint32_t last_slow = osKernelGetTickCount();
  for(;;)
  {
      ICM42688_ReadSensorData(&imu) ;
      MahonyAHRS_Update(&mahony , imu.gyro , imu.accle , 0.001) ;
      MahonyAHRS_GetEulerAngles(&mahony , &balance.status) ;
      balance.gyro_pitch_v = imu.gyro[1];
      uint32_t now = osKernelGetTickCount();
      if (now - last_slow >= 20) {
          last_slow = now;
          Encoder_Update(&encoder) ;
          balance.speed_now_left = encoder.speed_left ;
          balance.speed_now_right = encoder.speed_right ;
          Balance_UpdateSlow(&balance) ;
      }
      Balance_UpdateFast(&balance) ;
      Motor_DutySet(&motorL , balance.left_duty) ;
      Motor_DutySet(&motorR ,balance.right_duty) ;
    osDelay(1);
  }
  /* USER CODE END ControlTaskStart */
}

/* USER CODE BEGIN Header_UITaskStart */
/**
* @brief Function implementing the UITask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UITaskStart */
void UITaskStart(void *argument)
{
  /* USER CODE BEGIN UITaskStart */
  /* Infinite loop */
  for(;;)
  {
    osDelay(20);
  }
  /* USER CODE END UITaskStart */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

