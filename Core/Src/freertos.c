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
#include "gimbal_control.h"
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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Process_Data */
osThreadId_t Process_DataHandle;
const osThreadAttr_t Process_Data_attributes = {
  .name = "Process_Data",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for PID */
osThreadId_t PIDHandle;
const osThreadAttr_t PID_attributes = {
  .name = "PID",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vofa */
osThreadId_t vofaHandle;
const osThreadAttr_t vofa_attributes = {
  .name = "vofa",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for launch */
osThreadId_t launchHandle;
const osThreadAttr_t launch_attributes = {
  .name = "launch",
  .stack_size = 256 * 4,   /* 512B 偏小易溢出导致 launch 卡死(表现为 M2006 不更新+电机疯转) */
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Target_Angle */
osMessageQueueId_t Target_AngleHandle;
const osMessageQueueAttr_t Target_Angle_attributes = {
  .name = "Target_Angle"
};
/* Definitions for Update_PID_para */
osMessageQueueId_t Update_PID_paraHandle;
const osMessageQueueAttr_t Update_PID_para_attributes = {
  .name = "Update_PID_para"
};
/* Definitions for Update_launch_para */
osMessageQueueId_t Update_launch_paraHandle;
const osMessageQueueAttr_t Update_launch_para_attributes = {
  .name = "Update_launch_para"
};
/* Definitions for wake_launch */
osSemaphoreId_t wake_launchHandle;
const osSemaphoreAttr_t wake_launch_attributes = {
  .name = "wake_launch"
};
/* Definitions for wake_launch_motor */
osSemaphoreId_t wake_launch_motorHandle;
const osSemaphoreAttr_t wake_launch_motor_attributes = {
  .name = "wake_launch_motor"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Data_Process(void *argument);
void PID_calc(void *argument);
void VOFA_print(void *argument);
void Launch_Task(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of wake_launch */
  wake_launchHandle = osSemaphoreNew(1, 0, &wake_launch_attributes);

  /* creation of wake_launch_motor */
  wake_launch_motorHandle = osSemaphoreNew(1, 0, &wake_launch_motor_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Target_Angle */
  Target_AngleHandle = osMessageQueueNew (8, sizeof(TargetAngleMessage_t), &Target_Angle_attributes);

  /* creation of Update_PID_para */
  Update_PID_paraHandle = osMessageQueueNew (8, sizeof(PidParameterUpdate_t), &Update_PID_para_attributes);

  /* creation of Update_launch_para */
  Update_launch_paraHandle = osMessageQueueNew (4, sizeof(LaunchParameterUpdate_t), &Update_launch_para_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Process_Data */
  Process_DataHandle = osThreadNew(Data_Process, NULL, &Process_Data_attributes);

  /* creation of PID */
  PIDHandle = osThreadNew(PID_calc, NULL, &PID_attributes);

  /* creation of vofa */
  vofaHandle = osThreadNew(VOFA_print, NULL, &vofa_attributes);

  /* creation of launch */
  launchHandle = osThreadNew(Launch_Task, NULL, &launch_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Data_Process */
/**
* @brief Function implementing the Process_Data thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Data_Process */
__weak void Data_Process(void *argument)
{
  /* USER CODE BEGIN Data_Process */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Data_Process */
}

/* USER CODE BEGIN Header_PID_calc */
/**
* @brief Function implementing the PID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PID_calc */
__weak void PID_calc(void *argument)
{
  /* USER CODE BEGIN PID_calc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PID_calc */
}

/* USER CODE BEGIN Header_VOFA_print */
/**
* @brief Function implementing the vofa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VOFA_print */
__weak void VOFA_print(void *argument)
{
  /* USER CODE BEGIN VOFA_print */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END VOFA_print */
}

/* USER CODE BEGIN Header_Launch_Task */
/**
* @brief Function implementing the launch thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Launch_Task */
__weak void Launch_Task(void *argument)
{
  /* USER CODE BEGIN Launch_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Launch_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

