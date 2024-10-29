/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "usart.h"
#include "rng.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rcl/init_options.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_srvs/srv/set_bool.h>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/twist.h>

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
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;
rcl_ret_t rc;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_publisher_t robot1_publisher;
rcl_publisher_t robot2_publisher;
rcl_publisher_t robot3_publisher;
rcl_publisher_t robot4_publisher;
//rcl_subscription_t subscriber;

geometry_msgs__msg__Point cmd;
geometry_msgs__msg__Twist robot1_cmd_vel;
geometry_msgs__msg__Twist robot2_cmd_vel;
geometry_msgs__msg__Twist robot3_cmd_vel;
geometry_msgs__msg__Twist robot4_cmd_vel;

uint16_t ADC_RawRead[300];
int x = 0;
int y = 0;
int d8 = 0;
int a = 0;
int b = 0;
int c = 0;
int d = 0;
GPIO_PinState prev_button = GPIO_PIN_RESET;
int toggle_a = 0, toggle_b = 0, toggle_c = 0, toggle_d = 0, toggle_d8 = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	if (timer != NULL)
	{
        int a_pressed = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET);
        int b_pressed = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET);
        int c_pressed = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET);
        int d_pressed = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET);

        // Toggle logic - only one button can be active at a time
        if (a_pressed && !toggle_a) {
            toggle_a = 1; toggle_b = toggle_c = toggle_d = toggle_d8 = 0;
        } else if (b_pressed && !toggle_b) {
            toggle_b = 1; toggle_a = toggle_c = toggle_d = toggle_d8 = 0;
        } else if (c_pressed && !toggle_c) {
            toggle_c = 1; toggle_a = toggle_b = toggle_d = toggle_d8 = 0;
        } else if (d_pressed && !toggle_d) {
            toggle_d = 1; toggle_a = toggle_b = toggle_c = toggle_d8 = 0;
        }

        // Set the velocity based on the active toggle
        geometry_msgs__msg__Twist* robots[4] = {&robot1_cmd_vel, &robot2_cmd_vel, &robot3_cmd_vel, &robot4_cmd_vel};

        for (int i = 0; i < 4; ++i) {
            robots[i]->linear.x = 0;
            robots[i]->angular.z = 0;
        }

        if (toggle_a) {
            robot1_cmd_vel.linear.x = cmd.y;
            robot1_cmd_vel.angular.z = cmd.x;
        } else if (toggle_b) {
            robot2_cmd_vel.linear.x = cmd.y;
            robot2_cmd_vel.angular.z = cmd.x;
        } else if (toggle_c) {
            robot3_cmd_vel.linear.x = cmd.y;
            robot3_cmd_vel.angular.z = cmd.x;
        } else if (toggle_d) {
            robot4_cmd_vel.linear.x = cmd.y;
            robot4_cmd_vel.angular.z = cmd.x;
        }
		if (d8 == 1)
		{

		}

		if (x < 1023.75){x = 0.0;}
		else if (x > 3071.25){x = 4095;}
		else {x = 2047.5;}

		if (y < 1023.75){y = 0.0;}
		else if (y > 3071.25){y = 4095;}
		else {y = 2047.5;}

		cmd.x = ((x / 2047.5) - 1.0)* -1.0;
		cmd.y = (y / 2047.5) - 1.0;


		rc = rcl_publish(&robot1_publisher, &robot1_cmd_vel, NULL);
		rc = rcl_publish(&robot2_publisher, &robot2_cmd_vel, NULL);
		rc = rcl_publish(&robot3_publisher, &robot3_cmd_vel, NULL);
		rc = rcl_publish(&robot4_publisher, &robot4_cmd_vel, NULL);
		HAL_IWDG_Refresh(&hiwdg);
		x = ADC_RawRead[0];
		y = ADC_RawRead[1];
	}
}

//void subscription_callback(const void *msgin)
//{
//	const std_msgs__msg__Int32 *sub_msg = (const std_msgs__msg__Int32*) msgin;
//	step = sub_msg->data;
//}

void save_path_client_callback(const void * save_path_resp){
    std_srvs__srv__SetBool_Response * resp = (std_srvs__srv__SetBool_Response *)save_path_resp;

    if (resp->success) {
        printf("SavePath success: %s\n", resp->message.data);
    } else {
        printf("SavePath failed: %s\n", resp->message.data);
    }
}
void ref_client_callback(const void * ref_resp){
	std_srvs__srv__SetBool_Response * resp = (std_srvs__srv__SetBool_Response *)ref_resp;

    if (resp->success) {
        printf("Change Mode success: %s\n", resp->message.data);
    } else {
        printf("Change Mode failed: %s\n", resp->message.data);
    }
}
void mode_client_callback(const void * mode_resp){
	std_srvs__srv__SetBool_Response * resp = (std_srvs__srv__SetBool_Response *)mode_resp;

    if (resp->success) {
        printf("Mode success: %s\n", resp->message.data);
    } else {
        printf("Mode failed: %s\n", resp->message.data);
    }
}

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  // micro-ROS configuration

	rmw_uros_set_custom_transport(
			true,
			(void *) &hlpuart1,
			cubemx_transport_open,
			cubemx_transport_close,
			cubemx_transport_write,
			cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		printf("Error on default allocators (line %d)\n", __LINE__);
	}

	// micro-ROS app

	allocator = rcl_get_default_allocator();

	//create init_options
	init_options = rcl_get_zero_initialized_init_options();
	rc = rcl_init_options_init(&init_options, allocator);
	rc = rcl_init_options_set_domain_id(&init_options, 28);

	rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

	// create node
	rclc_node_init_default(&node, "joy", "", &support);

	// create timer
	rclc_timer_init_default(
			&timer,
			&support,
			RCL_MS_TO_NS(10),
			timer_callback
	);

	// create massage


	//	 create publisher
	rclc_publisher_init_default(
			&robot1_publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist) ,
			"/robot1/cmd_vel"
	);
	// publisher replace service problem
	rclc_publisher_init_default(
			&robot2_publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist) ,
			"/robot2/cmd_vel"
	);
	rclc_publisher_init_default(
			&robot3_publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist) ,
			"/robot3/cmd_vel"
	);
	rclc_publisher_init_default(
			&robot4_publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist) ,
			"/robot4/cmd_vel"
	);

//	// create subscription
//	rclc_subscription_init_default(
//			&subscriber,
//			&node,
//			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//			"cubemx_subscriber"
//	);

	// create client for SavePath service
//	rclc_client_init_default(
//	    &save_path_client,
//	    &node,
//	    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
//	    "/save_path"
//	);
//
//	// create client for Ref service
//	rclc_client_init_default(
//	    &ref_client,
//	    &node,
//	    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
//	    "/ref"
//	);
//
//	// create client for Mode service
//	rclc_client_init_default(
//		&mode_client,
//		&node,
//		ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
//		"/mode"
//	);

	// create executor
	executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, 4, &allocator);
	rclc_executor_add_timer(&executor, &timer);
//	rclc_executor_add_client(&executor, &save_path_client, &save_path_resp, save_path_client_callback);
//	rclc_executor_add_client(&executor, &ref_client, &ref_resp, ref_client_callback);
//	rclc_executor_add_client(&executor, &mode_client, &mode_resp, mode_client_callback);
//	rclc_executor_spin(&executor);

	for(;;)
	{
		osDelay(10);
	    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
	}
  /* USER CODE END 5 */
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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_RNG_Init();
  MX_IWDG_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc2, ADC_RawRead, 300);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
