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
#include "MPU6050.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846 //pi value with high precision
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
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
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t IMU_ADDR = 0x68 << 1; //address of MPU6050, and shift left by 1 bit
IMU_Data imu;
float pitch = 0, roll = 0;
char sbuf[100];
uint32_t millisOld, millisNow; //time value
float dt; // time elapse
float gpitch = 0;
float groll = 0;
float gyaw = 0;
float alpha = 0.95;
float pitch_CF = 0;
float roll_CF = 0;
//float roll_KF;
//float pitch_KF;
//float prev_roll_KF = 0.0;
//float prev_pitch_KF = 0.0;
//float KG_roll;
//float KG_pitch;
//float roll_var_g = 4;
//float pitch_var_g = 4;
//float var_gyro = 16.0;
//float var_acc = 9.0;
//int gyro_variance = 16;
//int accel_variance = 9;
//float kalman_gain = 0;
/* USER CODE END 0 */

//uint8_t IMU_ADDR = 0x68 << 1;  // MPU6050 I2C address
//IMU_Data imu;  // Structure to store sensor data
//float pitch = 0, roll = 0;  // Variables for pitch and roll
//char sbuf[100];  // Buffer for UART transmission
//uint32_t millisOld, millisNow;  // Time value
//float dt;  // Time difference between measurements

// Kalman Filter variables
float roll_KF = 0, pitch_KF = 0;  // Kalman filtered pitch and roll (final corrected values)
float roll_gyro_pred = 0, pitch_gyro_pred = 0;  // Gyroscope predicted roll and pitch
float P_roll = 4.0, P_pitch = 4.0;  // Initial uncertainty (variance) for roll and pitch
float Q_gyro = 16.0;  // Gyroscope noise variance
float R_acc = 9.0;  // Accelerometer noise variance
float K_roll, K_pitch;  // Kalman gains for roll and pitch


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	/* uint8_t sbuf[15] = "Hello World!\n\r";
	uint8_t *OLED_buf; [Lab 1] */

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
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  uint8_t status = IMU_Initialise(&imu, &hi2c2, &huart3);

  /*OLED_Init();
  OLED_ShowString(10, 5, "SC2104/CE3002"); //show message on OLED display at line 10

  OLED_buf = "Lab 1"; //another way to show message through buffer
  OLED_ShowString(40, 30, OLED_buf); //another message at line 40
  OLED_Refresh_Gram(); [Lab1] */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Read Accelerometer data
	  IMU_AccelRead(&imu);  // Call function to read accelerometer values

	  // Calculate Pitch and Roll
	  pitch = atan2(imu.acc[0], imu.acc[2]) * 180 / M_PI;
	  roll = atan2(imu.acc[1], imu.acc[2]) * 180 / M_PI;

	  //read Gyroscope
	  IMU_GyroRead(&imu);

	  millisNow = HAL_GetTick(); // get the current time
	  dt = (millisNow - millisOld)*0.001; // time elapsed in millisecond
	  millisOld = millisNow; // store the current time for next round

	  gpitch = gpitch + imu.gyro[1]*dt; //current value + change
	  groll = groll + imu.gyro[0]*dt;
	  gyaw = gyaw + imu.gyro[2]*dt;

	  //complementary filter pitch and roll
	  pitch_CF = (1 - alpha) * pitch + alpha * (pitch_CF + imu.gyro[0] * dt);
	  roll_CF = (1 - alpha) * roll + alpha * (roll_CF + imu.gyro[1] * dt);

	  //Kalman Filter

	  roll_KF += imu.gyro[0] * dt;
	  pitch_KF += imu.gyro[1] * dt;
	  P_roll += Q_gyro * dt * dt;
	  P_pitch += Q_gyro * dt * dt;
	  K_roll = P_roll / (P_roll + R_acc);
	  K_pitch = P_pitch / (P_pitch + R_acc);
	  roll_KF = roll_KF + K_roll * (roll - roll_KF);  // Correct roll estimate
	  pitch_KF = pitch_KF + K_pitch * (pitch - pitch_KF);  // Correct pitch estimate
	  P_roll = (1 - K_roll) * P_roll;  // Update roll uncertainty
	  P_pitch = (1 - K_pitch) * P_pitch;  // Update pitch uncertainty





	  //	// Kalman filter: might not be correct, check again
	  	//	// step 1
	  	//	roll_KF = roll_KF + imu.gyro[0] * dt;
	  	//	pitch_KF = pitch_KF + imu.gyro[1] * dt;
	  	//
	  	//	// step 3
	  	//	roll_var_g = roll_var_g + dt * dt * var_gyro;
	  	//	pitch_var_g = pitch_var_g + dt * dt * var_gyro;
	  	//
	  	//	// step 4
	  	//	KG_roll = roll_var_g / (roll_var_g + var_acc);
	  	//	KG_pitch = pitch_var_g / (pitch_var_g + var_acc);
	  	//
	  	//	roll_KF = roll_KF + KG_roll * (acc_roll - roll_KF);
	  	//	pitch_KF = pitch_KF + KG_pitch * (acc_pitch -  pitch_KF);
	  	//
	  	//	roll_var_g = (1 - KG_roll) * roll_var_g;
	  	//	pitch_var_g = (1 - KG_pitch) * pitch_var_g;

	  	//sprintf(sbuf, "%5.2f, %5.2f, %5.2f, %5.2f, %5.2f\r\n", pitch, roll, gpitch, groll, gyaw);
	  	sprintf(sbuf, "%5.2f, %5.2f, %5.2f, %5.2f, %5.2f\r\n", pitch, roll, pitch_CF, roll_CF, gyaw);// For Complementary
	  	//sprintf(sbuf, "%5.2f, %5.2f, %5.2f, %5.2f, %5.2f\r\n", pitch, roll, roll_KF, pitch_KF, gyaw);// For Kalmain
	  	 //sprintf(sbuf, "%5.2f, %5.2f, %5.2f\r\n", gyro_pitch, gyro_roll, gyro_yaw);
	  	// sprintf(sbuf, "%5.2f, %5.2f\r\n", acc_pitch, acc_roll);
	  	// sprintf(sbuf, "%5.2f, %5.2f, %5.2f\r\n", acc_gyro_pitch, acc_gyro_roll, acc_roll);
	  	// sprintf(sbuf, "%5.2f, %5.2f\r\n", KG_roll, KG_pitch);
	  // Format the output string and send it over UART
	  //sprintf(sbuf, "Pitch: %5.2f, Roll: %5.2f, Pitch_CF: %5.2f, Roll_CF: %5.2f, Gyro_Pitch: %5.2f, Gyro_Roll: %5.2f, Yaw: %5.2f\r\n",
	  //        pitch, roll, pitch_CF, roll_CF, gpitch, groll, gyaw);
	  //HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, strlen(sbuf), HAL_MAX_DELAY);


	  	HAL_UART_Transmit(&huart3, sbuf, strlen(sbuf), HAL_MAX_DELAY);



//	 //kalman filter implementation
//	  gyro_variance = gyro_variance + (dt*dt) * gyro_variance;
//	  kalman_gain = gyro_variance / (gyro_variance + accel_variance);
//	  // update pitch and roll angles of gyroscope
//	  gpitch = gpitch + kalman_gain * (pitch - gpitch);
//	  grow = grow + kalman_gain * (roll - grow);
//	  //update uncertainty of gyro readings
//	  gyro_variance = (1 - kalman_gain) * gyro_variance;
//
//
//	  //sprintf(sbuf, "%.2f,%.2f,%.2f\r\n", grow, gpitch, gyaw);  // comma-separated values
//	  //HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, strlen(sbuf), HAL_MAX_DELAY);
//	  //HAL_Delay(500);
//	  sprintf(sbuf, "%.2f,%.2f,%.2f,%.2f,%.2f\r\n", pitch_CF, roll_CF, grow, gpitch, gyaw);  // comma-separated values
//	  HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, strlen(sbuf), HAL_MAX_DELAY);
//
//
//
//	  /* HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
//  	   HAL_Delay(1000); //1000 msec
//  	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
//  	   HAL_Delay(1000);
//
//  	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
//  	   HAL_Delay(1000); //1000 msec
//  	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
//  	   HAL_Delay(1000);
//
//  	   HAL_UART_Transmit(&huart3, sbuf, sizeof(sbuf), HAL_MAX_DELAY); [Lab1] */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DC_Pin|RESET__Pin|SDIN_Pin|SCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DC_Pin RESET__Pin SDIN_Pin SCLK_Pin */
  GPIO_InitStruct.Pin = DC_Pin|RESET__Pin|SDIN_Pin|SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin LED3_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
