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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ICM_20948.h"

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//static const uint8_t TMP102_ADDR = 0x52 << 1; // Use 8-bit address
//static const uint8_t REG_TEMP = 0x00;
uint8_t addr = 0b1101001 << 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void i2cscanner() {
//
//	int i, ret;
//
//	sendToUsb((uint8_t*) "Scan start.\n");
//
//	for (i = 1; i < 128; i++) {
//
//		ret = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t) (i << 1), 3, 5);
//
//		if (ret == HAL_OK) /* No ACK Received At That Address */
//
//		{
//
//			sprintf((char*) buffer, "0x%X ", i);
//
//			sendToUsb(buffer);
//
//		}
//
//	}
//
//	sendToUsb((uint8_t*) "Scan done.\n");
//}

void Uprint(char message[]){
	char uart_buf[100];
	int uart_buf_len;
	uart_buf_len = sprintf(uart_buf, message);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
}

int16_t twoComplementToDec(uint16_t val) {
	if ((val & 0x8000) == 0) {
		return val;
	} else {
		val = ~(val) + 1;
		return -val;
	}
}

uint16_t addBinary(uint8_t lowByte, uint8_t highByte) {
	return ((highByte << 8) | lowByte);
}

HAL_StatusTypeDef readReg(int regAddress, uint8_t *buf,
		int dataAmount) {
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Read(&hi2c1, addr, regAddress, 1, buf, dataAmount,
	HAL_MAX_DELAY);
	return ret;
}

int16_t getGyroX() {
	uint8_t buf[2];
	HAL_StatusTypeDef ret;

	ret = readReg(0x33, buf, 2);
	int xGyro = twoComplementToDec(addBinary(buf[1], buf[0]));

	if (ret == HAL_OK) {
		return xGyro / 1;
	} else {
		return -99;
	}
}

int getTemp(){
	uint8_t buf[2];
	HAL_StatusTypeDef ret;

//	ret = HAL_I2C_Mem_Read(&hi2c1, addr, 0x39, 1, buf, 2, HAL_MAX_DELAY);
	ret = readReg(0x39, buf, 2);
	int temp = twoComplementToDec(addBinary(buf[1], buf[0]));

	if (ret == HAL_OK) {
		return ((temp - 21) / 400) + 21;
	} else {
		return -99;
	}

//	uint8_t buf[1];
//	uint8_t buf2[1];
//	uint16_t ans;
//	HAL_StatusTypeDef ret;
//	ret = HAL_I2C_Mem_Read(&hi2c1, addr, 0x3A, 1, buf, 1, HAL_MAX_DELAY);
//	ret = HAL_I2C_Mem_Read(&hi2c1, addr, 0x39, 1, buf2, 1, HAL_MAX_DELAY);
//
//	if (ret == HAL_OK) {
//		ans = ((((buf2[0]<<8) | buf[0]) - 21)/400) + 21;
//	} else {
//		ans = 0;
//	}
//	return ans;
}

HAL_StatusTypeDef configReg(int regAddress, int regSize, int data){
	HAL_StatusTypeDef ret;
	uint8_t temp = data;
	ret = HAL_I2C_Mem_Write(&hi2c1, addr, regAddress, regSize, &temp, regSize, HAL_MAX_DELAY);

	return ret;
}

void configIMU(){
	HAL_StatusTypeDef ret;


//	Clock and sleep settings
	ret = configReg(0x06, 1, 0b00000001);
//	Ensure that accelerometer and gyroscope are both enabled
	ret = configReg(0x07, 1, 0b00000000);
//	Gyro Config
//	ret = configReg(0x01, 1, 0b00110111);
//	ret = configReg(0x02, 1, 0b00000000);
//	Accel Config
//	ret = configReg(0x20, 1, 0b00000001);
//	ret = configReg(0x21, 1, 0b00000000);

//	Mag Config
//	ret = configReg(0x31, 1, 0b00000010);


	if (ret == HAL_OK) {
		Uprint("write worked\r\n");
	} else {
		Uprint("write failed\r\n");
	}

	uint8_t buf[1];
	char str[10];

	ret = HAL_I2C_Mem_Read(&hi2c1, addr, 0x01, 1, buf, 1, HAL_MAX_DELAY);
	if (ret == HAL_OK) {
			Uprint("read worked\r\n");
			sprintf(str, "%d\r\n", buf[0]);
			Uprint(str);
		} else {
			Uprint("read failed\r\n");
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */



  HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, addr, 1, 100);
  if(ret == HAL_OK){
	  Uprint("The device is ready\r\n");
  }
  else {
	  Uprint("The device is not ready. Check cables\r\n");
  }

  configIMU();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//	HAL_StatusTypeDef ret;
//	uint8_t buf[12];
//	int16_t val;
//	float temp_c;

  	ICM_20948 IMU(hi2c1, addr);


	while (1)
	{
    /* USER CODE END WHILE */
//		uint8_t buf[1];
//		uint8_t buf2[1];
		char str[100];
//		ret = HAL_I2C_Mem_Read(&hi2c1, addr, 0x30, 1, buf, 1, HAL_MAX_DELAY);
//		ret = HAL_I2C_Mem_Read(&hi2c1, addr, 0x2F, 1, buf2, 1, HAL_MAX_DELAY);
//		if (ret == HAL_OK) {
////			Uprint("read worked\r\n");
//			uint16_t ans = ((buf2[0]<<8) | buf[0]);
//			sprintf(str, "%d\r\n", ans);
//			Uprint(str);
//		} else {
//			Uprint("read failed\r\n");
//		}

//		sprintf(str, "ACCEL X:%f Y:%f Z:%f\r\n", IMU.getAccelX(), IMU.getAccelY(), IMU.getAccelZ());
//		Uprint(str);
//		sprintf(str, "GYRO X:%d Y:%d Z:%d\r\n", IMU.getGyroX(), IMU.getGyroY(), IMU.getGyroZ());
//		Uprint(str);


		sprintf(str, "%f\r\n", IMU.getPitch());
		Uprint(str);
		HAL_Delay(100);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
			while (1) {
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
