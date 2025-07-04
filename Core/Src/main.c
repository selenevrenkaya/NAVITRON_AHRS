/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"

#include "bno055.h"
#include "neo_m8n.h"
#include "ahrs.h"
#include "ms5611.h"
#include "ekf.h"

//#include "usbd_cdc_if.h"
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
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* sd card functions */
static void write_to_sd_card(float head, float roll, float pitch);
void read_from_sd_card(void);
void clear_sd_card_file(void);
void delete_sd_card_file(void);

static void send_data_uart(UART_HandleTypeDef *huart, tAHRS* ahrs, tBNO055* bno);

FATFS FatFs;
FIL Fil;

/* GPS */
t_circ_buffer circ_buffer = {	.buffer = { 0 },
								.size = circ_buffer_size,
								.write_pos = 0,
								.read_pos = 0
							};

/*
t_gps_gga gga = { 0 };
t_gps_rmc rmc = { 0 };
*/

t_gps gps_data = {	.cbuffer = &circ_buffer				};

unsigned char gga_received[100];
unsigned char rmc_received[100];


/* Barometer */
t_ms5611 ms5611 = {		.hi2c = &hi2c1,
						.temp = 0.00,
						.pressure = 0.00

					};


/* IMU */
tBNO055 bno = {0};


/* AHRS Component */
tAHRS ahrs = { 0 };


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t RATE_10Hz[] =  		{0XB5, 0X62, 0X06, 0X08, 0X06, 0X00, 0X64, 0X00, 0X01, 0X00, 0X01, 0X00, 0X7A, 0X12};

char usb_buffer[100];
char sd_reading_buffer[500];
char sd_writing_buffer[100];


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
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_I2C3_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* init function for barometer */
  ms5611_init();

  /* init function for IMU sensor */
  BNO055_Init(&bno, &hi2c3);

  HAL_Delay(500);

  /* Configure GPS for data rate */
  HAL_UART_Transmit(&huart4, RATE_10Hz, 14, 500);
  HAL_Delay(100);

  if (HAL_UART_GetState(&huart4) == HAL_UART_STATE_READY) {
	  HAL_UART_Receive_IT(&huart4, gps_data.cbuffer->buffer, circ_buffer_size);
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(gps_data.receive_check == true){
		  if(check_sentence(&gps_data, "GGA")){

			  if(parse_gps_data(&gps_data, '*', gga_received)){

				  if(decode_gga(gga_received, &(gps_data.gga)) == 1){

					  set_ahrs_data(&gps_data, &ahrs);
					  gps_data.receive_check = false;

					  /* for serial */
					  /*
					  sprintf(serial_buffer, "%.2f%c, %.2f%c, %02d:%02d:%02d  ", gps_data.gga->location.latitude, gps_data.gga->location.NS,\
							  gps_data.gga->location.longitude, gps_data.gga->location.EW, gps_data.gga->time.hour, \
							  gps_data.gga->time.min, gps_data.gga->time.sec );
						*/

				  }

			  }
		  }
		  else if(check_sentence(&gps_data, "RMC")){

			  if(parse_gps_data(&gps_data, '*', rmc_received)){

				  if(decode_rmc(rmc_received, &(gps_data.rmc)) == 1){

					  set_ahrs_data(&gps_data, &ahrs);
					  gps_data.receive_check = false;

					  /* for serial */
					  /*
					  sprintf(serial_buffer, " %.2f, %.2f, %02d/%02d/%04d ", gps_data.rmc->speed, gps_data.rmc->course, \
							  gps_data.rmc->date.day, gps_data.rmc->date.month, gps_data.rmc->date.year );
						*/

				  }
			  }

		  }
	  }


	  /* Barometer Task */
	  ms5611_task(&ahrs);

	  /* IMU Task */
	  BNO055_Task(&bno, &ahrs);

	  /* Extended Kalman Filter */
	  ekf_task(&gps_data, &bno, &ahrs);


	  /* USB Communication */
	  //sprintf(usb_buffer, " %.2f, %.2f, %.2f ", bno.euler.H, bno.euler.R, bno.euler.P);
	  //sprintf(usb_buffer, " %.2f, %.2f ", bno.eulerf.R, ahrs.telemetry.roll);
	  //CDC_Transmit_FS((uint8_t*)usb_buffer, strlen(usb_buffer));


	  /* Save AHRS data to SD card */
	  write_to_sd_card(ahrs.telemetry.head, ahrs.telemetry.roll, ahrs.telemetry.pitch);	// try increasing baud rate
	  //HAL_Delay(78);

	  //read_from_sd_card();		/* read if you need for checking */

	  //send_data_uart(&huart2, &ahrs, &bno);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/* SD Card Functions */
static void write_to_sd_card(float head, float roll, float pitch) {
    FRESULT FR_Status;
    UINT WWC;  // Write Word Counter

    FR_Status = f_mount(&FatFs, "", 1);
    if (FR_Status != FR_OK) {
        return;
    }

    // Dosyayı açma (her seferinde veri eklemek için append)
    FR_Status = f_open(&Fil, "ahrs_data.txt", FA_OPEN_APPEND | FA_WRITE);
    if (FR_Status != FR_OK) {
        return;
    }

    // Veri formatı: head, roll, pitch
    snprintf(sd_writing_buffer, sizeof(sd_writing_buffer), "%.2f, %.2f, %.2f\n", head, roll, pitch);

    // Veriyi dosyaya yaz ve WWC'yi güncelle
    FR_Status = f_write(&Fil, sd_writing_buffer, strlen(sd_writing_buffer), &WWC);
    if (FR_Status != FR_OK) {
        return;
    }

    // Yazılan byte sayısını WWC ile takip et
    //printf("Written %d bytes to the file.\n", WWC);

    f_close(&Fil);
}




void read_from_sd_card(void) {
    FRESULT FR_Status;
    UINT RWC;  // Read Word Counter

    // Dosyayı açma
    FR_Status = f_open(&Fil, "ahrs_data.txt", FA_READ);
    if (FR_Status != FR_OK) {
        return;
    }

    DWORD file_size = f_size(&Fil);

    // Dosyanın sonuna kadar okuma işlemi
    UINT read_position = 0;
    while (read_position < file_size) {
        // Veriyi okuma (WWC kadar)
        FR_Status = f_read(&Fil, sd_reading_buffer, sizeof(sd_reading_buffer) - 1, &RWC);
        if (FR_Status != FR_OK) {
            return;
        }

        // Eğer okuma işlemi yapılmazsa (dosyanın sonuna geldiyse), çık
        if (RWC == 0) {
            break;
        }

        // Okunan veriyi ekrana yaz
        sd_reading_buffer[RWC] = '\0';  // Null terminator ekleyelim
        //printf("Read data: %s\n", sd_reading_buffer);

        read_position += RWC;

        // Eğer okuma işleminden sonra daha fazla veri yoksa çık
        if (RWC < sizeof(sd_reading_buffer) - 1) {
            break;
        }

    }

    f_close(&Fil);
}



void delete_sd_card_file(void){
	FRESULT FR_Status;

	FR_Status = f_unlink("ahrs_data.txt");

	if (FR_Status != FR_OK){
		return;
	}

}


/* Observe filtering IMU data and GPS position */
static void send_data_uart(UART_HandleTypeDef *huart, tAHRS* ahrs, tBNO055* bno) {
    char tx_buffer[150];
    memset(tx_buffer, '\0', sizeof(tx_buffer));

    snprintf(tx_buffer, sizeof(tx_buffer),
             "R:%.2f,%.2f,P:%.2f,%.2f,Y:%.2f,%.2f,Lat:%.8f,Lon:%.8f,Q:%.3f,%.3f,%.3f,%.3f\n",
             bno->eulerf.R, ahrs->telemetry.roll, bno->eulerf.P, ahrs->telemetry.pitch, bno->eulerf.H, ahrs->telemetry.head,
			 ahrs->position.latitude, ahrs->position.longitude, ahrs->telemetry.q0, ahrs->telemetry.q1,
			 ahrs->telemetry.q2, ahrs->telemetry.q3);

    HAL_UART_Transmit(huart, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);

}


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
