/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"

#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "bno055.h"
#include "neo_m8n.h"
#include "ahrs.h"
#include "ms5611.h"
#include "ekf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* sd card functions */
static void write_to_sd_card(float head, float roll, float pitch);
void read_from_sd_card(void);
void clear_sd_card_file(void);
void delete_sd_card_file(void);

static void send_data_uart(UART_HandleTypeDef *huart, tAHRS* ahrs, tBNO055* bno);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t RATE_10Hz[] =  		{0XB5, 0X62, 0X06, 0X08, 0X06, 0X00, 0X64, 0X00, 0X01, 0X00, 0X01, 0X00, 0X7A, 0X12};

char usb_buffer[100];
char sd_reading_buffer[500];
char sd_writing_buffer[100];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* SD Card */
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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GPS_Task */
osThreadId_t GPS_TaskHandle;
uint32_t GPS_TaskBuffer[ 128 ];
osStaticThreadDef_t GPS_TaskControlBlock;
const osThreadAttr_t GPS_Task_attributes = {
  .name = "GPS_Task",
  .cb_mem = &GPS_TaskControlBlock,
  .cb_size = sizeof(GPS_TaskControlBlock),
  .stack_mem = &GPS_TaskBuffer[0],
  .stack_size = sizeof(GPS_TaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
uint32_t IMU_TaskBuffer[ 128 ];
osStaticThreadDef_t IMU_TaskControlBlock;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .cb_mem = &IMU_TaskControlBlock,
  .cb_size = sizeof(IMU_TaskControlBlock),
  .stack_mem = &IMU_TaskBuffer[0],
  .stack_size = sizeof(IMU_TaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for Barometer_Task */
osThreadId_t Barometer_TaskHandle;
uint32_t Barometer_TaskBuffer[ 128 ];
osStaticThreadDef_t Barometer_TaskControlBlock;
const osThreadAttr_t Barometer_Task_attributes = {
  .name = "Barometer_Task",
  .cb_mem = &Barometer_TaskControlBlock,
  .cb_size = sizeof(Barometer_TaskControlBlock),
  .stack_mem = &Barometer_TaskBuffer[0],
  .stack_size = sizeof(Barometer_TaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for SD_Card_Task */
osThreadId_t SD_Card_TaskHandle;
uint32_t SD_Card_TaskBuffer[ 128 ];
osStaticThreadDef_t SD_Card_TaskControlBlock;
const osThreadAttr_t SD_Card_Task_attributes = {
  .name = "SD_Card_Task",
  .cb_mem = &SD_Card_TaskControlBlock,
  .cb_size = sizeof(SD_Card_TaskControlBlock),
  .stack_mem = &SD_Card_TaskBuffer[0],
  .stack_size = sizeof(SD_Card_TaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for USB_Task */
osThreadId_t USB_TaskHandle;
uint32_t USB_TaskBuffer[ 128 ];
osStaticThreadDef_t USB_TaskControlBlock;
const osThreadAttr_t USB_Task_attributes = {
  .name = "USB_Task",
  .cb_mem = &USB_TaskControlBlock,
  .cb_size = sizeof(USB_TaskControlBlock),
  .stack_mem = &USB_TaskBuffer[0],
  .stack_size = sizeof(USB_TaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for EKF_Task */
osThreadId_t EKF_TaskHandle;
uint32_t EKF_TaskBuffer[ 128 ];
osStaticThreadDef_t EKF_TaskControlBlock;
const osThreadAttr_t EKF_Task_attributes = {
  .name = "EKF_Task",
  .cb_mem = &EKF_TaskControlBlock,
  .cb_size = sizeof(EKF_TaskControlBlock),
  .stack_mem = &EKF_TaskBuffer[0],
  .stack_size = sizeof(EKF_TaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime6,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

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

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartGPSTask(void *argument);
void StartIMUTask(void *argument);
void StartBarometerTask(void *argument);
void StartSDCardTask(void *argument);
void StartUSBCommTask(void *argument);
void StartEKFTask(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of GPS_Task */
  GPS_TaskHandle = osThreadNew(StartGPSTask, NULL, &GPS_Task_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(StartIMUTask, NULL, &IMU_Task_attributes);

  /* creation of Barometer_Task */
  Barometer_TaskHandle = osThreadNew(StartBarometerTask, NULL, &Barometer_Task_attributes);

  /* creation of SD_Card_Task */
  SD_Card_TaskHandle = osThreadNew(StartSDCardTask, NULL, &SD_Card_Task_attributes);

  /* creation of USB_Task */
  USB_TaskHandle = osThreadNew(StartUSBCommTask, NULL, &USB_Task_attributes);

  /* creation of EKF_Task */
  EKF_TaskHandle = osThreadNew(StartEKFTask, NULL, &EKF_Task_attributes);

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

/* USER CODE BEGIN Header_StartGPSTask */
/**
* @brief Function implementing the GPS_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGPSTask */
void StartGPSTask(void *argument)
{
  /* USER CODE BEGIN StartGPSTask */

	  /* Configure GPS for data rate */
	  HAL_UART_Transmit(&huart4, RATE_10Hz, 14, 500);
	  HAL_Delay(100);

	  if (HAL_UART_GetState(&huart4) == HAL_UART_STATE_READY) {
		  HAL_UART_Receive_IT(&huart4, gps_data.cbuffer->buffer, circ_buffer_size);
	  }

  /* Infinite loop */
  for(;;)
  {

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

    osDelay(1);
  }
  /* USER CODE END StartGPSTask */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */

	  /* init function for IMU sensor */
	  BNO055_Init(&bno, &hi2c3);

  /* Infinite loop */
  for(;;)
  {

	  /* IMU Task */
	  BNO055_Task(&bno, &ahrs);

    osDelay(1);
  }
  /* USER CODE END StartIMUTask */
}

/* USER CODE BEGIN Header_StartBarometerTask */
/**
* @brief Function implementing the Barometer_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBarometerTask */
void StartBarometerTask(void *argument)
{
  /* USER CODE BEGIN StartBarometerTask */

	  /* init function for barometer */
	  ms5611_init();

  /* Infinite loop */
  for(;;)
  {

	  /* Barometer Task */
	  ms5611_task(&ahrs);

    osDelay(1);
  }
  /* USER CODE END StartBarometerTask */
}

/* USER CODE BEGIN Header_StartSDCardTask */
/**
* @brief Function implementing the SD_Card_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSDCardTask */
void StartSDCardTask(void *argument)
{
  /* USER CODE BEGIN StartSDCardTask */
  /* Infinite loop */
  for(;;)
  {

	  /* Save AHRS data to SD card */
	  write_to_sd_card(ahrs.telemetry.head, ahrs.telemetry.roll, ahrs.telemetry.pitch);	// try increasing baud rate
	  //HAL_Delay(78);

	  //read_from_sd_card();		/* read if you need for checking */

    osDelay(1);
  }
  /* USER CODE END StartSDCardTask */
}

/* USER CODE BEGIN Header_StartUSBCommTask */
/**
* @brief Function implementing the USB_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUSBCommTask */
void StartUSBCommTask(void *argument)
{
  /* USER CODE BEGIN StartUSBCommTask */
  /* Infinite loop */
  for(;;)
  {

	  /* USB Communication */
	  //sprintf(usb_buffer, " %.2f, %.2f, %.2f ", bno.euler.H, bno.euler.R, bno.euler.P);
	  //sprintf(usb_buffer, " %.2f, %.2f ", ms5611.temp, ms5611.pressure);
	  //CDC_Transmit_FS((uint8_t*)usb_buffer, strlen(usb_buffer));

	  send_data_uart(&huart2, &ahrs, &bno);

    osDelay(1);
  }
  /* USER CODE END StartUSBCommTask */
}

/* USER CODE BEGIN Header_StartEKFTask */
/**
* @brief Function implementing the EKF_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEKFTask */
void StartEKFTask(void *argument)
{
  /* USER CODE BEGIN StartEKFTask */
  /* Infinite loop */
  for(;;)
  {

	  /* Extended Kalman Filter */
	  ekf_task(&gps_data, &ahrs);

    osDelay(1);
  }
  /* USER CODE END StartEKFTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

