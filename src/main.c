

#include <mik32_msp.h>
#include <pad_config.h>
#include <gpio.h>
#include <power_manager.h>

#include <gpio_irq.h>
#include <epic.h>
#include <scr1_csr_encoding.h>
//#include "riscv_csr_encoding.h"

#include "mik32_hal_usart.h"
#include "mik32_hal_irq.h"
#include "mik32_hal_spi.h"
#include "mik32_hal_timer32.h"
#include "mik32_hal_dma.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "mik32_icm42688p.h"
#include "mik32_pid_bf.h"
#include "mik32_mix_bf.h"
#include "dshot.h"
#include "mik32_msp.h"

#define MILLISECONDS configTICK_RATE_HZ/1000
#define XYZ_AXIS_COUNT 3
#define MOTOR_COUNT 4
#define GYRO_DATA_HEAP 100
#define FLIGHT 1

typedef struct gyroDataStorage_s {
	float xData[GYRO_DATA_HEAP];
	float yData[GYRO_DATA_HEAP];
	float zData[GYRO_DATA_HEAP];
	uint16_t actualData;
	uint16_t lastData;
} gyroDataStorage_t;

float pidSums[XYZ_AXIS_COUNT];
uint16_t pidSumLimit;
uint16_t pidSumLimitYaw;
float motorSignals[MOTOR_COUNT];
gyroDataStorage_t gyroDataStorage;
SemaphoreHandle_t xMotorSemaphore; 
TIMER32_HandleTypeDef ht32_2;
TIMER32_CHANNEL_HandleTypeDef ch_1;
TIMER32_CHANNEL_HandleTypeDef ch_2;
TIMER32_CHANNEL_HandleTypeDef ch_3;
TIMER32_CHANNEL_HandleTypeDef ch_4;
DMA_InitTypeDef hd;
DMA_ChannelHandleTypeDef hd_ch_0;
DMA_ChannelHandleTypeDef hd_ch_1;
DMA_ChannelHandleTypeDef hd_ch_2;
DMA_ChannelHandleTypeDef hd_ch_3;
SPI_HandleTypeDef hspi0;
USART_HandleTypeDef husart;
SemaphoreHandle_t xSemaphore;

static void getDataTask(void* param);
static void PIDTask(void* param);
static void mixTask(void* param);
static void DSHOTTask(void* param);

static void MSP_Task(void *param);
void SystemClockConfig(void);
static void LED_Task(void *param);

SemaphoreHandle_t xSemaphore;

void ext_trap_handler()
{
    if (EPIC_CHECK_UART_0()) {
		MSP_ISR();
    }
	if (EPIC_CHECK_DMA()) {
		dshotDMAIRQ();
	}

   HAL_EPIC_Clear(0xFFFFFFFF);
   xSemaphoreGiveFromISR(xSemaphore, NULL); 
}


int main()
{
	SystemClockConfig();
	__HAL_PCC_GPIO_2_CLK_ENABLE();
	HAL_GPIO_PinConfig(GPIO_2, GPIO_PIN_7, HAL_GPIO_MODE_GPIO_OUTPUT, HAL_GPIO_PULL_UP, HAL_GPIO_DS_2MA);
	
#if defined(FLIGHT)
    //xTaskCreate ( getDataTask, "getDataTask", 128, ( void * ) 2, tskIDLE_PRIORITY + 1 , NULL );
#endif    
	xTaskCreate ( PIDTask, "PIDTask", 128, ( void * ) 1, tskIDLE_PRIORITY + 1 , NULL );
	xTaskCreate ( mixTask, "mixTask", 128, ( void * ) 1, tskIDLE_PRIORITY + 1 , NULL );
	xTaskCreate ( DSHOTTask, "DSHOTTask", 128, ( void * ) 1, tskIDLE_PRIORITY + 1 , NULL );
	xTaskCreate ( MSP_Task, "MSP_Task", 128, ( void * ) 1, tskIDLE_PRIORITY + 1 , NULL );
	xTaskCreate ( LED_Task, "LED_Task", 128, ( void * ) 1, tskIDLE_PRIORITY + 1 , NULL );
	xSemaphore = xSemaphoreCreateBinary();
 	xMotorSemaphore = xSemaphoreCreateBinary();
	MSP_Init(&husart);
#if defined(FLIGHT)
	ICM42688P_Init(&hspi0);
#endif 
	pidBFInitDefault();
	mixBfInitDefault();
	pidSumLimit = pidBFGetPidSumLimit();
	pidSumLimitYaw = pidBFGetPidSumLimitYaw();
 	dshotInit(DSHOT150, 
					&ht32_2,
					&ch_1,
					&ch_2,
					&ch_3,
					&ch_4,
					&hd,
					&hd_ch_0,
					&hd_ch_1,
					&hd_ch_2,
					&hd_ch_3); 
	gyroDataStorage.actualData = 0;
	gyroDataStorage.lastData = 0; 
	gyroDataStorage.xData[GYRO_DATA_HEAP] = 0.0f;
	gyroDataStorage.yData[GYRO_DATA_HEAP] = 0.0f;
	gyroDataStorage.zData[GYRO_DATA_HEAP] = 0.0f;
	
	__HAL_PCC_EPIC_CLK_ENABLE();
    HAL_EPIC_MaskLevelSet(HAL_EPIC_UART_0_MASK | HAL_EPIC_DMA_MASK); 
	// 
	//HAL_EPIC_DMA_CHANNELS_MASK - необходимо обсудить с поддержкой микроконтроллера.
	HAL_IRQ_EnableInterrupts();
    vTaskStartScheduler();

    while (1){
	}
}

void SystemClockConfig() {
	PCC_InitTypeDef PCC_OscInit = {0};
    PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
    PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
    PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
    PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
    PCC_OscInit.AHBDivider = 0;
    PCC_OscInit.APBMDivider = 0;
    PCC_OscInit.APBPDivider = 0;
    PCC_OscInit.HSI32MCalibrationValue = 128;
    PCC_OscInit.LSI32KCalibrationValue = 8;
    PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_AUTO;
    PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
    HAL_PCC_Config(&PCC_OscInit);
}

static void getDataTask(void* param) {
	while(1) {
		float gyroData[XYZ_AXIS_COUNT];
		ICM42688P_ReadGyroData(gyroData);
		gyroDataStorage.xData[gyroDataStorage.lastData] = gyroData[0];
		gyroDataStorage.yData[gyroDataStorage.lastData] = gyroData[1];
		gyroDataStorage.zData[gyroDataStorage.lastData] = gyroData[2];
		gyroDataStorage.lastData++;
		if (gyroDataStorage.lastData > GYRO_DATA_HEAP) {
			gyroDataStorage.lastData = 0;
		}
	}
}

static void PIDTask(void* param) {
	float gyroData[XYZ_AXIS_COUNT];
	while(1) {
		gyroData[0] = gyroDataStorage.xData[gyroDataStorage.actualData];
		gyroData[1] = gyroDataStorage.yData[gyroDataStorage.actualData];
		gyroData[2] = gyroDataStorage.zData[gyroDataStorage.actualData];
		if (gyroDataStorage.lastData < gyroDataStorage.actualData) {
			gyroDataStorage.actualData = gyroDataStorage.lastData;
		} else {
			gyroDataStorage.actualData++;
		}
		if (gyroDataStorage.actualData > GYRO_DATA_HEAP) {
			gyroDataStorage.actualData = 0;
		}
		pidController(gyroData, pidSums);
	}
}

static void mixTask(void* param) {
	while(1) {
		if (xSemaphoreTake(xMotorSemaphore, portMAX_DELAY) == pdTRUE) {
			mixTable(pidSums, pidSumLimit, pidSumLimitYaw, motorSignals);
		}
	}
}

static void DSHOTTask(void* param) {
	uint16_t motorSignalsUint16[MOTOR_COUNT];
	uint8_t i;
	while(1) {
		for (i = 0; i < MOTOR_COUNT; i++) {
			motorSignalsUint16[i] = ((motorSignals[i] - (float)((int)motorSignals[i])) > 0.5f ? ((uint16_t)motorSignals[i]) + 1 : (uint16_t)motorSignals[i]) - 1;
		}
		dshotWrite(motorSignalsUint16);
		xSemaphoreGive(xMotorSemaphore);
	}
}

static void MSP_Task(void *param)
{
    while (1)
    {
		if (xSemaphore && xSemaphoreTake(xSemaphore, 100) == pdTRUE)
		{
			MSP_GetMessage();
		}			
    }
}


static void LED_Task(void *param)
{
    while(1)
    {
		vTaskDelay(1000 * MILLISECONDS);
		HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_7);
		xSemaphoreGive(xSemaphore);
    }
}