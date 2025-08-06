#include <mik32_msp.h>
#include <pad_config.h>
#include <gpio.h>
#include <power_manager.h>

#include <gpio_irq.h>
#include <epic.h>
#include <scr1_csr_encoding.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "mik32_dshot.h"
#include "mik32_hal_timer32.h"
#include "mik32_hal_dma.h"


SemaphoreHandle_t xSemaphore;
float mixer[] = {1500.0f, 1500.0f, 1500.0f, 1500.0f};

void SystemClockConfig(void);
static void LED_task(void *param);
static void MPU_task(void *param);

void ext_trap_handler()
{
    HAL_EPIC_Clear(0xFFFFFFFF);
	xSemaphoreGiveFromISR(xSemaphore, NULL);
}

int main()
{
	SystemClockConfig();
	//DSHOT_init(0);
	xTaskCreate (LED_task, "LED_task", 128, ( void * ) 1, tskIDLE_PRIORITY + 1 , NULL );
	xTaskCreate (MPU_task, "MPU_task", 128, ( void * ) 1, tskIDLE_PRIORITY + 2 , NULL );
	xSemaphore = xSemaphoreCreateBinary();
	
	__HAL_PCC_GPIO_2_CLK_ENABLE();
	__HAL_PCC_GPIO_0_CLK_ENABLE();
	HAL_GPIO_PinConfig(GPIO_2, GPIO_PIN_7, HAL_GPIO_MODE_GPIO_OUTPUT, HAL_GPIO_PULL_UP, HAL_GPIO_DS_2MA);
	HAL_GPIO_PinConfig(GPIO_0, GPIO_PIN_8, HAL_GPIO_MODE_GPIO_OUTPUT, HAL_GPIO_PULL_NONE, HAL_GPIO_DS_2MA);
	
	
	__HAL_PCC_EPIC_CLK_ENABLE();
	//HAL_EPIC_MaskLevelSet();
	HAL_IRQ_EnableInterrupts();
	

	
	vTaskStartScheduler();
	

    while (1){
		//HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_7);
	}
}

void SystemClockConfig(void) {
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

static void LED_task(void *param)
{
	while(1)
	{
		for(int i = 0; i < 10000000; i++);
		xSemaphoreGive(xSemaphore);
	}
}

static void MPU_task(void *param)
{

	while(1)
	{
		//DSHOT_send(mixer);
		HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_7);
		for(volatile int i = 0; i < 1000000; i++);
		
	}
	
}

