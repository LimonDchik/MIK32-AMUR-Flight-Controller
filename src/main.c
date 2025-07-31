
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

#include "mik32_hal_timer32.h"
#include "mik32_hal_dma.h"

TIMER32_HandleTypeDef htimer32_1;
TIMER32_CHANNEL_HandleTypeDef htimer32_channel0;
TIMER32_CHANNEL_HandleTypeDef htimer32_channel1;
DMA_InitTypeDef hdma;
DMA_ChannelHandleTypeDef hdma_ch0;
DMA_ChannelHandleTypeDef hdma_ch1;
SemaphoreHandle_t xSemaphore;

void Timer32_1_Init(void);
void SystemClockConfig(void);
void DMA_CH0_Init(DMA_InitTypeDef *hdma);
void DMA_Init(void);
void dshotPrepareDMAbuffer(uint32_t* motorDMAbuffer, uint16_t value);
uint16_t dshotPreparePacket(uint16_t value);
uint16_t dshotRound(float floatNum);
static void LED_task(void *param);
static void DSHOT_task(void *param);


uint32_t OCR_value = 2;
bool OCR_vector = true;
uint8_t INT_counter = 0;
uint32_t OCR_codes_Ch0[] = {0, 87, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 0};
uint32_t OCR_codes_Ch1[] = {0, 87, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 0};
//void ext_trap_handler()
//{
/*     if (EPIC_CHECK_UART_0()) {
		MSP_ISR();
    }
	if (EPIC_CHECK_DMA()) {
		dshotDMAIRQ();
	}

   HAL_EPIC_Clear(0xFFFFFFFF);
   xSemaphoreGiveFromISR(xSemaphore, NULL); */ 
//}

void ext_trap_handler()
{
     if (EPIC_CHECK_DMA())
    {
		if (HAL_DMA_GetChannelIrq(&hdma_ch0)) {
			HAL_Timer32_Channel_Disable(&htimer32_channel0);
		}
		if (HAL_DMA_GetChannelIrq(&hdma_ch1)) {
			HAL_Timer32_Channel_Disable(&htimer32_channel1);
		}
		//HAL_GPIO_WritePin(GPIO_2, GPIO_PIN_7, GPIO_PIN_HIGH);
		
/* 		HAL_Timer32_Stop(&htimer32_1);
		HAL_Timer32_Channel_Disable(&htimer32_channel0);
		HAL_Timer32_Channel_Disable(&htimer32_channel1);
		HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_7); */
		
		HAL_DMA_ClearLocalIrq(&hdma); 
    } 
/* 	if (EPIC_CHECK_TIMER32_1())
    {
		//HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_7);
		if (INT_counter == 4) {

			
			//INT_counter = 0;
		} else {
			//INT_counter++;
		}
		HAL_TIMER32_INTERRUPTFLAGS_CLEAR(&htimer32_1);
		HAL_GPIO_WritePin(GPIO_2, GPIO_PIN_7, GPIO_PIN_HIGH);
		
    } */
    HAL_EPIC_Clear(0xFFFFFFFF);
	xSemaphoreGiveFromISR(xSemaphore, NULL);
}

int main()
{
	SystemClockConfig();
	
	xTaskCreate (LED_task, "LED_task", 128, ( void * ) 1, tskIDLE_PRIORITY + 1 , NULL );
	xTaskCreate (DSHOT_task, "DSHOT_task", 128, ( void * ) 1, tskIDLE_PRIORITY + 2 , NULL );
	xSemaphore = xSemaphoreCreateBinary();
	
	__HAL_PCC_GPIO_2_CLK_ENABLE();
	__HAL_PCC_GPIO_0_CLK_ENABLE();
	HAL_GPIO_PinConfig(GPIO_2, GPIO_PIN_7, HAL_GPIO_MODE_GPIO_OUTPUT, HAL_GPIO_PULL_UP, HAL_GPIO_DS_2MA);
	HAL_GPIO_PinConfig(GPIO_0, GPIO_PIN_8, HAL_GPIO_MODE_GPIO_OUTPUT, HAL_GPIO_PULL_NONE, HAL_GPIO_DS_2MA);
	
	Timer32_1_Init();
	
	DMA_Init();
	//HAL_DMA_LocalIRQEnable(&hdma_ch0, DMA_IRQ_ENABLE);
	//HAL_DMA_LocalIRQEnable(&hdma_ch1, DMA_IRQ_ENABLE);
	
	__HAL_PCC_EPIC_CLK_ENABLE();
	//HAL_EPIC_MaskLevelSet(HAL_EPIC_DMA_MASK); 
	//HAL_EPIC_MaskLevelSet(HAL_EPIC_TIMER32_1_MASK); 
	HAL_IRQ_EnableInterrupts();
	
    
    HAL_Timer32_Value_Clear(&htimer32_1);
    
	//HAL_Timer32_PWM_Start_IT(&htimer32_1, &htimer32_channel0);
	
	
	int top = 640;
	//top = 213 => f = 150 kHz
	//OCR = 160 => bit = 1
	//OCR = 80 => bit = 0
	HAL_Timer32_Top_Set(&htimer32_1, 213);
    HAL_Timer32_Channel_OCR_Set(&htimer32_channel0, 80);
	HAL_Timer32_Channel_OCR_Set(&htimer32_channel1, 80);
    HAL_Timer32_Value_Clear(&htimer32_1);

	HAL_GPIO_WritePin(GPIO_0, GPIO_PIN_8, GPIO_PIN_HIGH);
	
	uint16_t num0 = 0b0000011100001111;
	float num1 = 1900.319;
	dshotPrepareDMAbuffer(OCR_codes_Ch0, num0);
	dshotPrepareDMAbuffer(OCR_codes_Ch1, dshotRound(num1));
	
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

void Timer32_1_Init(void)
{
    htimer32_1.Instance = TIMER32_1;
    htimer32_1.Top = 213;
    htimer32_1.State = TIMER32_STATE_DISABLE;
    htimer32_1.Clock.Source = TIMER32_SOURCE_PRESCALER;
    htimer32_1.Clock.Prescaler = 0;
    htimer32_1.InterruptMask = 0;//TIMER32_INT_OC_M(TIMER32_CHANNEL_0);
    htimer32_1.CountMode = TIMER32_COUNTMODE_FORWARD;
    HAL_Timer32_Init(&htimer32_1);

    htimer32_channel0.TimerInstance = htimer32_1.Instance;
    htimer32_channel0.ChannelIndex = TIMER32_CHANNEL_0;
    htimer32_channel0.PWM_Invert = TIMER32_CHANNEL_NON_INVERTED_PWM;
    htimer32_channel0.Mode = TIMER32_CHANNEL_MODE_PWM;
    htimer32_channel0.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    htimer32_channel0.OCR = 80;
    htimer32_channel0.Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(&htimer32_channel0);
	
	htimer32_channel1.TimerInstance = htimer32_1.Instance;
    htimer32_channel1.ChannelIndex = TIMER32_CHANNEL_1;
    htimer32_channel1.PWM_Invert = TIMER32_CHANNEL_NON_INVERTED_PWM;
    htimer32_channel1.Mode = TIMER32_CHANNEL_MODE_PWM;
    htimer32_channel1.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    htimer32_channel1.OCR = 80;
    htimer32_channel1.Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(&htimer32_channel1);
}

void DMA_CH0_Init(DMA_InitTypeDef *hdma)
{
    hdma_ch0.dma = hdma;

    /* Настройки канала */
    hdma_ch0.ChannelInit.Channel = DMA_CHANNEL_0;
    hdma_ch0.ChannelInit.Priority = DMA_CHANNEL_PRIORITY_VERY_HIGH;

    hdma_ch0.ChannelInit.ReadMode = DMA_CHANNEL_MODE_MEMORY;
    hdma_ch0.ChannelInit.ReadInc = DMA_CHANNEL_INC_ENABLE;
    hdma_ch0.ChannelInit.ReadSize = DMA_CHANNEL_SIZE_WORD; /* data_len должно быть кратно read_size */
    hdma_ch0.ChannelInit.ReadBurstSize = 2;                /* read_burst_size должно быть кратно read_size */
    hdma_ch0.ChannelInit.ReadRequest = DMA_CHANNEL_TIMER32_1_REQUEST;
    hdma_ch0.ChannelInit.ReadAck = DMA_CHANNEL_ACK_DISABLE;

    hdma_ch0.ChannelInit.WriteMode = DMA_CHANNEL_MODE_PERIPHERY;
    hdma_ch0.ChannelInit.WriteInc = DMA_CHANNEL_INC_DISABLE;
    hdma_ch0.ChannelInit.WriteSize = DMA_CHANNEL_SIZE_WORD; /* data_len должно быть кратно write_size */
    hdma_ch0.ChannelInit.WriteBurstSize = 2;                /* write_burst_size должно быть кратно read_size */
    hdma_ch0.ChannelInit.WriteRequest = DMA_CHANNEL_TIMER32_1_REQUEST;
    hdma_ch0.ChannelInit.WriteAck = DMA_CHANNEL_ACK_ENABLE;
	
	
	hdma_ch1.dma = hdma;

    /* Настройки канала */
    hdma_ch1.ChannelInit.Channel = DMA_CHANNEL_1;
    hdma_ch1.ChannelInit.Priority = DMA_CHANNEL_PRIORITY_VERY_HIGH;

    hdma_ch1.ChannelInit.ReadMode = DMA_CHANNEL_MODE_MEMORY;
    hdma_ch1.ChannelInit.ReadInc = DMA_CHANNEL_INC_ENABLE;
    hdma_ch1.ChannelInit.ReadSize = DMA_CHANNEL_SIZE_WORD; /* data_len должно быть кратно read_size */
    hdma_ch1.ChannelInit.ReadBurstSize = 2;                /* read_burst_size должно быть кратно read_size */
    hdma_ch1.ChannelInit.ReadRequest = DMA_CHANNEL_TIMER32_1_REQUEST;
    hdma_ch1.ChannelInit.ReadAck = DMA_CHANNEL_ACK_DISABLE;

    hdma_ch1.ChannelInit.WriteMode = DMA_CHANNEL_MODE_PERIPHERY;
    hdma_ch1.ChannelInit.WriteInc = DMA_CHANNEL_INC_DISABLE;
    hdma_ch1.ChannelInit.WriteSize = DMA_CHANNEL_SIZE_WORD; /* data_len должно быть кратно write_size */
    hdma_ch1.ChannelInit.WriteBurstSize = 2;                /* write_burst_size должно быть кратно read_size */
    hdma_ch1.ChannelInit.WriteRequest = DMA_CHANNEL_TIMER32_1_REQUEST;
    hdma_ch1.ChannelInit.WriteAck = DMA_CHANNEL_ACK_ENABLE;
}

void DMA_Init(void)
{

    /* Настройки DMA */
    hdma.Instance = DMA_CONFIG;
    hdma.CurrentValue = DMA_CURRENT_VALUE_ENABLE;
    if (HAL_DMA_Init(&hdma) != HAL_OK)
    {
       // xprintf("DMA_Init Error\n");
    }

    /* Инициализация канала */
    DMA_CH0_Init(&hdma);

}

static void LED_task(void *param)
{
	while(1)
	{
		for(int i = 0; i < 10000000; i++);
		xSemaphoreGive(xSemaphore);
	}
}

static void DSHOT_task(void *param)
{
	HAL_Timer32_Channel_Enable(&htimer32_channel0);
	HAL_Timer32_Channel_Enable(&htimer32_channel1);
	while(1)
	{
		//HAL_Timer32_InterruptMask_Set(&htimer32_1, TIMER32_INT_OC_M(htimer32_channel0.ChannelIndex));

		//HAL_Timer32_Compare_Start_IT(&htimer32_1, &htimer32_channel0);
		HAL_DMA_Start(&hdma_ch0, (void *)&OCR_codes_Ch0, (void *)&htimer32_channel0.Instance->OCR, sizeof(OCR_codes_Ch0) - 1);
		HAL_DMA_Start(&hdma_ch1, (void *)&OCR_codes_Ch1, (void *)&htimer32_channel1.Instance->OCR, sizeof(OCR_codes_Ch1) - 1);
		//HAL_Timer32_Channel_Enable(&htimer32_channel0);
		//HAL_Timer32_Channel_Enable(&htimer32_channel1);		
		HAL_Timer32_Start(&htimer32_1);
		if (HAL_DMA_Wait(&hdma_ch0, 10 * DMA_TIMEOUT_DEFAULT) != HAL_OK)
        {
            //xprintf("Timeout\n");
        }
		if (HAL_DMA_Wait(&hdma_ch1, 10 * DMA_TIMEOUT_DEFAULT) != HAL_OK)
        {
            //xprintf("Timeout\n");
        }
		HAL_Timer32_Stop(&htimer32_1);
		//HAL_Timer32_Channel_Disable(&htimer32_channel0);
		//HAL_Timer32_Channel_Disable(&htimer32_channel1);
/* 		HAL_Timer32_Stop(&htimer32_1);
		HAL_Timer32_Channel_Disable(&htimer32_channel0);
		HAL_Timer32_Channel_Disable(&htimer32_channel1); */
		HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_7);
		
		//HAL_DMA_ClearLocalIrq(&hdma);
		//HAL_Timer32_InterruptMask_Clear(&htimer32_1, TIMER32_INT_OC_M(htimer32_channel0.ChannelIndex));
		//HAL_Timer32_Compare_Stop_IT(&htimer32_1, &htimer32_channel0);
		//HAL_Timer32_Channel_Disable(&htimer32_channel0);
		//HAL_Timer32_Stop(&htimer32_1);
		
		for(int i = 0; i < 1000000; i++);
		
	}
	
}

uint16_t dshotPreparePacket(uint16_t value)
{
	uint16_t packet;
	bool dshotTelemetry = false;

	packet = (value << 1);

	unsigned int csum = 0;
	unsigned int csumData = packet;

	for(int i = 0; i < 3; i++)
	{
        csum ^=  csumData;
        csumData >>= 4;
	}

	csum &= 0xf;
	packet = (packet << 4) | csum;

	return packet;
}

void dshotPrepareDMAbuffer(uint32_t* motorDMAbuffer, uint16_t value)
{
	uint16_t packet;
	packet = dshotPreparePacket(value);
	
	motorDMAbuffer[1] = (packet & 0x8000) ? 167 : 87;
	packet <<= 1;
	
	for(int i = 2; i < 17; i++)
	{
		motorDMAbuffer[i] = (packet & 0x8000) ? 160 : 80;
		packet <<= 1;
	}

	motorDMAbuffer[17] = 0;
	motorDMAbuffer[18] = 0;
}

uint16_t dshotRound(float floatNum) {
	return (floatNum - (uint16_t)floatNum) >= 0.5 ? (uint16_t)(floatNum + 1) : (uint16_t)floatNum;
}