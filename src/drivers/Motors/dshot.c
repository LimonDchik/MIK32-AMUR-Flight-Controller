
#include "dshot.h"

TIMER32_HandleTypeDef* htimer32_2;
TIMER32_CHANNEL_HandleTypeDef* channel_1;
TIMER32_CHANNEL_HandleTypeDef* channel_2;
TIMER32_CHANNEL_HandleTypeDef* channel_3;
TIMER32_CHANNEL_HandleTypeDef* channel_4;
DMA_InitTypeDef* hdma;
DMA_ChannelHandleTypeDef* hdma_channel_0;
DMA_ChannelHandleTypeDef* hdma_channel_1;
DMA_ChannelHandleTypeDef* hdma_channel_2;
DMA_ChannelHandleTypeDef* hdma_channel_3;

uint32_t motor1DMAbuffer[DSHOT_DMA_BUFFER_SIZE];
uint32_t motor2DMAbuffer[DSHOT_DMA_BUFFER_SIZE];
uint32_t motor3DMAbuffer[DSHOT_DMA_BUFFER_SIZE];
uint32_t motor4DMAbuffer[DSHOT_DMA_BUFFER_SIZE];

uint32_t dshotChooseType(dshotType_e dshotType);

void dshotInitTimer(dshotType_e dshotType, 
						TIMER32_HandleTypeDef* ht32_2,
						TIMER32_CHANNEL_HandleTypeDef* ch_1,
						TIMER32_CHANNEL_HandleTypeDef* ch_2,
						TIMER32_CHANNEL_HandleTypeDef* ch_3,
						TIMER32_CHANNEL_HandleTypeDef* ch_4,
						DMA_InitTypeDef* hd,
						DMA_ChannelHandleTypeDef* hd_ch_0,
						DMA_ChannelHandleTypeDef* hd_ch_1,
						DMA_ChannelHandleTypeDef* hd_ch_2,
						DMA_ChannelHandleTypeDef* hd_ch_3);
void dshotDMAIRQInit(void);
void dshotStartPWM(void);
void DMA_CH_Init(DMA_ChannelHandleTypeDef *hdma_ch, DMA_InitTypeDef *controller_hdma, HAL_DMA_ChannelIndexTypeDef DMA_CHANNEL_X);
void DMA_Init(DMA_InitTypeDef* hd, 
				DMA_ChannelHandleTypeDef* hd_ch_0,
				DMA_ChannelHandleTypeDef* hd_ch_1,
				DMA_ChannelHandleTypeDef* hd_ch_2,
				DMA_ChannelHandleTypeDef* hd_ch_3);

uint16_t dshotPreparePacket(uint16_t value);
void dshotPrepareDMAbuffer(uint32_t* motorDMAbuffer, uint16_t value);
void dshotPrepareDMAbufferAll(uint16_t* motorValue);
void dshotDMAStart(void);
void dshotEnableDMARequest(void);


/**
 * @brief Функция инициализации DSHOT.
 * 
 * @param dshotType тип DSHOT: DSHOT150, DSHOT300 или DSHOT600.
 * @param ht32_2 Конфигурация таймера, которая будет перенастроена в этой инициализации.
 * @param ch_1 Конфигурация канала таймера 1, которая будет перенастроена в этой инициализации.
 * @param ch_2 Конфигурация канала таймера 2, которая будет перенастроена в этой инициализации.
 * @param ch_3 Конфигурация канала таймера 3, которая будет перенастроена в этой инициализации.
 * @param ch_4 Конфигурация канала таймера 4, которая будет перенастроена в этой инициализации.
 * @param hd Конфигурация DMA, которая будет перенастроена в этой инициализации.
 * @param hd_ch_0 Конфигурация канала DMA 0, которая будет перенастроена в этой инициализации. 
 * @param hd_ch_1 Конфигурация канала DMA 1, которая будет перенастроена в этой инициализации. 
 * @param hd_ch_2 Конфигурация канала DMA 2, которая будет перенастроена в этой инициализации.
 * @param hd_ch_3 Конфигурация канала DMA 3, которая будет перенастроена в этой инициализации.
 */
void dshotInit(const dshotType_e dshotType, 
					TIMER32_HandleTypeDef* ht32_2,
					TIMER32_CHANNEL_HandleTypeDef* ch_1,
					TIMER32_CHANNEL_HandleTypeDef* ch_2,
					TIMER32_CHANNEL_HandleTypeDef* ch_3,
					TIMER32_CHANNEL_HandleTypeDef* ch_4,
					DMA_InitTypeDef* hd,
					DMA_ChannelHandleTypeDef* hd_ch_0,
					DMA_ChannelHandleTypeDef* hd_ch_1,
					DMA_ChannelHandleTypeDef* hd_ch_2,
					DMA_ChannelHandleTypeDef* hd_ch_3) {
						
 	dshotInitTimer(dshotType, 
						ht32_2,
						ch_1,
						ch_2,
						ch_3,
						ch_4,
						hd,
						hd_ch_0,
						hd_ch_1,
						hd_ch_2,
						hd_ch_3);  
	dshotDMAIRQInit(); 
	dshotStartPWM();
}

/**
 * @brief Отправка сообщения на ESC по протоколу DSHOT.
 * 
 * @param motorValue рассчитаные значения управляющих сигналов для моторов.
 */
void dshotWrite(uint16_t* motorValue)
{
	dshotPrepareDMAbufferAll(motorValue);
	dshotEnableDMARequest();
	dshotDMAStart();	
}

uint32_t dshotChooseType(dshotType_e dshotType)
{
	switch (dshotType)
	{
		case(DSHOT600):
				return DSHOT600_HZ;

		case(DSHOT300):
				return DSHOT300_HZ;

		default:
		case(DSHOT150):
				return DSHOT150_HZ;
	}
}

void dshotInitTimer(dshotType_e dshotType, 
						TIMER32_HandleTypeDef* ht32_2,
						TIMER32_CHANNEL_HandleTypeDef* ch_1,
						TIMER32_CHANNEL_HandleTypeDef* ch_2,
						TIMER32_CHANNEL_HandleTypeDef* ch_3,
						TIMER32_CHANNEL_HandleTypeDef* ch_4,
						DMA_InitTypeDef* hd,
						DMA_ChannelHandleTypeDef* hd_ch_0,
						DMA_ChannelHandleTypeDef* hd_ch_1,
						DMA_ChannelHandleTypeDef* hd_ch_2,
						DMA_ChannelHandleTypeDef* hd_ch_3) {
	DMA_Init(hd, 
				hd_ch_0,
				hd_ch_1,
				hd_ch_2,
				hd_ch_3);
	
	uint32_t dshotPrescaler;
	uint32_t timerClock = TIMER_CLOCK;
	float divResult;
	divResult = (float)timerClock / (float)dshotChooseType(dshotType);
	dshotPrescaler = ((divResult - (float)((int)divResult)) > 0.5f ? ((uint32_t)divResult) + 1 : (uint32_t)divResult) - 1;
 	
	htimer32_2 = ht32_2;
	htimer32_2->Instance = TIMER32_2;
    htimer32_2->Top = MOTOR_BITLENGTH;
    htimer32_2->State = TIMER32_STATE_DISABLE;
    htimer32_2->Clock.Source = TIMER32_SOURCE_PRESCALER;
    htimer32_2->Clock.Prescaler = dshotPrescaler;
    htimer32_2->InterruptMask = 0;
    htimer32_2->CountMode = TIMER32_COUNTMODE_FORWARD;
    HAL_Timer32_Init(htimer32_2);
	
	channel_1 = ch_1;
	channel_1->TimerInstance = htimer32_2->Instance;
    channel_1->ChannelIndex = TIMER32_CHANNEL_0;
    channel_1->PWM_Invert = TIMER32_CHANNEL_NON_INVERTED_PWM;
    channel_1->Mode = TIMER32_CHANNEL_MODE_PWM;
    channel_1->CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    channel_1->OCR = MOTOR_BITLENGTH >> 1;
    channel_1->Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(channel_1);
	
	channel_2 = ch_2;
	channel_2->TimerInstance = htimer32_2->Instance;
    channel_2->ChannelIndex = TIMER32_CHANNEL_1;
    channel_2->PWM_Invert = TIMER32_CHANNEL_NON_INVERTED_PWM;
    channel_2->Mode = TIMER32_CHANNEL_MODE_PWM;
    channel_2->CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    channel_2->OCR = MOTOR_BITLENGTH >> 1;
    channel_2->Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(channel_2);
	
	channel_3 = ch_3;
	channel_3->TimerInstance = htimer32_2->Instance;
    channel_3->ChannelIndex = TIMER32_CHANNEL_2;
    channel_3->PWM_Invert = TIMER32_CHANNEL_NON_INVERTED_PWM;
    channel_3->Mode = TIMER32_CHANNEL_MODE_PWM;
    channel_3->CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    channel_3->OCR = MOTOR_BITLENGTH >> 1;
    channel_3->Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(channel_3);
	
	channel_4 = ch_4;
	channel_4->TimerInstance = htimer32_2->Instance;
    channel_4->ChannelIndex = TIMER32_CHANNEL_3;
    channel_4->PWM_Invert = TIMER32_CHANNEL_NON_INVERTED_PWM;
    channel_4->Mode = TIMER32_CHANNEL_MODE_PWM;
    channel_4->CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    channel_4->OCR = MOTOR_BITLENGTH >> 1;
    channel_4->Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(channel_4);
}

void DMA_Init(DMA_InitTypeDef* hd, 
				DMA_ChannelHandleTypeDef* hd_ch_0,
				DMA_ChannelHandleTypeDef* hd_ch_1,
				DMA_ChannelHandleTypeDef* hd_ch_2,
				DMA_ChannelHandleTypeDef* hd_ch_3) {
	hdma = hd;
    hdma->Instance = DMA_CONFIG;
    hdma->CurrentValue = DMA_CURRENT_VALUE_ENABLE;
	HAL_DMA_Init(hdma);
	HAL_DMA_GlobalIRQEnable(hdma, DMA_IRQ_ENABLE);
	HAL_DMA_ErrorIRQEnable(hdma, DMA_IRQ_ENABLE);
	DMA_CH_Init(hd_ch_0, hdma, DMA_CHANNEL_0);
	hdma_channel_0 = hd_ch_0;
	DMA_CH_Init(hd_ch_1, hdma, DMA_CHANNEL_1);
	hdma_channel_1 = hd_ch_1;
	DMA_CH_Init(hd_ch_2, hdma, DMA_CHANNEL_2);
	hdma_channel_2 = hd_ch_2;
	DMA_CH_Init(hd_ch_3, hdma, DMA_CHANNEL_3);
	hdma_channel_3 = hd_ch_3;
}

void DMA_CH_Init(DMA_ChannelHandleTypeDef *hdma_ch, DMA_InitTypeDef *controller_hdma, HAL_DMA_ChannelIndexTypeDef DMA_CHANNEL_X)
{
    hdma_ch->dma = controller_hdma;

    hdma_ch->ChannelInit.Channel = DMA_CHANNEL_X;
    hdma_ch->ChannelInit.Priority = DMA_CHANNEL_PRIORITY_VERY_HIGH;

    hdma_ch->ChannelInit.ReadMode = DMA_CHANNEL_MODE_MEMORY;
    hdma_ch->ChannelInit.ReadInc = DMA_CHANNEL_INC_ENABLE;
    hdma_ch->ChannelInit.ReadSize = DMA_CHANNEL_SIZE_WORD; /* data_len должно быть кратно read_size */
    hdma_ch->ChannelInit.ReadBurstSize = 2;                /* read_burst_size должно быть кратно read_size */
    hdma_ch->ChannelInit.ReadRequest = DMA_CHANNEL_TIMER32_1_REQUEST;
    hdma_ch->ChannelInit.ReadAck = DMA_CHANNEL_ACK_DISABLE;

    hdma_ch->ChannelInit.WriteMode = DMA_CHANNEL_MODE_PERIPHERY;
    hdma_ch->ChannelInit.WriteInc = DMA_CHANNEL_INC_DISABLE;
    hdma_ch->ChannelInit.WriteSize = DMA_CHANNEL_SIZE_WORD; /* data_len должно быть кратно write_size */
    hdma_ch->ChannelInit.WriteBurstSize = 2;                /* write_burst_size должно быть кратно read_size */
    hdma_ch->ChannelInit.WriteRequest = DMA_CHANNEL_TIMER32_1_REQUEST;
    hdma_ch->ChannelInit.WriteAck = DMA_CHANNEL_ACK_ENABLE;
}

void dshotDMAIRQ()
{
	if (HAL_DMA_GetChannelIrq(hdma_channel_0))
	{
		HAL_DMA_ChannelDisable(hdma_channel_0);
	}
	if(HAL_DMA_GetChannelIrq(hdma_channel_1))
	{
		HAL_DMA_ChannelDisable(hdma_channel_1);
	}
	if(HAL_DMA_GetChannelIrq(hdma_channel_2))
	{
		HAL_DMA_ChannelDisable(hdma_channel_2);
	}
	if(HAL_DMA_GetChannelIrq(hdma_channel_3))
	{
		HAL_DMA_ChannelDisable(hdma_channel_3);
	}
    HAL_DMA_ClearLocalIrq(hdma);
    HAL_DMA_ClearGlobalIrq(hdma);
    HAL_DMA_ClearErrorIrq(hdma);;
}

void dshotDMAIRQInit()
{
	HAL_DMA_LocalIRQEnable(hdma_channel_0, DMA_IRQ_ENABLE);
	HAL_DMA_LocalIRQEnable(hdma_channel_1, DMA_IRQ_ENABLE);
	HAL_DMA_LocalIRQEnable(hdma_channel_2, DMA_IRQ_ENABLE);
	HAL_DMA_LocalIRQEnable(hdma_channel_3, DMA_IRQ_ENABLE);
}

void dshotStartPWM()
{
    HAL_Timer32_Channel_Enable(channel_1);
	HAL_Timer32_Channel_Enable(channel_2);
	HAL_Timer32_Channel_Enable(channel_3);
	HAL_Timer32_Channel_Enable(channel_4);
	HAL_Timer32_Value_Clear(htimer32_2);
    HAL_Timer32_Start(htimer32_2);
}

uint16_t dshotPreparePacket(uint16_t value)
{
	uint16_t packet;
	bool dshotTelemetry = false;

	packet = (value << 1) | (dshotTelemetry ? 1 : 0);

	unsigned csum = 0;
	unsigned csumData = packet;

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

	for(int i = 0; i < 16; i++)
	{
		motorDMAbuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
		packet <<= 1;
	}

	motorDMAbuffer[16] = 0;
	motorDMAbuffer[17] = 0;
}

void dshotPrepareDMAbufferAll(uint16_t* motorValue)
{
	dshotPrepareDMAbuffer(motor1DMAbuffer, motorValue[0]);
	dshotPrepareDMAbuffer(motor2DMAbuffer, motorValue[1]);
	dshotPrepareDMAbuffer(motor3DMAbuffer, motorValue[2]);
	dshotPrepareDMAbuffer(motor4DMAbuffer, motorValue[3]);
}

void dshotDMAStart()
{
	HAL_DMA_Start(hdma_channel_0, (void* )motor1DMAbuffer, (void* )&channel_1->Instance->OCR, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start(hdma_channel_1, (void* )motor2DMAbuffer, (void* )&channel_2->Instance->OCR, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start(hdma_channel_2, (void* )motor3DMAbuffer, (void* )&channel_3->Instance->OCR, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start(hdma_channel_3, (void* )motor4DMAbuffer, (void* )&channel_4->Instance->OCR, DSHOT_DMA_BUFFER_SIZE);
}

void dshotEnableDMARequest()
{
	HAL_DMA_ChannelEnable(hdma_channel_0);
	HAL_DMA_ChannelEnable(hdma_channel_1);
	HAL_DMA_ChannelEnable(hdma_channel_2);
	HAL_DMA_ChannelEnable(hdma_channel_3);
	HAL_Timer32_Value_Clear(htimer32_2);
}
