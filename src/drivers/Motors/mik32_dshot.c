#include "mik32_dshot.h"

//Таймеры и DMA
volatile TIMER32_HandleTypeDef htimer32_1;
volatile TIMER32_HandleTypeDef htimer32_2;
volatile TIMER32_CHANNEL_HandleTypeDef htimer32_channel0;
volatile TIMER32_CHANNEL_HandleTypeDef htimer32_channel1;
volatile TIMER32_CHANNEL_HandleTypeDef htimer32_channel2;
volatile TIMER32_CHANNEL_HandleTypeDef htimer32_channel3;
volatile DMA_InitTypeDef hdma;
volatile DMA_ChannelHandleTypeDef hdma_ch0;
volatile DMA_ChannelHandleTypeDef hdma_ch1;
volatile DMA_ChannelHandleTypeDef hdma_ch2;
volatile DMA_ChannelHandleTypeDef hdma_ch3;

//Переменные вида DHSOT
volatile uint8_t DSHOT_zero;
volatile uint8_t DSHOT_one;
volatile uint8_t DSHOT_Timer_Top;
volatile uint8_t DSHOT_OCR_correct;

//DMA буфферы
volatile uint32_t OCR_codes_Ch0[] = {0, 0, 0, 0, 0, 0, 0, 87, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 0, 0};
volatile uint32_t OCR_codes_Ch1[] = {0, 0, 0, 0, 0, 87, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 0, 0};
volatile uint32_t OCR_codes_Ch2[] = {0, 0, 0, 0, 87, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 0, 0};
volatile uint32_t OCR_codes_Ch3[] = {0, 0, 87, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 80, 160, 0, 0};

//Функции инициализации
void DSHOT_timer_init();
void DSHOT_dma_init();

//Функции обработки пакета
uint16_t DSHOT_round(float floatNum);
uint16_t DSHOT_preparePacket(uint16_t value);
void DSHOT_prepareDMAbuffer(uint32_t* motorDMAbuffer, uint16_t value, uint8_t delay);

//Инициализация DSHOT
void DSHOT_init(uint8_t type) {
	//Инициализация типа DSHOT
	if (type == 1) {
		//DSHOT300
		DSHOT_Timer_Top = 107;
		DSHOT_zero = 40;
		DSHOT_one = 80;
		DSHOT_OCR_correct = 4;
	} else if (type == 2) {
		//DSHOT600
		DSHOT_Timer_Top = 53;
		DSHOT_zero = 20;
		DSHOT_one = 40;
		DSHOT_OCR_correct = 2;
	} else {
		//DSHOT150
		DSHOT_Timer_Top = 213;
		DSHOT_zero = 80;
		DSHOT_one = 160;
		DSHOT_OCR_correct = 7;
	}
	//Инициализация таймера и DMA
	DSHOT_timer_init();
	DSHOT_dma_init();
	
	//Стартовые настройки
	HAL_Timer32_Value_Clear(&htimer32_1);
	HAL_Timer32_Channel_Enable(&htimer32_channel0);
	HAL_Timer32_Channel_Enable(&htimer32_channel1);
	HAL_Timer32_Channel_Enable(&htimer32_channel2);
	HAL_Timer32_Channel_Enable(&htimer32_channel3);
	for(volatile int i = 0; i < 10; i++);	
	HAL_Timer32_Start(&htimer32_1);
	for(volatile int i = 0; i < 19; i++);
	HAL_Timer32_Start(&htimer32_2);
}

//Инициализация таймера32 и его каналов
void DSHOT_timer_init() {
    htimer32_1.Instance = TIMER32_1;
    htimer32_1.Top = DSHOT_Timer_Top;
    htimer32_1.State = TIMER32_STATE_DISABLE;
    htimer32_1.Clock.Source = TIMER32_SOURCE_PRESCALER;
    htimer32_1.Clock.Prescaler = 0;
    htimer32_1.InterruptMask = 0;
    htimer32_1.CountMode = TIMER32_COUNTMODE_FORWARD;
    HAL_Timer32_Init(&htimer32_1);

    htimer32_channel0.TimerInstance = htimer32_1.Instance;
    htimer32_channel0.ChannelIndex = TIMER32_CHANNEL_0;
    htimer32_channel0.PWM_Invert = TIMER32_CHANNEL_NON_INVERTED_PWM;
    htimer32_channel0.Mode = TIMER32_CHANNEL_MODE_PWM;
    htimer32_channel0.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    htimer32_channel0.OCR = DSHOT_zero;
    htimer32_channel0.Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(&htimer32_channel0);
	
	htimer32_channel1.TimerInstance = htimer32_1.Instance;
    htimer32_channel1.ChannelIndex = TIMER32_CHANNEL_1;
    htimer32_channel1.PWM_Invert = TIMER32_CHANNEL_NON_INVERTED_PWM;
    htimer32_channel1.Mode = TIMER32_CHANNEL_MODE_PWM;
    htimer32_channel1.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    htimer32_channel1.OCR = DSHOT_zero;
    htimer32_channel1.Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(&htimer32_channel1);

    htimer32_2.Instance = TIMER32_2;
    htimer32_2.Top = DSHOT_Timer_Top;
    htimer32_2.State = TIMER32_STATE_DISABLE;
    htimer32_2.Clock.Source = TIMER32_SOURCE_PRESCALER;
    htimer32_2.Clock.Prescaler = 0;
    htimer32_2.InterruptMask = 0;
    htimer32_2.CountMode = TIMER32_COUNTMODE_FORWARD;
    HAL_Timer32_Init(&htimer32_2);
	
	htimer32_channel2.TimerInstance = htimer32_1.Instance;
    htimer32_channel2.ChannelIndex = TIMER32_CHANNEL_2;
    htimer32_channel2.PWM_Invert = TIMER32_CHANNEL_NON_INVERTED_PWM;
    htimer32_channel2.Mode = TIMER32_CHANNEL_MODE_PWM;
    htimer32_channel2.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    htimer32_channel2.OCR = DSHOT_zero;
    htimer32_channel2.Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(&htimer32_channel2);
	
	htimer32_channel3.TimerInstance = htimer32_1.Instance;
    htimer32_channel3.ChannelIndex = TIMER32_CHANNEL_3;
    htimer32_channel3.PWM_Invert = TIMER32_CHANNEL_NON_INVERTED_PWM;
    htimer32_channel3.Mode = TIMER32_CHANNEL_MODE_PWM;
    htimer32_channel3.CaptureEdge = TIMER32_CHANNEL_CAPTUREEDGE_RISING;
    htimer32_channel3.OCR = DSHOT_zero;
    htimer32_channel3.Noise = TIMER32_CHANNEL_FILTER_OFF;
    HAL_Timer32_Channel_Init(&htimer32_channel3);
}

//Инициализация DMA и его каналов
void DSHOT_dma_init() {
	//Инициализация DMA
    hdma.Instance = DMA_CONFIG;
    hdma.CurrentValue = DMA_CURRENT_VALUE_ENABLE;
    if (HAL_DMA_Init(&hdma) != HAL_OK)
    {
       //Error handler
    }

    //Инициализация канала 0
    hdma_ch0.dma = &hdma;
    hdma_ch0.ChannelInit.Channel = DMA_CHANNEL_0;
    hdma_ch0.ChannelInit.Priority = DMA_CHANNEL_PRIORITY_VERY_HIGH;
    hdma_ch0.ChannelInit.ReadMode = DMA_CHANNEL_MODE_MEMORY;
    hdma_ch0.ChannelInit.ReadInc = DMA_CHANNEL_INC_ENABLE;
    hdma_ch0.ChannelInit.ReadSize = DMA_CHANNEL_SIZE_WORD; 
    hdma_ch0.ChannelInit.ReadBurstSize = 2;                
    hdma_ch0.ChannelInit.ReadRequest = DMA_CHANNEL_TIMER32_1_REQUEST;
    hdma_ch0.ChannelInit.ReadAck = DMA_CHANNEL_ACK_DISABLE;
    hdma_ch0.ChannelInit.WriteMode = DMA_CHANNEL_MODE_PERIPHERY;
    hdma_ch0.ChannelInit.WriteInc = DMA_CHANNEL_INC_DISABLE;
    hdma_ch0.ChannelInit.WriteSize = DMA_CHANNEL_SIZE_WORD; 
    hdma_ch0.ChannelInit.WriteBurstSize = 2;                
    hdma_ch0.ChannelInit.WriteRequest = DMA_CHANNEL_TIMER32_1_REQUEST;
    hdma_ch0.ChannelInit.WriteAck = DMA_CHANNEL_ACK_ENABLE;
	
	//Инициализация канала 1
	hdma_ch1.dma = &hdma;
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
	
	//Инициализация канала 2
	hdma_ch2.dma = &hdma;
    hdma_ch2.ChannelInit.Channel = DMA_CHANNEL_2;
    hdma_ch2.ChannelInit.Priority = DMA_CHANNEL_PRIORITY_VERY_HIGH;
    hdma_ch2.ChannelInit.ReadMode = DMA_CHANNEL_MODE_MEMORY;
    hdma_ch2.ChannelInit.ReadInc = DMA_CHANNEL_INC_ENABLE;
    hdma_ch2.ChannelInit.ReadSize = DMA_CHANNEL_SIZE_WORD; /* data_len должно быть кратно read_size */
    hdma_ch2.ChannelInit.ReadBurstSize = 2;                /* read_burst_size должно быть кратно read_size */
    hdma_ch2.ChannelInit.ReadRequest = DMA_CHANNEL_TIMER32_2_REQUEST;
    hdma_ch2.ChannelInit.ReadAck = DMA_CHANNEL_ACK_DISABLE;
    hdma_ch2.ChannelInit.WriteMode = DMA_CHANNEL_MODE_PERIPHERY;
    hdma_ch2.ChannelInit.WriteInc = DMA_CHANNEL_INC_DISABLE;
    hdma_ch2.ChannelInit.WriteSize = DMA_CHANNEL_SIZE_WORD; /* data_len должно быть кратно write_size */
    hdma_ch2.ChannelInit.WriteBurstSize = 2;                /* write_burst_size должно быть кратно read_size */
    hdma_ch2.ChannelInit.WriteRequest = DMA_CHANNEL_TIMER32_2_REQUEST;
    hdma_ch2.ChannelInit.WriteAck = DMA_CHANNEL_ACK_ENABLE;
	
	//Инициализация канала 3
	hdma_ch3.dma = &hdma;
    hdma_ch3.ChannelInit.Channel = DMA_CHANNEL_3;
    hdma_ch3.ChannelInit.Priority = DMA_CHANNEL_PRIORITY_VERY_HIGH;
    hdma_ch3.ChannelInit.ReadMode = DMA_CHANNEL_MODE_MEMORY;
    hdma_ch3.ChannelInit.ReadInc = DMA_CHANNEL_INC_ENABLE;
    hdma_ch3.ChannelInit.ReadSize = DMA_CHANNEL_SIZE_WORD; /* data_len должно быть кратно read_size */
    hdma_ch3.ChannelInit.ReadBurstSize = 2;                /* read_burst_size должно быть кратно read_size */
    hdma_ch3.ChannelInit.ReadRequest = DMA_CHANNEL_TIMER32_2_REQUEST;
    hdma_ch3.ChannelInit.ReadAck = DMA_CHANNEL_ACK_DISABLE;
    hdma_ch3.ChannelInit.WriteMode = DMA_CHANNEL_MODE_PERIPHERY;
    hdma_ch3.ChannelInit.WriteInc = DMA_CHANNEL_INC_DISABLE;
    hdma_ch3.ChannelInit.WriteSize = DMA_CHANNEL_SIZE_WORD; /* data_len должно быть кратно write_size */
    hdma_ch3.ChannelInit.WriteBurstSize = 2;                /* write_burst_size должно быть кратно read_size */
    hdma_ch3.ChannelInit.WriteRequest = DMA_CHANNEL_TIMER32_2_REQUEST;
    hdma_ch3.ChannelInit.WriteAck = DMA_CHANNEL_ACK_ENABLE;
}

//Подготовка пакета DSHOT
uint16_t DSHOT_preparePacket(uint16_t value) {
	volatile uint16_t packet;
		
	packet = (value << 1); //Добавление нулевого бита телеметрии
	
	volatile unsigned int csum = 0;
	volatile unsigned int csumData = packet;
	
	//Вычисление проверочной суммы
	for(volatile int i = 0; i < 3; i++)
	{
        csum ^=  csumData;
        csumData >>= 4;
	}
	csum &= 0xf;
	packet = (packet << 4) | csum;
	
	//Отправка
	return packet;
}

//Подготовка буфферов DMA
void DSHOT_prepareDMAbuffer(uint32_t* motorDMAbuffer, uint16_t packet, uint8_t delay) {
	//Первый бит длиннее, так как DMA его сокращает
	//Я не знаю почему
	motorDMAbuffer[0 + delay] = (packet & 0x8000) ? (DSHOT_one + DSHOT_OCR_correct) : (DSHOT_zero + DSHOT_OCR_correct);
	packet <<= 1;
	
	//Преобразование битов в скважность импульсов
	for(volatile int i = 1 + delay; i < 16 + delay; i++)
	{
		motorDMAbuffer[i] = (packet & 0x8000) ? DSHOT_one : DSHOT_zero;
		packet <<= 1;
	}
	
	//Конечные биты для установки сигнала в область нуля по окончании отправки
	motorDMAbuffer[16 + delay] = 0;
	motorDMAbuffer[17 + delay] = 0;
}

//Округление
uint16_t DSHOT_round(float floatNum) {
	return (floatNum - (uint16_t)floatNum) >= 0.5 ? (uint16_t)(floatNum + 1) : (uint16_t)floatNum;
}

//Отправка сообщения DSHOT
void DSHOT_send(float* mixersSignals) {
	//Подготовка сообщений
	DSHOT_prepareDMAbuffer(OCR_codes_Ch0, DSHOT_preparePacket(DSHOT_round(mixersSignals[0])), 7);
	DSHOT_prepareDMAbuffer(OCR_codes_Ch1, DSHOT_preparePacket(DSHOT_round(mixersSignals[1])), 5);
	DSHOT_prepareDMAbuffer(OCR_codes_Ch2, DSHOT_preparePacket(DSHOT_round(mixersSignals[2])), 4);
	DSHOT_prepareDMAbuffer(OCR_codes_Ch3, DSHOT_preparePacket(DSHOT_round(mixersSignals[3])), 2);	
	for(volatile int i = 0; i < 10; i++);
	
	//Отправка
	//for(volatile int i = 0; i < 5; i++);
	
	//for(volatile int i = 0; i < 10; i++);
	HAL_DMA_Start(&hdma_ch0, (void *)&OCR_codes_Ch0, (void *)&htimer32_channel0.Instance->OCR, sizeof(OCR_codes_Ch0) - 1);
	//for(volatile int i = 0; i < 10; i++);
	HAL_DMA_Start(&hdma_ch1, (void *)&OCR_codes_Ch1, (void *)&htimer32_channel1.Instance->OCR, sizeof(OCR_codes_Ch1) - 1);
	//for(volatile int i = 0; i < 10; i++);
	HAL_DMA_Start(&hdma_ch2, (void *)&OCR_codes_Ch2, (void *)&htimer32_channel2.Instance->OCR, sizeof(OCR_codes_Ch2) - 1);
	//for(volatile int i = 0; i < 10; i++);
	HAL_DMA_Start(&hdma_ch3, (void *)&OCR_codes_Ch3, (void *)&htimer32_channel3.Instance->OCR, sizeof(OCR_codes_Ch3) - 1);
	
	//HAL_DMA_ChannelEnable(&hdma_ch0);
	//HAL_DMA_ChannelEnable(&hdma_ch1);
	//HAL_DMA_ChannelEnable(&hdma_ch2);
	//HAL_Timer32_Channel_Enable(&hdma_ch0);
	//HAL_Timer32_Channel_Enable(&hdma_ch1);
	//HAL_Timer32_Channel_Enable(&hdma_ch2);

	
	
	
	if (HAL_DMA_Wait(&hdma_ch0, 2 * DMA_TIMEOUT_DEFAULT) != HAL_OK) {
		//Error handler
    }
	if (HAL_DMA_Wait(&hdma_ch1, 2 * DMA_TIMEOUT_DEFAULT) != HAL_OK) {
		//Error handler
    }
	if (HAL_DMA_Wait(&hdma_ch2, 2 * DMA_TIMEOUT_DEFAULT) != HAL_OK) {
		//Error handler
    }
	if (HAL_DMA_Wait(&hdma_ch3, 2 * DMA_TIMEOUT_DEFAULT) != HAL_OK) {
		//Error handler
    }
	//HAL_DMA_ChannelDisable(&hdma_ch0);
	//HAL_DMA_ChannelDisable(&hdma_ch1);
	//HAL_DMA_ChannelDisable(&hdma_ch2);
	
//	for(volatile int i = 0; i < 20; i++); 
/*	HAL_DMA_Start(&hdma_ch2, (void *)&OCR_codes_Ch2, (void *)&htimer32_channel2.Instance->OCR, sizeof(OCR_codes_Ch2) - 1);*/
	//for(volatile int i = 0; i < 20; i++);
 //	HAL_DMA_Start(&hdma_ch3, (void *)&OCR_codes_Ch3, (void *)&htimer32_channel3.Instance->OCR, sizeof(OCR_codes_Ch3) - 1);	
		 
	//for(volatile int i = 0; i < 1000000; i++);
 
 	//if (HAL_DMA_Wait(&hdma_ch1, 2 * DMA_TIMEOUT_DEFAULT) != HAL_OK) {
		//Error handler
    //} 
 /*	if (HAL_DMA_Wait(&hdma_ch2, 2 * DMA_TIMEOUT_DEFAULT) != HAL_OK) {
		//Error handler
    } */
// 	if (HAL_DMA_Wait(&hdma_ch3, 20 * DMA_TIMEOUT_DEFAULT) != HAL_OK) {
		//Error handler
//    }
	//for(volatile int i = 0; i < 10; i++);
	//HAL_Timer32_Stop(&htimer32_1);
	//HAL_Timer32_Stop(&htimer32_2); 

	
	
	
	//HAL_Timer32_Value_Clear(&htimer32_1);
	//HAL_Timer32_Value_Clear(&htimer32_2);
/* 	HAL_Timer32_Channel_Disable(&hdma_ch0);
	HAL_Timer32_Channel_Disable(&hdma_ch1);
	HAL_Timer32_Channel_Disable(&hdma_ch2); */
}