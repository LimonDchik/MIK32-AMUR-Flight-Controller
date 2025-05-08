#include "mik32_msp.h" 

/**
 * @brief   Максимальная длина сообщения.
 */
#define BUFFER_LENGTH    512

/**
 * @name Адреса символов пакета MSP.
 * @{
 */
#define START_SYMBOL_ADDRESS    0
#define M_SYMBOL_ADDRESS    1
#define VECTOR_SYMBOL_ADDRESS    2	
#define DATA_LENGTH_ADDRESS    3
#define CODE_SYMBOL_ADDRESS    4
/** @} */

/**
 * @name Символы пакета MSP.
 * @{
 */
#define START_SYMBOL    0x24
#define M_SYMBOL    0x4D
#define VECTOR_SYMBOL_IN    0x3C
#define VECTOR_SYMBOL_OUT    0x3E
/** @} */

/**
 * @brief Буфер для хранения вводимых данных
 */
static uint8_t buf[BUFFER_LENGTH];


/**
 * @brief Указатель на текущий элемент буфера
 */
uint8_t bufPointer;

/**
 * @brief Флаг готовности принятых данных
 */
bool bufReady = false;

/**
 * @brief Длина полученного сообщения.
 */
uint8_t gottenMessageLength;

USART_HandleTypeDef* husartMSP;

void EMPTY_FUNCTION(void);
bool ChecksumCheck(void);
void checksumAdd(void);
void MSPUART_Init(void);

/**
 * @brief Функция инициализации протокола MSP.
 * 
 * @param user_husart Конфигурация UART, к которой будет привязан протокол.
 */
void MSP_Init(USART_HandleTypeDef* user_husart) {
	husartMSP = user_husart;
	MSPUART_Init();
}

/**
 * @brief Проверка контрольной суммы пришедшего сообщения.
 * 
 * @return bool результат проверки контрольной суммы.
 */
bool ChecksumCheck() {
	uint8_t checksum = buf[3] ^ buf[4];
	uint32_t i;
	for (i = 5; i < 5 + buf[DATA_LENGTH_ADDRESS]; i++) {    
		checksum ^= buf[i];
	}
	if (buf[5 + buf[DATA_LENGTH_ADDRESS]] == checksum) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief Функция получения и обработки сообщений.
 */
void MSP_GetMessage() {
	HAL_USART_RXNE_EnableInterrupt(husartMSP);
	while(!bufReady);
	HAL_USART_RXNE_DisableInterrupt(husartMSP);
	bufReady = false;
	if(buf[START_SYMBOL_ADDRESS] == START_SYMBOL &&
	   buf[M_SYMBOL_ADDRESS] == M_SYMBOL &&
	   buf[VECTOR_SYMBOL_ADDRESS] == VECTOR_SYMBOL_IN) {
		   if (ChecksumCheck()) {
			    switch(buf[CODE_SYMBOL_ADDRESS]) {
				case 0x45: 
						MSP_SendMessage();
						break;
				default:
						EMPTY_FUNCTION();
						break;
				}
		   } else {
			   //CHECKSUM ERROR!
		   }
	} else {
		//PACKET ERROR!
	}
}

/**
 * @brief Функция отправки сообщения.
 */
void MSP_SendMessage(){
	HAL_USART_TXC_EnableInterrupt(husartMSP);
	
	/* Передача первого байта строки, последующие будут отправляться
	обработчиком прерываний */
	
	bufPointer = 1;
	if (buf[0] != '\0') HAL_USART_WriteByte(husartMSP, buf[0]);

	/* Задержка */
	vTaskDelay(pdMS_TO_TICKS(100));
	
	/* Запретить прерывания по передаче данных */
	HAL_USART_TXC_DisableInterrupt(husartMSP);	
}

/**
 * @brief Пустая функция для отладки.
 */
void EMPTY_FUNCTION(){
	uint8_t x;
	x = 1+2;
}

/**
 * @brief Функция, которая содержит алгоритм, исполняющийся во время прерывания по UART.
 */
void MSP_ISR() {
        if (HAL_USART_RXNE_ReadFlag(husartMSP))
        {
            buf[bufPointer] = HAL_USART_ReadByte(husartMSP);
            if (bufPointer == DATA_LENGTH_ADDRESS) {
				gottenMessageLength = buf[bufPointer] + 1;
			}
			if (gottenMessageLength == 0 && bufPointer > DATA_LENGTH_ADDRESS)
            {
				checksumAdd();
                bufPointer = 0;
                bufReady = true;
            }
            else
            {
                bufPointer += 1;
				gottenMessageLength -= 1;
                if (bufPointer >= BUFFER_LENGTH) bufPointer = 5;
                bufReady = false;
            }
            HAL_USART_RXNE_ClearFlag(husartMSP);
        }

        if (HAL_USART_TXC_ReadFlag(husartMSP))
        {
            if (buf[bufPointer] != '\0')
            {
                HAL_USART_WriteByte(husartMSP, buf[bufPointer]);
                bufPointer += 1;
            }
            else bufPointer = 0;
            HAL_USART_TXC_ClearFlag(husartMSP);
        }
}

/**
 * @brief Расчёт и добавление контрольной суммы.
 */
void checksumAdd() {
	  uint8_t checksum = buf[3] ^ buf[4];
	  uint32_t i;
	  for (i = 5; i < 5 + buf[DATA_LENGTH_ADDRESS]; i++) {    
		checksum ^= buf[i];
	  }
	  bufPointer += 1;
	  buf[bufPointer] = checksum;
	  bufPointer += 1;
}

void MSPUART_Init() {
    husartMSP->Instance = UART_0;
    husartMSP->transmitting = Enable;
    husartMSP->receiving = Enable;
    husartMSP->frame = Frame_8bit;
    husartMSP->parity_bit = Disable;
    husartMSP->parity_bit_inversion = Disable;
    husartMSP->bit_direction = LSB_First;
    husartMSP->data_inversion = Disable;
    husartMSP->tx_inversion = Disable;
    husartMSP->rx_inversion = Disable;
    husartMSP->swap = Disable;
    husartMSP->lbm = Disable;
    husartMSP->stop_bit = StopBit_1;
    husartMSP->mode = Asynchronous_Mode;
    husartMSP->xck_mode = XCK_Mode3;
    husartMSP->last_byte_clock = Disable;
    husartMSP->overwrite = Disable;
    husartMSP->rts_mode = AlwaysEnable_mode;
    husartMSP->dma_tx_request = Disable;
    husartMSP->dma_rx_request = Disable;
    husartMSP->channel_mode = Duplex_Mode;
    husartMSP->tx_break_mode = Disable;
    husartMSP->Interrupt.ctsie = Disable;
    husartMSP->Interrupt.eie = Disable;
    husartMSP->Interrupt.idleie = Disable;
    husartMSP->Interrupt.lbdie = Disable;
    husartMSP->Interrupt.peie = Disable;
    husartMSP->Interrupt.rxneie = Disable;
    husartMSP->Interrupt.tcie = Disable;
    husartMSP->Interrupt.txeie = Disable;
    husartMSP->Modem.rts = Disable; 
    husartMSP->Modem.cts = Disable; 
    husartMSP->Modem.dtr = Disable; 
    husartMSP->Modem.dcd = Disable; 
    husartMSP->Modem.dsr = Disable; 
    husartMSP->Modem.ri = Disable;  
    husartMSP->Modem.ddis = Disable;
    husartMSP->baudrate = 9600;
    HAL_USART_Init(husartMSP);
}
