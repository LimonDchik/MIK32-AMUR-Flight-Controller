#ifndef MSP_MIK32
#define MSP_MIK32

#include "FreeRTOS.h"
#include "task.h"
#include "mik32_hal_usart.h"
#include "mik32_hal_irq.h"

/**
 * @brief Функция инициализации протокола MSP.
 * 
 * @param bool Конфигурация UART, к которой будет привязан протокол.
 */
void MSP_Init(USART_HandleTypeDef* user_husart);

/**
 * @brief Функция, которая содержит алгоритм, исполняющийся во время прерывания по UART.
 */
void MSP_ISR(void);

/**
 * @brief Функция получения и обработки сообщений.
 */
void MSP_GetMessage(void);

/**
 * @brief Функция отправки сообщения.
 */
void MSP_SendMessage(void);

#endif //MSP_MIK32