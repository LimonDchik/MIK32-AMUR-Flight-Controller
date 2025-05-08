#ifndef MIK32_BMP280
#define MIK32_BMP280

#include <stdint.h>
#include "mik32_hal_i2c.h"

/**
 * @brief Функция записи данных во внутренние регистры BMP280.
 * 
 * @param BMP280_hi2c Указатель на конфигурацию интерфейса I2C.
 * @param addres Адрес первого регистра для записи.
 * @param data Указатель на данные для записи.
 * @param data_length Длина данных.
 */
void BMP280_I2C_Write(I2C_HandleTypeDef *hi2c, uint8_t addres, uint8_t* data, uint16_t data_length);

/**
 * @brief Функция чтения данных из внутренних регистров BMP280.
 * 
 * @param BMP280_hi2c Указатель на конфигурацию интерфейса I2C.
 * @param addres Адрес первого регистра для чтения.
 * @param data Указатель на место хранения считанных данных.
 * @param data_length Длина данных для чтения.
 */
void BMP280_I2C_Read(I2C_HandleTypeDef *hi2c, uint8_t addres, uint8_t* data, uint16_t data_length);
/**
 * @brief Функция инициализации барометра.
 * 
 * @param BMP280_hi2c Указатель на конфигурацию интерфейса I2C.
 */
void BMP280_Init (I2C_HandleTypeDef *BMP280_hi2c);

/**
 * @brief Функция чтения необработанных данных давления с барометра.
 * 
 * @param data Указатель на массив, в который будут сохранены необработанные данные давления.
 */
void BMP280_ReadPressure (uint8_t *data);

/**
 * @brief Функция чтения необработанных данных температуры с барометра.
 * 
 * @param data Указатель на массив, в который будут сохранены необработанные данные температуры.
 */
void BMP280_ReadTemp (uint8_t *data);

/**
 * @brief Функция обработки необработанных данных температуры (из документации).
 * 
 * @param adc_T Необработанные данные температуры, объединённые в одно значение.
 * @return int32_t Компенсированное значение температуры.
 */
int32_t bmp280_compensate_T_int32(int32_t adc_T);

/**
 * @brief Функция обработки необработанных данных давления (из документации).
 * 
 * @param adc_P Необработанные данные давления, объединённые в одно значение.
 * @return uint32_t Компенсированное значение давления.
 */
uint32_t bmp280_compensate_P_int64(int32_t adc_P);

/**
 * @brief Функция получения данных давления.
 * 
 * @return uint32_t Компенсированное значение давления.
 */
uint32_t BMP280_ReadCookedPressure(void);



#endif //MIK32_BMP280