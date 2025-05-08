#include "mik32_bmp280.h"

/**
 * @brief I2C-адрес BMP280.
 */
#define BMP280_ADRESS 0x76

/**
 * @brief Адрес регистра CTRL_MEAS (оверсэмплирование давления, оверсэмплирование температуры, режим).
 */
#define CTRL_MEAS_ADRESS 0xF4

/**
 * @brief Адрес регистра CONFIG (ожидание, фильтр, SPI).
 */
#define CONFIG_ADRESS 0xF5

/**
 * @name Адреса регистров измерения давления.
 * @{
 */
#define PRESS_MSB_ADRESS 0xF7
#define PRESS_LSB_ADRESS 0xF8
#define PRESS_XLSB_ADRESS 0xF9
/** @} */

/**
 * @name Адреса регистров измерения температуры.
 * @{
 */
#define TEMP_MSB_ADRESS 0xFA
#define TEMP_LSB_ADRESS 0xFB
#define TEMP_XLSB_ADRESS 0xFC
/** @} */

/**
 * @name Адреса регистров калибровки.
 * @{
 */
#define CALIB00 0x88
#define CALIB01 0x89
#define CALIB02 0x8A
#define CALIB03 0x8B
#define CALIB04 0x8C
#define CALIB05 0x8D
#define CALIB06 0x8E
#define CALIB07 0x8F
#define CALIB08 0x90
#define CALIB09 0x91
#define CALIB10 0x92
#define CALIB11 0x93
#define CALIB12 0x94
#define CALIB13 0x95
#define CALIB14 0x96
#define CALIB15 0x97
#define CALIB16 0x98
#define CALIB17 0x99
#define CALIB18 0x9A
#define CALIB19 0x9B
#define CALIB20 0x9C
#define CALIB21 0x9D
#define CALIB22 0x9E
#define CALIB23 0x9F
#define CALIB24 0xA0
#define CALIB25 0xA1
/** @} */

/**
 * @brief Коэффициент, основанный на текущей температуре.
 */
static int32_t t_fine;

/**
 * @name Коэффициенты калибровки температуры.
 * @{
 */
static uint16_t dig_T1;
static int16_t  dig_T2;
static int16_t  dig_T3;
/** @} */

/**
 * @name Коэффициенты калибровки давления.
 * @{
 */
static uint16_t dig_P1;
static int16_t  dig_P2;
static int16_t  dig_P3;
static int16_t  dig_P4;
static int16_t  dig_P5;
static int16_t  dig_P6;
static int16_t  dig_P7;
static int16_t  dig_P8;
static int16_t  dig_P9;
/** @} */

I2C_HandleTypeDef *BMP280_hi2c;

/**
 * @brief Функция записи данных во внутренние регистры BMP280.
 * 
 * @param BMP280_hi2c Указатель на конфигурацию интерфейса I2C.
 * @param addres Адрес первого регистра для записи.
 * @param data Указатель на данные для записи.
 * @param data_length Длина данных.
 */
void BMP280_I2C_Write(I2C_HandleTypeDef *hi2c, uint8_t addres, uint8_t* data, uint16_t data_length) {
	HAL_I2C_Master_Transmit(hi2c, addres, data, data_length, I2C_TIMEOUT_DEFAULT);
}

/**
 * @brief Функция чтения данных из внутренних регистров BMP280.
 * 
 * @param BMP280_hi2c Указатель на конфигурацию интерфейса I2C.
 * @param addres Адрес первого регистра для чтения.
 * @param data Указатель на место хранения считанных данных.
 * @param data_length Длина данных для чтения.
 */
void BMP280_I2C_Read(I2C_HandleTypeDef *hi2c, uint8_t addres, uint8_t* data, uint16_t data_length) {
	HAL_I2C_Master_Receive(hi2c, addres, data, data_length, I2C_TIMEOUT_DEFAULT);
}

/**
 * @brief Функция инициализации барометра.
 * 
 * @param BMP280_hi2c Указатель на конфигурацию интерфейса I2C.
 */
void BMP280_Init (I2C_HandleTypeDef *hi2c) {
	BMP280_hi2c = hi2c;
	
	uint8_t data[2];

	// Настройки регистра CTRL_MEAS (стандартное разрешение).
	data[0] = CTRL_MEAS_ADRESS;
	data[1] = 0x25; // Настройки: давление x1, температура x1, forced mode.
	BMP280_I2C_Write(BMP280_hi2c, BMP280_ADRESS, data, 2);

	// Настройки регистра CONFIG (стандартное разрешение).
	data[0] = CONFIG_ADRESS;
	data[1] = 0x00; // Ожидание 0.5 мс, фильтр выключен, SPI отключен.
	BMP280_I2C_Write(BMP280_hi2c, BMP280_ADRESS, data, 2);

	// Чтение калибровочных коэффициентов.
	uint8_t calib_coefficients[26];
	uint8_t start_adress = CALIB00;
	BMP280_I2C_Write(BMP280_hi2c, BMP280_ADRESS, &start_adress, 1);
	BMP280_I2C_Read(BMP280_hi2c, BMP280_ADRESS, calib_coefficients, 26);

	dig_T1 = (uint16_t)(calib_coefficients[1] << 8 | calib_coefficients[0]);
	dig_T2 = (int16_t)(calib_coefficients[3] << 8 | calib_coefficients[2]);
	dig_T3 = (int16_t)(calib_coefficients[5] << 8 | calib_coefficients[4]);

	dig_P1 = (uint16_t)(calib_coefficients[7] << 8 | calib_coefficients[6]);
	dig_P2 = (int16_t)(calib_coefficients[9] << 8 | calib_coefficients[8]);
	dig_P3 = (int16_t)(calib_coefficients[11] << 8 | calib_coefficients[10]);
	dig_P4 = (int16_t)(calib_coefficients[13] << 8 | calib_coefficients[12]);
	dig_P5 = (int16_t)(calib_coefficients[15] << 8 | calib_coefficients[14]);
	dig_P6 = (int16_t)(calib_coefficients[17] << 8 | calib_coefficients[16]);
	dig_P7 = (int16_t)(calib_coefficients[19] << 8 | calib_coefficients[18]);
	dig_P8 = (int16_t)(calib_coefficients[21] << 8 | calib_coefficients[20]);
	dig_P9 = (int16_t)(calib_coefficients[23] << 8 | calib_coefficients[22]);
}

/**
 * @brief Функция чтения необработанных данных давления с барометра.
 * 
 * @param data Указатель на массив, в который будут сохранены необработанные данные давления.
 */
void BMP280_ReadPressure (uint8_t *data) {
	uint8_t start_adress = PRESS_MSB_ADRESS;
	BMP280_I2C_Write(BMP280_hi2c, BMP280_ADRESS, &start_adress, 1);
	BMP280_I2C_Read(BMP280_hi2c, BMP280_ADRESS, data, 3);
}

/**
 * @brief Функция чтения необработанных данных температуры с барометра.
 * 
 * @param data Указатель на массив, в который будут сохранены необработанные данные температуры.
 */
void BMP280_ReadTemp (uint8_t *data) {
	uint8_t start_adress = TEMP_MSB_ADRESS;
	BMP280_I2C_Write(BMP280_hi2c, BMP280_ADRESS, &start_adress, 1);
	BMP280_I2C_Read(BMP280_hi2c, BMP280_ADRESS, data, 3);
}

/**
 * @brief Функция обработки необработанных данных температуры (из документации).
 * 
 * @param adc_T Необработанные данные температуры, объединённые в одно значение.
 * @return int32_t Компенсированное значение температуры.
 */
int32_t bmp280_compensate_T_int32(int32_t adc_T) {
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

/**
 * @brief Функция обработки необработанных данных давления (из документации).
 * 
 * @param adc_P Необработанные данные давления, объединённые в одно значение.
 * @return uint32_t Компенсированное значение давления.
 */
uint32_t bmp280_compensate_P_int64(int32_t adc_P) {
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0) {
		return 0; // Avoid division by zero.
	}
	p = 1048576 - adc_P;
	p = (((p<<31) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)(p >> 8);
}

/**
 * @brief Функция получения данных давления.
 * 
 * @param hi2c Указатель на конфигурацию интерфейса I2C.
 * @return uint32_t Компенсированное значение давления.
 */
uint32_t BMP280_ReadCookedPressure() {
	uint8_t rawTemp[3];
	uint8_t rawPressure[3];
	int32_t adcT;
	int32_t adcP;
	uint32_t realPressure;

	BMP280_ReadTemp(rawTemp);
	BMP280_ReadPressure(rawPressure);
	
	adcT = ((int32_t)rawTemp[0] << 12) | ((int32_t)rawTemp[1] << 4) | ((int32_t)(rawTemp[2] >> 4));
    adcP = ((int32_t)rawPressure[0] << 12) | ((int32_t)rawPressure[1] << 4) | ((int32_t)(rawPressure[2] >> 4));

	bmp280_compensate_T_int32(adcT);
	realPressure = bmp280_compensate_P_int64(adcP);
	
	return realPressure;
}
