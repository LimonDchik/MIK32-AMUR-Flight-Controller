#ifndef MIK32_QMC5883L
#define MIK32_QMC5883L

#include <stdint.h>
#include "mik32_hal_i2c.h"


/**
 * @brief Инициализация датчика.
 * 
 */
int QMC5883L_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Считывает показания датчика.
 * 
 * @param mag_data массив данных, где data[0] = X; data[1] = Y; data[2] = Z.
 */
void QMC5883L_ReadData(int16_t* magData);


#endif // MIK32_QMC5883L