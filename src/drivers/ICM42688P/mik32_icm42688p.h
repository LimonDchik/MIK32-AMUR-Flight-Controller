#ifndef MIK32_ICM42688
#define MIK32_ICM42688

#include "mik32_hal_spi.h"

void ICM42688P_Init(SPI_HandleTypeDef* user_hspi);
void ICM42688P_ReadGyroData(float* gyroADC);


#endif // MIK32_ICM42688