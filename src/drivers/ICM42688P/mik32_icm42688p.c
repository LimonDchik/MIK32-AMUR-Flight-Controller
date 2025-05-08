#include "mik32_icm42688p.h"
#include "mik32_pilot.h"

#define ICM426XX_RA_REG_BANK_SEL                    0x76
#define ICM426XX_RA_PWR_MGMT0                       0x4E
#define ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF      0x00
#define ICM426XX_RA_GYRO_ACCEL_CONFIG0              0x52
#define ICM426XX_RA_GYRO_CONFIG0                    0x4F

#define ICM426XX_BANK_SELECT0                       0x00
#define ICM426XX_BANK_SELECT1                       0x01
#define ICM426XX_BANK_SELECT2                       0x02
#define ICM426XX_BANK_SELECT3                       0x03
#define ICM426XX_BANK_SELECT4                       0x04

#define ICM426XX_RA_ACCEL_CONFIG_STATIC2            0x03
#define ICM426XX_RA_ACCEL_CONFIG_STATIC3            0x04
#define ICM426XX_RA_ACCEL_CONFIG_STATIC4            0x05

#define ICM426XX_RA_GYRO_CONFIG_STATIC3             0x0C
#define ICM426XX_RA_GYRO_CONFIG_STATIC4             0x0D
#define ICM426XX_RA_GYRO_CONFIG_STATIC5             0x0E

#define ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY       (15 << 4)
#define ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY        (15 << 0)

#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)

#define ICM426XX_INTF_CONFIG1_AFSR_MASK             0xC0
#define ICM426XX_INTF_CONFIG1_AFSR_DISABLE          0x40
#define ICM426XX_INTF_CONFIG1                       0x4D

#define ICM426XX_RA_GYRO_DATA_X1                    0x25

#define XYZ_AXIS_COUNT 3

SPI_HandleTypeDef* ICM42688P_hspi;

uint8_t initOutput[2];
uint8_t initInput[2];
uint8_t dataOutput[6];
uint8_t dataInput[6];
uint32_t cyclesRemaining;
float gyroZero[XYZ_AXIS_COUNT];

void performGyroCalibration(int16_t* gyroADCRaw, uint32_t cyclesRemaining, float* sum);
void SPI0_Init(void);

/**
 * @brief Инициализация ICM42688P.
 *
 * @param user_hspi Указатель на конфигурацию SPI.
 */
void ICM42688P_Init(SPI_HandleTypeDef* user_hspi) {
	//Инициализация SPI
	ICM42688P_hspi = user_hspi;
 	SPI0_Init();
	
	//Выбор банка 0.
	initOutput[0] = ICM426XX_RA_REG_BANK_SEL;
	initOutput[1] = ICM426XX_BANK_SELECT0;
	HAL_SPI_Exchange(ICM42688P_hspi, initOutput, initInput, 2, SPI_TIMEOUT_DEFAULT);
	
	//Выключение гироскопа и акселерометра.
	initOutput[0] = ICM426XX_RA_PWR_MGMT0;
	initOutput[1] = ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF;
	HAL_SPI_Exchange(ICM42688P_hspi, initOutput, initInput, 2, SPI_TIMEOUT_DEFAULT);	
	
	//Выбор банка 1.
	initOutput[0] = ICM426XX_RA_REG_BANK_SEL;
	initOutput[1] = ICM426XX_BANK_SELECT1;
	HAL_SPI_Exchange(ICM42688P_hspi, initOutput, initInput, 2, SPI_TIMEOUT_DEFAULT);	
	
	//Инициализация Anti-alias фильтра.
	//Данные взяты из секции 5.3 документации.
	uint8_t GYRO_AAF_DELT = 6;
	uint8_t GYRO_AAF_DELTSQR = 36;
	uint8_t GYRO_AAF_BITSHIFT = 10;
	initOutput[0] = ICM426XX_RA_GYRO_CONFIG_STATIC3;
	initOutput[1] = GYRO_AAF_DELT;
	HAL_SPI_Exchange(ICM42688P_hspi, initOutput, initInput, 2, SPI_TIMEOUT_DEFAULT);
	initOutput[0] = ICM426XX_RA_GYRO_CONFIG_STATIC4;
	initOutput[1] = GYRO_AAF_DELTSQR & 0xFF;
	HAL_SPI_Exchange(ICM42688P_hspi, initOutput, initInput, 2, SPI_TIMEOUT_DEFAULT);
	initOutput[0] = ICM426XX_RA_GYRO_CONFIG_STATIC5;
	initOutput[1] = (GYRO_AAF_DELTSQR >> 8) | (GYRO_AAF_BITSHIFT << 4);
	HAL_SPI_Exchange(ICM42688P_hspi, initOutput, initInput, 2, SPI_TIMEOUT_DEFAULT);	

	//Выбор банка 0.
	initOutput[0] = ICM426XX_RA_REG_BANK_SEL;
	initOutput[1] = ICM426XX_BANK_SELECT0;
	HAL_SPI_Exchange(ICM42688P_hspi, initOutput, initInput, 2, SPI_TIMEOUT_DEFAULT);	
	
	//Настройка UI фильтра.
	initOutput[0] = ICM426XX_RA_GYRO_ACCEL_CONFIG0;
	initOutput[1] = ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY;
	HAL_SPI_Exchange(ICM42688P_hspi, initOutput, initInput, 2, SPI_TIMEOUT_DEFAULT);	
	
	//Отключение AFSR.
	initOutput[0] = ICM426XX_INTF_CONFIG1;
	initOutput[1] = 0x00;
	HAL_SPI_Exchange(ICM42688P_hspi, initOutput, initInput, 1, SPI_TIMEOUT_DEFAULT);		
	uint8_t intfConfig1Value = initInput[0];
	intfConfig1Value &= ~ICM426XX_INTF_CONFIG1_AFSR_MASK;
    intfConfig1Value |= ICM426XX_INTF_CONFIG1_AFSR_DISABLE;
	initOutput[0] = ICM426XX_INTF_CONFIG1;
	initOutput[1] = intfConfig1Value;
	HAL_SPI_Exchange(ICM42688P_hspi, initOutput, initInput, 2, SPI_TIMEOUT_DEFAULT);		
	
	//Включение гироскопа.
	initOutput[0] = ICM426XX_RA_PWR_MGMT0;
	initOutput[1] = ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_GYRO_MODE_LN;
	HAL_SPI_Exchange(ICM42688P_hspi, initOutput, initInput, 2, SPI_TIMEOUT_DEFAULT);		
	
	//Установка максимального FSR и установка ODR.
	uint8_t odrConfig;
	odrConfig = 6;
	initOutput[0] = ICM426XX_RA_GYRO_CONFIG0;
	initOutput[1] = (0 << 5) | (odrConfig & 0x0F);
	HAL_SPI_Exchange(ICM42688P_hspi, initOutput, initInput, 2, SPI_TIMEOUT_DEFAULT);
	
	//Выбор банка 0.
	initOutput[0] = ICM426XX_RA_REG_BANK_SEL;
	initOutput[1] = ICM426XX_BANK_SELECT0;
	HAL_SPI_Exchange(ICM42688P_hspi, initOutput, initInput, 2, SPI_TIMEOUT_DEFAULT);

 	//Калибровка.
	int16_t gyroADCRaw[XYZ_AXIS_COUNT];
	float sumRaw[XYZ_AXIS_COUNT];
	uint32_t cyclesRemaining;
	dataInput[0] = 0x00;
	dataInput[1] = 0x00;
	dataInput[2] = 0x00;
	dataInput[3] = 0x00;
	dataInput[4] = 0x00;
	dataInput[5] = 0x00;
	dataOutput[0] = ICM426XX_RA_GYRO_DATA_X1 | 0x80; 
	dataOutput[1] = 0x00;        
	dataOutput[2] = 0x00;
	dataOutput[3] = 0x00;
	dataOutput[4] = 0x00;
	dataOutput[5] = 0x00;
	for(cyclesRemaining = MAX_ICM_CALIBR_CYCLES; cyclesRemaining > 0; --cyclesRemaining) {
		HAL_SPI_Exchange(ICM42688P_hspi, initOutput, dataInput, 6, SPI_TIMEOUT_DEFAULT);	
		gyroADCRaw[0] = (int16_t)((dataInput[0] << 8) | dataInput[1]);
		gyroADCRaw[1] = (int16_t)((dataInput[2] << 8) | dataInput[3]);
		gyroADCRaw[2] = (int16_t)((dataInput[4] << 8) | dataInput[5]);
		performGyroCalibration(gyroADCRaw, cyclesRemaining, sumRaw);
	} 
}

void performGyroCalibration(int16_t* gyroADCRaw, uint32_t cyclesRemaining, float* sum) {
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        if (cyclesRemaining == MAX_ICM_CALIBR_CYCLES) {
            sum[axis] = 0.0f;
            gyroZero[axis] = 0.0f;
        }
        sum[axis] += gyroADCRaw[axis];
        if (cyclesRemaining == 1) {
            gyroZero[axis] = sum[axis] / MAX_ICM_CALIBR_CYCLES;
        }
    }
}

/**
 * @brief Чтение регистров данных гироскопа ICM42688P.
 *
 * @param gyroADCf массив, в который будут сохранены данные.
 */
void ICM42688P_ReadGyroData(float* gyroADCf) {
	int16_t gyroADC[XYZ_AXIS_COUNT];
	dataOutput[0] = ICM426XX_RA_GYRO_DATA_X1 | 0x80; 
	dataOutput[1] = 0x00;        
	dataOutput[2] = 0x00;
	dataOutput[3] = 0x00;
	dataOutput[4] = 0x00;
	dataOutput[5] = 0x00;
	dataOutput[6] = 0x00;
	HAL_SPI_Exchange(ICM42688P_hspi, initOutput, dataInput, 6, SPI_TIMEOUT_DEFAULT);
	gyroADC[0] = (int16_t)((dataInput[0] << 8) | dataInput[1]);
    gyroADC[1] = (int16_t)((dataInput[2] << 8) | dataInput[3]);
    gyroADC[2] = (int16_t)((dataInput[4] << 8) | dataInput[5]);
	gyroADCf[0] = gyroADC[0] - gyroZero[0];
	gyroADCf[1] = gyroADC[1] - gyroZero[1];
	gyroADCf[2] = gyroADC[2] - gyroZero[2];
}


void SPI0_Init() {
    ICM42688P_hspi->Instance = SPI_0;
    ICM42688P_hspi->Init.SPI_Mode = HAL_SPI_MODE_MASTER;
    ICM42688P_hspi->Init.CLKPhase = SPI_PHASE_ON;
    ICM42688P_hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
    ICM42688P_hspi->Init.ThresholdTX = 4;
    ICM42688P_hspi->Init.BaudRateDiv = SPI_BAUDRATE_DIV64;
    ICM42688P_hspi->Init.Decoder = SPI_DECODER_NONE;
    ICM42688P_hspi->Init.ManualCS = SPI_MANUALCS_OFF;
    ICM42688P_hspi->Init.ChipSelect = SPI_CS_0;      
}