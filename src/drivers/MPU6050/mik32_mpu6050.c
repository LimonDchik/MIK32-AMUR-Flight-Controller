#include "mik32_mpu6050.h"
#include "fixmath.h"
#include "mik32_hal_i2c.h"

#define GYRO_SENSITIVITY            65.5f   // LSB/(grad/s)
#define GYRO_ADDR  0x68
#define I2C_TIMEOUT (I2C_TIMEOUT_DEFAULT)
#define GYRO_CALIB_ITERATIONS_NUM   2000
#define GYRO_FILTER_RATE 1.0f

volatile I2C_HandleTypeDef hi2c1;

volatile fix32_t  gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
volatile fix32_t  gyro_roll_flt = 0, gyro_pitch_flt = 0, gyro_yaw_flt = 0;
volatile fix32_t  gyro_roll_bias = 0, gyro_pitch_bias = 0, gyro_yaw_bias = 0;
volatile fix32_t  acc_x = 0, acc_y = 0, acc_z = 0;

void I2C1_Init(void);

void I2C1_Init()
{
    /* Общие настройки */
    hi2c1.Instance = I2C_1;
    hi2c1.Init.Mode = HAL_I2C_MODE_MASTER;
    hi2c1.Init.DigitalFilter = I2C_DIGITALFILTER_OFF;
    hi2c1.Init.AnalogFilter = I2C_ANALOGFILTER_DISABLE;
    hi2c1.Init.AutoEnd = I2C_AUTOEND_ENABLE;
    /* Настройка частоты */
    hi2c1.Clock.PRESC  = 8;
    hi2c1.Clock.SCLDEL = 15;
    hi2c1.Clock.SDADEL = 0;
    hi2c1.Clock.SCLH   = 15;
    hi2c1.Clock.SCLL   = 15;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        xprintf("I2C_Init error\n\r");
    }
}

void gyro_init()
{
	I2C1_Init();
	
    uint8_t initBuf[2]       = {0x6B, 0};
    uint8_t confGyroBuf[2]   = {0x1B, 0x08};
    uint8_t confAccBuf[2]    = {0x1C, 0x10};
    uint8_t confFilterBuf[2] = {0x1A, 0x03}; // фильтр на 43 Гц
    xprintf("\n\rGyro Init\n\r");
    // Включение MPU-6050
    HAL_StatusTypeDef error_code = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, initBuf, sizeof(initBuf), I2C_TIMEOUT);
    // Настройка гироскопа на пределы ±500 градусов/сек
    error_code = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, confGyroBuf, sizeof(confGyroBuf), I2C_TIMEOUT);
    // Настройка акселерометра на пределы ±8g 
    error_code = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, confAccBuf, sizeof(confAccBuf), I2C_TIMEOUT);
    // Настройка цифрового фильтра нижних частот
    error_code = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, confFilterBuf, sizeof(confFilterBuf), I2C_TIMEOUT);
}

void gyro_processing(int* gyro_user)
{
    volatile uint8_t txBuf[1] = {0x3B};
    volatile uint8_t data[14] = {0};
    volatile uint32_t acc_axis[3], gyro_axis[3];
    HAL_StatusTypeDef error_code = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDR, txBuf, sizeof(txBuf), I2C_TIMEOUT);
    error_code = HAL_I2C_Master_Receive(&hi2c1, GYRO_ADDR, data, sizeof(data), I2C_TIMEOUT);
    // TODO: как-то обрабатывать ошибки в будущем
    acc_axis[0]  = (int)((int16_t)((uint16_t)data[0] << 8 | data[1]));
    acc_axis[1]  = (int)((int16_t)((uint16_t)data[2] << 8 | data[3]));
    acc_axis[2]  = (int)((int16_t)((uint16_t)data[4] << 8 | data[5]));
    // temperature  = (int)((int16_t)((uint16_t)data[6] << 8 | data[7]));
    gyro_axis[0] = (int)((int16_t)((uint16_t)data[8] << 8 | data[9]));
    gyro_axis[1] = (int)((int16_t)((uint16_t)data[10] << 8 | data[11]));
    gyro_axis[2] = (int)((int16_t)((uint16_t)data[12] << 8 | data[13]));
    gyro_roll  = fix32_mul(fix32_from_int(-gyro_axis[1]),fix32_from_float(1/GYRO_SENSITIVITY));
    gyro_pitch = fix32_mul(fix32_from_int(-gyro_axis[0]),fix32_from_float(1/GYRO_SENSITIVITY));
    gyro_yaw   = fix32_mul(fix32_from_int(-gyro_axis[2]),fix32_from_float(1/GYRO_SENSITIVITY));
    gyro_roll  -= gyro_roll_bias;
    gyro_pitch -= gyro_pitch_bias;
    gyro_yaw   -= gyro_yaw_bias;
    // фильтруем гироскоп для определения углов
    gyro_roll_flt   = fix32_mul(gyro_roll_flt, fix32_from_float(1-GYRO_FILTER_RATE)) + fix32_mul(gyro_roll, fix32_from_float(GYRO_FILTER_RATE));
    gyro_pitch_flt  = fix32_mul(gyro_pitch_flt,fix32_from_float(1-GYRO_FILTER_RATE)) + fix32_mul(gyro_pitch,fix32_from_float(GYRO_FILTER_RATE));
    gyro_yaw_flt    = fix32_mul(gyro_yaw_flt,  fix32_from_float(1-GYRO_FILTER_RATE)) + fix32_mul(gyro_yaw  ,fix32_from_float(GYRO_FILTER_RATE));
    gyro_user[0] = fix32_to_int(gyro_roll_flt);
	gyro_user[1] = fix32_to_int(gyro_pitch_flt);
	gyro_user[2] = fix32_to_int(gyro_yaw_flt);
   
    acc_x = fix32_from_int(-acc_axis[0]);
    acc_y = fix32_from_int(-acc_axis[1]);
    acc_z = fix32_from_int(-acc_axis[2]);
}

void gyro_calibration()
{
	volatile int empty[] = {0, 0, 0};
    volatile fix32_t gyro_roll_buf = 0, gyro_pitch_buf = 0, gyro_yaw_buf = 0;
    for (volatile uint32_t cal_int = 0; cal_int < GYRO_CALIB_ITERATIONS_NUM; cal_int++)
    {
        gyro_processing(empty);
        gyro_roll_buf  += (gyro_roll);
        gyro_pitch_buf += (gyro_pitch);
        gyro_yaw_buf   += (gyro_yaw);
        if (fix32_mod(fix32_mul(fix32_from_int((int)cal_int), fix32_from_float(100.0f / (float)GYRO_CALIB_ITERATIONS_NUM)), fix32_from_int((int)10)) > fix32_from_float(9.9f)){};
            xprintf("\e[1A\rCalibration progress: %d                    \n\r", fix32_to_int(fix32_mul(fix32_from_int((int)cal_int), fix32_from_float(100.0f / (float)GYRO_CALIB_ITERATIONS_NUM))));
    }
    gyro_roll_bias  = fix32_mul(gyro_roll_buf,  fix32_from_float(1.0f/GYRO_CALIB_ITERATIONS_NUM));
    gyro_pitch_bias = fix32_mul(gyro_pitch_buf, fix32_from_float(1.0f/GYRO_CALIB_ITERATIONS_NUM));
    gyro_yaw_bias   = fix32_mul(gyro_yaw_buf,   fix32_from_float(1.0f/GYRO_CALIB_ITERATIONS_NUM));
    xprintf("\n Calibration done! \n");
}