#include <mik32_msp.h>
#include <pad_config.h>
#include <gpio.h>
#include <power_manager.h>

#include <gpio_irq.h>
#include <epic.h>
#include <scr1_csr_encoding.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "uart_lib.h"
#include "xprintf.h"
#include "fixmath.h"

#include "mik32_hal_i2c.h"

#define GYRO_SENSITIVITY            65.5f   // LSB/(grad/s)
#define GYRO_ADDR  0x68
#define I2C_TIMEOUT (I2C_TIMEOUT_DEFAULT)
#define GYRO_CALIB_ITERATIONS_NUM   2000
#define GYRO_FILTER_RATE 1.0f

I2C_HandleTypeDef hi2c1;

SemaphoreHandle_t xSemaphore;

//float mixer[] = {1500.0f, 1500.0f, 1500.0f, 1500.0f};
//fix32_t - это int64_t
volatile fix32_t  gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
volatile fix32_t  gyro_roll_flt = 0, gyro_pitch_flt = 0, gyro_yaw_flt = 0;
volatile fix32_t  gyro_roll_bias = 0, gyro_pitch_bias = 0, gyro_yaw_bias = 0;
volatile fix32_t  acc_x = 0, acc_y = 0, acc_z = 0;

void SystemClockConfig(void);
static void LED_task(void *param);
static void MPU_task(void *param);

void I2C1_Init(void);
void gyro_init(void);
void gyro_processing(void);
void gyro_calibration(void);

void ext_trap_handler()
{
    HAL_EPIC_Clear(0xFFFFFFFF);
	xSemaphoreGiveFromISR(xSemaphore, NULL);
}

int main()
{
	SystemClockConfig();
	
	UART_Init(UART_0, 278, UART_CONTROL1_RE_M | UART_CONTROL1_TE_M | UART_CONTROL1_M_8BIT_M, 0, UART_CONTROL3_DMAR_M | UART_CONTROL3_DMAT_M);
	I2C1_Init();
	gyro_init();
	
	//DSHOT_init(0);
	xTaskCreate (LED_task, "LED_task", 128, ( void * ) 1, tskIDLE_PRIORITY + 1 , NULL );
	xTaskCreate (MPU_task, "MPU_task", 128, ( void * ) 1, tskIDLE_PRIORITY + 2 , NULL );
	xSemaphore = xSemaphoreCreateBinary();
	
	__HAL_PCC_GPIO_2_CLK_ENABLE();
	__HAL_PCC_GPIO_0_CLK_ENABLE();
	HAL_GPIO_PinConfig(GPIO_2, GPIO_PIN_7, HAL_GPIO_MODE_GPIO_OUTPUT, HAL_GPIO_PULL_UP, HAL_GPIO_DS_2MA);
	HAL_GPIO_PinConfig(GPIO_0, GPIO_PIN_8, HAL_GPIO_MODE_GPIO_OUTPUT, HAL_GPIO_PULL_NONE, HAL_GPIO_DS_2MA);
	
	
	__HAL_PCC_EPIC_CLK_ENABLE();
	//HAL_EPIC_MaskLevelSet();
	HAL_IRQ_EnableInterrupts();
	
	gyro_calibration();
	
	vTaskStartScheduler();
	

    while (1){
		//HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_7);
	}
}

void SystemClockConfig(void) {
	PCC_InitTypeDef PCC_OscInit = {0};
    PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
    PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
    PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
    PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
    PCC_OscInit.AHBDivider = 0;
    PCC_OscInit.APBMDivider = 0;
    PCC_OscInit.APBPDivider = 0;
    PCC_OscInit.HSI32MCalibrationValue = 128;
    PCC_OscInit.LSI32KCalibrationValue = 8;
    PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_AUTO;
    PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
    HAL_PCC_Config(&PCC_OscInit);
}

static void LED_task(void *param)
{
	while(1)
	{
		for(int i = 0; i < 10000000; i++);
		xSemaphoreGive(xSemaphore);
	}
}

static void MPU_task(void *param)
{

	while(1)
	{
		//DSHOT_send(mixer);
		gyro_processing();
		
		xprintf("gyro_roll_flt = %d\n", fix32_to_int(gyro_roll_flt));
		xprintf("gyro_pitch_flt = %d\n", fix32_to_int(gyro_pitch_flt));
		xprintf("gyro_yaw_flt = %d\n", fix32_to_int(gyro_yaw_flt));
		HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_7);
		for(volatile int i = 0; i < 1000000; i++);
		
	}
	
}

void I2C1_Init(void)
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


void gyro_init(void)
{
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

void gyro_processing(void)
{
    volatile uint8_t txBuf[1] = {0x3B};
    volatile uint8_t data[14] = {0};
    uint32_t acc_axis[3], gyro_axis[3];
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
   
    acc_x = fix32_from_int(-acc_axis[0]);
    acc_y = fix32_from_int(-acc_axis[1]);
    acc_z = fix32_from_int(-acc_axis[2]);
}

void gyro_calibration(void)
{
    fix32_t gyro_roll_buf = 0, gyro_pitch_buf = 0, gyro_yaw_buf = 0;
    for (volatile uint32_t cal_int = 0; cal_int < GYRO_CALIB_ITERATIONS_NUM; cal_int++)
    {
        gyro_processing();
        gyro_roll_buf  += (gyro_roll);
        gyro_pitch_buf += (gyro_pitch);
        gyro_yaw_buf   += (gyro_yaw);
        if (fix32_mod(fix32_mul(fix32_from_int((int)cal_int), fix32_from_float(100.0f / (float)GYRO_CALIB_ITERATIONS_NUM)), fix32_from_int((int)10)) > fix32_from_float(9.9f))
            xprintf("\e[1A\rCalibration progress: %d                    \n\r", fix32_to_int(fix32_mul(fix32_from_int((int)cal_int), fix32_from_float(100.0f / (float)GYRO_CALIB_ITERATIONS_NUM))));
    }
    gyro_roll_bias  = fix32_mul(gyro_roll_buf,  fix32_from_float(1.0f/GYRO_CALIB_ITERATIONS_NUM));
    gyro_pitch_bias = fix32_mul(gyro_pitch_buf, fix32_from_float(1.0f/GYRO_CALIB_ITERATIONS_NUM));
    gyro_yaw_bias   = fix32_mul(gyro_yaw_buf,   fix32_from_float(1.0f/GYRO_CALIB_ITERATIONS_NUM));
    xprintf("\n Calibration done! \n");
}