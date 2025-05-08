#include "mik32_qmc5883l.h"
#include "mik32_qmc5883l.h"

/**
 * @brief   I2C-адрес QMC5883L.
 */
#define QMC5883L_ADDR           (0x0d)

/**
 * @name   Флаги состояния устройства.
 * @{
 */
#define QMC5883L_DRDY       (0x01)
#define QMC5883L_OVL        (0x02)
#define QMC5883L_DOR        (0x04)
/** @} */

/**
 * @name   Поля конфигурации.
 * @{
 */
#define QMC5883L_INT_DISB    (0x01)
#define QMC5883L_ROL_PNT     (0x40)
#define QMC5883L_SOFT_RST    (0x80)

#define QMC5883L_RNG_2G (0x00 << 4)
#define QMC5883L_RNG_8G (0x01 << 4)

// Частота передискретизации.
// Чем больше значение OSR, тем уже полоса пропускания фильтра,
// меньше шум в полосе пропускания и выше энергопотребление.
#define QMC5883L_OSR_512 (0x00 << 6)
#define QMC5883L_OSR_256 (0x01 << 6)
#define QMC5883L_OSR_128 (0x10 << 6)
#define QMC5883L_OSR_64  (0x11 << 6)

#define QMC5883L_ODR_10HZ  (0x00 << 2)
#define QMC5883L_ODR_50HZ  (0x01 << 2)
#define QMC5883L_ODR_100HZ (0x02 << 2)
#define QMC5883L_ODR_200HZ (0x03 << 2)
/** @} */

/**
 * @name   Карта регистров.
 * @{
 */
#define QMC5883L_DOXL       (0x00)
#define QMC5883L_STATUS     (0x06)
#define QMC5883L_CTRL1      (0x09)
#define QMC5883L_CTRL2      (0x0a)
#define QMC5883L_SETRESET   (0x0b)
/** @} */

/**
 * @name   Режимы устройства.
 * @{
 */
#define QMC5883L_STANDBY    (0x00)
#define QMC5883L_CONT       (0x01)
/** @} */

// объединяет два uint8_t в один int16_t.
// H - страшие 8 бит.
// L - младшие 8 бит.
#define MERG_NUM(H, L) (((int)H << 8) | L)

I2C_HandleTypeDef *QMC5883L_hi2c;

void QMC5883L_WriteReg(const uint8_t reg, const uint8_t value);
void QMC5883L_ReadReg(const uint8_t reg, uint8_t* data, const uint16_t dataLen);

/**
 * @brief Функция для записи значений в регистры датчика.
 * 
 * @param reg адрес регистра.
 * @param data данные, которые нужно записать по адресу.
 */
void QMC5883L_WriteReg(const uint8_t reg, const uint8_t value) {
    uint8_t data[2] = {0};
    data[0] = reg; 
	data[1] = value;
    HAL_I2C_Master_Transmit(QMC5883L_hi2c, QMC5883L_ADDR, data, 2, I2C_TIMEOUT_DEFAULT);
}

/**
 * @brief Функция для чтения данных с датчика.
 * 
 * @param reg адрес регистра из которого нужно считать данные.
 * @param data массив, в который будут записаны данные.
 * @param dataLen длина массива.
 */
void QMC5883L_ReadReg(const uint8_t reg, uint8_t* data, const uint16_t dataLen) {
    uint8_t startAdress = reg;
    // Переводится указатель на начало данных.
    HAL_I2C_Master_Transmit(QMC5883L_hi2c, QMC5883L_ADDR, &startAdress, 1, I2C_TIMEOUT_DEFAULT);
    // Считываются данные.
    HAL_I2C_Master_Receive(QMC5883L_hi2c, QMC5883L_ADDR, data, dataLen, I2C_TIMEOUT_DEFAULT);
}

/**
 * @brief Инициализация датчика.
 * 
 */
int QMC5883L_Init(I2C_HandleTypeDef *hi2c) {
    QMC5883L_hi2c = hi2c;

    // soft reset датчика.
    QMC5883L_WriteReg(QMC5883L_CTRL2, QMC5883L_SOFT_RST);

    // Проверка, что reset прошел успшено.
    uint8_t status_reg_val = 42; // Такое число, чтобы было понятно, что read сработал.
    QMC5883L_ReadReg(QMC5883L_STATUS, &status_reg_val, 1);
    if (status_reg_val != 0) { return -1; }

    // Устанавливается период SET/RESET.
    // Рекомендуется 0х01.
    QMC5883L_WriteReg(QMC5883L_SETRESET, 0x01);

    // Выключается DRDY пин.
    QMC5883L_WriteReg(QMC5883L_CTRL2, QMC5883L_INT_DISB);

    QMC5883L_WriteReg(QMC5883L_CTRL1, (QMC5883L_CONT 
                                    | QMC5883L_ODR_200HZ
                                    | QMC5883L_OSR_512
                                    | QMC5883L_RNG_8G));

    return 0;
}

/**
 * @brief Считывает показания датчика.
 * 
 * @param magData массив данных, где data[0] = X; data[1] = Y; data[2] = Z.
 */
void QMC5883L_ReadData(int16_t* magData) {
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};

    QMC5883L_ReadReg(QMC5883L_DOXL, data, 6);

    magData[0] = MERG_NUM(data[0], data[1]); // X
    magData[1] = MERG_NUM(data[4], data[5]); // Y
    magData[2] = MERG_NUM(data[2], data[3]); // Z
}
