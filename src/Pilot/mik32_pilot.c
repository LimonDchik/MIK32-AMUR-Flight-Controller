#include "mik32_pilot.h"

const float rcCommand[4] = {0.0f, 0.0f, 0.0f, 1500.0f};

/**
 * @brief Получение команды от пилота через массив rcCommand. 
 * @return const float* Команда пилота, переданная через массив данных.
 */
const float* rcCommandGet() {
    return rcCommand;
}
