#ifndef MIK32_DSHOT
#define MIK32_DSHOT

#include "mik32_hal_timer32.h"
#include "mik32_hal_dma.h"



void DSHOT_init(uint8_t type);
void DSHOT_send();

#endif //MIK32_DSHOT