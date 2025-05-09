#ifndef MIK32_DSHOT
#define MIK32_DSHOT

#include <stdbool.h>	
#include <math.h>		

#include "mik32_hal.h"
#include "mik32_hal_timer32.h"
#include "mik32_hal_dma.h"

#define TIMER_CLOCK				32000000	// 100MHz

#define MHZ_TO_HZ(x) 			((x) * 1000000)

#define DSHOT600_HZ     		MHZ_TO_HZ(12)
#define DSHOT300_HZ     		MHZ_TO_HZ(6)
#define DSHOT150_HZ     		MHZ_TO_HZ(3)

#define MOTOR_BIT_0            	7
#define MOTOR_BIT_1            	14
#define MOTOR_BITLENGTH        	20

#define DSHOT_FRAME_SIZE       	16
#define DSHOT_DMA_BUFFER_SIZE   18

#define DSHOT_MIN_THROTTLE      48
#define DSHOT_MAX_THROTTLE     	2047
#define DSHOT_RANGE 			(DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

typedef enum
{
    DSHOT150,
    DSHOT300,
    DSHOT600
} dshotType_e;

void dshotInit(const dshotType_e dshot_type, 
					TIMER32_HandleTypeDef* ht32_2,
					TIMER32_CHANNEL_HandleTypeDef* ch_1,
					TIMER32_CHANNEL_HandleTypeDef* ch_2,
					TIMER32_CHANNEL_HandleTypeDef* ch_3,
					TIMER32_CHANNEL_HandleTypeDef* ch_4,
					DMA_InitTypeDef* hd,
					DMA_ChannelHandleTypeDef* hd_ch_0,
					DMA_ChannelHandleTypeDef* hd_ch_1,
					DMA_ChannelHandleTypeDef* hd_ch_2,
					DMA_ChannelHandleTypeDef* hd_ch_3);
void dshotWrite(uint16_t* motorValue);
void dshotDMAIRQ(void);


#endif // MIK32_DSHOT