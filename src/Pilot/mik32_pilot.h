#ifndef MIK32_PILOT
#define MIK32_PILOT

#define MAX_ICM_CALIBR_CYCLES 1250

typedef enum rcAlias {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE
} rcAlias_e;

const float* rcCommandGet(void);

#endif
