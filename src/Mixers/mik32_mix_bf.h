#ifndef MIK32_MIX_BF
#define MIK32_MIX_BF

#include "mik32_pid_bf.h"

#define MAX_SUPPORTED_MOTORS 4

typedef enum {
    THROTTLE_LIMIT_TYPE_OFF = 0,
    THROTTLE_LIMIT_TYPE_SCALE,
    THROTTLE_LIMIT_TYPE_CLIP,
    THROTTLE_LIMIT_TYPE_COUNT 
} throttleLimitType_e;

typedef struct motorMixer_s {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

typedef struct controlRateConfig_s {
    uint8_t throttleLimitType;
    uint8_t throttleLimitPercent;
} controlRateConfig_t;

/**
 * @brief Основная структура, содержащая настройки для расчёта миксеров.
 */
typedef struct mixerRuntime_s {
    uint8_t motorCount;
    motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];
    float motorOutputLow;
    float motorOutputHigh;
    float disarmMotorOutput;
} mixerRuntime_t;

void mixTable(const float* pidSum, const uint16_t pidSumLimit, const uint16_t yawPidSumLimit, float* motor);
void mixBfInitDefault(void);

void mixBFSetThrottleLimitType(const throttleLimitType_e userThrottleLimitType);
void mixBFSetThrottleLimitPercent(const uint8_t userThrottleLimitPercent);
void mixBFSetMotorCount(const uint8_t userMotorCount);
void mixBFSetCurrentMixer(const float* frontLeft, const float* frontRight, const float* rearLeft, const float* rearRight);
void mixBFSetMotorOutputLow(const float userMotorOutputLow);
void mixBFSetMotorOutputHigh(const float userMotorOutputHigh);
void mixBFSetDisarmMotorOutput(const float userDisarmMotorOutput);
void mixerSetThrottleAngleCorrection(const int correctionValue);
void mixBFSetYawMotorsReversed(const bool userYawMotorsReversed);

#endif //MIK32_MIX_BF
