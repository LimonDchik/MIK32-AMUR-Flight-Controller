#ifndef MIK32_PID_BF
#define MIK32_PID_BF

#include <stdint.h>
#include <stdbool.h>
#include "mik32_filters.h"

#define XYZ_AXIS_COUNT 3

typedef struct pidAxisData_s {
    float P;
    float I;
    float D;
    float F;
    float S;
    float Sum;
} pidAxisData_t;

typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidCoefficient_t;

typedef enum {
    FD_ROLL = 0,
    FD_PITCH,
    FD_YAW
} flightDynamicsIndex_t;

/**
 * @brief Основная структура, содержащая настройки для ПИД-регуляции.
 */
typedef struct pidRuntime_s {
    float dT;
    float pidFrequency;
    bool pidStabilisationEnabled;
    float previousPidSetpoint[XYZ_AXIS_COUNT];
    pt1Filter_t dtermLowpass[XYZ_AXIS_COUNT];
    pt1Filter_t ptermYawLowpass;
    pt2Filter_t antiGravityLpf;
    bool antiGravityEnabled;
    float antiGravityThrottleD;
    float itermAccelerator;
    uint8_t antiGravityGain;
    float antiGravityPGain;
    pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];
    float maxVelocity[XYZ_AXIS_COUNT];
    float itermLimit;
    float itermLimitYaw;
    bool zeroThrottleItermReset;
    float tpaFactor;
    float tpaLowBreakpoint;
    float tpaMultiplier;
    bool tpaLowAlways;
    float tpaLowMultiplier;
    float tpaBreakpoint;
    uint16_t pidSumLimit;
    uint16_t pidSumLimitYaw;
} pidRuntime_t;

void pidBFInitDefault(void);
void pidController(const float* gyroADCf, float* pidSums);

void pidBFSetPidFrequencyDt(const float pidFrequencyUser);
void pidBFSetStabilisationEnabled(const bool enabled);
void pidBFSetPidCoefficient(const uint8_t axis, const float kp, const float ki, const float kd);
void pidBFSetDtermLowpass(const uint8_t axis, const float cutoffFrequencyHz);
void pidBFSetMaxVelocity(const uint8_t axis, const float maxVel);
void pidBFSetPtermYawLowpass(const float cutoffFrequencyHz);
void pidBFSetAntiGravity(const bool enabled);
void pidBFSetAntiGravityGain(const float gain);
void pidBFSetAntiGravityPGain(const float pGain);
void pidBFSetItermAccelerator(const float value);
void pidBFSetAntiGravityThrottleD(const float value);
void pidBFSetAntiGravityLPF(const float cutoffFrequencyHz);
void pidBFSetItermLimit(const float limit);
void pidBFSetItermLimitYaw(const float limitYaw);
void pidBFSetZeroThrottleItermReset(bool reset);
void pidBFSetTpaFactor(const float tpa);
void pidBFSetTpaLowBreakpoint(const float tpaLowBreakpoint);
void pidBFSetTpaMultiplier(const float multiplier);
void pidBFSetTpaLowAlways(const bool always);
void pidBFSetTpaLowMultiplier(const float multiplier);
void pidBFSetTpaBreakpoint(const float tpaBreakpoint);

uint16_t pidBFGetPidSumLimit(void); //Требуется для миксеров.
uint16_t pidBFGetPidSumLimitYaw(void); //Требуется для миксеров.

void pidUpdateAntiGravityThrottleFilter(const float throttle); //Требуется для миксеров.
void pidUpdateTpaFactor(float throttle); //Требуется для миксеров.

void rawSetpointUpdate(const float roll, const float pitch, const float yaw);

#endif // MIK32_PID_BF
