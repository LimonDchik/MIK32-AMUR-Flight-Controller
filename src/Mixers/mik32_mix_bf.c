#include "mik32_mix_bf.h"
#include "mik32_pilot.h"


#define PID_MIXER_SCALING 1000.0f
#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
#define PWM_RANGE (PWM_RANGE_MAX - PWM_RANGE_MIN)
#define MIN(a, b) ((b)>(a)?(a):(b))

controlRateConfig_t currentControlRateProfile;

mixerRuntime_t mixerRuntime;

float throttle = 0;
float motorMixRange = 0;
float motorRangeMin = 0;
float motorRangeMax = 0;
float motorOutputMin = 0;
float motorOutputRange = 0;
int8_t motorOutputMixSign = 0;
int throttleAngleCorrection = 0;
bool yawMotorsReversed = false;

float constrainfMIX(const float amt, const float low, const float high);
float applyThrottleLimit(const float throttle);
float scaleRangef(const float x, const float srcFrom, const float srcTo, const float destFrom, const float destTo);
void applyMixerAdjustment(float *motorMix, const float motorMixMin, const float motorMixMax, const bool airmodeEnabled);
void calculateThrottleAndCurrentMotorEndpoints();
void applyMixToMotors(const float motorMix[MAX_SUPPORTED_MOTORS], const motorMixer_t *activeMixer, float* motor);


float constrainfMIX(const float amt, const float low, const float high) {
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

float applyThrottleLimit(const float throttle) {
    if (currentControlRateProfile.throttleLimitPercent < 100) {
        const float throttleLimitFactor = currentControlRateProfile.throttleLimitPercent / 100.0f;
        switch (currentControlRateProfile.throttleLimitType) {
            case THROTTLE_LIMIT_TYPE_SCALE:
                return throttle * throttleLimitFactor;
            case THROTTLE_LIMIT_TYPE_CLIP:
                return MIN(throttle, throttleLimitFactor);
        }
    }
    return throttle;
}

float scaleRangef(const float x, const float srcFrom, const float srcTo, const float destFrom, const float destTo) {
    float a = (destTo - destFrom) * (x - srcFrom);
    float b = srcTo - srcFrom;
    return (a / b) + destFrom;
}

void applyMixerAdjustment(float *motorMix, const float motorMixMin, const float motorMixMax, const bool airmodeEnabled) {
    float airmodeTransitionPercent = 1.0f;
    if (!airmodeEnabled && throttle < 0.5f) {
        airmodeTransitionPercent = scaleRangef(throttle, 0.0f, 0.5f, 0.5f, 1.0f);
    }
    const float motorMixNormalizationFactor = motorMixRange > 1.0f ? airmodeTransitionPercent / motorMixRange : airmodeTransitionPercent;

    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motorMix[i] *= motorMixNormalizationFactor;
    }
    const float normalizedMotorMixMin = motorMixMin * motorMixNormalizationFactor;
    const float normalizedMotorMixMax = motorMixMax * motorMixNormalizationFactor;
    throttle = constrainfMIX(throttle, -normalizedMotorMixMin, 1.0f - normalizedMotorMixMax);
}

void calculateThrottleAndCurrentMotorEndpoints() {
    float currentThrottleInputRange = 0;
    const float* rcCommand = rcCommandGet();
    throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN + throttleAngleCorrection;
    currentThrottleInputRange = PWM_RANGE;
    motorRangeMax = mixerRuntime.motorOutputHigh;
    motorRangeMin = mixerRuntime.motorOutputLow;
    motorOutputMin = motorRangeMin;
    motorOutputRange = motorRangeMax - motorRangeMin;
    motorOutputMixSign = 1;
    throttle = constrainfMIX(throttle / currentThrottleInputRange, 0.0f, 1.0f);
}

void applyMixToMotors(const float motorMix[MAX_SUPPORTED_MOTORS], const motorMixer_t *activeMixer, float* motor) {
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        float motorOutput = motorOutputMixSign * motorMix[i] + throttle * activeMixer[i].throttle;
        motorOutput = motorOutputMin + throttle * motorOutput;
        motorOutput = constrainfMIX(motorOutput, motorRangeMin, motorRangeMax);
        motor[i] = motorOutput;
    }
}

/**
 * @brief Основная функция для вычисления миксера.
 * 
 * @param pidSum Сумма PID значений.
 * @param pidSumLimit Ограничение для суммы PID.
 * @param yawPidSumLimit Ограничение для суммы yaw PID.
 * @param motor Массив выходных значений для моторов.
 */
void mixTable(const float* pidSum, const uint16_t pidSumLimit, const uint16_t yawPidSumLimit, float* motor) {
    const bool launchControlActive = false;
    const bool airmodeEnabled = false;
    calculateThrottleAndCurrentMotorEndpoints();
    motorMixer_t* activeMixer = &mixerRuntime.currentMixer[0];
    const float scaledAxisPidRoll = constrainfMIX(pidSum[FD_ROLL], -pidSumLimit, pidSumLimit) / PID_MIXER_SCALING;
    const float scaledAxisPidPitch = constrainfMIX(pidSum[FD_PITCH], -pidSumLimit, pidSumLimit) / PID_MIXER_SCALING;
    float scaledAxisPidYaw = constrainfMIX(pidSum[FD_YAW], -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;

    if (!yawMotorsReversed) {
        scaledAxisPidYaw = -scaledAxisPidYaw;
    }
    if (currentControlRateProfile.throttleLimitType != THROTTLE_LIMIT_TYPE_OFF) {
        throttle = applyThrottleLimit(throttle);
    }
    pidUpdateAntiGravityThrottleFilter(throttle);
    pidUpdateTpaFactor(throttle);
    float motorMix[MAX_SUPPORTED_MOTORS];
    float motorMixMax = 0, motorMixMin = 0;
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        float mix = scaledAxisPidRoll * activeMixer[i].roll +
                    scaledAxisPidPitch * activeMixer[i].pitch +
                    scaledAxisPidYaw * activeMixer[i].yaw;
        if (mix > motorMixMax) {
            motorMixMax = mix;
        } else if (mix < motorMixMin) {
            motorMixMin = mix;
        }
        motorMix[i] = mix;
    }
    motorMixRange = motorMixMax - motorMixMin;
    applyMixerAdjustment(motorMix, motorMixMin, motorMixMax, airmodeEnabled);
    applyMixToMotors(motorMix, activeMixer, motor);
}

/**
 * @brief Инициализация значений по умолчанию для миксера.
 */
void mixBfInitDefault() {
    currentControlRateProfile.throttleLimitType = THROTTLE_LIMIT_TYPE_OFF;
    currentControlRateProfile.throttleLimitPercent = 100;
    mixerRuntime.motorCount = MAX_SUPPORTED_MOTORS;
    mixerRuntime.currentMixer[0] = (motorMixer_t){ .throttle = 1.0f, .roll = -1.0f, .pitch =  1.0f, .yaw = -1.0f }; // Front Left
    mixerRuntime.currentMixer[1] = (motorMixer_t){ .throttle = 1.0f, .roll =  1.0f, .pitch =  1.0f, .yaw =  1.0f }; // Front Right
    mixerRuntime.currentMixer[2] = (motorMixer_t){ .throttle = 1.0f, .roll = -1.0f, .pitch = -1.0f, .yaw =  1.0f }; // Rear Left
    mixerRuntime.currentMixer[3] = (motorMixer_t){ .throttle = 1.0f, .roll =  1.0f, .pitch = -1.0f, .yaw = -1.0f }; // Rear Right
    mixerRuntime.motorOutputLow = 1050.0f;
    mixerRuntime.motorOutputHigh = 1950.0f;
    mixerRuntime.disarmMotorOutput = 1000.0f;
}

void mixBFSetThrottleLimitType(const throttleLimitType_e userThrottleLimitType) {
    currentControlRateProfile.throttleLimitType = userThrottleLimitType;
}

void mixBFSetThrottleLimitPercent(const uint8_t userThrottleLimitPercent) {
    currentControlRateProfile.throttleLimitPercent = userThrottleLimitPercent;
}

void mixBFSetMotorCount(const uint8_t userMotorCount) {
    mixerRuntime.motorCount = userMotorCount;
}

void mixBFSetCurrentMixer(const float* frontLeft, const float* frontRight, const float* rearLeft, const float* rearRight) {
    mixerRuntime.currentMixer[0] = (motorMixer_t){ .throttle = frontLeft[0], .roll = frontLeft[1], .pitch =  frontLeft[2], .yaw = frontLeft[3]}; // Front Left
    mixerRuntime.currentMixer[1] = (motorMixer_t){ .throttle = frontRight[0], .roll = frontRight[1], .pitch =  frontRight[2], .yaw = frontRight[3]}; // Front Right
    mixerRuntime.currentMixer[2] = (motorMixer_t){ .throttle = rearLeft[0], .roll = rearLeft[1], .pitch =  rearLeft[2], .yaw = rearLeft[3]}; // Rear Left
    mixerRuntime.currentMixer[3] = (motorMixer_t){ .throttle = rearRight[0], .roll = rearRight[1], .pitch =  rearRight[2], .yaw = rearRight[3]}; // Rear Right
}

void mixBFSetMotorOutputLow(const float userMotorOutputLow) {
    mixerRuntime.motorOutputLow = userMotorOutputLow;
}

void mixBFSetMotorOutputHigh(const float userMotorOutputHigh) {
    mixerRuntime.motorOutputHigh = userMotorOutputHigh;
}

void mixBFSetDisarmMotorOutput(const float userDisarmMotorOutput) {
    mixerRuntime.disarmMotorOutput = userDisarmMotorOutput;
}

void mixerSetThrottleAngleCorrection(const int correctionValue) {
    throttleAngleCorrection = correctionValue;
}

void mixBFSetYawMotorsReversed(const bool userYawMotorsReversed) {
    yawMotorsReversed = userYawMotorsReversed;
}
