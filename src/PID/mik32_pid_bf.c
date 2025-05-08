#include "mik32_pid_bf.h"

#define ANTIGRAVITY_KI 0.34f
#define MAX(a, b) ((a)>(b)?(a):(b))
#define PIDSUM_LIMIT 500
#define PIDSUM_LIMIT_YAW 400

float rawSetpoint[XYZ_AXIS_COUNT];
float previousGyroRateDterm[XYZ_AXIS_COUNT];

pidAxisData_t pidData[XYZ_AXIS_COUNT];

pidRuntime_t pidRuntime;

float accelerationLimit(const int axis, float currentPidSetpoint);
float constrainfPID(const float amt, const float low, const float high);
float getTpaFactorClassic(const float tpaArgument);


float accelerationLimit(const int axis, float currentPidSetpoint) { 
    static float previousSetpoint[XYZ_AXIS_COUNT];
    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];
	float absCurrentVelocity;
	if (currentVelocity < 0) {
		absCurrentVelocity =  currentVelocity * (-1);
	} else {
		absCurrentVelocity = currentVelocity;
	}
	
    if (absCurrentVelocity > pidRuntime.maxVelocity[axis]) {
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + pidRuntime.maxVelocity[axis] : previousSetpoint[axis] - pidRuntime.maxVelocity[axis];
    }

    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

float constrainfPID(const float amt, const float low, const float high) {
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

/**
 * @brief Основная функция, которая выполняет расчёт ПИД-регулятора для всех трёх осей.
 *
 * @param gyroADCf Указатель на массив отфильтрованных значений гироскопа по каждой оси.
 * @param pidSums Указатель на массив, в который сохраняется итоговая сумма ПИД-компонент.
 */
void pidController(const float* gyroADCf, float* pidSums) {
	// Anti Gravity
    if (pidRuntime.antiGravityEnabled) {
        pidRuntime.antiGravityThrottleD *= pidRuntime.antiGravityGain;
        pidRuntime.itermAccelerator = pidRuntime.antiGravityThrottleD * ANTIGRAVITY_KI;
    } else {
        pidRuntime.antiGravityThrottleD = 0.0f;
        pidRuntime.itermAccelerator = 0.0f;
    }
    float gyroRateDterm[XYZ_AXIS_COUNT];
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        gyroRateDterm[axis] = gyroADCf[axis];
		gyroRateDterm[axis] = pt1FilterApply(&pidRuntime.dtermLowpass[axis], gyroRateDterm[axis]);
	}
	// ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
		float currentPidSetpoint = rawSetpoint[axis];
		
        if (pidRuntime.maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }
		const float gyroRate = gyroADCf[axis];
		float errorRate = currentPidSetpoint - gyroRate;
		const float previousIterm = pidData[axis].I;
        float itermErrorRate = errorRate;
		
		// --------low-level gyro-based PID based on 2DOF PID controller. ----------

        // -----calculate P component
        pidData[axis].P = pidRuntime.pidCoefficient[axis].Kp * errorRate * pidRuntime.tpaFactor; //getTpaFactor(pidProfile, TERM_P); //Это нам вообще нужно? //PD и D
        if (axis == FD_YAW) {
			pidData[axis].P = pt1FilterApply(&pidRuntime.ptermYawLowpass, pidData[axis].P);
		}
		
        // -----calculate I component
        float Ki = pidRuntime.pidCoefficient[axis].Ki;
        float itermLimit = pidRuntime.itermLimit; // windup fraction of pidSumLimit
		
        // yaw iTerm has it's own limit based on pidSumLimitYaw
        if (axis == FD_YAW) {
            itermLimit = pidRuntime.itermLimitYaw;
            pidRuntime.itermAccelerator = 0.0f;
        }
		float iTermChange = (Ki + pidRuntime.itermAccelerator) * pidRuntime.dT * itermErrorRate;
		pidData[axis].I = constrainfPID(previousIterm + iTermChange, -itermLimit, itermLimit);

        // -----calculate D component
        float pidSetpointDelta = 0;
		pidRuntime.previousPidSetpoint[axis] = currentPidSetpoint; 
        if ((pidRuntime.pidCoefficient[axis].Kd > 0)) {
            const float delta = - (gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidRuntime.pidFrequency;
            float preTpaD = pidRuntime.pidCoefficient[axis].Kd * delta;
            pidData[axis].D = preTpaD * pidRuntime.tpaFactor; //getTpaFactor(pidProfile, TERM_D);
        } else {
            pidData[axis].D = 0;
        }
        previousGyroRateDterm[axis] = gyroRateDterm[axis];

        // -----calculate feedforward component
        const float feedforwardGain = pidRuntime.pidCoefficient[axis].Kf;
        pidData[axis].F = feedforwardGain * pidSetpointDelta;
		float agSetpointAttenuator;
        if (axis != FD_YAW) {
			if (currentPidSetpoint < 0) {
				agSetpointAttenuator = -currentPidSetpoint / 50.0f;
			} else {
				agSetpointAttenuator = currentPidSetpoint / 50.0f;
			}
            agSetpointAttenuator = MAX(agSetpointAttenuator, 1.0f);
            const float antiGravityPBoost = 1.0f + (pidRuntime.antiGravityThrottleD / agSetpointAttenuator) * pidRuntime.antiGravityPGain;
            pidData[axis].P *= antiGravityPBoost;
        }

        // calculating the PID sum
        const float pidSum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F /*+ pidData[axis].S*/;
        pidData[axis].Sum = pidSum;
		pidSums[axis] = pidSum;
    }
}

void pidBFInitDefault() {
	pidRuntime.pidFrequency = 1000.0f;
	pidRuntime.dT = 1.0f / pidRuntime.pidFrequency;
	pidRuntime.pidStabilisationEnabled  = true;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        pidRuntime.previousPidSetpoint[axis] = 0.0f;
        pt1FilterInit(&pidRuntime.dtermLowpass[axis], pt1FilterGain(100.0f, pidRuntime.dT));
        pidRuntime.pidCoefficient[axis].Kp = 40.0f;
        pidRuntime.pidCoefficient[axis].Ki = 60.0f;
        pidRuntime.pidCoefficient[axis].Kd = 30.0f;
        pidRuntime.maxVelocity[axis] = 200.0f;
    }
    pt1FilterInit(&pidRuntime.ptermYawLowpass, pt1FilterGain(100.0f, pidRuntime.dT));

    // Anti-gravity и iTerm
    pidRuntime.antiGravityEnabled = true;
    pidRuntime.antiGravityThrottleD = 0.0f;
    pidRuntime.itermAccelerator = 1.0f;
    pidRuntime.antiGravityGain = 200;             
    pidRuntime.antiGravityPGain = 1.0f;
	pt2FilterInit(&pidRuntime.antiGravityLpf, pt2FilterGain(100.0f, pidRuntime.dT));	

    // Лимиты
    pidRuntime.itermLimit = 200.0f;
    pidRuntime.itermLimitYaw = 50.0f;
    pidRuntime.zeroThrottleItermReset = true;

    // TPA (начинает снижать PID при высоком газе)
    pidRuntime.tpaFactor = 0.3f;
	pidRuntime.tpaLowBreakpoint = 1250.0f;
	pidRuntime.tpaMultiplier = 1.0f;
    pidRuntime.tpaLowAlways = false;
    pidRuntime.tpaLowMultiplier = 1.0f;
	pidRuntime.tpaBreakpoint = 0.5f;
	pidRuntime.pidSumLimit = PIDSUM_LIMIT;	
	pidRuntime.pidSumLimitYaw = PIDSUM_LIMIT_YAW;
	
	previousGyroRateDterm[0] = 0.0f;
	previousGyroRateDterm[1] = 0.0f;
	previousGyroRateDterm[2] = 0.0f;
}

void pidBFSetPidFrequencyDt(const float pidFrequency_user){
	pidRuntime.pidFrequency = pidFrequency_user;
	pidRuntime.dT = 1.0f / pidFrequency_user;
}

void pidBFSetStabilisationEnabled(const bool enabled) {
    pidRuntime.pidStabilisationEnabled = enabled;
}

void pidBFSetPidCoefficient(const uint8_t axis, const float Kp, const float Ki, const float Kd) {
    pidRuntime.pidCoefficient[axis].Kp = Kp;
    pidRuntime.pidCoefficient[axis].Ki = Ki;
    pidRuntime.pidCoefficient[axis].Kd = Kd;
}

void pidBFSetDtermLowpass(const uint8_t axis, const float cutoffFrequencyHz) {
    pt1FilterInit(&pidRuntime.dtermLowpass[axis], pt1FilterGain(cutoffFrequencyHz, pidRuntime.dT));
}

void pidBFSetMaxVelocity(const uint8_t axis, const float maxVel) {
    pidRuntime.maxVelocity[axis] = maxVel;
}

void pidBFSetPtermYawLowpass(const float cutoffFrequencyHz) {
    pt1FilterInit(&pidRuntime.ptermYawLowpass, pt1FilterGain(cutoffFrequencyHz, pidRuntime.dT));
}

void pidBFSetAntiGravity(const bool enabled) {
    pidRuntime.antiGravityEnabled = enabled;
}

void pidBFSetAntiGravityGain(const float gain) {
    pidRuntime.antiGravityGain = gain;
}

void pidBFSetAntiGravityPGain(const float pGain) {
    pidRuntime.antiGravityPGain = pGain;
}

void pidBFSetItermAccelerator(const float value) {
    pidRuntime.itermAccelerator = value;
}

void pidBFSetAntiGravityThrottleD(const float value) {
    pidRuntime.antiGravityThrottleD = value;
}

void pidBFSetAntiGravityLPF(const float cutoffFrequencyHz) {
    pt2FilterInit(&pidRuntime.antiGravityLpf, pt2FilterGain(cutoffFrequencyHz, pidRuntime.dT));
}

void pidBFSetItermLimit(const float limit) {
    pidRuntime.itermLimit = limit;
}

void pidBFSetItermLimitYaw(const float limitYaw) {
    pidRuntime.itermLimitYaw = limitYaw;
}

void pidBFSetZeroThrottleItermReset(const bool reset) {
    pidRuntime.zeroThrottleItermReset = reset;
}

void pidBFSetTpaFactor(const float tpa) {
    pidRuntime.tpaFactor = tpa;
}

void pidBFSetTpaLowBreakpoint(const float tpaLowBreakpoint) {
    pidRuntime.tpaLowBreakpoint = tpaLowBreakpoint;
}

void pidBFSetTpaMultiplier(const float multiplier){
    pidRuntime.tpaMultiplier = multiplier;
}

void pidBFSetTpaLowAlways(const bool always) {
    pidRuntime.tpaLowAlways = always;
}

void pidBFSetTpaLowMultiplier(const float multiplier) {
    pidRuntime.tpaLowMultiplier = multiplier;
}

void pidBFSetTpaBreakpoint(const float tpaBreakpoint) {
    pidRuntime.tpaBreakpoint = tpaBreakpoint;
}

uint16_t pidBFGetPidSumLimit() {
	return pidRuntime.pidSumLimit;
}

uint16_t pidBFGetPidSumLimitYaw() {
	return pidRuntime.pidSumLimitYaw;
}

void rawSetpointUpdate(const float roll, const float pitch, const float yaw) {
	rawSetpoint[FD_ROLL] = roll;
	rawSetpoint[FD_PITCH] = pitch;
	rawSetpoint[FD_YAW] = yaw;	
}

void pidUpdateAntiGravityThrottleFilter(const float throttle) {
    static float previousThrottle = 0.0f;
    const float throttleInv = 1.0f - throttle;
	float deltaThrottle = 0.0f;
	if (throttle > previousThrottle) {
		deltaThrottle = throttle - previousThrottle;
	} else {
		deltaThrottle = previousThrottle - throttle;
	}
    float throttleDerivative = deltaThrottle * pidRuntime.pidFrequency;
    throttleDerivative *= throttleInv * throttleInv;
    if (throttle > previousThrottle) {
        throttleDerivative *= throttleInv * 0.5f;
    }
    previousThrottle = throttle;
    throttleDerivative = pt2FilterApply(&pidRuntime.antiGravityLpf, throttleDerivative);
    pidRuntime.antiGravityThrottleD = throttleDerivative;
}

void pidUpdateTpaFactor(float throttle) {
    throttle = constrainfPID(throttle, 0.0f, 1.0f);
    float tpaFactor;
    const float tpaArgument = throttle;
    tpaFactor = getTpaFactorClassic(tpaArgument);
    pidRuntime.tpaFactor = tpaFactor;
}

float getTpaFactorClassic(const float tpaArgument)
{
    bool isTpaLowFaded = false;
    bool isThrottlePastTpaLowBreakpoint = (tpaArgument >= pidRuntime.tpaLowBreakpoint || pidRuntime.tpaLowBreakpoint <= 0.01f);
    float tpaRate = 0.0f;
    if (isThrottlePastTpaLowBreakpoint || isTpaLowFaded) {
        tpaRate = pidRuntime.tpaMultiplier * MAX(tpaArgument - pidRuntime.tpaBreakpoint, 0.0f);
        if (!pidRuntime.tpaLowAlways && !isTpaLowFaded) {
            isTpaLowFaded = true;
        }
    } else {
        tpaRate = pidRuntime.tpaLowMultiplier * (pidRuntime.tpaLowBreakpoint - tpaArgument);
    }

    return 1.0f - tpaRate;
}