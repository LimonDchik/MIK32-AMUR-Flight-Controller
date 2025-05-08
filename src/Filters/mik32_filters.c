#include "mik32_filters.h"

// PT1 Low Pass filter

// Расчёт коэффициента.
float pt1FilterGain(float f_cut, float dT)
{
    float omega = 2.0f * M_PIf * f_cut * dT;
    return omega / (omega + 1.0f);
}

// Вычисляет коэффициент фильтра на основе задержки (постоянной времени фильтра) — времени, за которое отклик фильтра достигает 63,2% от величины ступенчатого входного сигнала.
float pt1FilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0) {
        return 1.0f; // gain = 1 means no filtering
    }

    const float cutoffHz = 1.0f / (2.0f * M_PIf * delay);
    return pt1FilterGain(cutoffHz, dT);
}

// Инициализация с рассчитанным коэффициентом.
void pt1FilterInit(pt1Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->k = k;
}

// Обновить коэффициент.
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k)
{
    filter->k = k;
}

// Применение.
float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}

// PT2 Low Pass filter

// Расчёт коэффициента.
float pt2FilterGain(float f_cut, float dT)
{
    // shift f_cut to satisfy -3dB cutoff condition
    return pt1FilterGain(f_cut * CUTOFF_CORRECTION_PT2, dT);
}

// Вычисляет коэффициент фильтра на основе задержки (постоянной времени фильтра) — времени, за которое отклик фильтра достигает 63,2% от величины ступенчатого входного сигнала.
float pt2FilterGainFromDelay(float delay, float dT)
{
    if (delay <= 0) {
        return 1.0f; // gain = 1 means no filtering
    }

    const float cutoffHz = 1.0f / (M_PIf * delay * CUTOFF_CORRECTION_PT2);
    return pt2FilterGain(cutoffHz, dT);
}

// Инициализация с рассчитанным коэффициентом.
void pt2FilterInit(pt2Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->k = k;
}

// Обновить коэффициент.
void pt2FilterUpdateCutoff(pt2Filter_t *filter, float k)
{
    filter->k = k;
}

// Применение.
float pt2FilterApply(pt2Filter_t *filter, float input)
{
    filter->state1 = filter->state1 + filter->k * (input - filter->state1);
    filter->state = filter->state + filter->k * (filter->state1 - filter->state);
    return filter->state;
}