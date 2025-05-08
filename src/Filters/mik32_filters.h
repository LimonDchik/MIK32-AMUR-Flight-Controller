#ifndef MIK32_FILTERS
#define MIK32_FILTERS

#define M_PIf 3.14159265358979323846f

#define CUTOFF_CORRECTION_PT2 1.553773974f

typedef struct pt1Filter_s {
    float state;
    float k;
} pt1Filter_t;

typedef struct pt2Filter_s {
    float state;
    float state1;
    float k;
} pt2Filter_t;

float pt1FilterGain(float f_cut, float dT); // Расчёт коэффициента.
float pt1FilterGainFromDelay(float delay, float dT); //Вычисляет коэффициент фильтра на основе задержки (постоянной времени фильтра) — времени, за которое отклик фильтра достигает 63,2% от величины ступенчатого входного сигнала.
void pt1FilterInit(pt1Filter_t *filter, float k); // Инициализация с рассчитанным коэффициентом.
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k); // Обновить коэффициент.
float pt1FilterApply(pt1Filter_t *filter, float input); // Применение.

float pt2FilterGain(float f_cut, float dT);  // Расчёт коэффициента.                                   
float pt2FilterGainFromDelay(float delay, float dT); // Вычисляет коэффициент фильтра на основе задержки (постоянной времени фильтра) — времени, за которое отклик фильтра достигает 63,2% от величины ступенчатого входного сигнала.
void pt2FilterInit(pt2Filter_t *filter, float k); // Инициализация с рассчитанным коэффициентом.
void pt2FilterUpdateCutoff(pt2Filter_t *filter, float k); // Обновить коэффициент.
float pt2FilterApply(pt2Filter_t *filter, float input); // Применение.

#endif //MIK32_FILTERS