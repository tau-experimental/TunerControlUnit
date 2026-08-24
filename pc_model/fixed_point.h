#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#include <stdint.h>

/* Тип данных фиксированной точки Q16.16 */
typedef int32_t fixed_t;

#define FX_SHIFT 16
#define FX_ONE   ((fixed_t)(1 << FX_SHIFT))
#define FX_HALF  ((fixed_t)(1 << (FX_SHIFT - 1)))

/* Макросы для быстрого преобразования тривиальных типов */
#define TO_FX(x)   ((fixed_t)((x) * (double)FX_ONE))
#define INT_TO_FX(x) ((fixed_t)((x) << FX_SHIFT))
#define FX_TO_INT(x) ((int32_t)((x) >> FX_SHIFT))
#define FX_TO_DOUBLE(x) ((double)(x) / (double)FX_ONE)

/* Прототипы функций математики Fixed Point */
fixed_t fx_mul(fixed_t a, fixed_t b);
fixed_t fx_div(fixed_t a, fixed_t b);

/* Быстрый целочисленный синус для DDS генератора (ШИМ) */
/* Принимает фазу в формате Q16.16 (от 0 до 2*PI в формате fixed) */
fixed_t fx_sin(fixed_t phase);
fixed_t fx_cos(fixed_t phase);

#endif /* FIXED_POINT_H */
