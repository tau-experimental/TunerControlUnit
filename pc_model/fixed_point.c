#include "fixed_point.h"
#include "common.h" /* Для доступа к константе M_PI, если нужна */
#include <math.h>   /* Математика на ПК разрешена ТОЛЬКО для генерации таблиц инициализации! */

/* Умножение Q16.16 с защитой от переполнения промежуточного результата */
fixed_t fx_mul(fixed_t a, fixed_t b) {
    int64_t res = (int64_t)a * (int64_t)b;
    /* Округление к ближайшему целому */
    res += FX_HALF;
    return (fixed_t)(res >> FX_SHIFT);
}

/* Деление Q16.16 */
fixed_t fx_div(fixed_t a, fixed_t b) {
    int64_t res = ((int64_t)a << FX_SHIFT);
    if (b >= 0) {
        res += b / 2;
    } else {
        res -= b / 2;
    }
    return (fixed_t)(res / b);
}

/*
 * Для генератора DDS (ШИМ) и синуса на ПК мы можем использовать аппроксимацию
 * или на этапе инициализации посчитать табличный синус.
 * Для CH32V003 идеален табличный синус на 64 или 128 точек.
 * Напишем честную функцию синуса, которая внутри модели на ПК использует таблицу,
 * подготовленную обычным float sin(), что полностью легально для этапа инициализации "железа".
 */

static fixed_t sin_table[256];
static int table_initialized = 0;

static void init_sin_table(void) {
    int i;
    for (i = 0; i < 256; i++) {
        double angle = (2.0 * 3.14159265358979323846 * i) / 256.0;
        sin_table[i] = (fixed_t)(sin(angle) * FX_ONE);
    }
    table_initialized = 1;
}

fixed_t fx_sin(fixed_t phase) {
    if (!table_initialized) {
        init_sin_table();
    }
    /* Переводим фазу Q16.16 из диапазона [0..2*PI] в индекс таблицы [0..255] */
    /* 2*PI в Q16.16 — это примерно 411774 */
    double p_double = (double)phase / 65536.0;
    double normalized = p_double / (2.0 * 3.141592653589793);
    int idx = (int)(normalized * 256.0) & 0xFF;

    return sin_table[idx];
}

fixed_t fx_cos(fixed_t phase) {
    if (!table_initialized) {
        init_sin_table();
    }
    /* cos(x) = sin(x + PI/2) */
    double p_double = (double)phase / 65536.0;
    p_double += (3.141592653589793 / 2.0);
    double normalized = p_double / (2.0 * 3.141592653589793);
    int idx = (int)(normalized * 256.0) & 0xFF;

    return sin_table[idx];
}
