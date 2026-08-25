#ifndef RECEIVER_H
#define RECEIVER_H

#include <inttypes.h>

#include "fixed_point.h"

// Goertzel state structures for our 3 frequencies
typedef struct {
    int32_t v1;
    int32_t v2;
} GoertzelState_t;

// Порог подбирается экспериментально для 10-бит (например, если размах ~200,
// то пиковая энергия Гёрцеля будет в районе нескольких тысяч или десятков тысяч).
#define SIGNAL_THRESHOLD  1000
/* Если max_energy < SIGNAL_THRESHOLD, приёмник обязан принудительно
 * выставлять freq_idx = -1 (сигнала нет) и сбрасывать автомат в состояние поиска. */

// Модифицируем вывод Гёрцеля, чтобы он отдавал структуру
typedef struct {
    int8_t freq_idx;
    int32_t energy;
} GoertzelResult_t;

void process_incoming_bit(uint8_t incoming_bit);
int8_t process_goertzel_sample_10bit(uint16_t sample_10bit);
void process_adc_sample_stream(uint16_t sample_10bit);

#endif /* RECEIVER_H */
