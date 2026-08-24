#include "receiver.h"
#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

void goertzel_fx_init(Goertzel_Fx *g, double target_freq, int len) {
    double k = ((double)len * target_freq) / (double)FS;
    g->coeff = TO_FX(2.0 * cos(2.0 * M_PI * k / (double)len));
    g->q0 = 0; g->q1 = 0; g->q2 = 0;
    g->count = 0;
    g->len = len;
}

void goertzel_fx_reset(Goertzel_Fx *g) {
    g->q0 = 0; g->q1 = 0; g->q2 = 0;
    g->count = 0;
}

fixed_t goertzel_fx_process(Goertzel_Fx *g, fixed_t sample) {
    g->q0 = sample + fx_mul(g->coeff, g->q1) - g->q2;
    g->q2 = g->q1;
    g->q1 = g->q0;
    g->count++;

    if (g->count >= g->len) {
        fixed_t q1_scaled = g->q1 >> 3;
        fixed_t q2_scaled = g->q2 >> 3;

        fixed_t q1_sq = fx_mul(q1_scaled, q1_scaled);
        fixed_t q2_sq = fx_mul(q2_scaled, q2_scaled);
        fixed_t coeff_q1_q2 = fx_mul(fx_mul(g->coeff, q1_scaled), q2_scaled);

        fixed_t magnitude_sq = q1_sq + q2_sq - coeff_q1_q2;

        goertzel_fx_reset(g);
        return magnitude_sq;
    }
    return -INT_TO_FX(1);
}

/* Обновленный потоковый Sliding Goertzel.
   Принимает текущий отсчет и САМЫЙ СТАРЫЙ отсчет из буфера истории (16 шагов назад) */
fixed_t goertzel_sliding_process(Goertzel_Fx *g, fixed_t sample, fixed_t old_sample) {
    /* Входной дифференциальный сигнал */
    fixed_t delta = sample - old_sample;

    /* Основное рекуррентное уравнение Гёрцеля */
    g->q0 = delta + fx_mul(g->coeff, g->q1) - g->q2;
    g->q2 = g->q1;
    g->q1 = g->q0;

    /*
     * КРИТИЧЕСКИЙ ФИКС ДЛЯ ДЛИННЫХ ОКОН (N >= 32):
     * Увеличиваем сдвиг с >> 3 до >> 5 (деление на 32).
     * Это гарантированно удержит q1_scaled и q2_scaled в диапазоне,
     * где их квадраты не переполнят верхнюю границу int32_t.
     */
    fixed_t q1_scaled = g->q1 >> 5;
    fixed_t q2_scaled = g->q2 >> 5;

    fixed_t q1_sq = fx_mul(q1_scaled, q1_scaled);
    fixed_t q2_sq = fx_mul(q2_scaled, q2_scaled);
    fixed_t coeff_q1_q2 = fx_mul(fx_mul(g->coeff, q1_scaled), q2_scaled);

    fixed_t magnitude_sq = q1_sq + q2_sq - coeff_q1_q2;

    /* Возвращаем чистую, защищенную от переполнений энергию */
    return magnitude_sq;
}

int decode_fsk_wav(const char *filename, const double *freqs, unsigned char *out_payload) {
    FILE *f_in;
    WavHeader header;
    int num_samples, payload_len, idx, i, t;
    short *samples;
    Goertzel_Fx detectors[3];

    /* Кольцевой буфер истории АЦП на 32 элемента */
    fixed_t history_buffer[SYMBOL_LEN] = {0};
    int history_idx = 0;

    if (!filename || !freqs || !out_payload) return -1;

    f_in = fopen(filename, "rb");
    if (!f_in) return -1;
    if (fread(&header, sizeof(WavHeader), 1, f_in) != 1) { fclose(f_in); return -1; }

    num_samples = header.subchunk2Size / 2;
    samples = (short *)malloc(num_samples * sizeof(short));
    if (fread(samples, sizeof(short), num_samples, f_in) != num_samples) { free(samples); fclose(f_in); return -1; }
    fclose(f_in);

    for (i = 0; i < 3; i++) {
        goertzel_fx_init(&detectors[i], freqs[i], SYMBOL_LEN);
    }

    /* Вычисляем, сколько байт заложено в файле */
    payload_len = num_samples / (SYMBOL_LEN * BITS_PER_BYTE);

    unsigned char prev_decoded = 0; /* Стартовая опора вращения (0..2) */
    int byte_idx = 0;
    int total_symbol_counter = 0;

    /* Выделяем буфер под принятые биты для текущего байта (8 данных + 1 четность) */
    unsigned char rx_bits[9];

    /* Сквозной линейный проход по файлу без прыжков индексов */
    for (byte_idx = 0; byte_idx < payload_len; byte_idx++) {
        unsigned char assembled_byte = 0;
        int bit_pos;

        /* Читаем ровно 9 символов подряд (8 данных + 1 паритет) */
        for (bit_pos = 0; bit_pos < 9; bit_pos++) {
            /* Вычисляем стартовый индекс текущего символа в файле */
            idx = total_symbol_counter * SYMBOL_LEN;
            total_symbol_counter++;

            fixed_t amplitudes[3] = {0, 0, 0};
            fixed_t max_amp = -1;
            int winner_sym = 0;

            /* Прокачиваем 32 отсчета синуса через скользящий Гёрцель */
            for (t = 0; t < SYMBOL_LEN; t++) {
                int adc_sample = (int)samples[idx + t] / 256;
                fixed_t fx_sample = INT_TO_FX(adc_sample);

                fixed_t old_sample = history_buffer[history_idx];
                history_buffer[history_idx] = fx_sample;
                history_idx = (history_idx + 1) % SYMBOL_LEN;

                for (i = 0; i < 3; i++) {
                    amplitudes[i] = goertzel_sliding_process(&detectors[i], fx_sample, old_sample);
                }
            }

            /* На последнем отсчете символа определяем доминирующую частоту */
            for (i = 0; i < 3; i++) {
                if (amplitudes[i] > max_amp) {
                    max_amp = amplitudes[i];
                    winner_sym = i;
                }
            }

            /* Декодируем направление вращения относительно ПРЕДЫДУЩЕГО символа линии */
            rx_bits[bit_pos] = descramble_bit(winner_sym, prev_decoded);

            /* Важно: текущий символ становится опорой для СЛЕДУЮЩЕГО шага (кольцо непрерывно!) */
            prev_decoded = winner_sym;
        }

        /* Собираем первые 8 бит обратно в байт данных (от MSB к LSB) */
        for (i = 0; i < 8; i++) {
            assembled_byte |= (rx_bits[i] << (7 - i));
        }

        /* 9-й принятый бит — это Parity */
        unsigned char rx_parity_bit = rx_bits[8];
        unsigned char local_parity_bit = calculate_odd_parity(assembled_byte);

        /* Проверяем целостность нечётности кадра */
        if (rx_parity_bit != local_parity_bit) {
            free(samples);
            return -2; /* Ошибка Parity: неверно распознан вектор вращения */
        }

        out_payload[byte_idx] = assembled_byte;
    }

    free(samples);
    return payload_len;
}
