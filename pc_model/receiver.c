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
    /* Формула Sliding DFT / Goertzel:
       Входным сигналом для рекурсии становится разность текущего и старого отсчетов */
    fixed_t delta = sample - old_sample;

    g->q0 = delta + fx_mul(g->coeff, g->q1) - g->q2;
    g->q2 = g->q1;
    g->q1 = g->q0;

    /* На скользящем окне энергия ДОСТУПНА НА КАЖДОМ ОТСЧЕТЕ! */
    fixed_t q1_scaled = g->q1 >> 3;
    fixed_t q2_scaled = g->q2 >> 3;

    fixed_t q1_sq = fx_mul(q1_scaled, q1_scaled);
    fixed_t q2_sq = fx_mul(q2_scaled, q2_scaled);
    fixed_t coeff_q1_q2 = fx_mul(fx_mul(g->coeff, q1_scaled), q2_scaled);

    return (q1_sq + q2_sq - coeff_q1_q2);
}

int decode_fsk_wav(const char *filename, const double *freqs, unsigned char *out_payload) {
    FILE *f_in;
    WavHeader header;
    int num_samples, max_possible_syms, rx_sym_count, in_packet, expected_symbols, payload_len, idx, i;
    short *samples;
    unsigned char *rx_symbols;
    Goertzel_Fx detectors[4];

    /* Кольцевой буфер истории АЦП на 16 элементов */
    fixed_t history_buffer[SYMBOL_LEN] = {0};
    int history_idx = 0;

    if (!filename || !freqs || !out_payload) return -1;

    f_in = fopen(filename, "rb");
    if (!f_in) return -1;

    if (fread(&header, sizeof(WavHeader), 1, f_in) != 1) {
        fclose(f_in); return -1;
    }

    num_samples = header.subchunk2Size / 2;
    samples = (short *)malloc(num_samples * sizeof(short));
    if (!samples) { fclose(f_in); return -1; }

    if (fread(samples, sizeof(short), num_samples, f_in) != num_samples) {
        free(samples); fclose(f_in); return -1;
    }
    fclose(f_in);

    for (i = 0; i < 4; i++) {
        goertzel_fx_init(&detectors[i], freqs[i], SYMBOL_LEN);
    }

    max_possible_syms = num_samples;
    rx_symbols = (unsigned char *)malloc(max_possible_syms);

    rx_sym_count = 0;
    in_packet = 0;
    expected_symbols = 9999;
    payload_len = 0;

    /* Железный порог энергии для непрерывного резонанса скользящего окна */
    fixed_t ENERGY_THRESHOLD = INT_TO_FX(5000);
    int lock_countdown = 0;

    printf("\n--- СТАРТ ЧЕСТНОГО СКОЛЬЗЯЩЕГО ЦОС-ОКНА: %s ---\n", filename);

    for (idx = 0; idx < num_samples; idx++) {
        int adc_sample = (int)samples[idx] / 256;
        fixed_t fx_sample = INT_TO_FX(adc_sample); /* Полный масштаб АЦП */

        /* Извлекаем самый старый отсчет из кольцевого буфера истории */
        fixed_t old_sample = history_buffer[history_idx];
        /* Записываем текущий отсчет на его место */
        history_buffer[history_idx] = fx_sample;
        history_idx = (history_idx + 1) % SYMBOL_LEN;

        fixed_t amplitudes[4] = {0};
        fixed_t max_amp = -1;
        int winner_sym = 0;

        /* Обновляем Sliding Goertzel на КАЖДОМ шаге АЦП */
        for (i = 0; i < 4; i++) {
            amplitudes[i] = goertzel_sliding_process(&detectors[i], fx_sample, old_sample);
            if (amplitudes[i] > max_amp) {
                max_amp = amplitudes[i];
                winner_sym = i;
            }
        }

        /* Защитный тайм-аут удержания символа (чтобы не двоить один и тот же символ 16 раз) */
        if (lock_countdown > 0) {
            lock_countdown--;
            continue;
        }

        /* Фиксация превышения порога */
        if (max_amp > ENERGY_THRESHOLD) {
            rx_symbols[rx_sym_count++] = (unsigned char)winner_sym;

            /* Блокируем чтение на следующие 16 отсчетов, так как мы считали
               центр текущего символа и перескакиваем через его тело */
            lock_countdown = SYMBOL_LEN - 1;

            if (!in_packet) {
                printf("SampleIdx: %5d | WIN: %d | MaxAmp: %7.2f [CLOCK CAPTURED]\n",
                       idx, winner_sym, FX_TO_DOUBLE(max_amp));
            } else {
                printf("SymIdx: %3d | WIN: %d | Amp: %7.2f | ", rx_sym_count, winner_sym, FX_TO_DOUBLE(max_amp));
            }

            /* Сборка байта из 4-х дискретных символов */
            if (in_packet || rx_sym_count >= 4) {
                int last_idx = rx_sym_count - 4;
                unsigned char assembled_byte = (rx_symbols[last_idx] << 6) |
                                               (rx_symbols[last_idx+1] << 4) |
                                               (rx_symbols[last_idx+2] << 2) |
                                               rx_symbols[last_idx+3];

                if (in_packet && rx_sym_count % 4 == 0) {
                    printf("Assembled Byte: 0x%02X\n", assembled_byte);
                }

                if (!in_packet && assembled_byte == SOF_BYTE) {
                    in_packet = 1;
                    rx_sym_count = 0; /* Синхронизируем буфер под полезные данные кадра */
                    printf("\n   ===> [SYNC] Маркер SOF (0x7E) успешно пойман! Читаем пакет... <===\n");
                }
            }

            if (in_packet) {
                /* Извлечение длины */
                if (rx_sym_count == 4 && expected_symbols == 9999) {
                    payload_len = (rx_symbols[0] << 6) | (rx_symbols[1] << 4) | (rx_symbols[2] << 2) | rx_symbols[3];
                    printf("   -> [PACKET] Длина полезной нагрузки: %d байт\n", payload_len);
                    if (payload_len > MAX_PAYLOAD || payload_len <= 0) {
                        /* Защита от мусора */
                        in_packet = 0; rx_sym_count = 0; expected_symbols = 9999;
                    } else {
                        expected_symbols = (1 + payload_len + 1) * 4;
                    }
                }

                /* Валидация пакета по CRC-8 */
                if (rx_sym_count >= expected_symbols) {
                    unsigned char calc_crc = 0;
                    int b_idx = 0;

                    for (i = 4; i < expected_symbols - 4; i += 4) {
                        unsigned char b = (rx_symbols[i] << 6) | (rx_symbols[i+1] << 4) | (rx_symbols[i+2] << 2) | rx_symbols[i+3];
                        out_payload[b_idx++] = b;
                        calc_crc = update_crc8(calc_crc, b);
                    }

                    int crc_pos = expected_symbols - 4;
                    unsigned char rx_crc = (rx_symbols[crc_pos] << 6) | (rx_symbols[crc_pos+1] << 4) | (rx_symbols[crc_pos+2] << 2) | rx_symbols[crc_pos+3];

                    free(samples); free(rx_symbols);
                    if (calc_crc == rx_crc) return payload_len;
                    return -2;
                }
            }
        }
    }

    free(samples); free(rx_symbols);
    return -1;
}
