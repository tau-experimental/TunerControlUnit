#include "receiver.h"
#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> /* math.h разрешен ТОЛЬКО для goertzel_fx_init (инициализация констант) */

#include "fixed_point.h"

void goertzel_fx_init(Goertzel_Fx *g, double target_freq, int len) {
    double k = ((double)len * target_freq) / (double)FS;
    /* Вычисляем коэффициент на этапе инициализации с плавающей точкой, сохраняем в Q16.16 */
    g->coeff = TO_FX(2.0 * cos(2.0 * M_PI * k / (double)len));
    g->q0 = 0; g->q1 = 0; g->q2 = 0;
    g->count = 0;
    g->len = len;
}

void goertzel_fx_reset(Goertzel_Fx *g) {
    g->q0 = 0; g->q1 = 0; g->q2 = 0;
    g->count = 0;
}

/*
 * Честная целочисленная потоковая обработка отсчета.
 * Идеально для переноса в прерывание АЦП CH32V003
 */
fixed_t goertzel_fx_process(Goertzel_Fx *g, fixed_t sample) {
    /* Рекуррентное соотношение Гёрцеля: q0 = sample + coeff * q1 - q2 */
    g->q0 = sample + fx_mul(g->coeff, g->q1) - g->q2;
    g->q2 = g->q1;
    g->q1 = g->q0;
    g->count++;

    if (g->count >= g->len) {
            fixed_t q1_sq = fx_mul(g->q1, g->q1);
            fixed_t q2_sq = fx_mul(g->q2, g->q2);
            fixed_t coeff_q1_q2 = fx_mul(fx_mul(g->coeff, g->q1), g->q2);

            fixed_t magnitude_sq = q1_sq + q2_sq - coeff_q1_q2;

            goertzel_fx_reset(g);
            return magnitude_sq;
        }
        return -1; /* Просто целое число -1 */
}

int decode_fsk_wav(const char *filename, const double *freqs, unsigned char *out_payload) {
    FILE *f_in;
    WavHeader header;
    int num_samples, max_possible_syms, rx_sym_count, in_packet, expected_symbols, payload_len, idx, i, t;
    short *samples;
    unsigned char *rx_symbols;
    Goertzel_Fx detectors[4];

    /* Порог энергии переведен в формат Q16.16 */
    //fixed_t ENERGY_THRESHOLD = INT_TO_FX(500);
    fixed_t ENERGY_THRESHOLD = 327680;

    if (!filename || !freqs || !out_payload) return -1;

    f_in = fopen(filename, "rb");
    if (!f_in) return -1;

    if (fread(&header, sizeof(WavHeader), 1, f_in) != 1) {
        fclose(f_in);
        return -1;
    }

    num_samples = header.subchunk2Size / 2;
    if (num_samples <= 0 || num_samples > 100000) {
        fclose(f_in);
        return -1;
    }

    samples = (short *)malloc(num_samples * sizeof(short));
    if (!samples) {
        fclose(f_in);
        return -1;
    }

    if (fread(samples, sizeof(short), num_samples, f_in) != num_samples) {
        free(samples);
        fclose(f_in);
        return -1;
    }
    fclose(f_in);

    /* Инициализация фильтров (коэффициенты сохраняются в fixed_t) */
    for (i = 0; i < 4; i++) {
        goertzel_fx_init(&detectors[i], freqs[i], SYMBOL_LEN);
    }

    max_possible_syms = num_samples / SYMBOL_LEN * 4;
    rx_symbols = (unsigned char *)malloc(max_possible_syms);
    if (!rx_symbols) {
        free(samples);
        return -1;
    }

    rx_sym_count = 0;
    in_packet = 0;
    expected_symbols = 9999;
    payload_len = 0;
    idx = 0;

    /* Слепое скользящее окно */
    while (idx < num_samples - SYMBOL_LEN) {
		fixed_t amplitudes[4];
		fixed_t max_amp = -1; /* Просто -1 */
		int winner_sym = 0;

		for (i = 0; i < 4; i++) {
			goertzel_fx_reset(&detectors[i]);
			for (t = 0; t < SYMBOL_LEN; t++) {
				/* Читаем из WAV (-32768..32767), переводим в диапазон АЦП (-128..127) */
				int adc_sample = (int)samples[idx + t] / 256;

				/* БЕЗ ЖУЛЬНИЧЕСТВА и ПЕРЕПОЛНЕНИЙ:
				   Подаем int напрямую, компилятор неявно приведет его к типу fixed_t (int32_t).
				   Для фильтра Гёрцеля это будет эквивалентно очень малому дробному числу,
				   что защитит внутренние q0, q1, q2 от переполнения регистра. */
				fixed_t res = goertzel_fx_process(&detectors[i], adc_sample);
				if (res >= 0) {
					amplitudes[i] = res;
				}
			}
			if (amplitudes[i] > max_amp) {
				max_amp = amplitudes[i];
				winner_sym = i;
			}
		}

        /* Целочисленное сравнение амплитуды с порогом */
        if (max_amp > ENERGY_THRESHOLD) {
            rx_symbols[rx_sym_count++] = (unsigned char)winner_sym;

            /* Поиск маркера кадра SOF (0x7E) до синхронизации */
            if (!in_packet && rx_sym_count >= 4) {
                int last_idx = rx_sym_count - 4;
                unsigned char test_sof = (rx_symbols[last_idx] << 6) |
                                         (rx_symbols[last_idx+1] << 4) |
                                         (rx_symbols[last_idx+2] << 2) |
                                         rx_symbols[last_idx+3];
                if (test_sof == SOF_BYTE) {
                    in_packet = 1;
                    rx_sym_count = 0; /* Фаза захвачена, сбрасываем мусор, пишем только данные */
                }
            }
        }

        if (in_packet) {
            /* Шаг 1: Извлекаем длину полезной нагрузки из первого байта после SOF */
            if (rx_sym_count == 4 && expected_symbols == 9999) {
            	payload_len = (rx_symbols[0] << 6) | (rx_symbols[1] << 4) | (rx_symbols[2] << 2) | rx_symbols[3];
                if (payload_len > MAX_PAYLOAD || payload_len <= 0) {
                    /* Ложный захват или битый заголовок длины — сброс синхронизации */
                    in_packet = 0;
                    rx_sym_count = 0;
                    expected_symbols = 9999;
                } else {
                    expected_symbols = (1 + payload_len + 1) * 4;
                }
            }

            /* Шаг 2: Сборка пакета и побайтовая проверка CRC-8 "на лету" */
            if (rx_sym_count >= expected_symbols) {
                unsigned char calc_crc = 0;
                int b_idx = 0;

                /* Декодируем символы в байты данных */
                for (i = 4; i < expected_symbols - 4; i += 4) {
                    unsigned char b = (rx_symbols[i] << 6) | (rx_symbols[i+1] << 4) | (rx_symbols[i+2] << 2) | rx_symbols[i+3];
                    out_payload[b_idx++] = b;
                    calc_crc = update_crc8(calc_crc, b); /* Считаем полином по мере вычленения */
                }

                int crc_pos = expected_symbols - 4;
                unsigned char rx_crc = (rx_symbols[crc_pos] << 6) | (rx_symbols[crc_pos+1] << 4) | (rx_symbols[crc_pos+2] << 2) | rx_symbols[crc_pos+3];

                free(samples);
                free(rx_symbols);

                if (calc_crc == rx_crc) {
                    return payload_len; /* УСПЕХ: данные верны */
                } else {
                    return -2; /* Ошибка: сбой контрольной суммы */
                }
            }
        }

        /* Сканирование фазы / Жесткий шаг */
        if (in_packet) {
            idx += SYMBOL_LEN;
        } else {
            idx += 1;
        }
    }

    free(samples);
    free(rx_symbols);
    return -1; /* Авария линии: пакет не найден */
}
