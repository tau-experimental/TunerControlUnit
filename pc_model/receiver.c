#include "receiver.h"
#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> /* math.h разрешен ТОЛЬКО для goertzel_fx_init (инициализация констант) */

#include "fixed_point.h"

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
    /* Базовое рекуррентное уравнение Гёрцеля на fixed_t */
    g->q0 = sample + fx_mul(g->coeff, g->q1) - g->q2;
    g->q2 = g->q1;
    g->q1 = g->q0;
    g->count++;

    if (g->count >= g->len) {
        /* Вычисляем энергию. Чтобы промежуточные квадраты q1*q1 не переполняли fixed_t,
           мы уменьшаем масштаб q1 и q2 в 8 раз (сдвиг >> 3) перед умножением. */
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

int decode_fsk_wav(const char *filename, const double *freqs, unsigned char *out_payload) {
    FILE *f_in;
    WavHeader header;
    int num_samples, max_possible_syms, rx_sym_count, in_packet, expected_symbols, payload_len, idx, i, t;
    short *samples;
    unsigned char *rx_symbols;
    Goertzel_Fx detectors[4];

    /* Базовый порог для новой 64-битной математики */

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

	/* Переменная состояния синхронизации битовых часов */
	int bit_clock_synced = 0;
	fixed_t ENERGY_THRESHOLD = INT_TO_FX(150); /* Порог уверенного захвата преамбулы */

	printf("\n--- СТАРТ ДВУХФАЗНОЙ ДЕМОДУЛЯЦИИ ФАЙЛА: %s ---\n", filename);

	idx = 0;
	while (idx < num_samples) {
		int adc_sample = (int)samples[idx] / 256;
		fixed_t fx_sample = INT_TO_FX(adc_sample) / 4;

		fixed_t amplitudes[4] = {-1, -1, -1, -1};
		int window_ready = 0;

		for (i = 0; i < 4; i++) {
			fixed_t res = goertzel_fx_process(&detectors[i], fx_sample);
			if (res >= 0) {
				amplitudes[i] = res;
				window_ready = 1;
			}
		}

		if (window_ready) {
			fixed_t max_amp = -1;
			int winner_sym = 0;

			for (i = 0; i < 4; i++) {
				if (amplitudes[i] > max_amp) {
					max_amp = amplitudes[i];
					winner_sym = i;
				}
			}

			/* Если мы еще не синхронизировали символьные часы по преамбуле */
			if (!bit_clock_synced) {
				/* Ждем, когда в кабель прилетит сильный сигнал частоты преамбулы (WIN: 2, 1800Гц) */
				if (max_amp > ENERGY_THRESHOLD && winner_sym == 2) {
					bit_clock_synced = 1;
					printf("   ===> [CLOCK] Символьные часы захвачены на SampleIdx: %d (Amp: %.2f) <===\n",
						   idx, FX_TO_DOUBLE(max_amp));

					/* Забираем первый символ преамбулы */
					rx_symbols[rx_sym_count++] = (unsigned char)winner_sym;

					/* Мгновенно переключаем шаг на жесткий дискретный режим!
					   Следующая выборка будет сделана ровно через 16 отсчетов, в центре следующего символа */
					idx += SYMBOL_LEN;
					continue;
				}
			} else {
				/* МЫ В РЕЖИМЕ ЖЕСТКИХ ЧАСОВ: Каждая детекция — это один честный физический символ */
				rx_symbols[rx_sym_count++] = (unsigned char)winner_sym;

				/* Печатаем чистый шаг символов */
				printf("SymIdx: %3d | WIN: %d | Amp: %7.2f | ", rx_sym_count, winner_sym, FX_TO_DOUBLE(max_amp));

				/* Сборка байта из 4 символов для проверки структуры кадра */
				if (rx_sym_count % 4 == 0) {
					int last_idx = rx_sym_count - 4;
					unsigned char assembled_byte = (rx_symbols[last_idx] << 6) |
												   (rx_symbols[last_idx+1] << 4) |
												   (rx_symbols[last_idx+2] << 2) |
												   rx_symbols[last_idx+3];

					printf("Assembled Byte: 0x%02X\n", assembled_byte);

					/* Ищем маркер SOF */
					if (!in_packet && assembled_byte == SOF_BYTE) {
						in_packet = 1;
						rx_sym_count = 0; /* Сбрасываем преамбулу, начинаем копить тело пакета */
						printf("   ===> [SYNC] Маркер SOF (0x7E) успешно распознан! Начинаем прием данных кадра.\\n");
					}
				} else {
					printf("\n");
				}
			}

			if (in_packet) {
				/* Шаг 1: Извлекаем длину пакета (когда накопилось первые 4 символа данных после SOF) */
				if (rx_sym_count == 4 && expected_symbols == 9999) {
					payload_len = (rx_symbols[0] << 6) | (rx_symbols[1] << 4) | (rx_symbols[2] << 2) | rx_symbols[3];
					printf("   -> [PACKET] Длина полезной нагрузки: %d байт\n", payload_len);

					if (payload_len > MAX_PAYLOAD || payload_len <= 0) {
						printf("   -> [ERROR] Битый заголовок длины. Сброс линии.\n");
						in_packet = 0;
						bit_clock_synced = 0;
						rx_sym_count = 0;
						expected_symbols = 9999;
					} else {
						expected_symbols = (1 + payload_len + 1) * 4;
					}
				}

				/* Шаг 2: Валидация пакета по CRC-8 */
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

					free(samples);
					free(rx_symbols);

					if (calc_crc == rx_crc) {
						return payload_len; /* ПОЛНЫЙ УСПЕХ! */
					} else {
						return -2; /* Ошибка контрольной суммы */
					}
				}
			}

			/* Если символьные часы уже запущены, мы всегда прыгаем блоками по 16 отсчетов */
			if (bit_clock_synced) {
				idx += SYMBOL_LEN;
				continue;
			}
		}

		/* Если часы не запущены или тишина — скользим плавно по 1 отсчету */
		idx++;
	}

	free(samples);
	free(rx_symbols);
	return -1;
}
