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

fixed_t goertzel_sliding_process(Goertzel_Fx *g, fixed_t sample, fixed_t old_sample) {
    /* Для синхронного посимвольного анализа (Этап 3) убираем дифференциальный занос.
       Работаем по классической формуле Гёрцеля: q0 = sample + coeff * q1 - q2 */
    g->q0 = sample + fx_mul(g->coeff, g->q1) - g->q2;
    g->q2 = g->q1;
    g->q1 = g->q0;

    g->count++;

    /* Вычисляем энергию, когда окно закрылось */
    fixed_t q1_scaled = g->q1 >> 4;
    fixed_t q2_scaled = g->q2 >> 4;

    fixed_t q1_sq = fx_mul(q1_scaled, q1_scaled);
    fixed_t q2_sq = fx_mul(q2_scaled, q2_scaled);
    fixed_t coeff_q1_q2 = fx_mul(fx_mul(g->coeff, q1_scaled), q2_scaled);

    return (q1_sq + q2_sq - coeff_q1_q2);
}

/*
 * КЛАССИЧЕСКИЙ БЛОЧНЫЙ ГЁРЦЕЛЬ (Идеален для жесткой сетки и АРУ меандра)
 * Не использует буфер истории, накапливает резонанс строго с нуля.
 */
fixed_t goertzel_block_process(Goertzel_Fx *g, fixed_t sample) {
    /* Чистая каноническая формула БИХ-резонатора */
    g->q0 = sample + fx_mul(g->coeff, g->q1) - g->q2;
    g->q2 = g->q1;
    g->q1 = g->q0;
    g->count++;

    if (g->count >= g->len) {
        /* Окно закрылось, вычисляем финальную энергию */
        fixed_t q1_scaled = g->q1 >> 4;
        fixed_t q2_scaled = g->q2 >> 4;

        fixed_t q1_sq = fx_mul(q1_scaled, q1_scaled);
        fixed_t q2_sq = fx_mul(q2_scaled, q2_scaled);
        fixed_t coeff_q1_q2 = fx_mul(fx_mul(g->coeff, q1_scaled), q2_scaled);

        return (q1_sq + q2_sq - coeff_q1_q2);
    }
    return -INT_TO_FX(1);
}

fixed_t apply_hard_limiter(int adc_sample) {
    /* Если в линии тишина или микрошум — возвращаем ноль */
    if (adc_sample > -2 && adc_sample < 2) {
        return 0;
    }

    if (adc_sample > 0) {
        return INT_TO_FX(4);
    } else {
        return -INT_TO_FX(4);
    }
}

int decode_fsk_wav(const char *filename, const double *freqs, unsigned char *out_payload) {
    FILE *f_in;
    WavHeader header;
    int num_samples, payload_len, idx, i, b, t;
    short *samples;
    Goertzel_Fx detectors[3];

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

    payload_len = num_samples / (SYMBOL_LEN * BITS_PER_BYTE);

    unsigned char prev_decoded = 0;
    int byte_idx = 0;
    int total_symbol_counter = 0;

    for (byte_idx = 0; byte_idx < payload_len; byte_idx++) {
        unsigned char assembled_byte = 0;
        unsigned char rx_parity_bit = 0;
        int bit_pos;

        for (bit_pos = 0; bit_pos < 9; bit_pos++) {
            idx = total_symbol_counter * SYMBOL_LEN;
            total_symbol_counter++;

            fixed_t amplitudes[3] = {-INT_TO_FX(1), -INT_TO_FX(1), -INT_TO_FX(1)};
            fixed_t max_amp = -1;
            int winner_sym = 0;

            /* ОЧИЩАЕМ РЕЗОНАТОРЫ ПЕРЕД КАЖДЫМ СИМВОЛОМ */
            for (i = 0; i < 3; i++) {
                goertzel_fx_reset(&detectors[i]);
            }

            /* Прокачиваем 32 отсчета строго с нуля, без влияния буфера истории */
            for (t = 0; t < SYMBOL_LEN; t++) {
                int adc_sample = (int)samples[idx + t] / 256;

                /* АРУ-Лимитер включен для защиты от шума грязного канала */
                fixed_t fx_sample = apply_hard_limiter(adc_sample);

                for (i = 0; i < 3; i++) {
                    fixed_t res = goertzel_block_process(&detectors[i], fx_sample);
                    if (res >= 0) {
                        amplitudes[i] = res;
                    }
                }
            }

            for (i = 0; i < 3; i++) {
                if (amplitudes[i] > max_amp) {
                    max_amp = amplitudes[i];
                    winner_sym = i;
                }
            }

            unsigned char decoded_bit = descramble_bit(winner_sym, prev_decoded);
            prev_decoded = winner_sym;

            if (bit_pos < 8) {
                assembled_byte |= (decoded_bit << (7 - bit_pos));
            } else {
                rx_parity_bit = decoded_bit;
            }
        }

        unsigned char local_parity_bit = calculate_odd_parity(assembled_byte);

        if (rx_parity_bit != local_parity_bit) {
            printf("    [DEBUG FAIL] БайтIdx: %d | Собран: 0x%02X | Принятый Parity: %d | Ожидаемый Parity: %d\n",
                   byte_idx, assembled_byte, rx_parity_bit, local_parity_bit);
            free(samples);
            return -2;
        }

        out_payload[byte_idx] = assembled_byte;
    }

    free(samples);
    return payload_len;
}

int decode_fsk_wav_dynamic(const char *filename, const double *freqs, unsigned char *out_payload) {
    FILE *f_in;
    WavHeader header;
    int num_samples, rx_sym_count, in_packet, expected_symbols, payload_len, idx, i, t;
    short *samples;
    unsigned char *rx_symbols;
    Goertzel_Fx detectors[3];

    /* Буфер истории для скользящего окна Гёрцеля */
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

    for (i = 0; i < 3; i++) goertzel_fx_init(&detectors[i], freqs[i], SYMBOL_LEN);
    rx_symbols = (unsigned char *)malloc(num_samples);
    rx_sym_count = 0; in_packet = 0; expected_symbols = 9999; payload_len = 0;

    /* Состояния автомата синхронизации битовых часов */
    int bit_clock_synced = 0;
    int last_winner = -1;
    int transition_idx = -1;

    /* Благодаря жесткому ограничителю амплитуд, порог энергии теперь СТАТИЧЕН
       и равен стабильным 1500 единицам, независимо от затухания кабеля! */
    fixed_t ENERGY_THRESHOLD = INT_TO_FX(100);

    printf("\n--- СТАРТ ЭТАПА №5 (АРУ/ЛИМИТЕР + СЛЕМПОЙ ПОИСК): %s ---\n", filename);

    idx = 0;
    while (idx < num_samples) {
        int adc_sample = (int)samples[idx] / 256;

        /* КРИТИЧЕСКИЙ ФИКС: Пропускаем сигнал через жесткий ограничитель!
           Он полностью уничтожает затухание 10 дБ и выравнивает шкалу энергий. */
        fixed_t fx_sample = apply_hard_limiter(adc_sample);

        fixed_t old_sample = history_buffer[history_idx];
        history_buffer[history_idx] = fx_sample;
        history_idx = (history_idx + 1) % SYMBOL_LEN;

        fixed_t amplitudes[3] = {0};
        fixed_t max_amp = -1;
        int winner_sym = 0;

        for (i = 0; i < 3; i++) {
            amplitudes[i] = goertzel_sliding_process(&detectors[i], fx_sample, old_sample);
            if (amplitudes[i] > max_amp) {
                max_amp = amplitudes[i];
                winner_sym = i;
            }
        }

        /* ФАЗА 0: СЛЕПОЙ ПОИСК ПЕРЕКЛЮЧЕНИЯ ЧАСТОТ ПРЕАМБУЛЫ */
        if (!bit_clock_synced) {
            if (max_amp > ENERGY_THRESHOLD) {
                if (last_winner != -1 && winner_sym != last_winner) {
                    /* Зафиксирован четкий частотный переход на отсчете 'idx'! */
                    if (transition_idx == -1) {
                        transition_idx = idx;
                    } else {
                        int delta_time = idx - transition_idx;
                        /* Если расстояние между переключениями частот равно длине символа (+/- 2 отсчета) */
                        if (delta_time >= (SYMBOL_LEN - 2) && delta_time <= (SYMBOL_LEN + 2)) {
                            printf("   ===> [CLOCK] Битовые часы зафиксированы! Переключение на шаге: %d (Delta: %d) <===\n", idx, delta_time);
                            bit_clock_synced = 1;

                            /* Прыгаем на SYMBOL_LEN / 2 отсчетов вперед, прямо в центр стабильного плато частоты */
                            idx += (SYMBOL_LEN / 2);
                            last_winner = winner_sym;
                            continue;
                        } else {
                            transition_idx = idx; /* Перезапуск захвата при ложном шуме */
                        }
                    }
                }
                last_winner = winner_sym;
            }
            idx++; /* До захвата часов плавно скользим по 1 отсчету */
            continue;
        }

        /* ФАЗА 1: ДИСКРЕТНОЕ ДЕКОДИРОВАНИЕ СИМВОЛОВ ВРАЩЕНИЯ ПО ЦЕНТРАМ */
        unsigned char decoded_bit = descramble_bit(winner_sym, last_winner);
        last_winner = winner_sym;

        rx_symbols[rx_sym_count++] = decoded_bit;

        /* Побайтовая сборка кадра */
        if (rx_sym_count >= BITS_PER_BYTE && rx_sym_count % BITS_PER_BYTE == 0) {
            int byte_start_pos = rx_sym_count - BITS_PER_BYTE;
            unsigned char assembled_byte = 0;

            for (i = 0; i < 8; i++) {
                assembled_byte |= (rx_symbols[byte_start_pos + i] << (7 - i));
            }

            unsigned char rx_parity = rx_symbols[byte_start_pos + 8];
            unsigned char local_parity = calculate_odd_parity(assembled_byte);

            if (rx_parity != local_parity) {
                /* Если скремблированный мусор в паузе выдал ложную контрольную сумму — сбрасываем */
                if (!in_packet) {
                    bit_clock_synced = 0; transition_idx = -1; rx_sym_count = 0; idx++; continue;
                } else {
                    free(samples); free(rx_symbols); return -2; /* Реальная ошибка паритета в пакете */
                }
            }

            /* Ищем маркер SOF кадра данных */
            if (!in_packet && assembled_byte == SOF_BYTE) {
                in_packet = 1;
                rx_sym_count = 0; /* Синхронизируем буфер под полезный Payload */
                printf("   ===> [SYNC] Маркер кадра SOF (0x7E) успешно распознан ограничителем! Принимаем данные... <===\n");
                idx += SYMBOL_LEN;
                continue;
            }

            if (in_packet) {
                /* Извлекаем длину полезных данных пакета */
                if (rx_sym_count == BITS_PER_BYTE) {
                    payload_len = assembled_byte;
                    printf("   -> [PACKET] Длина полезной нагрузки: %d байт\n", payload_len);
                    if (payload_len > MAX_PAYLOAD || payload_len <= 0) {
                        in_packet = 0; bit_clock_synced = 0; transition_idx = -1; rx_sym_count = 0; expected_symbols = 9999;
                    } else {
                        /* Всего символов кадра: (Len_byte + Payload + CRC_byte) * 9 бит */
                        expected_symbols = (1 + payload_len + 1) * BITS_PER_BYTE;
                    }
                }

                /* Когда весь пакет полностью собран на дискретной сетке */
                if (rx_sym_count >= expected_symbols) {
                    unsigned char calc_crc = 0;
                    int p_idx = BITS_PER_BYTE; /* Пропускаем байт длины */
                    int out_idx = 0;

                    for (i = 0; i < payload_len; i++) {
                        unsigned char b = 0;
                        for (t = 0; t < 8; t++) b |= (rx_symbols[p_idx + t] << (7 - t));
                        out_payload[out_idx++] = b;
                        calc_crc = update_crc8(calc_crc, b);
                        p_idx += BITS_PER_BYTE;
                    }

                    unsigned char rx_crc = 0;
                    for (t = 0; t < 8; t++) rx_crc |= (rx_symbols[p_idx + t] << (7 - t));

                    free(samples); free(rx_symbols);
                    if (calc_crc == rx_crc) return payload_len; /* ПОЛНЫЙ УСПЕХ! */
                    return -3; /* Ошибка CRC */
                }
            }
        }

        idx += SYMBOL_LEN; /* В режиме фиксации часов прыгаем строго по центрам плато */
    }

    free(samples); free(rx_symbols);
    return -1;
}

