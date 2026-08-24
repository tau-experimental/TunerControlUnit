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
    /*
     * High-gain linear amplifier instead of pure square-wave clipping.
     * We scale the small signal up by a factor of 4 to combat the -10 dB drop,
     * but clip the absolute ceiling to prevent internal fixed-point overflow.
     */
    int scaled = adc_sample * 4;

    if (scaled > 127)  scaled = 127;
    if (scaled < -128) scaled = -128;

    return INT_TO_FX(scaled) / 16; /* Clean, safe Q16.16 range */
}

int decode_fsk_wav(const char *filename, const double *freqs, unsigned char *out_payload) {
    FILE *f_in;
    WavHeader header;
    int num_samples, total_bytes_in_file, idx, i, b, t;
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

    total_bytes_in_file = num_samples / (SYMBOL_LEN * BITS_PER_BYTE);

    unsigned char prev_decoded = 0;
    int total_symbol_counter = 0;
    int byte_idx = 0;

    int payload_len = 0;
    int payload_counter = 0;
    unsigned char calc_crc = 0;
    unsigned char rx_crc = 0;

    printf("\n======================================================================\n");
    printf("[ЦОС ТЕЛЕМЕТРИЯ] Анализ кадра: %s (Байт в файле: %d)\n", filename, total_bytes_in_file);
    printf("======================================================================\n");

    for (byte_idx = 0; byte_idx < total_bytes_in_file; byte_idx++) {
        unsigned char assembled_byte = 0;
        unsigned char rx_parity_bit = 0;
        int bit_pos;

        printf("\n--- Чтение байта №%d (SampleIdx старта: %d) ---\n", byte_idx, total_symbol_counter * SYMBOL_LEN);

        for (bit_pos = 0; bit_pos < 9; bit_pos++) {
            idx = total_symbol_counter * SYMBOL_LEN;
            total_symbol_counter++;

            fixed_t amplitudes[3] = {0, 0, 0};
            fixed_t max_amp = -1;
            int winner_sym = 0;

            /* Принудительный сброс БИХ-резонаторов перед новым символом */
            for (i = 0; i < 3; i++) {
                goertzel_fx_reset(&detectors[i]);
            }

            /* Интегрируем 32 отсчета */
            for (t = 0; t < SYMBOL_LEN; t++) {
                int adc_sample = (int)samples[idx + t] / 256;
                fixed_t fx_sample = apply_hard_limiter(adc_sample);

                for (i = 0; i < 3; i++) {
                    fixed_t res = goertzel_block_process(&detectors[i], fx_sample);
                    if (res >= 0) {
                        amplitudes[i] = res;
                    }
                }
            }

            /* Находим доминирующий спектральный пик */
            for (i = 0; i < 3; i++) {
                if (amplitudes[i] > max_amp) {
                    max_amp = amplitudes[i];
                    winner_sym = i;
                }
            }

            /* Вычисляем относительный шаг вращения (дифференциальный декодер) */
            unsigned char decoded_bit = descramble_bit(winner_sym, prev_decoded);

            /* Вывод побитовой диагностики ЦОС-движка */
            printf("  Символ %d/9 | E0(1000Hz):%6.1f | E1(1400Hz):%6.1f | E2(1800Hz):%6.1f | WIN: F%d | Шаг относительно F%d -> БИТ: %d\n",
                   bit_pos + 1,
                   FX_TO_DOUBLE(amplitudes[0]),
                   FX_TO_DOUBLE(amplitudes[1]),
                   FX_TO_DOUBLE(amplitudes[2]),
                   winner_sym, prev_decoded, decoded_bit);

            prev_decoded = winner_sym; /* Сохраняем непрерывность фазового кольца */

            if (bit_pos < 8) {
                assembled_byte |= (decoded_bit << (7 - bit_pos));
            } else {
                rx_parity_bit = decoded_bit;
            }
        }

        /* Вычисляем локальный паритет от собранного байта */
        unsigned char local_parity_bit = calculate_odd_parity(assembled_byte);
        int parity_ok = (rx_parity_bit == local_parity_bit);

        /* СТРОГО ТРЕБУЕМЫЙ ФОРМАТ ВЫВОДА: [ XX.P ] */
        printf(" => ПОЛУЧЕН ПАКЕТ БАЙТА: [ %02X.%d ] | Локальный Odd Parity: %d -> %s\n",
               assembled_byte, rx_parity_bit, local_parity_bit,
               parity_ok ? "УСПЕХ (Чётность совпала)" : "КРИТИЧЕСКАЯ ОШИБКА ЧЁТНОСТИ!");

        if (!parity_ok) {
            printf("[FAIL] Прерывание кадра из-за сбоя чётности.\n");
            free(samples);
            return -2; /* Ошибка паритета */
        }

        /* Распределение байт по логическим ролям внутри кадра */
        if (byte_idx == 0) {
            printf("    [СЛУЖЕБНЫЙ] Байт распознан как маркер начала кадра SOF.\n");
            if (assembled_byte != SOF_BYTE) { free(samples); return -1; }
        }
        else if (byte_idx == 1) {
            payload_len = assembled_byte;
            printf("    [СЛУЖЕБНЫЙ] Байт распознан как длина Payload: Ожидаем %d байт данных.\n", payload_len);
            if (payload_len > MAX_PAYLOAD || payload_len <= 0) { free(samples); return -1; }
        }
        else if (byte_idx < 2 + payload_len) {
            out_payload[payload_counter++] = assembled_byte;
            calc_crc = update_crc8(calc_crc, assembled_byte);
            printf("    [ДАННЫЕ] Сохранено в буфер полезной нагрузки. Текущий CRC-8: %02X\n", calc_crc);
        }
        else {
            rx_crc = assembled_byte;
            free(samples);

            printf("\n--- ПРОВЕРКА КОНТРОЛЬНОЙ СУММЫ КАДРА ---\n");
            printf("    Рассчитано приемником: 0x%02X\n", calc_crc);
            printf("    Принято из линии (CRC):  0x%02X\n", rx_crc);

            if (calc_crc == rx_crc) {
                printf("=== КАДР ВЕРИФИЦИРОВАН УСПЕШНО ===\n");
                return payload_counter;
            } else {
                printf("=== [FAIL] СБОЙ КОНТРОЛЬНОЙ СУММЫ ПАКЕТА (CRC МИСМАТЧ) ===\n");
                return -3;
            }
        }
    }

    free(samples);
    return -1;
}

int decode_fsk_wav_dynamic(const char *filename, const double *freqs, unsigned char *out_payload) {
    FILE *f_in;
    WavHeader header;
    int num_samples, rx_sym_count, in_packet, expected_symbols, payload_len, idx, i, t;
    short *samples;
    unsigned char *rx_symbols;
    Goertzel_Fx detectors[3];

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

    int bit_clock_synced = 0;
    int last_winner = -1;
    int transition_idx = -1;

    /*
     * ТЮНИНГ ПОРОГА ДЛЯ СЛУЧАЙНОЙ ПАУЗЫ:
     * Выставляем порог в 40 единиц. Наш полезный сигнал (94..108) пробьет его
     * со стопроцентной гарантией, а шум в паузе (0.0) останется снизу.
     */
    fixed_t ENERGY_THRESHOLD = INT_TO_FX(40);

    printf("\n--- СТАРТ ДИНАМИЧЕСКОГО ПРИЕМНИКА (ЭТАП 5) ---\n");

    idx = 0;
    while (idx < num_samples - SYMBOL_LEN) {
        fixed_t amplitudes[3] = {0, 0, 0};
        fixed_t max_amp = -1;
        int winner_sym = 0;

        /* Очищаем Блочный Гёрцель */
        for (i = 0; i < 3; i++) goertzel_fx_reset(&detectors[i]);

        /* Анализируем окно из 32 отсчетов от текущего 'idx' */
        for (t = 0; t < SYMBOL_LEN; t++) {
            int adc_sample = (int)samples[idx + t] / 256;
            fixed_t fx_sample = apply_hard_limiter(adc_sample);

            for (i = 0; i < 3; i++) {
                fixed_t res = goertzel_block_process(&detectors[i], fx_sample);
                if (res >= 0) amplitudes[i] = res;
            }
        }

        for (i = 0; i < 3; i++) {
            if (amplitudes[i] > max_amp) {
                max_amp = amplitudes[i];
                winner_sym = i;
            }
        }

        /* ФАЗА 0: СЛЕПОЙ ПОИСК ПЕРЕКЛЮЧЕНИЙ ПРЕАМБУЛЫ (ШАГ ПО 1 ОТСЧЕТУ) */
        if (!bit_clock_synced) {
            if (max_amp > ENERGY_THRESHOLD) {
                if (last_winner != -1 && winner_sym != last_winner) {
                    /* Нашли фронт переключения частот! */
                    if (transition_idx == -1) {
                        transition_idx = idx;
                    } else {
                        int delta_time = idx - transition_idx;
                        /* Если расстояние между переключениями совпало с длительностью символа */
                        if (delta_time >= (SYMBOL_LEN - 3) && delta_time <= (SYMBOL_LEN + 3)) {
                            printf("   ===> [CLOCK] Сетка битовых часов защелкнута на отсчете: %d! (Delta: %d) <===\n", idx, delta_time);
                            bit_clock_synced = 1;

                            /* Прыгаем в центр следующего стабильного символа */
                            idx += (SYMBOL_LEN / 2);
                            last_winner = winner_sym;
                            continue;
                        } else {
                            transition_idx = idx; /* Глитч шума, перезахват */
                        }
                    }
                }
                last_winner = winner_sym;
            }
            idx++; /* В режиме поиска скользим плавно по 1 отсчету */
            continue;
        }

        /* ФАЗА 1: ДИСКРЕТНЫЙ ПРИЕМ ПАКЕТА (ЖЕСТКИЙ ШАГ ПО СИМВОЛАМ) */
        unsigned char decoded_bit = descramble_bit(winner_sym, last_winner);
        last_winner = winner_sym;

        rx_symbols[rx_sym_count++] = decoded_bit;

        if (rx_sym_count >= BITS_PER_BYTE && rx_sym_count % BITS_PER_BYTE == 0) {
            int byte_start_pos = rx_sym_count - BITS_PER_BYTE;
            unsigned char assembled_byte = 0;

            for (i = 0; i < 8; i++) {
                assembled_byte |= (rx_symbols[byte_start_pos + i] << (7 - i));
            }

            unsigned char rx_parity = rx_symbols[byte_start_pos + 8];
            unsigned char local_parity = calculate_odd_parity(assembled_byte);

            if (rx_parity != local_parity) {
                if (!in_packet) {
                    /* Мусор в паузе ложно прикинулся байтом — сбрасываем часы */
                    bit_clock_synced = 0; transition_idx = -1; rx_sym_count = 0; idx++; continue;
                } else {
                    free(samples); free(rx_symbols); return -2; /* Ошибка четности в пакете */
                }
            }

            if (!in_packet && assembled_byte == SOF_BYTE) {
                in_packet = 1;
                rx_sym_count = 0;
                printf("   ===> [SYNC] Маркер SOF (0x7E) успешно распознан из шума! Принимаем данные... <===\n");
                idx += SYMBOL_LEN;
                continue;
            }

            if (in_packet) {
                if (rx_sym_count == BITS_PER_BYTE) {
                    payload_len = assembled_byte;
                    if (payload_len > MAX_PAYLOAD || payload_len <= 0) {
                        in_packet = 0; bit_clock_synced = 0; transition_idx = -1; rx_sym_count = 0; expected_symbols = 9999;
                    } else {
                        expected_symbols = (1 + payload_len + 1) * BITS_PER_BYTE;
                    }
                }

                if (rx_sym_count >= expected_symbols) {
                    unsigned char calc_crc = 0;
                    int p_idx = BITS_PER_BYTE;
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
                    if (calc_crc == rx_crc) return payload_len;
                    return -3;
                }
            }
        }

        idx += SYMBOL_LEN; /* Прыгаем по центрам плато символов */
    }

    free(samples); free(rx_symbols);
    return -1;
}

