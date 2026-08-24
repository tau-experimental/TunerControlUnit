#include "transmitter.h"
#include "common.h"
#include "fixed_point.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int generate_fsk_wav(const unsigned char *payload, int payload_len, const double *freqs, const char *filename, int dirty) {
    int total_bytes, total_symbols, preamble_symbols, data_symbols, pause_samples, end_samples, signal_samples, total_samples;
    short *buffer;
    unsigned char *raw_symbols;
    unsigned char *tx_symbols;
    unsigned char crc = 0;
    int i, b, idx, s_idx, t, out_idx;
    double pause_sec, double_rand;
    FILE *f_out;
    WavHeader header;

    fixed_t fx_phase = 0;
    fixed_t fx_phase_steps[3];
    fixed_t fx_two_pi = TO_FX(2.0 * 3.141592653589793);

    if (payload_len < 0 || payload_len > MAX_PAYLOAD || !payload || !freqs || !filename) {
        return -1;
    }

    for (i = 0; i < 3; i++) {
        fx_phase_steps[i] = TO_FX(2.0 * 3.141592653589793 * freqs[i] / (double)FS);
    }

    /*
     * ОПРЕДЕЛЕНИЕ СТРУКТУРЫ КАДРА В ЗАВИСИМОСТИ ОТ РЕЖИМА
     * Если dirty == 1, нам нужна преамбула (16 символов чередования 0 и 2) для захвата часов.
     * Если dirty == 0, мы генерируем голый пакет "в стык" для Этапа 3 и 4.
     */
    if (dirty) {
        preamble_symbols = 16; /* Чередование частот для Clock Recovery */
    } else {
        preamble_symbols = 0;  /* Без преамбулы */
    }

    /* Структура кадра: SOF (1 байт) + Len (1 байт) + Payload + CRC (1 байт) */
    total_bytes = 1 + 1 + payload_len + 1;
    data_symbols = total_bytes * BITS_PER_BYTE;
    total_symbols = preamble_symbols + data_symbols;

    /* Расчет пауз */
    if (dirty) {
        double_rand = (double)rand() / (double)RAND_MAX;
        pause_sec = 0.3 + double_rand * 0.4; /* Рандомная пауза 0.3 ... 0.7 сек */
        pause_samples = (int)(pause_sec * FS);
        end_samples = (int)(0.1 * FS);
    } else {
        pause_samples = 0;
        end_samples = 0;
    }

    signal_samples = total_symbols * SYMBOL_LEN;
    total_samples = pause_samples + signal_samples + end_samples;

    buffer = (short *)calloc(total_samples, sizeof(short));
    raw_symbols = (unsigned char *)malloc(total_symbols);
    tx_symbols = (unsigned char *)malloc(total_symbols);

    if (!buffer || !raw_symbols || !tx_symbols) {
        if (buffer) free(buffer);
        if (raw_symbols) free(raw_symbols);
        if (tx_symbols) free(tx_symbols);
        return -1;
    }

    /* 1. ГЕНЕРАЦИЯ ПРЕАМБУЛЫ (если включена) */
    for (i = 0; i < preamble_symbols; i++) {
        /* Чередуем символы 0 и 2 (максимальный разнос частот 1000 Гц и 1800 Гц) */
        raw_symbols[i] = (i % 2 == 0) ? 0 : 2;
    }

    /* 2. СБОРКА СЫРЫХ ДАННЫХ ПАКЕТА */
    for (i = 0; i < payload_len; i++) {
        crc = update_crc8(crc, payload[i]);
    }

    s_idx = preamble_symbols;
    byte_to_symbols(SOF_BYTE, &raw_symbols[s_idx]); s_idx += 4; /* Помним, что byte_to_symbols бьет на 4 части, перепишем ниже под биты! */

    /*
     * Стоп! Функция byte_to_symbols из common.c бьет байт на 4 двухбитных символа!
     * Но в нашей новой 9-битной дифференциальной схеме мы не должны использовать byte_to_symbols!
     * Мы упаковываем байты побитно прямо в массив raw_symbols!
     */
    s_idx = preamble_symbols;

    // Упаковка SOF (0x7E)
    unsigned char sof_parity = calculate_odd_parity(SOF_BYTE);
    for (b = 7; b >= 0; b--) raw_symbols[s_idx++] = (SOF_BYTE >> b) & 0x01;
    raw_symbols[s_idx++] = sof_parity;

    // Упаковка Length
    unsigned char len_parity = calculate_odd_parity((unsigned char)payload_len);
    for (b = 7; b >= 0; b--) raw_symbols[s_idx++] = ((unsigned char)payload_len >> b) & 0x01;
    raw_symbols[s_idx++] = len_parity;

    // Упаковка Payload
    for (i = 0; i < payload_len; i++) {
        unsigned char p_parity = calculate_odd_parity(payload[i]);
        for (b = 7; b >= 0; b--) raw_symbols[s_idx++] = (payload[i] >> b) & 0x01;
        raw_symbols[s_idx++] = p_parity;
    }

    // Упаковка CRC
    unsigned char crc_parity = calculate_odd_parity(crc);
    for (b = 7; b >= 0; b--) raw_symbols[s_idx++] = (crc >> b) & 0x01;
    raw_symbols[s_idx++] = crc_parity;

    /*
     * 3. ДИФФЕРЕНЦИАЛЬНОЕ КОДИРОВАНИЕ ВРАЩЕНИЕМ ЧАСТОТ
     * Кодируем побитовую ленту raw_symbols в физические символы tx_symbols (0..2)
     */
    unsigned char prev_encoded = 0;
    for (i = 0; i < total_symbols; i++) {
        if (i < preamble_symbols) {
            /* Преамбулу не скремблируем, она уже жестко задана как 0, 2, 0, 2 */
            tx_symbols[i] = raw_symbols[i];
            prev_encoded = tx_symbols[i];
        } else {
            /* Тело кадра кодируем вращением (0 - вправо, 1 - влево) */
            tx_symbols[i] = scramble_bit(raw_symbols[i], prev_encoded);
            prev_encoded = tx_symbols[i];
        }
    }

    /* 4. ЦИФРОВОЙ СИНТЕЗ СИГНАЛА (DDS) */
    out_idx = pause_samples;
    for (i = 0; i < total_symbols; i++) {
        fixed_t step = fx_phase_steps[tx_symbols[i]];
        for (t = 0; t < SYMBOL_LEN; t++) {
            fixed_t fx_sin_val = fx_sin(fx_phase);
            fixed_t fx_scaled = fx_mul(fx_sin_val, INT_TO_FX(127));
            int val_8bit = FX_TO_INT(fx_scaled);

            buffer[out_idx++] = (short)(val_8bit << 8);

            fx_phase += step;
            if (fx_phase >= fx_two_pi) fx_phase -= fx_two_pi;
        }
    }

    /* 5. ИМИТАЦИЯ ГРЯЗНОГО КАБЕЛЯ (Ослабление 10 дБ + Шум Найквиста + Щелчки) */
    if (dirty) {
        double brown_noise = 0.0;
        for (i = 0; i < total_samples; i++) {
            /* Ослабление синуса на 10 дБ по напряжению (деление на 3) */
            int attenuated_signal = buffer[i] / 3;

            /* Коричневый шум линии */
            brown_noise += ((double)rand() / (double)RAND_MAX) * 800.0 - 400.0;
            if (brown_noise > 4000.0)  brown_noise = 4000.0;
            if (brown_noise < -4000.0) brown_noise = -4000.0;

            int mixed = attenuated_signal + (int)brown_noise;

            /* Импульсные высоковольтные щелчки Найквиста (0.1% шанс на отсчет) */
            if (((double)rand() / (double)RAND_MAX) < 0.001) {
                mixed = (rand() % 2) ? 25000 : -25000;
            }

            if (mixed > 32767)  mixed = 32767;
            if (mixed < -32768) mixed = -32768;
            buffer[i] = (short)mixed;
        }
    }

    f_out = fopen(filename, "wb");
    if (!f_out) {
        free(buffer); free(raw_symbols); free(tx_symbols);
        return -1;
    }

    memcpy(header.chunkID, "RIFF", 4);
    header.chunkSize = 36 + total_samples * 2;
    memcpy(header.format, "WAVE", 4);
    memcpy(header.subchunk1ID, "fmt ", 4);
    header.subchunk1Size = 16;
    header.audioFormat = 1;
    header.numChannels = 1;
    header.sampleRate = FS;
    header.byteRate = FS * 2;
    header.blockAlign = 2;
    header.bitsPerSample = 16;
    memcpy(header.subchunk2ID, "data", 4);
    header.subchunk2Size = total_samples * 2;

    fwrite(&header, sizeof(WavHeader), 1, f_out);
    fwrite(buffer, sizeof(short), total_samples, f_out);
    fclose(f_out);

    free(buffer); free(raw_symbols); free(tx_symbols);
    return 0;
}
