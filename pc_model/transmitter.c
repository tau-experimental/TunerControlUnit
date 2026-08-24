#include "transmitter.h"
#include "common.h"
#include "fixed_point.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int generate_fsk_wav(const unsigned char *payload, int payload_len, const double *freqs, const char *filename, int dirty) {
    int total_bytes, total_symbols, pause_samples, end_samples, signal_samples, total_samples;
    short *buffer;
    unsigned char *symbols;
    unsigned char crc = 0;
    int i, s_idx, t, out_idx;
    double pause_sec, double_rand;
    FILE *f_out;
    WavHeader header;

    /* Переменные для фиксированной точки DDS */
    fixed_t fx_phase = 0;
    fixed_t fx_phase_steps[4];
    fixed_t fx_two_pi = TO_FX(2.0 * 3.141592653589793);

    if (payload_len < 0 || payload_len > MAX_PAYLOAD || !payload || !freqs || !filename) {
        return -1;
    }

    /*
     * Предвычисляем фазовые шаги DDS для четырех частот 4-FSK.
     * На МК эти коэффициенты будут вычислены заранее и лежать в памяти как константы fixed_t!
     * phase_step = 2 * PI * f / FS
     */
    for (i = 0; i < 4; i++) {
        fx_phase_steps[i] = TO_FX(2.0 * 3.141592653589793 * freqs[i] / (double)FS);
    }

    total_bytes = 4 + payload_len + 1; /* 2 Preamble + SOF + Len + Payload + CRC */
    total_symbols = total_bytes * 4;

    double_rand = (double)rand() / (double)RAND_MAX;
    pause_sec = 0.3 + double_rand * 0.4;
    pause_samples = (int)(pause_sec * FS);
    end_samples = (int)(0.1 * FS);
    signal_samples = total_symbols * SYMBOL_LEN;
    total_samples = pause_samples + signal_samples + end_samples;

    buffer = (short *)calloc(total_samples, sizeof(short));
    symbols = (unsigned char *)malloc(total_symbols);

    if (!buffer || !symbols) {
        if (buffer) free(buffer);
        if (symbols) free(symbols);
        return -1;
    }

    /* Расчет контрольной суммы пакета */
    for (i = 0; i < payload_len; i++) {
        crc = update_crc8(crc, payload[i]);
    }

    /* Сборка структуры кадра */
    byte_to_symbols(PREAMBLE_BYTE, &symbols[0]);
    byte_to_symbols(PREAMBLE_BYTE, &symbols[4]);
    byte_to_symbols(SOF_BYTE,       &symbols[8]);
    byte_to_symbols((unsigned char)payload_len, &symbols[12]);

    s_idx = 16;
    for (i = 0; i < payload_len; i++) {
        byte_to_symbols(payload[i], &symbols[s_idx]);
        s_idx += 4;
    }
    byte_to_symbols(crc, &symbols[s_idx]);

    /*
     * ЧЕСТНЫЙ ЦЕЛОЧИСЛЕННЫЙ ЦИФРОВОЙ СИНТЕЗ (DDS)
     * Имитирует генерацию синусоиды через таймер ШИМ с 8-битным квантованием амплитуды.
     */
    out_idx = pause_samples;
    for (i = 0; i < total_symbols; i++) {
        fixed_t step = fx_phase_steps[symbols[i]];
        for (t = 0; t < SYMBOL_LEN; t++) {
            /* Вычисляем синус средствами целочисленной библиотеки fixed_t (Q16.16) */
            fixed_t fx_sin_val = fx_sin(fx_phase);

            /*
             * Превращаем результат Q16.16 в честное 8-битное знаковое число от -128 до 127
             * (Имитация ШИМ-ЦАП разрядности 8 бит).
             * Так как fx_sin_val лежит в диапазоне [-FX_ONE .. FX_ONE], то умножение на 127
             * через fx_mul вернет значение в шкале Q16.16, которое мы превращаем в обычное int.
             */
            fixed_t fx_scaled = fx_mul(fx_sin_val, INT_TO_FX(127));
            int val_8bit = FX_TO_INT(fx_scaled);

            /* Масштабируем до 16-битного PCM WAV файла (сдвиг влево на 8 бит) */
            buffer[out_idx++] = (short)(val_8bit << 8);

            /* Инкремент фазы аккумулятора DDS */
            fx_phase += step;
            if (fx_phase >= fx_two_pi) {
                fx_phase -= fx_two_pi;
            }
        }
    }

    /* Наложение реалистичной грязи в кабель (Коричневый шум и импульсные щелчки) */
    if (dirty) {
        double brown_noise = 0.0;
        for (i = 0; i < total_samples; i++) {
            brown_noise += ((double)rand() / (double)RAND_MAX) * 800.0 - 400.0;
            if (brown_noise > 4000.0)  brown_noise = 4000.0;
            if (brown_noise < -4000.0) brown_noise = -4000.0;

            int mixed = buffer[i] + (int)brown_noise;
            if (((double)rand() / (double)RAND_MAX) < 0.001) {
                mixed = (rand() % 2) ? 25000 : -25000;
            }
            if (mixed > 32767)  mixed = 32767;
            if (mixed < -32768) mixed = -32768;
            buffer[i] = (short)mixed;
        }
    }

    /* Запись эмулированного кабеля в WAV файл */
    f_out = fopen(filename, "wb");
    if (!f_out) {
        free(buffer);
        free(symbols);
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

    free(buffer);
    free(symbols);
    return 0;
}
