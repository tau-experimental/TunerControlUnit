#include "transmitter.h"
#include "common.h"
#include "fixed_point.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int generate_fsk_wav(const unsigned char *payload, int payload_len, const double *freqs, const char *filename, int dirty) {
    int total_bytes, total_symbols, total_samples;
    short *buffer;
    unsigned char *tx_symbols;
    int i, b, sym_idx, t, out_idx;
    FILE *f_out;
    WavHeader header;

    fixed_t fx_phase = 0;
    fixed_t fx_phase_steps[3]; /* У нас теперь ровно 3 рабочие частоты */
    fixed_t fx_two_pi = TO_FX(2.0 * 3.141592653589793);

    if (payload_len < 0 || payload_len > MAX_PAYLOAD || !payload || !freqs || !filename) {
        return -1;
    }

    /* Инициализируем шаги фазы для 3-х частот (константы Flash памяти для МК) */
    for (i = 0; i < 3; i++) {
        fx_phase_steps[i] = TO_FX(2.0 * 3.141592653589793 * freqs[i] / (double)FS);
    }

    /* На каждый байт приходится ровно 9 символов вращения частоты */
    total_symbols = payload_len * BITS_PER_BYTE;
    total_samples = total_symbols * SYMBOL_LEN;

    buffer = (short *)calloc(total_samples, sizeof(short));
    tx_symbols = (unsigned char *)malloc(total_symbols);

    if (!buffer || !tx_symbols) {
        if (buffer) free(buffer);
        if (tx_symbols) free(tx_symbols);
        return -1;
    }

    /* СБОРКА И ДИФФЕРЕНЦИАЛЬНОЕ КОДИРОВАНИЕ ПАКЕТА */
    sym_idx = 0;
    unsigned char prev_encoded = 0; /* Стартовая точка кольца вращения */

    for (i = 0; i < payload_len; i++) {
        unsigned char parity_bit = calculate_odd_parity(payload[i]);

        /* 8 информационных бит (от старшего к младшему) */
        for (b = 7; b >= 0; b--) {
            unsigned char current_bit = (payload[i] >> b) & 0x01;
            tx_symbols[sym_idx] = scramble_bit(current_bit, prev_encoded);
            prev_encoded = tx_symbols[sym_idx];
            sym_idx++;
        }

        /* 9-й бит четности */
        tx_symbols[sym_idx] = scramble_bit(parity_bit, prev_encoded);
        prev_encoded = tx_symbols[sym_idx];
        sym_idx++;
    }

    /* ЧЕСТНЫЙ ЦЕЛОЧИСЛЕННЫЙ ЦИФРОВОЙ СИНТЕЗ СИГНАЛА (DDS) */
    out_idx = 0;
    for (i = 0; i < total_symbols; i++) {
        fixed_t step = fx_phase_steps[tx_symbols[i]];
        for (t = 0; t < SYMBOL_LEN; t++) {
            fixed_t fx_sin_val = fx_sin(fx_phase);

            /* Имитируем 8-битное квантование ШИМ-ЦАП (-127..127) */
            fixed_t fx_scaled = fx_mul(fx_sin_val, INT_TO_FX(127));
            int val_8bit = FX_TO_INT(fx_scaled);

            /* Пишем в 16-битный PCM WAV */
            buffer[out_idx++] = (short)(val_8bit << 8);

            fx_phase += step;
            if (fx_phase >= fx_two_pi) {
                fx_phase -= fx_two_pi;
            }
        }
    }

    /* Запись на диск */
    f_out = fopen(filename, "wb");
    if (!f_out) {
        free(buffer); free(tx_symbols);
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
    free(tx_symbols);
    return 0;
}
