#include "transmitter.h"
#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

int generate_fsk_wav(const unsigned char *payload, int payload_len, const double *freqs, const char *filename, int dirty) {
    int total_bytes, total_symbols, pause_samples, end_samples, signal_samples, total_samples;
    short *buffer;
    unsigned char *symbols;
    unsigned char crc = 0;
    int i, s_idx, t, out_idx;
    double pause_sec, phase, double_rand;
    FILE *f_out;
    WavHeader header;

    if (payload_len < 0 || payload_len > MAX_PAYLOAD || !payload || !freqs || !filename) {
        return -1;
    }

    total_bytes = 4 + payload_len + 1;
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

    for (i = 0; i < payload_len; i++) {
        crc = update_crc8(crc, payload[i]);
    }

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

    phase = 0.0;
    out_idx = pause_samples;

    for (i = 0; i < total_symbols; i++) {
        double f = freqs[symbols[i]];
        double phase_step = 2.0 * M_PI * f / (double)FS;
        for (t = 0; t < SYMBOL_LEN; t++) {
            int val_8bit = (int)(sin(phase) * 127.0);
            buffer[out_idx++] = (short)(val_8bit << 8);
            phase += phase_step;
            if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
        }
    }

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
