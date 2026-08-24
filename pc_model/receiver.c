#include "receiver.h"
#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

void goertzel_init(Goertzel *g, double target_freq, int len) {
    double k = ((double)len * target_freq) / (double)FS;
    g->coeff = 2.0 * cos(2.0 * M_PI * k / (double)len);
    g->q0 = 0.0; g->q1 = 0.0; g->q2 = 0.0;
    g->count = 0;
    g->len = len;
}

void goertzel_reset(Goertzel *g) {
    g->q0 = 0.0; g->q1 = 0.0; g->q2 = 0.0;
    g->count = 0;
}

double goertzel_process(Goertzel *g, double sample) {
    g->q0 = sample + g->coeff * g->q1 - g->q2;
    g->q2 = g->q1;
    g->q1 = g->q0;
    g->count++;

    if (g->count >= g->len) {
        double mag_sq = g->q1 * g->q1 + g->q2 * g->q2 - g->coeff * g->q1 * g->q2;
        goertzel_reset(g);
        return mag_sq;
    }
    return -1.0;
}

int decode_fsk_wav(const char *filename, const double *freqs, unsigned char *out_payload) {
    FILE *f_in;
    WavHeader header;
    int num_samples, max_possible_syms, rx_sym_count, in_packet, expected_symbols, payload_len, idx, i, t;
    short *samples;
    unsigned char *rx_symbols;
    Goertzel detectors[4];

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
        goertzel_init(&detectors[i], freqs[i], SYMBOL_LEN);
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

    while (idx < num_samples - SYMBOL_LEN) {
        double amplitudes[4];
        double max_amp = -1.0;
        int winner_sym = 0;

        for (i = 0; i < 4; i++) {
            goertzel_reset(&detectors[i]);
            for (t = 0; t < SYMBOL_LEN; t++) {
                double norm_sample = (double)samples[idx + t] / 256.0;
                double res = goertzel_process(&detectors[i], norm_sample);
                if (res >= 0) amplitudes[i] = res;
            }
            if (amplitudes[i] > max_amp) {
                max_amp = amplitudes[i];
                winner_sym = i;
            }
        }

        if (max_amp > 500.0) {
            rx_symbols[rx_sym_count++] = (unsigned char)winner_sym;

            if (!in_packet && rx_sym_count >= 4) {
                int last_idx = rx_sym_count - 4;
                unsigned char test_sof = (rx_symbols[last_idx] << 6) |
                                         (rx_symbols[last_idx+1] << 4) |
                                         (rx_symbols[last_idx+2] << 2) |
                                         rx_symbols[last_idx+3];
                if (test_sof == SOF_BYTE) {
                    in_packet = 1;
                    rx_sym_count = 0;
                }
            }
        }

        if (in_packet) {
            if (rx_sym_count == 4 && expected_symbols == 9999) {
                payload_len = (rx_symbols[0] << 6) | (rx_symbols[1] << 4) | (rx_symbols[2] << 2) | rx_symbols[3];
                if (payload_len > MAX_PAYLOAD || payload_len <= 0) {
                    in_packet = 0;
                    rx_sym_count = 0;
                    expected_symbols = 9999;
                } else {
                    expected_symbols = (1 + payload_len + 1) * 4;
                }
            }

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
                    return payload_len;
                } else {
                    return -2;
                }
            }
        }

        if (in_packet) {
            idx += SYMBOL_LEN;
        } else {
            idx += 1;
        }
    }

    free(samples);
    free(rx_symbols);
    return -1;
}
