#ifndef RECEIVER_H
#define RECEIVER_H

#include <inttypes.h>

#include "fixed_point.h"

typedef struct {
    fixed_t coeff;
    fixed_t q0, q1, q2;
    int count;
    int len;
} Goertzel_Fx;

void goertzel_fx_init(Goertzel_Fx *g, double target_freq, int len);
void goertzel_fx_reset(Goertzel_Fx *g);
fixed_t goertzel_fx_process(Goertzel_Fx *g, fixed_t sample);

/* Обновленный прототип декодера (внутренняя обработка полностью на fixed_t) */
int decode_fsk_wav(const char *filename, const double *freqs, unsigned char *out_payload);

#endif /* RECEIVER_H */
