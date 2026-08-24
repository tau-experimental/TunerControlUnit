#ifndef RECEIVER_H
#define RECEIVER_H

typedef struct {
    double coeff;
    double q0, q1, q2;
    int count;
    int len;
} Goertzel;

void goertzel_init(Goertzel *g, double target_freq, int len);
void goertzel_reset(Goertzel *g);
double goertzel_process(Goertzel *g, double sample);

int decode_fsk_wav(const char *filename, const double *freqs, unsigned char *out_payload);

#endif /* RECEIVER_H */
