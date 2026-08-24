#ifndef TRANSMITTER_H
#define TRANSMITTER_H

int generate_fsk_wav(const unsigned char *payload, int payload_len, const double *freqs, const char *filename, int dirty);

#endif /* TRANSMITTER_H */
