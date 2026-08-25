#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <inttypes.h>

typedef enum {
	TX_IDLE,
	TX_PREAMBLE,
	TX_SOF,
    TX_LEN,
    TX_PAYLOAD,
	TX_CRC,
	TX_END
} TxState_t;

extern TxState_t tx_state;

extern uint8_t last_tx_frequency;
/* Обновленный прототип: передатчик полностью оперирует на фиксированной точке */
int generate_fsk_wav(const unsigned char *payload, int payload_len, const double *freqs, const char *filename, int dirty);

void modem_tx_start(uint8_t initial_preamble);
int modem_tx_tick(uint8_t *out_bit);
int16_t get_next_tabular_sample(void);

#endif /* TRANSMITTER_H */
