#ifndef COMMON_H
#define COMMON_H

#include <inttypes.h>

#define FS 8000
//#define SYMBOL_LEN 32
#define SYMBOL_LEN 64
#define EXPECTED_SYMBOL_LEN SYMBOL_LEN
#define MAX_PAYLOAD 16

#define BITS_PER_BYTE 9  /* 8 бит данных + 1 бит четности */

#define PREAMBLE_BYTE 0x55
#define SOF_BYTE      0x7E

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FREQ_BANDS	3
extern const double FREQ_DOWNLINK[FREQ_BANDS];
extern const double FREQ_UPLINK[FREQ_BANDS];

#pragma pack(push, 1)
typedef struct {
    char chunkID[4];         /* Должен быть массив из 4 символов, а не одиночный char! */
    unsigned int chunkSize;
    char format[4];          /* Тоже массив из 4 символов! */
    char subchunk1ID[4];     /* Массив */
    unsigned int subchunk1Size;
    unsigned short audioFormat;
    unsigned short numChannels;
    unsigned int sampleRate;
    unsigned int byteRate;
    unsigned short blockAlign;
    unsigned short bitsPerSample;
    char subchunk2ID[4];     /* Массив */
    unsigned int subchunk2Size;
} WavHeader;
#pragma pack(pop)

uint8_t odd(uint8_t b);

unsigned char update_crc8(unsigned char crc, unsigned char data);
void byte_to_symbols(unsigned char b, unsigned char *syms);

unsigned char calculate_odd_parity(unsigned char data_byte);

unsigned char scramble_bit(unsigned char bit, unsigned char prev_encoded_sym);
unsigned char descramble_bit(unsigned char encoded_sym, unsigned char prev_encoded_sym);

#endif /* COMMON_H */
