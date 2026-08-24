#ifndef COMMON_H
#define COMMON_H

#define FS 8000
#define SYMBOL_LEN 16
#define MAX_PAYLOAD 16

#define PREAMBLE_BYTE 0xAA
#define SOF_BYTE      0x7E

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

extern const double FREQ_DOWNLINK[4];
extern const double FREQ_UPLINK[4];

#pragma pack(push, 1)
typedef struct {
    char chunkID[4];
    unsigned int chunkSize;
    char format[4];
    char subchunk1ID[4];
    unsigned int subchunk1Size;
    unsigned short audioFormat;
    unsigned short numChannels;
    unsigned int sampleRate;
    unsigned int byteRate;
    unsigned short blockAlign;
    unsigned short bitsPerSample;
    char subchunk2ID[4];
    unsigned int subchunk2Size;
} WavHeader;
#pragma pack(pop)

unsigned char update_crc8(unsigned char crc, unsigned char data);
void byte_to_symbols(unsigned char b, unsigned char *syms);

#endif /* COMMON_H */
