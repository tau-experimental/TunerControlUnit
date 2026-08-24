#include "common.h"

const double FREQ_DOWNLINK[4] = {1200.0, 1500.0, 1800.0, 2100.0};
const double FREQ_UPLINK[4]   = {350.0,  450.0,  550.0,  650.0};

unsigned char update_crc8(unsigned char crc, unsigned char data) {
    int i;
    crc ^= data;
    for (i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = ((crc << 1) ^ 0x31) & 0xFF;
        } else {
            crc = (crc << 1) & 0xFF;
        }
    }
    return crc;
}

void byte_to_symbols(unsigned char b, unsigned char *syms) {
    syms[0] = (b >> 6) & 0x03;
    syms[1] = (b >> 4) & 0x03;
    syms[2] = (b >> 2) & 0x03;
    syms[3] = b & 0x03;
}
