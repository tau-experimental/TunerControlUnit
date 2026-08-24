#include "common.h"

const double FREQ_DOWNLINK[FREQ_BANDS] = {1000.0, 1400.0, 1800.0};
const double FREQ_UPLINK[FREQ_BANDS]   = {350.0,  750.0,  1150.0};

unsigned char update_crc8(unsigned char crc, unsigned char data) {
    int i;
    crc ^= data;
    for (i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0x31;
        } else {
            crc <<= 1;
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

/*
 * Вычисляет бит нечётности (Odd Parity) для 8-битного байта.
 * Возвращает 1, если количество единиц в байте чётное (чтобы сумма с битом стала нечётной),
 * и 0, если количество единиц уже нечётное.
 */
unsigned char calculate_odd_parity(unsigned char data_byte) {
    unsigned char count = 0;
    unsigned char temp = data_byte;
    while (temp) {
        count += (temp & 0x01);
        temp >>= 1;
    }
    /* Для Odd Parity: если сумма единиц чётная, нам нужна 1 */
    return (count % 2 == 0) ? 1 : 0;
}

unsigned char scramble_bit(unsigned char bit, unsigned char prev_encoded_sym) {
#if 1
    unsigned char p = prev_encoded_sym % 3;
    if ((bit & 0x01) == 0) {
        /* Бит 0: шаг по часовой стрелке (+1) */
        return (p + 1) % 3;
    } else {
        /* Бит 1: шаг против часовой стрелки (-1, что в поле %3 равно +2) */
        return (p + 2) % 3;
    }
#else

#endif
}

/*
 * Декодирование одного бита.
 * Принимает текущую частоту из линии (0..2) и предыдущую частоту (0..2).
 * Возвращает восстановленный бит (0 или 1).
 */
unsigned char descramble_bit(unsigned char encoded_sym, unsigned char prev_encoded_sym) {
#if 1
    unsigned char p = prev_encoded_sym % 3;
    unsigned char e = encoded_sym % 3;

    /* Вычисляем физический шаг сдвига в кольце */
    unsigned char shift = (e + 3 - p) % 3;

    if (shift == 1) {
        return 0; /* Шаг +1 означает, что передавали бит 0 */
    } else if (shift == 2) {
        return 1; /* Шаг +2 (или -1) означает, что передавали бит 1 */
    }

    return 0; /* Дефолтный возврат при ошибке (сдвиг 0 физически невозможен) */
#else
    int delta = encoded_sym - prev_encoded_sym;
    unsigned char current_bit = 0;
    if (delta < 0) {
        delta += 3;
    }

    // Теперь строго по вашему правилу:
    if (delta == 1) {
        // 0->1, 1->2, 2->0 (Частота увеличилась)
        current_bit = 1;
    }
    else if (delta == 2) {
        // 0->2, 2->1, 1->0 (Частота уменьшилась)
        current_bit = 0;
    }
    else {
        // delta == 0: Частота НЕ изменилась (по вашему правилу это ЗАПРЕЩЕНО)
        // Это маркер того, что мы либо стоим на шуме, либо просели по синхронизации!
        //signal_error_handler();
        printf ("Аномалия битового декодера: delta = %d\n", delta);
        exit(-100);
    }
    return current_bit;
#endif
}
