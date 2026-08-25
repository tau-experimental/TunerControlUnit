#include "transmitter.h"
#include "common.h"
#include "fixed_point.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "receiver.h"

/* Договоримся так:
 * передача 0 это уменьшение частоты (F0 -> F2 -> F1 -> F0)
 * передача 1 это увеличени частоты (F0 -> F1 -> F2 -> F0)
 * таким образом, вне зависимости от того, что именно передаётся,
 * частота следующего символа не может быть равна частоте предыдущего
 * и моменты переключения частоты будем использовать для ресинхронизации
 * */

int generate_fsk_preamble_test (void) {
    FILE *f_out;
    WavHeader header;
    uint8_t current_freq = 0; // с этой частоты начнём крутиться по треугольнику

    uint8_t bit_to_transmit = 0;
    uint8_t txbuf = PREAMBLE_BYTE;
    uint8_t Odd = odd(txbuf);
    int bitcount = 0, preamble_cycles;

    /* первые посылки преамбулы - намеренно "испорчены", с неправильным odd, потому что это просто чередующиеся 0 и 1.
     * простейший способ это сделать: передать сначала просто байт (8 бит) PREAMBLE_BYTE, а затем уже PREAMBLE_BYTE + odd
     * */
    /* т.к. я совсем отупел за годы бездействия, сейчас просто посмотрим на передаваемые биты! */
    printf ("В канал уйдут биты: [ ");
    for (preamble_cycles = 0; preamble_cycles < 2; preamble_cycles++) {
		for (bitcount = 0, txbuf = PREAMBLE_BYTE; bitcount < 8; bitcount ++, txbuf <<= 1) {
			bit_to_transmit = (txbuf & 0x80)>>7;
			printf ("%u ", bit_to_transmit);
		}
    };
    printf ("%u ]\n", bit_to_transmit = Odd);
    return 0;
}



uint8_t last_tx_frequency = 1;

uint8_t modulate_bit_to_frequency(uint8_t bit) {
    // Статическая переменная хранит состояние последней частоты в линии.
    // На старте инициализируем её, например, частотой F0 (индекс 0)

    if (bit == 1) {
        // Шаг вверх по треугольнику
        last_tx_frequency = (last_tx_frequency + 1) % 3;
    } else {
        // Шаг вниз по треугольнику
        last_tx_frequency = (last_tx_frequency + 2) % 3;
    }

    return last_tx_frequency;
}

// Глобальные переменные состояния передатчика (для CH32V003)
TxState_t tx_state = TX_IDLE;
uint8_t  tx_current_byte = 0;
uint8_t  tx_parity_bit = 0;
int      tx_cycles_left = 0;
int      tx_bit_idx = 0; // Счётчик бит внутри текущей 9-битной посылки (0..8)
int      tx_current_word_len = 8; // По умолчанию шлём обычные 8-битные байты
#define TEST_PAYLOAD_LENGTH	4
uint8_t tx_payload[TEST_PAYLOAD_LENGTH] = {0xAA, 0x55, 0x22, 0x77};
int     tx_payload_byte_idx = 0;
uint8_t tx_running_crc = 0;

// Полезная нагрузка для теста
uint8_t  test_payload_len = TEST_PAYLOAD_LENGTH; // Передадим тестовые 3 байта данных

void modem_tx_start(uint8_t initial_preamble) {
    tx_state = TX_PREAMBLE;
    tx_current_byte = PREAMBLE_BYTE;
    tx_parity_bit = odd(tx_current_byte);
    tx_cycles_left = initial_preamble;
    tx_bit_idx = 0;
    last_tx_frequency = 1; /* вс передачи начинаются с частоты №1 */
    // Если преамбула длинная, сначала шлём стандартные 8 бит.
    // Если передаем всего 1 байт преамбулы, он сразу должен быть 9-битным с ловушкой.
    tx_current_word_len = (tx_cycles_left > 1) ? 8 : 9;
    printf("[TX FSM] Старт передачи. Фаза: TX_PREAMBLE\n");
}

int modem_tx_tick(uint8_t *out_bit) {
    if (tx_state == TX_END) {
        return 0; // Передавать нечего
    }

    // 1. Формируем текущий бит на выдачу
    if (tx_bit_idx < 8) {
        // Выдаем биты байта от MSB к LSB (старший бит вперёд)
        *out_bit = (tx_current_byte >> (7 - tx_bit_idx)) & 1;
    } else {
        // 9-й шаг: выдаем бит нечётности
        *out_bit = tx_parity_bit;
    }

    // Отладочный вывод каждого выданного бита
    printf("[TX BIT] Состояние: %d | Бит %d/%d: %d\n", tx_state, tx_bit_idx, tx_current_word_len, *out_bit);

    // 2. Логика автомата шагов и переключения состояний
    tx_bit_idx++;

    if (tx_bit_idx == tx_current_word_len) {
        // Мы полностью выдали одну 9-битную посылку
        tx_bit_idx = 0; // Сброс для следующего байта
        tx_cycles_left--;

        // Если мы в преамбуле и остался ровно 1 цикл — включаем 9-битную ловушку!
        if (tx_state == TX_PREAMBLE && tx_cycles_left == 1) {
            tx_current_word_len = 9;
            printf("[TX FSM] Внимание: следующий байт преамбулы финальный, включаем 9-й бит (Odd)!\n");
        }

        if (tx_cycles_left <= 0) {
            // Текущая фаза завершена, планируем следующую
            switch (tx_state) {
                case TX_PREAMBLE: {
                    tx_state = TX_SOF;
                    tx_current_byte = SOF_BYTE;
                    tx_parity_bit = odd(tx_current_byte);
                    tx_cycles_left = 1; // 1 байт SOF
                    //tx_current_word_len = 9; // Все последующие системные байты и данные ВСЕГДА идут с паритетом (9 бит)

                    printf("[TX FSM] Переход на Фазу: TX_SOF (0x%02X)\n", tx_current_byte);
            	};break;

                case TX_SOF: {
                	tx_state = TX_LEN; // <-- Переходим к передаче длины
					tx_current_byte = test_payload_len; // Загружаем число 3
					tx_parity_bit = odd(tx_current_byte);
					tx_cycles_left = 1;
					tx_current_word_len = 9; // Длина тоже идет строго 9 бит
					tx_running_crc = update_crc8(0, test_payload_len);
					printf("[TX FSM] Переход -> TX_LEN (Длина полезной нагрузки: %d байта)\n", tx_current_byte);

                }; break;

                case TX_LEN: {
                    tx_state = TX_PAYLOAD;
                    tx_payload_byte_idx = 0; // Start at the first byte of payload
                    tx_current_byte = tx_payload[tx_payload_byte_idx];
                    tx_running_crc = update_crc8(tx_running_crc, tx_current_byte);
                    tx_parity_bit = odd(tx_current_byte);
                    tx_cycles_left = 1; // Load 1 byte at a time
                    tx_current_word_len = 9;
                    printf("[TX FSM] Переход -> TX_PAYLOAD [Byte 0: 0x%02X, running CRC: 0x%02X]\n", tx_current_byte, tx_running_crc);
                }; break;

                case TX_PAYLOAD:
                    tx_payload_byte_idx++;
                    if (tx_payload_byte_idx < test_payload_len) {
                        // We still have bytes left in the payload array
                        tx_current_byte = tx_payload[tx_payload_byte_idx];
                        tx_running_crc = update_crc8(tx_running_crc, tx_current_byte);
                        tx_parity_bit = odd(tx_current_byte);
                        tx_cycles_left = 1; // Process next byte
                        tx_current_word_len = 9;
                        printf("[TX FSM] TX_PAYLOAD continues [Byte %d: 0x%02X, running CRC: 0x%02X]\n",
                        		tx_payload_byte_idx,
                        		tx_current_byte,
                        		tx_running_crc);
                    } else {
                        // All payload bytes sent! For now, skip CRC and end the test
                        tx_state = TX_CRC;
                        tx_current_byte = tx_running_crc;
                        tx_parity_bit = odd(tx_current_byte);
                        tx_cycles_left = 1; // Process next byte
                        tx_current_word_len = 9;
                        printf("[TX FSM] All payload bytes sent. Transmiting CRC 0x%02X.\n", tx_current_byte);
                    }
                    break;

                case TX_CRC: {
                	tx_state = TX_END;
                	printf("[TX FSM] CRC sent. Ending test.\n");
                }; break;

                default:
                    tx_state = TX_END;
                    break;
            }
        }
    }

    return 1; // Бит успешно выдан
}

// Накопитель фазы DDS (32-битный регистр)
// Мы НЕ сбрасываем его при смене частот, обеспечивая непрерывность сигнала
uint32_t dds_phase = 0;
uint32_t dds_phase_step = 0;

// Счетчик отсчетов внутри текущей посылки (0..31)
int sample_in_symbol_counter = 0;

#define DDS_TUNING_WORD(fbin)	(uint32_t)(((uint64_t)4294967296*(uint64_t)(fbin))/(8000))
// Шаги приращения фазы для Goertzel Bins 4, 5, 6 при частоте дискретизации 8 кГц
// Формула: step = (2^32 * F_bin) / F_sampling
// F0 (Bin 4 = 1000 Гц), F1 (Bin 5 = 1250 Гц), F2 (Bin 6 = 1500 Гц)
const uint32_t DDS_STEPS[3] = {
    //536870912,  // F0 (1000 Гц)
    //671088640,  // F1 (1250 Гц)
    //805306368   // F2 (1500 Гц)
		DDS_TUNING_WORD(1000),
		DDS_TUNING_WORD((1000 + 2000)/2),
		DDS_TUNING_WORD(2000)
};

// 256-point 10-bit Sine Lookup Table (Centered at 511, peak amplitude ~200)
static const uint16_t SINE_LUT_10BIT[256] = {
    511, 516, 521, 526, 531, 536, 541, 546, 551, 556, 560, 565, 570, 575, 579, 584,
    589, 593, 598, 602, 607, 611, 615, 620, 624, 628, 632, 636, 640, 644, 648, 652,
    656, 659, 663, 666, 670, 673, 676, 680, 683, 686, 689, 692, 695, 697, 700, 702,
    705, 707, 709, 711, 713, 715, 716, 718, 719, 721, 722, 723, 724, 725, 725, 726,
    726, 726, 725, 725, 724, 723, 722, 721, 719, 718, 716, 715, 713, 711, 709, 707,
    705, 702, 700, 697, 695, 692, 689, 686, 683, 680, 676, 673, 670, 666, 663, 659,
    656, 652, 648, 644, 640, 636, 632, 628, 624, 620, 615, 611, 607, 602, 598, 593,
    589, 584, 579, 575, 570, 565, 560, 556, 551, 546, 541, 536, 531, 526, 521, 516,
    511, 506, 501, 496, 491, 486, 481, 476, 471, 466, 462, 457, 452, 447, 443, 438,
    433, 429, 424, 420, 415, 411, 407, 402, 398, 394, 390, 386, 382, 378, 374, 370,
    366, 363, 359, 356, 352, 349, 346, 342, 339, 336, 333, 330, 327, 325, 322, 320,
    317, 315, 313, 311, 309, 307, 306, 304, 303, 301, 300, 299, 298, 297, 297, 296,
    296, 296, 297, 297, 298, 299, 300, 301, 303, 304, 306, 307, 309, 311, 313, 315,
    317, 320, 322, 325, 327, 330, 333, 336, 339, 342, 346, 349, 352, 356, 359, 363,
    366, 370, 374, 378, 382, 386, 390, 394, 398, 402, 407, 411, 415, 420, 424, 429,
    433, 438, 443, 447, 452, 457, 462, 466, 471, 476, 481, 486, 491, 496, 501, 506
};

/**
 * Tabular Phase-Continuous DDS.
 * Outputs a signed 16-bit word derived from an 8-bit hardware simulation profile.
 */
int16_t get_next_tabular_sample_10bit(void) {
    if (sample_in_symbol_counter == 0) {
        uint8_t next_bit;
        if (modem_tx_tick(&next_bit)) {
            uint8_t current_freq_idx = modulate_bit_to_frequency(next_bit);
            dds_phase_step = DDS_STEPS[current_freq_idx];
        } else {
            return 0; // Конец передачи -> Тишина
        }
    }

    dds_phase += dds_phase_step;

    // Берем старшие 8 бит аккумулятора фазы для шага по таблице (256 точек)
    uint8_t lut_index = (uint8_t)(dds_phase >> 24);
    uint16_t raw_10bit_sample = SINE_LUT_10BIT[lut_index]; // Значение 0..1023

    // Для боевого МК: прямо здесь мы бы писали raw_10bit_sample в регистр ШИМ: TIM1->CH1CVR = raw_10bit_sample;

    // Для ПК-модели WAV: переводим в знаковый 16-битный PCM (сдвиг центра 511 -> 0)
    int16_t wav_compatible_sample = ((int16_t)raw_10bit_sample - 511) << 6;

    sample_in_symbol_counter++;
    if (sample_in_symbol_counter >= SYMBOL_LEN) {
        sample_in_symbol_counter = 0;
    }

    return wav_compatible_sample;
}


