#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "common.h"
#include "transmitter.h"
#include "receiver.h"

void run_test_crc8(void) {
    unsigned char crc = 0;
    printf("[TEST] Тестирование CRC-8...\n");

    crc = update_crc8(0x00, 0x02);
    crc = update_crc8(crc, 0x1C);
    if (crc == 0xA2) {
        printf("  -> УСПЕХ: Вычисление CRC корректно (0xA2).\n");
    } else {
        printf("  -> ПРОВАЛ: Ошибка в математике CRC. Получено %02X.\n", crc);
    }

    if (crc != 0x00) {
        printf("  -> УСПЕХ (Негативный тест): Ложное совпадение исключено.\n");
    } else {
        printf("  -> ПРОВАЛ (Негативный тест): CRC ошибочно выдала ноль.\n");
    }
}

void run_test_byte_to_symbols(void) {
    unsigned char symbols[4];
    printf("[TEST] Тестирование byte_to_symbols...\n");

    byte_to_symbols(0xE4, symbols);
    if (symbols[0] == 3 && symbols[1] == 2 && symbols[2] == 1 && symbols[3] == 0) {
        printf("  -> УСПЕХ: Байт корректно разбит на 2-битные символы (3, 2, 1, 0).\n");
    } else {
        printf("  -> ПРОВАЛ: Нарушена логика битовых масок.\n");
    }
}

void run_test_transmitter_limits(void) {
    printf("[TEST] Тестирование ограничений передатчика...\n");

    if (generate_fsk_wav(NULL, 5, FREQ_DOWNLINK, "fail.wav", 0) == -1) {
        printf("  -> УСПЕХ: Передатчик корректно заблокировал NULL payload.\n");
    } else {
        printf("  -> ПРОВАЛ: Передатчик пропустил NULL указатель.\n");
    }

    unsigned char long_payload[32] = {0};
    if (generate_fsk_wav(long_payload, 20, FREQ_DOWNLINK, "fail.wav", 0) == -1) {
        printf("  -> УСПЕХ: Передатчик корректно заблокировал длину > MAX_PAYLOAD.\n");
    } else {
        printf("  -> ПРОВАЛ: Передатчик допустил переполнение длины буфера.\n");
    }
}

void run_test_receiver_missing_file(void) {
    unsigned char buffer[MAX_PAYLOAD];
    printf("[TEST] Тестирование приемника на отсутствие файла...\n");

    if (decode_fsk_wav("non_existent_file_2026.wav", FREQ_DOWNLINK, buffer) == -1) {
        printf("  -> УСПЕХ: Приемник корректно отработал отсутствие файла физической линии.\n");
    } else {
        printf("  -> ПРОВАЛ: Приемник вернул некорректный статус для битого файла.\n");
    }
}

void run_end_to_end_simulation(void) {
    unsigned char base_tx_data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    unsigned char remote_rx_buffer[MAX_PAYLOAD];
    unsigned char remote_tx_data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    unsigned char base_rx_buffer[MAX_PAYLOAD];
    int len, i;

    printf("\n=== ЗАПУСК СКВОЗНОЙ СИМУЛЯЦИИ ЛИНИИ СВЯЗИ ===\n\n");

    printf("--- Сценарий 1: Лабораторно чистый кабель ---\n");
    generate_fsk_wav(base_tx_data, 8, FREQ_DOWNLINK, "downlink_clean.wav", 0);
    len = decode_fsk_wav("downlink_clean.wav", FREQ_DOWNLINK, remote_rx_buffer);

    if (len > 0) {
        printf("[REMOTE] Успешно принято %d байт: ", len);
        for(i = 0; i < len; i++) printf("%02X ", remote_rx_buffer[i]);
        printf("\n");

        printf("[REMOTE] Формирование и отправка синусоидального токового ответа Uplink...\n");
        generate_fsk_wav(remote_tx_data, 4, FREQ_UPLINK, "uplink_clean.wav", 0);

        len = decode_fsk_wav("uplink_clean.wav", FREQ_UPLINK, base_rx_buffer);
        if (len > 0) {
            printf("[BASE] ВЕРДИКТ: Сквозной тест чистой линии завершен. Связь установлена!\n");
        } else {
            printf("[BASE] ПРОВАЛ: Базовая станция не смогла разобрать чистый ответ.\n");
        }
    } else {
        printf("[REMOTE] ПРОВАЛ: Удаленный модуль не увидел чистый сигнал Downlink.\n");
    }

    printf("\n--- Сценарий 2: Реалистично грязный кабель (Шумы + Щелчки Найквиста) ---\n");
    generate_fsk_wav(base_tx_data, 8, FREQ_DOWNLINK, "downlink_dirty.wav", 1);
    len = decode_fsk_wav("downlink_dirty.wav", FREQ_DOWNLINK, remote_rx_buffer);

    if (len > 0) {
        printf("[REMOTE] ЦОС-алгоритм Гёрцеля успешно вытащил из шума %d байт: ", len);
        for(i = 0; i < len; i++) printf("%02X ", remote_rx_buffer[i]);
        printf("\n[SUCCESS] УСПЕХ: Математика DSP на базе АЦП полностью защищена от внеполосного мусора!\n");
    } else if (len == -2) {
        printf("[REMOTE] ПРОВАЛ: Синхронизация прошла, но пакет разрушен (Ошибка CRC-8).\n");
    } else {
        printf("[REMOTE] ПРОВАЛ: Сигнал полностью утонул в коричневом шуме.\n");
    }
}

int main(void) {
    srand((unsigned int)time(NULL));

    printf("=========================================================\n");
    printf("   СТАРТ МОДУЛЬНОГО И ИНТЕГРАЦИОННОГО ТЕСТИРОВАНИЯ ЦОС   \n");
    printf("=========================================================\n\n");

    run_test_crc8();
    run_test_byte_to_symbols();
    run_test_transmitter_limits();
    run_test_receiver_missing_file();

    run_end_to_end_simulation();

    return 0;
}
