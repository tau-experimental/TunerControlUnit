#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "common.h"
#include "fixed_point.h"
#include "transmitter.h"
#include "receiver.h"

/* Вспомогательная функция для генерации чистого тона в массив отсчетов (имитация АЦП) */
void generate_pure_tone(short *buffer, int num_samples, double freq) {
    int i;
    for (i = 0; i < num_samples; i++) {
        double angle = 2.0 * M_PI * freq * i / (double)FS;
        int val_8bit = (int)(sin(angle) * 127.0); /* 8-битное квантование ШИМ */
        buffer[i] = (short)(val_8bit << 8);        /* Масштаб под 16-бит WAV */
    }
}

/* ==============================================================================
 * ЭТАП 1: ТЕСТИРОВАНИЕ АНАЛОГОВЫХ ФИЛЬТРОВ ГЁРЦЕЛЯ (In-band / Out-of-band)
 * ==============================================================================
 */
void test_step_1_filters(void) {
    printf("[STEP 1] Тестирование избирательности фильтров Гёрцеля...\n");

    Goertzel_Fx detector;
    short sample_buffer[SYMBOL_LEN];
    int t;
    fixed_t energy;
    int bin;
    double freq_half_step = (FREQ_DOWNLINK[1] - FREQ_DOWNLINK[0])/2;

    for (bin = 0; bin < FREQ_BANDS; bin++) {
    	printf("Тест высокодобротного фильтра №%d, частота %.2f (полушаг %.2f): \n", bin, FREQ_DOWNLINK[bin], freq_half_step);
		/* Настроим фильтр на одну из частот Downlink */
		goertzel_fx_init(&detector, FREQ_DOWNLINK[bin], SYMBOL_LEN);

		/* Тест 1А: Подаем ИДЕАЛЬНЫЙ полезный сигнал */
		generate_pure_tone(sample_buffer, SYMBOL_LEN, FREQ_DOWNLINK[bin]);
		goertzel_fx_reset(&detector);
		for (t = 0; t < SYMBOL_LEN; t++) {
			int adc_sample = (int)sample_buffer[t] / 256;
			energy = goertzel_sliding_process(&detector, INT_TO_FX(adc_sample), 0);
		}
		printf("  -> Энергия на целевой частоте %d Гц: %.2f\n", (int)FREQ_DOWNLINK[bin], FX_TO_DOUBLE(energy));
		if (energy < INT_TO_FX(1600)) {
			printf("  [FAIL] Фильтр нечувствителен к собственной частоте!\n");
			exit(-1);
		}
		printf("  -> УСПЕХ: Фильтр отлично видит внутриполосный сигнал.\n");

		goertzel_fx_init(&detector, FREQ_DOWNLINK[bin], SYMBOL_LEN);

		if (bin == 0) { /* самый низкий тон: два теста */
			/* Тест 1Б-0: Подаем ВНЕПОЛОСНЫЙ сигнал ниже полосы */
			double freq = FREQ_DOWNLINK[bin] - freq_half_step;
			generate_pure_tone(sample_buffer, SYMBOL_LEN, freq);
			goertzel_fx_reset(&detector);
			for (t = 0; t < SYMBOL_LEN; t++) {
				int adc_sample = (int)sample_buffer[t] / 256;
				energy = goertzel_sliding_process(&detector, INT_TO_FX(adc_sample), 0);
			}
			printf("  -> Энергия на внеполосной частоте %d Гц: %.2f\n", (int)(freq), FX_TO_DOUBLE(energy));
			if (energy > INT_TO_FX(400)) {
				printf("  [FAIL] Фильтр пропускает внеполосный мусор!\n");
				exit(-1);
			}
			printf("  -> УСПЕХ: Фильтр полностью подавляет внеполосные частоты.\n");
		} else {
			/* Тест 1Б-1: Подаем ВНЕПОЛОСНЫЙ сигнал между или выше полосы */
			double freq = FREQ_DOWNLINK[bin] + freq_half_step;
			generate_pure_tone(sample_buffer, SYMBOL_LEN, freq);
			goertzel_fx_reset(&detector);
			for (t = 0; t < SYMBOL_LEN; t++) {
				int adc_sample = (int)sample_buffer[t] / 256;
				energy = goertzel_sliding_process(&detector, INT_TO_FX(adc_sample), 0);
			}
			printf("  -> Энергия на внеполосной частоте %d Гц: %.2f\n", (int)(freq), FX_TO_DOUBLE(energy));
			if (energy > INT_TO_FX(400)) {
				printf("  [FAIL] Фильтр пропускает внеполосный мусор!\n");
				exit(-1);
			}
			printf("  -> УСПЕХ: Фильтр полностью подавляет внеполосные частоты.\n");
		}
    }
    printf("=== ЭТАП 1 УСПЕШНО ПРОЙДЕН ===\n\n");
}

/* ==============================================================================
 * ЭТАП 2: ТЕСТИРОВАНИЕ КОДЕКА СВЯЗИ (Scrambler / Descrambler)
 * ==============================================================================
 */
void test_step_2_codec(void) {
	printf("[STEP 2] Тестирование 9-битного дифференциального кодека вращения частот (Данные + Odd Parity)...\n");

	unsigned char raw_data[128];
	unsigned char encoded_symbols[128 * BITS_PER_BYTE];
	unsigned char decoded_data[128];
	int run, i, b, sym_idx;
	unsigned char prev_encoded;

	for (run = 0; run < 3; run++) {
		prev_encoded = 0; /* Стартовая частота кольца (0..2) */
		sym_idx = 0;

		if (run == 0) {
			printf("  -> Прогон 2А: Случайный поток данных...\n");
			for (i = 0; i < 128; i++) raw_data[i] = rand() % 256;
		} else if (run == 1) {
			printf("  -> Прогон 2Б: Статический поток из нулей (0x00)...\n");
			for (i = 0; i < 128; i++) raw_data[i] = 0x00;
		} else {
			printf("  -> Прогон 2В: Статический поток из единиц (0xFF)...\n");
			for (i = 0; i < 128; i++) raw_data[i] = 0xFF;
		}

		/* 1. КОДИРОВАНИЕ: 8 бит данных + 1 бит Odd Parity */
		for (i = 0; i < 128; i++) {
			unsigned char parity_bit = calculate_odd_parity(raw_data[i]);

			/* Сначала кодируем 8 бит данных (от MSB к LSB) */
			for (b = 7; b >= 0; b--) {
				unsigned char current_bit = (raw_data[i] >> b) & 0x01;
				encoded_symbols[sym_idx] = scramble_bit(current_bit, prev_encoded);
				prev_encoded = encoded_symbols[sym_idx];
				sym_idx++;
			}

			/* 9-м шагом отправляем в линию бит чётности */
			encoded_symbols[sym_idx] = scramble_bit(parity_bit, prev_encoded);
			prev_encoded = encoded_symbols[sym_idx];
			sym_idx++;
		}

		/* 2. ПРИДИРЧИВАЯ ПРОВЕРКА ЛИНИИ */
		for (i = 1; i < 128 * BITS_PER_BYTE; i++) {
			if (encoded_symbols[i] == encoded_symbols[i-1]) {
				printf("    [FAIL] Нарушено правило линии на символе %d! Частота повторилась: %d\n", i, encoded_symbols[i]);
				exit(-1);
			}
			if (encoded_symbols[i] > 2) {
				printf("    [FAIL] Выход за границы 3-частотной сетки: %d\n", encoded_symbols[i]);
				exit(-1);
			}
		}
		printf("    -> Проверка коаксиального кабеля: %d символов вращаются идеально, 0%% повторов.\n", 128 * BITS_PER_BYTE);

		/* 3. ДЕКОДИРОВАНИЕ: Извлечение байта и валидация чётности */
		sym_idx = 0;
		prev_encoded = 0;
		for (i = 0; i < 128; i++) {
			unsigned char assembled_byte = 0;

			/* Восстанавливаем 8 бит данных */
			for (b = 7; b >= 0; b--) {
				unsigned char decoded_bit = descramble_bit(encoded_symbols[sym_idx], prev_encoded);
				assembled_byte |= (decoded_bit << b);
				prev_encoded = encoded_symbols[sym_idx];
				sym_idx++;
			}

			/* Читаем 9-й бит (пришедший Parity) */
			unsigned char rx_parity_bit = descramble_bit(encoded_symbols[sym_idx], prev_encoded);
			prev_encoded = encoded_symbols[sym_idx];
			sym_idx++;

			/* Математическая проверка чётности на приёме */
			unsigned char local_parity_bit = calculate_odd_parity(assembled_byte);
			if (rx_parity_bit != local_parity_bit) {
				printf("    [FAIL] Ошибка чётности на байте %d! Ожидали бит %d, приняли %d\n", i, local_parity_bit, rx_parity_bit);
				exit(-1);
			}

			decoded_data[i] = assembled_byte;
		}

		/* 4. ВЕРИФИКАЦИЯ ВОССТАНОВЛЕНИЯ ДАННЫХ */
		for (i = 0; i < 128; i++) {
			if (raw_data[i] != decoded_data[i]) {
				printf("    [FAIL] Искажение данных на байте %d! Исходный: %02X, Восстановленный: %02X\n", i, raw_data[i], decoded_data[i]);
				exit(-1);
			}
		}
		printf("    -> Прогон успешно завершен. Данные и чётность верифицированы.\n");
	}
	printf("=== ЭТАП 2 УСПЕШНО ПРОЙДЕН ===\n\n");
}

/* ==============================================================================
 * ЭТАП 3: ТЕСТИРОВАНИЕ ГЕНЕРАТОРА СИГНАЛА И ЕГО СИНХРОННОГО СЧИТЫВАНИЯ
 * ==============================================================================
 */
void test_step_3_hardware_synthesis(void) {
    printf("[STEP 3] Верификация 3-частотного DDS-синтезатора и синхронного ЦОС-распознавания...\n");

    unsigned char test_payload;
    unsigned char rx_buffer[MAX_PAYLOAD];
    int s, len;

    /* Экстремальные байты для проверки всех типов вращения фазы */
    unsigned char cases[4] = {0x00, 0xFF, 0x55, 0x96};
    const char *filenames[4] = {"test_rot_00.wav", "test_rot_FF.wav", "test_rot_55.wav", "test_rot_96.wav"};
    const char *descriptions[4] = {
        "сплошные нули (чистое вращение по часовой стрелке)",
        "сплошные единицы (чистое вращение против часовой стрелки)",
        "маятниковый паттерн 01010101 (реверс направления на каждом шаге)",
        "случайный информационный байт 0x96"
    };

    for (s = 0; s < 4; s++) {
        test_payload = cases[s];
        printf("  -> Тестирование физического синтеза для: %s...\n", descriptions[s]);

        /*
         * Передатчик генерирует WAV-файл "в стык" (без паузы),
         * кодируя байт в 9 физических символов вращения (8 данных + 1 Odd Parity).
         */
        if (generate_fsk_wav(&test_payload, 1, FREQ_DOWNLINK, filenames[s], 0) != 0) {
            printf("    [FAIL] Ошибка генерации WAV-файла %s!\n", filenames[s]);
            exit(-1);
        }

        /*
         * Синхронный приемник считывает файл блоками по SYMBOL_LEN (32 отсчета),
         * вычисляет векторы вращения и проверяет аппаратный бит четности.
         */
        len = decode_fsk_wav(filenames[s], FREQ_DOWNLINK, rx_buffer);

        if (len != 1) {
            if (len == -2) {
                printf("    [FAIL] Авария: Сбой валидации бита нечётности (Odd Parity)! Фильтры неверно распознали знак вращения частоты.\n");
            } else {
                printf("    [FAIL] Авария физического уровня: Файл поврежден или не прочитан. Код: %d\n", len);
            }
            exit(-1);
        }

        if (rx_buffer[0] != test_payload) {
            printf("    [FAIL] Ошибка декодирования! Отправили: %02X, Приняли: %02X\n", test_payload, rx_buffer[0]);
            exit(-1);
        }

        printf("    [SUCCESS] Байт %02X (вместе с Parity) успешно передан через синус, ШИМ-квантован и восстановлен ЦОС.\n", test_payload);
    }

    printf("=== ЭТАП 3 УСПЕШНО ПРОЙДЕН ===\n\n");
}


/* ==============================================================================
 * ЭТАП 4: СКВОЗНАЯ СИНХРОНИЗАЦИЯ КАДРА И ЛИНЕЙНЫЕ ТЕСТЫ
 * ==============================================================================
 */
void test_step_4_integration(void) {
    printf("[STEP 4] Тестирование сквозной динамической синхронизации кадра...\n");

    unsigned char base_tx_data[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    unsigned char remote_rx_buffer[MAX_PAYLOAD];
    int len, i;

    /* Генерируем грязь (Ослабление 10дБ, коричневый шум, щелчки Найквиста) */
    generate_fsk_wav(base_tx_data, 8, FREQ_DOWNLINK, "downlink_final_test.wav", 1);

    /* Приемник должен запустить Clock Recovery, поймать фронты переключений частот, встать на фазу и дескремблировать пакет */
    len = decode_fsk_wav("downlink_final_test.wav", FREQ_DOWNLINK, remote_rx_buffer);

    if (len == 8) {
        printf("  -> [REMOTE] Пакет успешно извлечен из грязного кабеля: ");
        for(i = 0; i < len; i++) printf("%02X ", remote_rx_buffer[i]);
        printf("\n=== ЭТАП 4 УСПЕШНО ПРОЙДЕН! ПОЛНЫЙ УСПЕХ СИСТЕМЫ ===\n");
    } else {
        printf("  [FAIL] Сквозная синхронизация кадра упала в шумах. Код ошибки приемника: %d\n", len);
        exit(-1);
    }
}

int main(void) {
    srand((unsigned int)time(NULL));

    printf("=========================================================\n");
    printf("  ПОШАГОВАЯ BOTTOM-UP ВЕРИФИКАЦИЯ ЦОС И ФИЗИЧЕСКОГО СЛОЯ \n");
    printf("=========================================================\n\n");

    /* Выполняем тесты строго от простого к сложному */
    test_step_1_filters();
    test_step_2_codec();
    test_step_3_hardware_synthesis();
    test_step_4_integration();

    return 0;
}
