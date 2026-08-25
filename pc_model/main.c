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

void dumb_dump_wav (const char *filename, int16_t *data, size_t length) {
    FILE *f_out;
    WavHeader header;

    f_out = fopen(filename, "wb");

    memcpy(header.chunkID, "RIFF", 4);
    header.chunkSize = 36 + length * 2;
    memcpy(header.format, "WAVE", 4);
    memcpy(header.subchunk1ID, "fmt ", 4);
    header.subchunk1Size = 16;
    header.audioFormat = 1;
    header.numChannels = 1;
    header.sampleRate = FS;
    header.byteRate = FS * 2;
    header.blockAlign = 2;
    header.bitsPerSample = 16;
    memcpy(header.subchunk2ID, "data", 4);
    header.subchunk2Size = length * 2;

    fwrite(&header, sizeof(WavHeader), 1, f_out);
    fwrite(data, sizeof(short), length, f_out);
    fclose(f_out);
}

/**
 * Simulates a brutal, noisy PLC electrical wire environment.
 * Expects a standard 10-bit simulated ADC input sample (0..1023).
 * Returns a corrupted 10-bit sample reflecting real wire conditions.
 */
uint16_t apply_realistic_plc_noise(uint16_t clean_sample, int noise_level) {
    // 1. Переводим в знаковый диапазон относительно центра 511
    int32_t signal = (int32_t)clean_sample - 511;

    // 2. Генерируем белый шум по Центральной Предельной Теореме
    // (сумма случайных величин дает красивое Гауссово распределение)
    int32_t awgn = 0;
    if (noise_level > 0) {
        for (int i = 0; i < 4; i++) {
            awgn += (rand() % noise_level) - (noise_level / 2);
        }
        awgn /= 2; // Нормализуем размах шума
    }

    // 3. Подмешиваем шум к нашему мощному сигналу
    signal += awgn;

    // 4. Жесткое ограничение под 10-битный диапазон АЦП (Clipping guard)
    if (signal > 512)  signal = 512;
    if (signal < -511) signal = -511;

    // 5. Возвращаем как беззнаковое значение АЦП
    return (uint16_t)(signal + 511);
}

/**
 * Подмешивает чистый белый шум к 16-битному знаковому PCM сигналу.
 * noise_amplitude задает размах шума в диапазоне 16-битного звука.
 * Рекомендуемые значения:
 *   0     - лабораторный чистый сигнал
 *   2000  - легкое шипение
 *   8000  - плотный шум
 *   15000 - жестокий грохот, сигнал на грани уничтожения
 */
int16_t apply_pcm_noise_16bit(int16_t clean_sample, int32_t noise_amplitude) {
    int32_t mixed_signal = clean_sample;

    if (noise_amplitude > 0) {
        // Суммируем 4 случайных числа для идеального Гауссова распределения шума
        int32_t noise = 0;
        for (int i = 0; i < 4; i++) {
            noise += (rand() % noise_amplitude) - (noise_amplitude / 2);
        }
        noise /= 2; // Нормализуем амплитуду

        mixed_signal += noise;
    }

    // Жесткое ограничение (Clipping) под стандартный 16-битный WAV-контейнер
    if (mixed_signal > 32767)  mixed_signal = 32767;
    if (mixed_signal < -32768) mixed_signal = -32768;

    return (int16_t)mixed_signal;
}

#define TOTAL_TEST_SAMPLES 25000 // High enough budget to capture all 71 bits * 32 samples

void generate_test_signal_file(void) {
    int16_t wav_buffer[TOTAL_TEST_SAMPLES];
    int recorded_samples = 0;
    int freq_index;
    int16_t clean_sample, dirty_sample;
    uint16_t adc_input_10bit;
    int noise_severity = 2;
    int spike_rate = 5;

    int32_t current_noise_level = 6000;

    // Initialize our deterministic FSM packet stack
    modem_tx_start(2); // Start with 2 bytes of preamble

    for (int i = 0; i < TOTAL_TEST_SAMPLES; i++) {
    	clean_sample = (i < 103) ? 511: get_next_tabular_sample_10bit();
    	dirty_sample = apply_pcm_noise_16bit(clean_sample, current_noise_level) + 4000; /* катастрофический DC-сдвиг */
    	//dirty_sample = clean_sample;
    	//dirty_sample = apply_pcm_noise_16bit(clean_sample, current_noise_level);

#if 0
        if ((freq_index = process_goertzel_sample_10bit (sample)) >= 0) {
        	printf ("Pretzel frequency index: %d\n", freq_index);
        }
#endif
#if 1
        adc_input_10bit = (uint16_t)((dirty_sample >> 6) + 511);
        process_adc_sample_stream (adc_input_10bit);
#endif

        // If transmission completes entirely, we fill the remaining budget with silence
        if (tx_state == TX_END && dirty_sample == 0) {
            wav_buffer[i] = 0;
        } else {
            wav_buffer[i] = adc_input_10bit*8;
            recorded_samples = i; // Track the exact endpoint of the real signal
        }
    }

    // At this point, write 'wav_buffer' to disk using your favorite standard
    // WAV-header write loop (e.g., standard write_wav("output.wav", wav_buffer, ...))
    dumb_dump_wav ("generated.wav", wav_buffer, TOTAL_TEST_SAMPLES);
    printf("[WAV GEN] Finished processing. Signal duration: %d samples.\n", recorded_samples);
}

int main(void) {
    srand((unsigned int)time(NULL));

    printf("=========================================================\n");
    printf("  ПОШАГОВАЯ BOTTOM-UP ВЕРИФИКАЦИЯ ЦОС И ФИЗИЧЕСКОГО СЛОЯ \n");
    printf("=========================================================\n\n");

    /* Выполняем тесты строго от простого к сложному */
    //test_step_1_filters();
    //test_step_2_codec();
    //test_step_3_hardware_synthesis();
    //test_step_4_integration();

    //test_step_5_1_limiter_noise_floor();
    //test_step_5_2_dirty_synchronous();

    //test_step_5_hardcore_cable();

    //generate_fsk_preamble_test();
    //test_receiver_fifo_step();

	// Переменная для межсоединения (наш виртуальный "кабель")
	uint8_t tx_wire_bit, rx_wire_bit;
	int total_bits = 0;
	int freq_index = last_tx_frequency, prev_freq_index = 0;

#if 0
    modem_tx_start(2);
	// Крутим цикл, пока передатчик выдает биты по одному
	while (modem_tx_tick(&tx_wire_bit)) {
		total_bits++;

		prev_freq_index = freq_index; /* сохраняем для диффф. демодулятора */
		freq_index = modulate_bit_to_frequency(tx_wire_bit); /* преобразуем бит в индекс частоты (прим.: внутри там статик) */
		printf ("F%d -> F%d : ", prev_freq_index, freq_index);
		rx_wire_bit = demodulate_frequency_to_bit (freq_index, prev_freq_index); /* преобраазуем дельту частоты в бит */

		/* Мгновенно скармливаем этот бит вашему FSM приёмника! */
		process_incoming_bit(rx_wire_bit);
	}
#else
	generate_test_signal_file();
#endif
	printf("--- ТЕСТ ЗАВЕРШЕН. Всего передано бит: %d ---\n", total_bits);


    return 0;
}
