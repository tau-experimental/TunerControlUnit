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

#define TOTAL_TEST_SAMPLES 5000 // High enough budget to capture all 71 bits * 32 samples

void generate_test_signal_file(void) {
    int16_t wav_buffer[TOTAL_TEST_SAMPLES];
    int recorded_samples = 0;
    int freq_index;
    int16_t sample;

    // Initialize our deterministic FSM packet stack
    modem_tx_start(3); // Start with 2 bytes of preamble

    for (int i = 0; i < TOTAL_TEST_SAMPLES; i++) {
    	sample = (i < 103) ? 511: get_next_tabular_sample_10bit();

#if 0
        if ((freq_index = process_goertzel_sample_10bit (sample)) >= 0) {
        	printf ("Pretzel frequency index: %d\n", freq_index);
        }
#endif
#if 1
        process_adc_sample_stream (sample);
#endif

        // If transmission completes entirely, we fill the remaining budget with silence
        if (tx_state == TX_END && sample == 0) {
            wav_buffer[i] = 0;
        } else {
            wav_buffer[i] = sample;
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
