#include "receiver.h"
#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static GoertzelState_t g_state_A[3], g_state_B[3];
static int goertzel_sample_count = 0;
static int goertzel_timer = 0;
static int cascade_timer = 0;

static const int16_t GOERTZEL_COEFFS[3] = {
#ifdef DOWNLINK
// Optimized Q12 Coefficients: 5793 (1000Hz), 3135 (1500Hz), 0 (2000Hz)
		5793,
		3135,
		0
#else
			7568,
			6811,
			5793
#endif
};

int8_t process_goertzel_sample_10bit(uint16_t sample_10bit) {
    // 1. Нормализуем 10-битный вход АЦП относительно центра 511
    int32_t x = (int32_t)sample_10bit - 511;

    // 2. Основной цикл фильтра Гёрцеля
    for (int f = 0; f < 3; f++) {
        // Вычисляем обратную связь в Q12 формате: (coeff * v1) / 4096
        int32_t feedback = (GOERTZEL_COEFFS[f] * g_state_A[f].v1) >> 12;

        int32_t v0 = x + feedback - g_state_A[f].v2;
        g_state_A[f].v2 = g_state_A[f].v1;
        g_state_A[f].v1 = v0;
    }

    goertzel_sample_count++;

    // 3. Граница символа: оцениваем энергии
    if (goertzel_sample_count >= SYMBOL_LEN) {
        goertzel_sample_count = 0;

        int32_t max_energy = -1;
        int8_t winner_index = 0;

        for (int f = 0; f < 3; f++) {
            // Вычисление квадрата амплитуды: E = v1^2 + v2^2 - (coeff * v1 * v2) >> 12
            int64_t v1_sq = (int64_t)g_state_A[f].v1 * g_state_A[f].v1;
            int64_t v2_sq = (int64_t)g_state_A[f].v2 * g_state_A[f].v2;
            int64_t cross_term = ((int64_t)GOERTZEL_COEFFS[f] * g_state_A[f].v1 * g_state_A[f].v2) >> 12;

            // Защита от переполнения: сдвигаем 64-битный результат в int32 диапазон.
            // При 10-битном входе сдвиг >> 12 или >> 14 даст отличный контролируемый масштаб энергии.
            int32_t energy = (int32_t)((v1_sq + v2_sq - cross_term) >> 12);

            if (energy > max_energy) {
                max_energy = energy;
                winner_index = f;
            }

            // Обнуляем детекторы для следующего символа кадра
            g_state_A[f].v1 = 0;
            g_state_A[f].v2 = 0;
        }

        // Возвращаем чистый индекс частоты для дифференциального FSM-демодулятора
        return winner_index;
    }

    return -1;
}

typedef enum { /* автомат приёмника */
    RXSTATE_SEARCH_CLK,  // Слепой поиск фронтов преамбулы и калибровка часов
    RXSTATE_WAIT_TRAP,    // Часы залочены, ждем маркер ловушки и SOF
    RXSTATE_RECEIVE_DATA
} RxState_t;

// Структура автомата синхронизации
typedef struct {
    RxState_t state;
    int32_t sample_counter;      // Абсолютный счетчик времени сэмпла
    int32_t last_transition_time;// Время последнего обнаруженного фронта
    int     refined_symbol_len;  // Уточненная длина символа (31, 32, 33...)
    int     sync_confidence;     // Счетчик доверия фронтам
    int8_t  last_raw_freq;       // Опора для фиксации смены частоты
    int     bits_captured;       // Сколько бит вдвинуто в FIFO
    uint16_t rx_fifo;            // 9-битный кольцевой буфер
} RxModemFsm_t;

static RxModemFsm_t fsm = {RXSTATE_SEARCH_CLK, 0, 0, EXPECTED_SYMBOL_LEN, 0, -1, 0, 0};


/*
 * Предполагаемая логика рааботы автомата приёма:
 * 1) сначала ждём энергетику преамбулы: в канале может быть тишина или шум, но пока нет заметной
 * энергии на частотах F0 и F2, продолжаем крутиться в ожидании.
 * 2) появилось подозрение н преамбулу (переключающиеся частоты F0->F2->F0 - начинаем синхронизацию,
 * сбрасывая таймер на каждом моменте, когда энергия на одной частоте оказываешься меньше, чем на другой.
 * У этого компаратора должен быть гистерезис, как я понимаю.
 * 3) принимаемые биты запихиваются в кольцевой буфер на 9 значений (для простоты:
 *  байтовый, т.е. на один бит данных расходуется аж 8 бит памяти). С каждым новым сохранённым битом
 *  проверяем нечётность. Пока преамбула продолжается, чётность будет принудительно некорректна,
 *  т.к. это просто чередующиеся 0 и 1, но последняя посылка преамбулы должна быть сделана так, чтобы получился
 *  байт 0x55, его odd = 1. Как только функция, следящая за кольцевым буфером, обнаруживает корректную нечётность
 *  это считается за момент синхронизации.
 * 4) передаатчик начинает работать с 8+1 посылками, формируя корректный odd бит для каждой.
 * Первым передаётся байт 0x7E (просто так решили), если приёмник видит этот байт - значит, синхронизация
 * это не случайное событие, а действительно данные (слишком маловероятное совпадение, но потом мы ещё проверим CRC8).
 * 5) следующий байт = Length (от 0 до 16 байт).
 * 6) последний байт = CRC8
 * 7) если за этим байтом ещё что-то передётся, то это garble. Но вообще передатчик должен замолчать и дать время приёмнику
 * на обработку пакета.
 */

/**
 * Дифференциальный ЧМ-демодулятор.
 * Принимает текущую победившую частоту (winner) и предыдущую частоту (ref).
 * Возвращает декодированный бит (0 или 1). Если частоты совпали, возвращает 0xFF (ошибка).
 */
uint8_t demodulate_frequency_to_bit(uint8_t winner, uint8_t ref) {
    int8_t delta = winner - ref;
    if (delta < 0) {
        delta += 3;
    }

    if (delta == 1) {
        return 1; // Частота увеличилась (0->1, 1->2, 2->0)
    } else if (delta == 2) {
        return 0; // Частота уменьшилась (0->2, 2->1, 1->0)
    }

    return 0xFF; // Ошибка: частоты соседних посылок совпали (в нашем коде запрещено)
}

typedef enum { /* автомат протокола */
    STATE_SEARCH_PREAMBLE,
    STATE_WAIT_SOF,
	STATE_RECEIVE_LEN,
    STATE_RECEIVE_DATA,
	STATE_RECEIVE_CRC
} FsmState;

// Переменные состояния (на CH32V003 это обычные глобальные/статические регистры)
FsmState current_state = STATE_SEARCH_PREAMBLE;
uint16_t rx_fifo = 0;
int bit_counter = 0;
int payload_length, payload_index;
uint8_t payload[MAX_PAYLOAD];
uint8_t rx_running_crc;

void process_incoming_bit(uint8_t incoming_bit) {
    // Вдвигаем бит в наш 9-битный FIFO-регистр
    rx_fifo = ((rx_fifo << 1) | incoming_bit) & 0x01FF;

    printf ("%u ", incoming_bit);

    switch (current_state) {

        case STATE_SEARCH_PREAMBLE: {
            uint8_t calculated_byte = (rx_fifo >> 1) & 0xFF;
            uint8_t received_parity = rx_fifo & 0x01;

            // Жёсткий двойной критерий: и байт наш, и нечётность сошлась
            if (calculated_byte == PREAMBLE_BYTE && odd(calculated_byte) == received_parity) {
                printf("[FSM] Преамбула 0x55 подтверждена! Переходим в STATE_WAIT_SOF.\n");

                current_state = STATE_WAIT_SOF;
                bit_counter = 0; // Сбрасываем счётчик для накопления ровно 9 бит SOF
            }
            break;
        }

        case STATE_WAIT_SOF: {
            bit_counter++;

            // Ждём, пока вдвинутся ровно 9 бит следующего символа
            if (bit_counter == 9) {
                uint8_t calculated_byte = (rx_fifo >> 1) & 0xFF;
                uint8_t received_parity = rx_fifo & 0x01;
                uint8_t calc_parity = odd(calculated_byte);

                // Проверяем маркер начала кадра SOF (0x7E) и его паритет
                if ((calculated_byte == SOF_BYTE) && (calc_parity == received_parity)) {
                    printf("[FSM] Маркер SOF (0x%02X) успешно принят! Синхронизация железобетонная.\n", calculated_byte);
                    printf("[FSM] Переходим к приёму полей пакета.\n");

                    current_state = STATE_RECEIVE_LEN;
                    bit_counter = 0;
                } else {

                    printf("[FSM] Ошибка! Вместо SOF прилетел мусор (0x%02X, parity = %d вместо %d). Сброс в поиск преамбулы.\n",
                    		calculated_byte, received_parity, calc_parity);

                    current_state = STATE_SEARCH_PREAMBLE;
                }
            }
            break;
        };

        case STATE_RECEIVE_LEN: {
            bit_counter++;

            // Ждём, пока вдвинутся ровно 9 бит следующего символа
            if (bit_counter == 9) {
                uint8_t calculated_byte = (rx_fifo >> 1) & 0xFF;
                uint8_t received_parity = rx_fifo & 0x01;
                uint8_t calc_parity = odd(calculated_byte);

                if (calc_parity == received_parity) {
                	payload_length = calculated_byte;
                	payload_index = 0;
                    printf("[FSM] LEN = %u\n", payload_length);
                    printf("[FSM] Переходим к приёму данных пакета.\n");
                    rx_running_crc = update_crc8 (0, payload_length);

                    current_state = STATE_RECEIVE_DATA;
                    bit_counter = 0;
                } else {
                    printf("[FSM] Сбой чётности в поле LEN (принято %d, ожидалось %d). Сброс в поиск преамбулы.\n",
                    		received_parity, calc_parity);

                    current_state = STATE_SEARCH_PREAMBLE;
                }
            }
       }; break;

        case STATE_RECEIVE_DATA: {
            bit_counter++;
            // Ждём, пока вдвинутся ровно 9 бит следующего символа
            if (bit_counter == 9) {
				uint8_t calculated_byte = (rx_fifo >> 1) & 0xFF;
				uint8_t received_parity = rx_fifo & 0x01;
				uint8_t calc_parity = odd(calculated_byte);

				bit_counter = 0;
				if (calc_parity == received_parity) {
					payload[payload_index] = calculated_byte;
					rx_running_crc = update_crc8 (rx_running_crc, calculated_byte);
					printf("[FSM] Payload[%d] = 0x%02X, ", payload_index, payload[payload_index]);
					if (++payload_index < payload_length) { /* приняты ещё не все байты */
						current_state = STATE_RECEIVE_DATA; /* замыкаемся обратно */
						printf("продолжаем приём данных пакета.\n");
					} else {
						current_state = STATE_RECEIVE_CRC; /* ждём CRC */
						printf("приём данных пакета завершён, переходим к приёму CRC.\n");
	                };
				} else {
					printf("[FSM] Сбой чётности в байте %d (принято %d, ожидалось %d). Сброс в поиск преамбулы.\n",
							payload_index, received_parity, calc_parity);

					current_state = STATE_SEARCH_PREAMBLE;
				};
            }
        }; break;

        case STATE_RECEIVE_CRC: {
        	bit_counter++;
            if (bit_counter == 9) {
				uint8_t calculated_byte = (rx_fifo >> 1) & 0xFF;
				uint8_t received_parity = rx_fifo & 0x01;
				uint8_t calc_parity = odd(calculated_byte);

				bit_counter = 0;
				if (calc_parity == received_parity) {
					printf("Проверка CRC: " );
					if (rx_running_crc == calculated_byte) { /* CRC сошлись! */
						current_state = STATE_SEARCH_PREAMBLE; /* ToDo: обработка данных пакета */
						printf("успешно (0x%02X)! ToDo: обработать пакет\n", rx_running_crc);
					} else {
						current_state = STATE_SEARCH_PREAMBLE; /* Сбой CRC: перезапрашиваем пакет или просто сбрасываем автомат */
						printf("сбой, вычислено 0x%02X, а в пакете указано 0x%02X.\n", rx_running_crc, calculated_byte);
					};
				};
            };
        	//current_state = STATE_SEARCH_PREAMBLE; // ToDo; пока просто сбрасываем автомат
        }; break;

        default: { /* невозможное состояние: сбрасываем автомат */
        	current_state = STATE_SEARCH_PREAMBLE;
        }
    }
}

static int32_t dc_x_prev = 0;
static int32_t dc_y_prev = 0;
/**
 * Обработка одного потокового 10-битного сэмпла из АЦП (0..1023)
 */
void process_adc_sample_stream(uint16_t sample_10bit) {
	GoertzelState_t *active_cascade = 0;
	int in_packet = 0;
	fsm.sample_counter++;
    goertzel_timer++;
    cascade_timer++;

    // =========================================================================
    // HARDWARE INSURANCE: THE ACTIVE DIGITAL DC BLOCKER
    // =========================================================================
    //int32_t x = (int32_t)sample_10bit - 511; /* такой способ компенсации  слишком тупой. А вдруг не 511? */
    int32_t x_raw = (int32_t)sample_10bit;
    // High-pass filter wipes out the massive DC pedestal completely on the fly!
    int32_t x = x_raw - dc_x_prev + dc_y_prev - (dc_y_prev >> 7);
    dc_x_prev = x_raw;
    dc_y_prev = x;

    // 1. Накапливаем текущий сэмпл в фильтрах Гёрцеля
    for (int f = 0; f < 3; f++) {
        int32_t feedback_A = (GOERTZEL_COEFFS[f] * g_state_A[f].v1) >> 12;
        int32_t feedback_B = (GOERTZEL_COEFFS[f] * g_state_B[f].v1) >> 12;
        int32_t v0_A = x + feedback_A - g_state_A[f].v2;
        int32_t v0_B = x + feedback_B - g_state_B[f].v2;
        g_state_A[f].v2 = g_state_A[f].v1;
        g_state_B[f].v2 = g_state_B[f].v1;
        g_state_A[f].v1 = v0_A;
        g_state_B[f].v1 = v0_B;
    }

    // =========================================================================
    // РЕЖИМ 0: СЛЕПОЙ ПОИСК ФРОНТОВ (Синхронизация по меандру преамбулы)
    // =========================================================================
    if (fsm.state == RXSTATE_SEARCH_CLK) {

        // Было:
    	// Каждые 8 сэмплов (четверть символа) мы оцениваем промежуточную энергию.
        // Это не дает фильтру "залипать" на старой частоте и позволяет четко видеть фронты.
    	// Надо сделать: два каскада (g_state_A и g_state_B) со сдвигом фаз на 16 сэмплов
    	//if (cascade_timer % 16 == 0) {
    	if (cascade_timer % 32 == 0) {
            //goertzel_timer = 0; // Сброс локального таймера окна поиска
    		//active_cascade = (cascade_timer % 32 == 0) ? g_state_A : g_state_B;
    		active_cascade = (cascade_timer % 64 == 0) ? g_state_A : g_state_B;

            int32_t max_energy = -1;
            int8_t current_freq = -1;

            for (int f = 0; f < 3; f++) {
                int64_t v1_sq_A = (int64_t)active_cascade[f].v1 * active_cascade[f].v1;
                int64_t v2_sq_A = (int64_t)active_cascade[f].v2 * active_cascade[f].v2;
                int64_t cross_term_A = ((int64_t)GOERTZEL_COEFFS[f] * active_cascade[f].v1 * active_cascade[f].v2) >> 12;
                int32_t energy_A = (int32_t)((v1_sq_A + v2_sq_A - cross_term_A) >> 12);

                if (energy_A > max_energy) {
                    max_energy = energy_A;
                    current_freq = f;
                }

                // ВАЖНО: Сбрасываем накопители! Нам нужна мгновенная реакция
                // на текущие 8 сэмплов, а не история за весь прошлый век.
                active_cascade[f].v1 = 0;
                active_cascade[f].v2 = 0;
            }

            // Вывод для контроля энергий в Audacity / консоли
            // printf("Энергия: %d, Частота candidate: %d\n", max_energy, current_freq);

            // Защита от шума (Squelch) по промежуточной энергии
            if (max_energy < (SIGNAL_THRESHOLD / 4)) {
                fsm.sync_confidence = 0;
                fsm.last_raw_freq = -1;
                return;
            }

            // Проверяем физическое переключение частоты
            if (fsm.last_raw_freq != -1 && current_freq != fsm.last_raw_freq) {

				// ВАЖНОЕ ДОПОЛНЕНИЕ: Игнорируем переключение, если энергия "победителя"
				// ничтожно мала (это просто шум на пустом месте). Порог для 8 сэмплов
				// должен быть пропорционально меньше, например, SIGNAL_THRESHOLD / 8
				if (max_energy > (SIGNAL_THRESHOLD / 8)) {

					int delta_time = fsm.sample_counter - fsm.last_transition_time;
					fsm.last_transition_time = fsm.sample_counter;

					//if (delta_time >= 24 && delta_time <= 40) {
					if (delta_time >= 48 && delta_time <= 80) {
						fsm.sync_confidence++;
						fsm.refined_symbol_len = (fsm.refined_symbol_len + delta_time) / 2;

						printf("[FSM SYNC] НАСТОЯЩИЙ ФРОНТ! Дельта: %d. Сетка: %d. Уверенность: %d/3\n",
							   delta_time, fsm.refined_symbol_len, fsm.sync_confidence);

						if (fsm.sync_confidence >= 3) {
							printf("[FSM LOCK] ===> ЧАСЫ ЗАХВАЧЕНЫ МАТЕМАТИЧЕСКИ! <===\n");
							fsm.state = RXSTATE_WAIT_TRAP;
							in_packet = 1;

							// СТРАХОВКА 1: Прыгаем строго в центр ("на плато") следующего символа.
							goertzel_timer = fsm.refined_symbol_len / 2;
							// СТРАХОВКА 2: Фиксируем текущего победителя как ЖЕСТКУЮ и чистую опору
							// для дифференциального декодера. Предыдущий хаотичный шум стирается!
							fsm.last_raw_freq = current_freq;

							// СТРАХОВКА 3: Полностью обнуляем ВСЕ накопители Гёрцеля обоих каскадов (А и Б).
							// Мы стираем "память" фильтров о переходных процессах преамбулы.
							// Следующие 64 сэмпла будут копиться с абсолютно чистого листа!
							for (int f = 0; f < 3; f++) {
								g_state_A[f].v1 = 0; g_state_A[f].v2 = 0;
								g_state_B[f].v1 = 0; g_state_B[f].v2 = 0;
								g_state_A[f].v1 = 0;   g_state_A[f].v2 = 0; // И основного каскада Фазы 1
							}
						}
					} else {
						// Дельта мусорная — значит, это был ложный чих.
						// Сбрасываем уверенность, только если дельта действительно аномальная
						fsm.sync_confidence = 0;
					}

					// Перезаписываем опору частоты только тогда, когда переключение БЫЛО ВАЛИДНЫМ по энергии
					fsm.last_raw_freq = current_freq;
				}
			} else {
				// Если частота не изменилась, но энергия хорошая,
				// мы просто подтверждаем текущую стабильную опору
				if (max_energy > (SIGNAL_THRESHOLD / 8)) {
					fsm.last_raw_freq = current_freq;
				}
			}
        }
        return;
    }

    // =========================================================================
    // РЕЖИМ 1: РАБОТА ПО УТОЧНЕННЫМ БИТОВЫМ ЧАСАМ (Дискретный прием)
    // =========================================================================
    // Мы вычисляем результат строго по нашей следящей сетке времени (goertzel_timer)
    if (goertzel_timer >= fsm.refined_symbol_len) {
        // РЕ-СИНХРОНИЗАЦИЯ (DPLL подстройка):
        // Если реальный физический фронт сместился, мы можем скорректировать goertzel_timer на +/- 1 сэмпл.
        // Для этого проверяем смену частоты. Если она произошла чуть раньше/позже — подтягиваем сетку.

        int32_t max_energy = -1;
        int8_t winner_freq = 0;
        int32_t total_energy = 0; // Сумма энергий всех частот для оценки шума

        for (int f = 0; f < 3; f++) {
            int64_t v1_sq = (int64_t)g_state_A[f].v1 * g_state_A[f].v1;
            int64_t v2_sq = (int64_t)g_state_A[f].v2 * g_state_A[f].v2;
            int64_t cross_term = ((int64_t)GOERTZEL_COEFFS[f] * g_state_A[f].v1 * g_state_A[f].v2) >> 12;

            int32_t energy = (int32_t)((v1_sq + v2_sq - cross_term) >> 14);
            if (energy < 0) energy = 0; // Защита от знакового переполнения

            total_energy += energy;

            if (energy > max_energy) {
                max_energy = energy;
                winner_freq = f;
            }
            // Чистим накопители для следующего такта
            g_state_A[f].v1 = 0;
            g_state_A[f].v2 = 0;
        }

        // ОТНОСИТЕЛЬНЫЙ SQUELCH (SNR-ДЕТЕКТОР):
        // Вычисляем среднюю энергию шума на "холостых" частотах.
        // Энергия шума = (Сумма всех - Максимальная) / 2
        int32_t noise_floor = (total_energy - max_energy) / 2;

        //if ((max_energy * 2) < (noise_floor * 3) && in_packet) {
        // Объединенная проверка: если энергии критически мало ИЛИ сигнал утонул в шуме
        printf("Total Energy: %d\n", total_energy);
        if ((total_energy < ABSOLUTE_MIN_ENERGY || (max_energy * 2) < (noise_floor * 3))) {

            printf("[FSM DROP] Канал пуст или забит шумом; возврат в поиск.\n");
            fsm.state = RXSTATE_SEARCH_CLK;
            current_state = STATE_SEARCH_PREAMBLE;
            fsm.sync_confidence = 0;
            goertzel_timer = 0;
            in_packet = 0;
            return;
        }

        // Вычисляем бит через наш треугольник частот
        uint8_t current_bit = demodulate_frequency_to_bit(winner_freq, fsm.last_raw_freq);
        fsm.last_raw_freq = winner_freq;

        if (current_bit != 0xFF) {
            // Вдвигаем бит в наш 9-битный FIFO
            //fsm.rx_fifo = ((fsm.rx_fifo << 1) | current_bit) & 0x01FF;
            //printf ("Current fifo: %d\n", fsm.rx_fifo );
        	process_incoming_bit(current_bit); /* протокольный FSM */
        }

        goertzel_timer = 0; // Сброс таймера сетки до следующего символа
    }
}
