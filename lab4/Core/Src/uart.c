#include <stdio.h>
#include "uart.h"
#include "math.h"
#include "stm32f4xx_hal_dac.h"
#include "stm32f4xx_hal_adc.h"

#define PI 3.14159
#define MAX_VAL 4095
#define PARTITION 200

extern  UART_HandleTypeDef huart2;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;

uint8_t rx_buff[MAX_PACKET_LENGTH]; // receive
uint8_t tx_buff[MAX_PACKET_LENGTH]; // transmit
uint8_t rx_length = 0;
uint8_t tx_length = 0;

uint8_t cur_freq = 10;
float prev_freq = 10;
float cur_amplitude = 3000.0f;
uint8_t counter = 0;
float sin_arr[PARTITION];
uint32_t prev_prescaler = 0;
uint32_t value = 0;

void nulify_buffs() {

	for(size_t i = 0; i < rx_length; i++) {
		rx_buff[i] = 0;
	}
	for(size_t i = 0; i < tx_length; i++) {
		tx_buff[i] = 0;
	}

	rx_length = 0;
	tx_length = 0;
}

void parse_packet(struct header packet) {
	switch(packet.request_id) {
	case Ping:
		break;
	case Freq:
		{
			uint8_t val = 0;
			for(size_t i = 0; i < packet.data_size; i++) {
				val += packet.data[i]*pow(10, packet.data_size-i-1);
			}

			prev_freq = cur_freq;
			cur_freq = val;

			if(!prev_prescaler)
				prev_prescaler = htim4.Init.Prescaler;

			__HAL_TIM_SET_PRESCALER(&htim4, prev_prescaler*prev_freq/cur_freq);
			prev_prescaler *= prev_freq/cur_freq;
			prev_freq = cur_freq;
			break;
		}
	case Amp:
		{
			uint8_t val = 0;
			for(size_t i = 0; i < packet.data_size; i++) {
				val += packet.data[i]*pow(10, packet.data_size-i-1);
			}

			cur_amplitude = val*1000;
			if((cur_amplitude < 0) || (cur_amplitude > MAX_VAL))
				cur_amplitude = MAX_VAL;
			break;
		}
	}

	for(size_t i = 0; i <= rx_length+1; i++) {
		tx_buff[i] = rx_buff[i];
	}

	tx_length = rx_length+1;
	tx_buff[tx_length] = '\n';
	tx_length++;
	tx_buff[tx_length] = '\r';
	tx_length++;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { // срабатывает в момент окончания передачи данных
	struct header packet;

	if(huart == &huart2) {

		if(rx_buff[rx_length] == START_BIT) {
			rx_length += 1;

			packet.request_id = rx_buff[rx_length];
			packet.data_size = rx_buff[rx_length+1] - NUM_ASCII_OFFSET;

			rx_length += 2;

			if(packet.data_size == 0) {
				parse_packet(packet);
			} else {
				for(size_t i = 0; i < packet.data_size; i++) {
					packet.data[i] = rx_buff[rx_length+i] - NUM_ASCII_OFFSET;
				}

				rx_length += (packet.data_size);
				parse_packet(packet);
			}
			if(rx_buff[rx_length] == STOP_BIT) {
				HAL_UART_Transmit(&huart2, tx_buff, tx_length, 20);
			}
		}
	}

	nulify_buffs();
	HAL_UART_Receive_IT(&huart2, rx_buff, PACKET_LENGTH);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM4) {
		counter += 1;

		if(counter >= PARTITION)
			counter = 0;

		//HAL_UART_Transmit(&huart2, tx_buff, tx_length, 20);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, cur_amplitude/2*(sinf(2*PI*counter/PARTITION)+1));
	}
	else if(htim->Instance == TIM3) {
		HAL_ADC_Start_DMA(&hadc1, &value, 1);
		HAL_UART_Transmit(&huart2, &value, 4, 20);
		//HAL_UART_Transmit(&huart2, 0xFF, 1, 5);
	}
}
