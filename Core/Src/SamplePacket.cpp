#include "SamplePacket.h"
#include "stdint.h"
#include "stm32f7xx_hal.h"
#include <algorithm>
//#include "mbed.h"


const uint32_t MAGIC_VALUES[3] = {0x0, 0xFFFFFFFF, 0x7A7ABFBF};

const int TRANSMIT_COUNT = INT16_MAX;
//const int TRANSMIT_COUNT = 128;

void SamplePacket::Transmit(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit(huart, (uint8_t*)MAGIC_VALUES, sizeof(MAGIC_VALUES), HAL_MAX_DELAY);
	//HAL_Delay(10);

	//HAL_UART_Transmit(huart, (uint8_t*)&Header, sizeof(Header), HAL_MAX_DELAY);
	HAL_UART_Transmit(huart, (uint8_t*)&Header, sizeof(Header), HAL_MAX_DELAY);
	//HAL_Delay(50);

	//HAL_UART_Transmit(huart, (uint8_t*)Samples, sizeof(uint16_t) * Header.SampleCount, HAL_MAX_DELAY);

	uint8_t* start = (uint8_t*)Samples;
	//uint8_t* end = start + (sizeof(uint16_t) * Header.SampleCount);


	//for(int i = 0; i < Header.SampleCount; i += INT16_MAX)
	for(int i = 0; i < Header.SampleCount; i += TRANSMIT_COUNT)
	{
		int thisCycleCount = std::min((long)TRANSMIT_COUNT, Header.SampleCount - i);
		HAL_UART_Transmit(huart, (uint8_t*)start, sizeof(uint16_t) * thisCycleCount, HAL_MAX_DELAY);
		start += sizeof(uint16_t) * thisCycleCount;
	}

	/*
	while(start <= end)
	{
		uint8_t diff = end - start;
		HAL_UART_Transmit(huart, (uint8_t*)start, sizeof(uint16_t) * Header.SampleCount, HAL_MAX_DELAY);
		start++;
	}
	*/

	//HAL_Delay(10);
}

SamplePacket::SamplePacket(uint16_t sampleCount)
{
	Samples = new uint16_t[sampleCount];
	Header.SampleCount = sampleCount;
}
     
SamplePacket::~SamplePacket()
{
	delete[] Samples;  
}

