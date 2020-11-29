#include "SamplePacket.h"
#include "stdint.h"
#include "stm32f7xx_hal.h"
//#include "mbed.h"


const uint32_t MAGIC_VALUES[3] = {0x0, 0xFFFFFFFF, 0x7A7ABFBF};

void SamplePacket::Transmit(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit(huart, (uint8_t*)MAGIC_VALUES, sizeof(MAGIC_VALUES), 10);
	HAL_Delay(10);

	HAL_UART_Transmit(huart, (uint8_t*)&Header, sizeof(Header), 10);
	HAL_Delay(10);

	HAL_UART_Transmit(huart, (uint8_t*)Samples, sizeof(uint16_t) * Header.SampleCount, 10);
	HAL_Delay(10);
}

SamplePacket::SamplePacket(int sampleCount)
{
	Samples = new uint16_t[sampleCount];
	Header.SampleCount = sampleCount;
}
     
SamplePacket::~SamplePacket()
{
	delete[] Samples;  
}

