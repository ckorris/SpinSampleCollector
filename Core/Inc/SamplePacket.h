#pragma once

#include "stdint.h"
#include "stm32f7xx_hal.h"
//#include "mbed.h"

struct SamplePacketHeader
{
	int DeviceID;
	int SampleID;

	int SamplingDurationUs;
	int AnalongInPins;  //Number of AnalogIn pins being read

	//int SampleCount;
	int32_t SampleCount;
};

class SamplePacket
{
public:
	SamplePacketHeader Header;

	uint16_t* Samples;
        
public:
     
	SamplePacket(uint16_t sampleCount);
     
	~SamplePacket();
     
	void Transmit(UART_HandleTypeDef *huart);
	
private:
	
	void WriteToSerial(UART_HandleTypeDef *huart, int value);
	
	void WriteToSerial(UART_HandleTypeDef *huart, uint16_t value);
};
