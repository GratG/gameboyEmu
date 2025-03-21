#pragma once

#include <cstdint>



class Timer
{
public:
	Timer();

	

	void tick();
	void timerWrite(uint16_t addr, uint8_t value);
	uint8_t timerRead(uint16_t addr);
	uint16_t div; //divider register
	uint8_t tima; //timer counter
	uint8_t tma; //timer modulo
	uint8_t tac; //timer controll
};

