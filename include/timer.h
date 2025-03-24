#pragma once

#include <cstdint>
#include "z80.h"

class z80;
class Timer
{
public:
	Timer(z80 *c);

	
	z80 *cpu;
	void tick();
	void timerWrite(uint16_t addr, uint8_t value);
	uint8_t timerRead(uint16_t addr);

private:
	uint16_t div; //divider register
	uint8_t tima; //timer counter
	uint8_t tma; //timer modulo
	uint8_t tac; //timer controll
};

