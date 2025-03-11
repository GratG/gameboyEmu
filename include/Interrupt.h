#pragma once

#include <iostream>
#include "z80.h"

class z80;

enum InterruptFlags {
	INT_VBLANK = (1 << 0),
	INT_LCD = (1 << 1),
	INT_TIMER = (1 << 2),
	INT_SERIAL = (1 << 3),
	INT_JOYPAD = (1 << 4)
};


class Interrupt
{
public:
	Interrupt(z80* c);
	~Interrupt();

	z80* cpu;

	void intHandle(uint16_t addr);
	bool intCheck(uint16_t addr, InterruptFlags f);
	void cpuRequestInterrupt(InterruptFlags f);
	void cpuHandleInterrupts();


private:
	
};

