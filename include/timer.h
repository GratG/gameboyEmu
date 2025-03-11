#pragma once

#include <cstdint>

#include "emu.h"


class Timer
{
public:
	Timer();

	Bus* bus;

	void tick();

	uint16_t div; //divider register
	uint8_t tima; //timer counter
	uint8_t tma; //timer modulo
	uint8_t tac; //timer controll
};

