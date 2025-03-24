#pragma once
#include <cstdint>
#include "timer.h"

class Timer;
class io
{
public: 
	io(Timer *t);
	~io();
	

	Timer *timer;
	void writeIO(uint16_t addr, uint8_t value);
	uint8_t readIO(uint16_t addr);
private:
	char serialData[2];
};

