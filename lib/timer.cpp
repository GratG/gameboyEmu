#include "timer.h"

#include <iostream>
Timer::Timer()
{
	std::cout << "initializing timer" << std::endl;
	div = 0xAC00;
}

void Timer::tick()
{
	uint16_t prevDiv = div;
	div++;
	bool timerUpdate;

	switch (tac & 0b11) {
	case 0b00:
		timerUpdate = (prevDiv & (1 << 9)) && (!(div & (1 << 9)));
		break;
	case 0b01:
		timerUpdate = (prevDiv & (1 << 3)) && (!(div & (1 << 3)));
		break;
	case 0b10:
		timerUpdate = (prevDiv & (1 << 5)) && (!(div & (1 << 5)));
		break;
	case 0b11:
		timerUpdate = (prevDiv & (1 << 7)) && (!(div & (1 << 7)));
		break;
	}

	if (timerUpdate && tac & (1 << 2)) {
		tima++;
		if (tima == 0xFF) {
			tima = tma;
			//request interrupt
		}

	}
}

void Timer::timerWrite(uint16_t addr, uint8_t value){
	switch(addr) {
        case 0xFF04:
			div = 0;
            break;

        case 0xFF05:
			tima = value;
            break;

        case 0xFF06:
			tma = value;
            break;

        case 0xFF07:
            tac = value;
            break;
    }
}

uint8_t Timer::timerRead(uint16_t addr) {
    switch(addr) {
        case 0xFF04:
            return div >> 8;
        case 0xFF05:
            return tima;
        case 0xFF06:
            return tma;
        case 0xFF07:
            return tac;
    }
	return 0;
}
