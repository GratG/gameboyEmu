#include "timer.h"

Timer::Timer()
{
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
