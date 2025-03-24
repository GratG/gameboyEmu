#include "io.h"
#include <iostream>


io::io(Timer *t)
{
	timer = t;
	std::cout << "initializing io" << std::endl;
}

io::~io()
{
}

void io::writeIO(uint16_t addr, uint8_t value)
{
	//std::cout << "IO WRITE " << serialData << std::endl;
	if (addr == 0xFF01) {
		serialData[0] = value;
		return;
	}
	if (addr == 0xFF02) {
		serialData[1] = value;
		return;
	}
	if(addr == 0xFF04){
		timer->timerWrite(addr, value);
		return;
	}
	if(addr == 0xFF05){
		timer->timerWrite(addr, value);
		return;
	}
	if(addr == 0xFF06){
		timer->timerWrite(addr, value);
		return;
	}
	if(addr == 0xFF07){
		timer->timerWrite(addr, value);
		return;
	}


}

uint8_t io::readIO(uint16_t addr)
{
	//std::cout << "IO READ " << +serialData[1] << std::endl;
	if (addr == 0xFF01) {
		return serialData[0];
	}
	if (addr == 0xFF02) {
		return serialData[1];
	}
	if(addr == 0xFF04){
		return timer->timerRead(addr);
	}
	if(addr == 0xFF05){
		return timer->timerRead(addr);
	}
	if(addr == 0xFF06){
		return timer->timerRead(addr);
	}
	if(addr == 0xFF07){
		return timer->timerRead(addr);
	}
	if (addr == 0xFF44) {
		return 0x0090;
	}
	return 0;
}
