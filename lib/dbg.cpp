#include "dbg.h"
#include <iostream>

dbg::dbg(Bus * b)
{
	bus = b;
}

void dbg::dbgUpdate()
{
	//std::cout << std::hex <<"0xFF02: " << +bus->busRead(0xFF01) <<std::endl;
	if (bus->busRead(0xFF02) == 0x81) {
		char c = bus->busRead(0xFF01);
		dbg_msg[msg_size++] = c;
		//msg_size++;

		bus->busWrite(0xFF02, 0);
	}
	//std::cout << +bus.busRead(0xFF01);
}

void dbg::dbgPrint()
{
	if (dbg_msg[0]) {
		std::cout << dbg_msg << std::endl;
	}
	//std::cout << "no change..." << std::endl;
}
