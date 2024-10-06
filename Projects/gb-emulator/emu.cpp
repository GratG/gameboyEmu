#include "emu.h"
#include <iostream>


 
Emu::Emu()
{
}

Emu::~Emu()
{
}

int Emu::emuRun(const std::string& file)
{

	running = true;
	paused = false;
	cycles = 0;
	Cartridge* cart = new Cartridge(file);
	while (running) {
		if (paused) {
			//delay(10);
			continue;
		}
		//std::cout << cycles;
		cycles++;
	}


	return 0;
}
