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
	//Cartridge* cartridge = new Cartridge(file);
	std::shared_ptr<Cartridge> cartridge = std::make_shared<Cartridge>(file);
	std::cout << "Cartridge Initialized..." << std::endl;
	bus.insertCartridge(cartridge);
	
	for (int i = 0; i < 1000; i++) {
		bus.cpu.clock();
	}

	
	while (running) {
		if (paused) {
			//delay(10);
			continue;
		}
		//std::cout << cycles;
		bus.cpu.clock();
		//cycles++;
		
	}


	return 0;
}
