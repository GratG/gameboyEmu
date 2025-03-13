#include "emu.h"
#include <iostream>


#include <pthread.h>


 
Emu::Emu()
{
	
}

Emu::~Emu()
{
}



int Emu::emuRun(const std::string& f)
{
	file = f;
	ui renderer(this);
	//renderer.init_window();
	
	//std::thread t1(cpu_run, this);
	//t1.join();
	
	running = true;
	paused = false;
	ticks = 0;
	//Cartridge* cartridge = new Cartridge(file);
	std::shared_ptr<Cartridge> cartridge = std::make_shared<Cartridge>(f);
	std::cout << "Cartridge Initialized..." << std::endl;
	bus.insertCartridge(cartridge);
	debug = new dbg(&bus);

	//for (int i = 0; i < 10000; i++) {
	//	bus.cpu.clock();
	//	debug->dbgUpdate();
	//	debug->dbgPrint();
	//}
	while (running) {

		if (paused) {
			//delay(10);
			continue;
		}

		//std::cout << cycles;
		bus.cpu.clock();
		if(bus.cpu.getCycles() == 0) {
			debug->dbgUpdate();
			debug->dbgPrint();
		}
		//check cpu interrupt
		bus.cpu.handleInterrupts();

		//renderer.handle_events();
		ticks++;

	}

	return 0;
}
