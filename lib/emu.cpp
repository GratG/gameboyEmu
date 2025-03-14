#include "emu.h"
#include <iostream>


//threading support for linux
#include <thread>
#include <unistd.h>


 
Emu::Emu()
{
	
}

Emu::~Emu()
{
}

void cpu_run(Emu *emu) {
	//cpu_init()
	emu->running = true;
	emu->paused = false;
	emu->ticks = 0;

	while(emu->running) {
		if (emu->paused) {
			//delay(10);
			continue;
		}

		if (!emu->bus.cpu.clock()) {
			   std::cout <<"CPU FAILED" <<std::endl;
			
		}
		if(emu->bus.cpu.getCycles() == 0) {
			//emu->debug->dbgUpdate();
			//emu->debug->dbgPrint();
		}
		emu->ticks++;
	}

	
}

int Emu::emuRun(const std::string& f)
{

	file = f;
	ui renderer(this);
	//renderer.init_window();
	

	
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

	//pthread_t t1;
//
	//if(pthread_create(&t1, NULL, &cpu_run,NULL)){
	//	std::cout << "FAILED TO START THREAD";
	//	return 0;
	//}
	std::thread t1 = std::thread(cpu_run, this);
	t1.join();
	//bus.cpu.clock();


	while (running) {

		if (paused) {
			//delay(10);
			continue;
		}

		//renderer.handle_events();
		ticks++;

	}

	return 0;
}
