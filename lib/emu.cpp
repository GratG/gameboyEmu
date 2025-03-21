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

void cpuRun(Emu *emu) {
	//cpu_init()
	emu->running = true;
	emu->paused = false;
	emu->ticks = 0;

	while(emu->running) {
		
		if (emu->paused) {
			//delay(10);
			continue;
		}

		if (!emu->bus.cpu->clock()) {
			   std::cout <<"CPU FAILED" <<std::endl;
			
		}
		if(emu->bus.cpu->getCycles() == 0) {
			emu->debug->dbgUpdate();
			emu->debug->dbgPrint();
		}
		emu->ticks++;
	}

	
}

int Emu::emuRun(const std::string& f)
{

	file = f;
	running = true;
	paused = false;
	ticks = 0;
	//Cartridge* cartridge = new Cartridge(file);
	std::shared_ptr<Cartridge> cartridge = std::make_shared<Cartridge>(f);
	std::cout << "Cartridge Initialized..." << std::endl;
	bus.insertCartridge(cartridge);
	debug = new dbg(&bus);


	std::thread t1 = std::thread(cpuRun, this);
	//t1.join();
	//bus.cpu.clock();

	ui->init_window(this);

	while(!die){
		usleep(1000);
		ui->handle_events(); 
	
	}
	return 0;
}
