#pragma once

#include <cstdint>

#include "bus.h"
#include "cartridge.h"

class Emu{

public:

	Emu();
	~Emu();

	//main bus
	Bus bus;

	bool paused;
	bool running;
	int cycles;
	int emuRun(const std::string& file);

private:
	
};

