#pragma once

#include <cstdint>


#include "cartridge.h"

class Emu{

public:

	Emu();
	~Emu();
	bool paused;
	bool running;
	int cycles;
	int emuRun(const std::string& file);

private:
	
};

