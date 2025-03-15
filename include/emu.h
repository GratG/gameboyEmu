#pragma once

#include <cstdint>
#include <string>

#include "ui.h"
#include "bus.h"
#include "cartridge.h"
#include "dbg.h"

class Ui;
class Emu{

public:

	Emu();
	~Emu();

	//main bus
	Bus bus;
	Ui *ui;
	dbg *debug;
	bool paused;
	bool running;
	int ticks;
	std::string file;
	int emuRun(const std::string& f);
	bool die;
	//void cpu_run();

private:
	
};

