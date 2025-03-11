
#include <SDL3/SDL.h>
#include "emu.h"
#include <string>
#include <iostream>


int main(int argc, char* argsv[]) {

	Emu emu;
	//std::string file = "C:/Dev/Projects/gb-emulator/ROMS/cpu_instrs.gb";
	std::string file = "../roms/01-special.gb";
	//std::cout << file;
	emu.emuRun(file);
	return 0;
}