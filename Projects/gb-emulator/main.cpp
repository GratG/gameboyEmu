
#include <SDL.h>
#include "emu.h"
#include <string>


int main(int argc, char* argsv[]) {

	Emu emu;
	//std::string file = "C:/Dev/Projects/gb-emulator/ROMS/Tetris.gb";
	std::string file = "C:/Dev/Projects/gb-emulator/ROMS/individual/01.gb";
	emu.emuRun(file);
	return 0;
}