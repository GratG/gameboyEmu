
#include <SDL.h>
#include "emu.h"
#include <string>


int main(int argc, char* argsv[]) {

	Emu emu;
	std::string file = "C:/Dev/Projects/gb-emulator/ROMS/Tetris.gb";

	emu.emuRun(file);
	return 0;
}