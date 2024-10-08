#include "ram.h"

Ram::Ram()
{
	for (auto i : hram) {
		hram[i] = 0x00;
	}
	for (auto i : wram) {
		wram[i] = 0x00;
	}
}

Ram::~Ram()
{
}

uint8_t Ram::read_wRam(uint16_t addr)
{
	return wram[addr - 0xC000];
}

void Ram::write_wRam(uint16_t addr, uint8_t value)
{
	wram[addr - 0xC000] = value;
}

uint8_t Ram::read_hRam(uint16_t addr)
{
	return hram[addr - 0xFF80];
}

void Ram::write_hRam(uint16_t addr, uint8_t value)
{
	hram[addr - 0xFF80] = value;
}
