#include "ram.h"
#include <iostream>
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
	if ((addr - 0xC000) >= 0x2000) {
		std::cout << "invalid wram addr";
	}
	//std::cout << "reading wram";
	return wram[addr - 0xC000];
}

void Ram::write_wRam(uint16_t addr, uint8_t value)
{
	wram[addr - 0xC000] = value;
	//std::cout << +wram[addr - 0xC000];
}

uint8_t Ram::read_hRam(uint16_t addr)
{
	if ((addr - 0xFF80) >= 0x2000) {
		std::cout << "invalid hram addr";
	}
	//std::cout << std::hex << "reading from hram addr: " << +addr
	//	<< " value: " << +hram[addr - 0xFF80] << std::endl;
	return hram[addr - 0xFF80];
}

void Ram::write_hRam(uint16_t addr, uint8_t value)
{
	//std::cout << std::hex << "writing to hram addr: " << +addr
	//	<< " value: " << +value << std::endl;
	hram[addr - 0xFF80] = value;
}
