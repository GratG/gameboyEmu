#pragma once

#include <cstdint>
class Ram
{
public:
	Ram();
	~Ram();
	uint8_t read_wRam(uint16_t addr);
	void write_wRam(uint16_t addr, uint8_t value);
	uint8_t read_hRam(uint16_t addr);
	void write_hRam(uint16_t addr, uint8_t value);

private:
	uint8_t wram[0x2000]; //working ram
	uint8_t hram[0x80]; //high ram
};

