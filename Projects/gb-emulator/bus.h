#pragma once
#include <cstdint>
#include <iostream>
#include <array>

#include "z80.h"
#include "cartridge.h"
#include "ram.h"



class Bus
{
public:
    Bus();
    ~Bus();

    //read write functions
    uint8_t busRead(uint16_t addr, bool bReadOnly = false);
    uint16_t busRead16(uint16_t addr);
    void busWrite(uint16_t addr, uint8_t data);
    void busWrite16(uint16_t addr, uint16_t data);

    void insertCartridge(const std::shared_ptr<Cartridge>& cartridge);

    //Devices on bus
    z80 cpu;
    //cartridge
    std::shared_ptr<Cartridge> cart;
    //Wram and Hram
    Ram memory;
    
};