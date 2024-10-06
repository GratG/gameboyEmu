#include "bus.h"


Bus::Bus() {
    //set ram contents to 0x00
    for (auto& i : ram)
        i = 0x00;

    cpu.ConnectBus(this);
    std::cout << cpu.C;
}

Bus::~Bus() {

}

void Bus::busWrite(uint16_t addr, uint8_t data) {

    if (addr < 0x8000) {
        //write to cartridge
        cart->cartWrite(addr, data);
    }
    else if ((addr >= 0xE000) && (addr < 0xFE00)) {
        //ram echo

        //write also in ram
        //cpuWrite(addr - 0x2000, data);
    }

    else if (addr >= 0x4000 && addr <= 0xFFFF)
        ram[addr] = data;
}

void Bus::insertCartridge(const std::shared_ptr<Cartridge>& cartridge)
{
    this->cart = cartridge;
    std::cout << "Cartridge Connected to bus..." << std::endl;
}

uint8_t Bus::busRead(uint16_t addr, bool bReadOnly) {

    //rom bank
    if (addr < 0x8000) {
        //read cartridge
        return cart->cartRead(addr);
    }
    if (addr >= 0x4000 && addr <= 0x7FFF)
        return ram[addr];

    return 0x00;
}