#include "z80.h"
#include "bus.h"
#include <iostream>
#include <format>

z80::z80() {
    registerAF.hi = 0x01;
    registerAF.lo = 0xB0;
    registerBC.hi = 0x00;
    registerBC.lo = 0x13;
    registerDE.hi = 0x00;
    registerDE.lo = 0xD8;
    registerHL.hi = 0x01;
    registerHL.lo = 0x4D;


}

z80::~z80() {

}

void z80::ConnectBus(Bus* n) {
    bus = n;
}

void z80::clock()
{
    
    if (!halted) {
        if (cycles == 0) {
            opcode = read(pc);
            std::cout << std::hex
                
                
                << " A: " << +registerAF.hi
                << " F: " << +registerAF.lo
                << " B: " << +registerBC.hi
                << " C: " << +registerBC.lo
                << " D: " << +registerDE.hi
                << " E: " << +registerDE.lo
                << " H: " << +registerHL.hi
                << " L: " << +registerHL.lo
                << " PC: " << pc
                << "Instruction : 0x" << +opcode
                << std::endl;
            pc++;
            cycles = executeOP(opcode);
        }
        cycles--;
    } 
    else { //is halted

    }
    //std::cout << +cycles << std::endl;
    //clock_cycles--;
    
    
}

uint8_t z80::getFlag(FLAGSz80 f)
{
    //extract f from lo reg, if f > 0 return 1 else return 0
    return ((registerAF.lo & f) > 0) ? 1 : 0;
}

bool z80::checkBit(uint8_t reg, int pos)
{
    uint8_t mask = 1 << pos;
    return(reg & mask) ? true : false;
}

bool z80::checkBit(uint16_t reg, int pos)
{
    uint16_t mask = 1 << pos;
    return(reg & mask) ? true : false;
}

void z80::setFlag(FLAGSz80 f, bool v) {
    if (v)
        registerAF.lo |= f; //set the f flag (1)
    else
        registerAF.lo &= ~f; //clear flag (0)
}


uint8_t z80::read(uint16_t a) {
    return bus->busRead(a, false);

}

uint16_t z80::read16(uint16_t a)
{
    return bus->busRead16(a);
}


uint16_t z80::read16()
{
    uint16_t v = read(pc + 1);
    v = v << 8;
    v += read(pc);
    return v;
}

void z80::pushSortToStack(uint16_t value)
{
    //obtain hi and lo bytes
    uint8_t hi = value >> 8;
    uint8_t lo = value & 0xFF;
    //shift stack (down) and write to that position
    sp--;
    write(sp, hi);
    sp--;
    write(sp, lo);
}

uint16_t z80::popShortFromStack()
{
    //hi byte
    nn = read(sp + 1) << 8;
    //lo byte
    nn |= read(sp);
    sp++; sp++;
    
    return nn;
}

void z80::write(uint16_t a, uint8_t d) {
    bus->busWrite(a, d);
}

void z80::write16(uint16_t a, uint16_t d)
{
    bus->busWrite16(a, d);
}

uint8_t z80::resetBit(uint8_t b, uint8_t x)
{
    b &= x;
    return b;
}

uint8_t z80::setBit(uint8_t b, uint8_t x)
{
    b |= x;
    return b;
}

void z80::BIT8_LOAD(uint8_t& reg) {
    //load immediate value of reg
    uint8_t n = read(pc);
    pc++;
    reg = n;
}
//void z80::BIT8_LOAD(uint16_t& reg) {
//    uint8_t n = read(reg);
//    //pc++;
//    reg = n;
//}
void z80::BIT8_LOAD(uint8_t& reg1, uint8_t& reg2) {
    reg1 = reg2;
    //pc++;
}



void z80::BIT8_LOAD_MEM(uint8_t& reg1, uint16_t addr)
{
    reg1 = read(addr);
}

//void z80::BIT8_LOAD(uint16_t& reg1, uint8_t& reg2) {
//    reg1 = reg2;
//    pc++;
//}
//
//void z80::BIT8_LOAD(uint16_t& reg1, uint16_t& reg2) {
//    reg1 = reg2;
//    pc++;
//}
void z80::BIT16_LOAD(uint16_t& reg)
{
    nn = read(pc + 1);
    nn <<= 8;
    nn |= read(pc);
    pc++; pc++;
    reg = nn;
}
void z80::BIT16_DEC(uint16_t& reg)
{
    reg--;
}
void z80::BIT16_INC(uint16_t& reg)
{
    reg++;
}
void z80::PUSH16(uint16_t b)
{
    //extract 8bit pairs from 16bit reg
    uint8_t hi = b >> 8;
    uint8_t lo = b & 0xFF;
    write(sp, hi);
    write(sp, lo);
    sp--;
    sp--;
}
void z80::POP16(uint16_t reg)
{
    nn = read(sp);
    write(reg, nn);
    sp--;
    sp--;
}
void z80::ADD_8BIT(uint8_t& reg1, uint8_t reg2, bool c)
{
    //keep the original value
    uint8_t temp = reg1;
    uint8_t addVal = 0;

    addVal = reg2;
    if (c) {
        if (getFlag(C) > 0)
            addVal++;
    }

    reg1 += addVal;

    //reset all flags to 0
    registerAF.lo = 0;

    //if sum == 0, set Z flag
    if (reg1 == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

    //half carry test
    uint16_t htest = (temp & 0xF); //set original 0xF bit of reg1
    htest += (addVal & 0xF); //add original 0xF bit of reg2 to reg1

    //test if carry occured
    if (htest > 0xF)
        registerAF.lo = setBit(registerAF.lo, H);
    if ((temp + addVal) > 0xFF)
        registerAF.lo = setBit(registerAF.lo, C);
}
void z80::ADD_16BIT(uint16_t& reg1, uint16_t reg2)
{
    //keep the original value
    uint16_t temp = reg1;
    uint16_t addVal = reg2;

    reg1 += addVal;

    registerAF.lo = resetBit(registerAF.lo, N);


    //half carry test
    if (((temp & 0xFF00) & 0xF) + ((addVal >> 8) & 0xF))
        registerAF.lo = setBit(registerAF.lo, H);
    else
        registerAF.lo = resetBit(registerAF.lo, H);

    //carry test
    if ((temp + addVal) > 0xFFFF)
        registerAF.lo = setBit(registerAF.lo, C);
    else
        registerAF.lo = resetBit(registerAF.lo, C);




}
void z80::SUB_8BIT(uint8_t& reg1, uint8_t reg2, bool c)
{
    uint8_t temp = reg1;

    uint8_t subVal = reg2;

    if (c) {
        if (getFlag(C) > 0)
            subVal++;
    }
    reg1 -= reg2;

    //reset flags
    registerAF.lo = 0;
    //set 0 flag
    if (reg1 == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    //set substitution flag
    registerAF.lo = setBit(registerAF.lo, N);
    //no borrow
    if (temp < subVal)
        registerAF.lo = setBit(registerAF.lo, C);

    int8_t htest = (temp & 0xF);
    htest -= (subVal & 0xF);

    if (htest < 0)
        registerAF.lo = setBit(registerAF.lo, H);


}
void z80::AND_8BIT(uint8_t& reg1, uint8_t reg2)
{
    uint8_t temp = reg2;

    reg1 &= temp;

    //reset flags
    registerAF.lo = 0;

    if (reg1 == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    //set H flag
    registerAF.lo = setBit(registerAF.lo, H);


}
void z80::OR_8BIT(uint8_t& reg1, uint8_t reg2)
{
    uint8_t temp = reg2;

    reg1 |= temp;

    //reset flags
    registerAF.lo = 0;

    if (reg1 == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

}
void z80::XOR_8BIT(uint8_t& reg1, uint8_t reg2)
{
    uint8_t temp = reg2;

    reg1 ^= temp;

    //reset flags
    registerAF.lo = 0;

    if (reg1 == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}
void z80::CP_8BIT(uint8_t reg1, uint8_t reg2)
{

    uint8_t temp = reg1;

    uint8_t subVal = reg2;

    reg1 -= reg2;

    //reset flags
    registerAF.lo = 0;
    //set 0 flag
    if (reg1 == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    //set substitution flag
    registerAF.lo = setBit(registerAF.lo, N);
    //no borrow
    if (temp < subVal)
        registerAF.lo = setBit(registerAF.lo, C);

    int8_t htest = (temp & 0xF);
    htest -= (subVal & 0xF);

    if (htest < 0)
        registerAF.lo = setBit(registerAF.lo, H);

}
void z80::INC_8BIT(uint8_t& reg)
{
    uint8_t temp = reg;
    reg++;

    //no flag reset
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    else
        registerAF.lo = resetBit(registerAF.lo, Z);

    registerAF.lo = resetBit(registerAF.lo, N);

    if ((temp & 0xF) == 0xF)
        registerAF.lo = setBit(registerAF.lo, H);
    else
        registerAF.lo = resetBit(registerAF.lo, H);

}
void z80::INC_8BIT_MEM(uint8_t addr)
{
    uint8_t temp = read(addr);
    write(addr, (temp + 1)); //inc value
    uint8_t now = temp + 1;

    //no flag reset
    if (now == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    else
        registerAF.lo = resetBit(registerAF.lo, Z);

    registerAF.lo = resetBit(registerAF.lo, N);

    if ((temp & 0xF) == 0xF)
        registerAF.lo = setBit(registerAF.lo, H);
    else
        registerAF.lo = resetBit(registerAF.lo, H);

}
void z80::INC_16BIT(uint16_t& reg)
{
    uint16_t temp = reg;
    reg++;

    //no flag reset
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    else
        registerAF.lo = resetBit(registerAF.lo, Z);

    registerAF.lo = resetBit(registerAF.lo, N);

    if ((temp & 0xF) == 0xF)
        registerAF.lo = setBit(registerAF.lo, H);
    else
        registerAF.lo = resetBit(registerAF.lo, H);

}
void z80::DEC_8BIT(uint8_t& reg)
{
    uint8_t temp = reg;
    reg--;

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    else
        registerAF.lo = resetBit(registerAF.lo, Z);

    //set substitution flag
    registerAF.lo = resetBit(registerAF.lo, N);

    if ((temp & 0xF) == 0xF)
        registerAF.lo = setBit(registerAF.lo, H);
    else
        registerAF.lo = resetBit(registerAF.lo, H);
}

void z80::DEC_8BIT_MEM(uint8_t addr)
{
    uint8_t temp = read(addr);
    write(addr, (temp - 1)); //inc value
    uint8_t now = temp - 1;

    if (now == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    else
        registerAF.lo = resetBit(registerAF.lo, Z);

    //set substitution flag
    registerAF.lo = resetBit(registerAF.lo, N);

    if ((temp & 0xF) == 0xF)
        registerAF.lo = setBit(registerAF.lo, H);
    else
        registerAF.lo = resetBit(registerAF.lo, H);
}



void z80::DEC_16BIT(uint16_t& reg)
{
    uint16_t temp = reg;
    reg--;

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    else
        registerAF.lo = resetBit(registerAF.lo, Z);

    //set substitution flag
    registerAF.lo = resetBit(registerAF.lo, N);

    if ((temp & 0xF) == 0xF)
        registerAF.lo = setBit(registerAF.lo, H);
    else
        registerAF.lo = resetBit(registerAF.lo, H);
}

void z80::SWAP_NIBBLES(uint8_t& reg)
{
    uint8_t temp = reg;
    uint8_t reglo = reg & 0x0F;
    uint8_t reghi = reg & 0xF0;
    reg = (reglo << 4) | (reghi >> 4);

    registerAF.lo = 0;
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

}

void z80::SWAP_NIB_MEM(uint16_t addr)
{
    uint8_t temp = read(addr);
    uint8_t reglo = temp & 0x0F;
    uint8_t reghi = temp & 0xF0;
    temp = (reglo << 4) | (reghi >> 4);

    write(addr, temp);

    registerAF.lo = 0;
    if (temp == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

}



void z80::DAA()
{//obtained from another emulator's code

    if (getFlag(N))
    {
        if ((registerAF.hi & 0x0F) > 0x09 || registerAF.lo & 0x20)
        {
            registerAF.hi -= 0x06; //Half borrow: (0-1) = (0xF-0x6) = 9
            if ((registerAF.hi & 0xF0) == 0xF0) registerAF.lo |= 0x10; else registerAF.lo &= ~0x10;
        }

        if ((registerAF.hi & 0xF0) > 0x90 || registerAF.lo & 0x10) registerAF.hi -= 0x60;
    }
    else
    {
        if ((registerAF.hi & 0x0F) > 9 || registerAF.lo & 0x20)
        {
            registerAF.hi += 0x06; //Half carry: (9+1) = (0xA+0x6) = 10
            if ((registerAF.hi & 0xF0) == 0) registerAF.lo |= 0x10; else registerAF.lo &= ~0x10;
        }

        if ((registerAF.hi & 0xF0) > 0x90 || registerAF.lo & 0x10) registerAF.hi += 0x60;
    }

    if (registerAF.hi == 0) registerAF.lo |= 0x80; else registerAF.lo &= ~0x80;
}

void z80::CPL()
{
    registerAF.hi = ~registerAF.hi;
    registerAF.lo = setBit(registerAF.hi, N);
    registerAF.lo = setBit(registerAF.lo, H);
}

void z80::CCF()
{
    if (getFlag(C))
        registerAF.lo = setBit(registerAF.lo, C);
    else
        registerAF.lo = setBit(registerAF.lo, C);

    registerAF.lo = resetBit(registerAF.lo, N);
    registerAF.lo = resetBit(registerAF.lo, H);

}

void z80::SCF()
{
    registerAF.lo = setBit(registerAF.lo, C);
    registerAF.lo = resetBit(registerAF.lo, H);
    registerAF.lo = resetBit(registerAF.lo, N);
}

void z80::JP()
{
    nn = read(pc + 1);
    nn <<= 8;
    nn |= read(pc);
    pc++; pc++;

    pc = nn;

}

void z80::JP(int flag, bool cond)
{

    nn = read(pc + 1);
    nn <<= 8;
    nn |= read(pc);
    pc++; pc++;

    if (checkBit(registerAF.lo, flag) == cond) {
        pc = nn;
    }       
}

void z80::JR()
{
    int8_t sn = (int8_t)read(pc);
    pc++;
    pc += sn;
}

void z80::JR(int flag, bool cond)
{
    int8_t sn = (int8_t)read(pc);
    pc++;

    if (checkBit(registerAF.lo, flag) == cond)
        pc += sn;
}

void z80::CALL()
{
    nn = read(pc + 1);
    nn <<= 8;
    nn |= read(pc);
    pc++; pc++;

    pushSortToStack(pc);
    pc = nn;

}

void z80::CALL(int flag, bool cond)
{
    nn = read(pc + 1);
    nn <<= 8;
    nn |= read(pc);
    pc++; pc++;
    if (checkBit(registerAF.lo, flag) == cond) {
        pushSortToStack(pc);
        pc = nn;
    }

}

void z80::RET()
{
    pc = popShortFromStack();
}

void z80::RET(int flag, bool cond)
{
    if (checkBit(registerAF.lo, flag) == cond) {
        pc = popShortFromStack();
    }
}

void z80::RST(uint8_t byte)
{
    pushSortToStack(pc);
    pc = byte;
}

void z80::RLC(uint8_t& reg)
{

    bool isSet = checkBit(reg, 7);

    registerAF.lo = 0;
    reg <<= 1;

    if (isSet) {
        registerAF.lo = setBit(registerAF.lo, C);
        reg = setBit(reg, 0);
    }

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}

void z80::RLCMem(uint16_t addr)
{
    uint8_t reg = read(addr);

    bool isSet = checkBit(reg, 7);

    registerAF.lo = 0;
    reg <<= 1;

    if (isSet) {
        registerAF.lo = setBit(registerAF.lo, C);
        reg = setBit(reg, 0);
    }

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    write(addr, reg);
}


void z80::RL(uint8_t& reg)
{
    bool isCarrySet = getFlag(C);
    bool isSet = checkBit(reg, 7);

    registerAF.lo = 0;
    reg <<= 1;

    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);

    if (isCarrySet)
        reg = setBit(reg, 0);

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}

void z80::RLMem(uint16_t addr)
{
    uint8_t reg = read(addr);
    bool isCarrySet = getFlag(C);
    bool isSet = checkBit(reg, 7);

    registerAF.lo = 0;
    reg <<= 1;

    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);

    if (isCarrySet)
        reg = setBit(reg, 0);

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

    write(addr, reg);
}



void z80::RRC(uint8_t& reg)
{
    bool isSet = checkBit(reg, 0);

    registerAF.lo = 0;
    reg >>= 1;

    if (isSet) {
        registerAF.lo = setBit(registerAF.lo, C);
        reg = setBit(reg, 7);
    }

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}

void z80::RRCMem(uint16_t addr)
{
    uint8_t reg = read(addr);
    bool isSet = checkBit(reg, 0);

    registerAF.lo = 0;
    reg >>= 1;

    if (isSet) {
        registerAF.lo = setBit(registerAF.lo, C);
        reg = setBit(reg, 7);
    }

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    
    write(addr, reg);
}

void z80::RR(uint8_t& reg)
{

    bool isCarrySet = checkBit(registerAF.lo, C);
    bool isSet = checkBit(reg, 0);

    registerAF.lo = 0;
    reg >>= 1;

    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);

    if (isCarrySet)
        reg = setBit(reg, 7);

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}

void z80::RRMem(uint16_t addr)
{
    uint8_t reg = read(addr);
    bool isCarrySet = checkBit(registerAF.lo, C);
    bool isSet = checkBit(reg, 0);

    registerAF.lo = 0;
    reg >>= 1;

    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);

    if (isCarrySet)
        reg = setBit(reg, 7);

    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    write(addr, reg);
}

void z80::SLA(uint8_t& reg)
{
    bool isSet = checkBit(reg, 7);

    reg <<= 1;

    registerAF.lo = 0;

    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}

void z80::SLAMem(uint16_t addr)
{
    uint8_t reg = read(addr);
    bool isSet = checkBit(reg, 7);

    reg <<= 1;

    registerAF.lo = 0;

    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
    
    write(addr, reg);
}

void z80::SRA(uint8_t& reg)
{
    bool isSet = checkBit(reg, 0);
    bool isHSet = checkBit(reg, 7);

    reg >>= 1;

    registerAF.lo = 0;

    if (isHSet)
        reg = setBit(reg, 7);
    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}

void z80::SRAMem(uint16_t addr)
{
    uint8_t reg = read(addr);
    bool isSet = checkBit(reg, 0);
    bool isHSet = checkBit(reg, 7);

    reg >>= 1;

    registerAF.lo = 0;

    if (isHSet)
        reg = setBit(reg, 7);
    if (isSet)
        registerAF.lo = setBit(registerAF.lo, C);
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

    write(addr, reg);
}

void z80::SRL(uint8_t& reg)
{

    bool isLSet = checkBit(reg, 0);

    registerAF.lo = 0;

    reg >>= 1;

    if (isLSet)
        registerAF.lo = setBit(registerAF.lo, C);
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);
}

void z80::SRLMem(uint16_t addr)
{
    uint8_t reg = read(addr);

    bool isLSet = checkBit(reg, 0);

    registerAF.lo = 0;

    reg >>= 1;

    if (isLSet)
        registerAF.lo = setBit(registerAF.lo, C);
    if (reg == 0)
        registerAF.lo = setBit(registerAF.lo, Z);

    write(addr, reg);
}

void z80::testBit(uint8_t reg, int bit)
{

    bool isSet = checkBit(reg, bit);

    if (isSet)
        registerAF.lo = resetBit(registerAF.lo, Z);
    else
        registerAF.lo = setBit(registerAF.lo, Z);
    
    registerAF.lo = setBit(registerAF.lo, H);
    registerAF.hi = resetBit(registerAF.lo, N);
}

void z80::testBitMem(uint16_t addr, int bit)
{
    uint8_t reg = read(addr);
    bool isSet = checkBit(reg, bit);

    if (isSet)
        registerAF.lo = resetBit(registerAF.lo, Z);
    else
        registerAF.lo = setBit(registerAF.lo, Z);

    registerAF.lo = setBit(registerAF.lo, H);
    registerAF.hi = resetBit(registerAF.lo, N);

  
}

void z80::SET(uint8_t& reg, int bit)
{

    reg = setBit(reg, bit);

}

void z80::SETMem(uint16_t addr, int bit)
{
    uint8_t reg = read(addr);
    reg = setBit(reg, bit);
    write(addr, reg);
}

void z80::RES(uint8_t& reg, int bit)
{

    reg = resetBit(reg, bit);

}

void z80::RESMem(uint16_t addr, int bit)
{
    uint8_t reg = read(addr);
    reg = resetBit(reg, bit);
    write(addr, reg);
}

// opcodes from http://www.codeslinger.co.uk/pages/projects/gameboy/files/GB.pdf
// switch statement for all opcodes...
// returns the cycle number which decrements in clock()
// instructions with parameters will be above this
// each opcode uses instructions in different static
int z80::executeOP(uint8_t opcode)
{
    //std::cout << std::hex << +opcode << std::endl;
    switch (opcode) {
        //all 8bit load opcodes:
    case 0x06:
        BIT8_LOAD(registerBC.hi);
        return 8;
    case 0x0E:
        BIT8_LOAD(registerBC.lo);
        return 8;
    case 0x16:
        BIT8_LOAD(registerDE.hi);
        return 8;
    case 0x1E:
        BIT8_LOAD(registerDE.lo);
        return 8;
    case 0x26:
        BIT8_LOAD(registerHL.hi);
        return 8;
    case 0x2E:
        BIT8_LOAD(registerHL.lo);
        return 8;
        //load r2 into r1
    case 0x7F:
        BIT8_LOAD(registerAF.hi, registerAF.hi);
        return 4;
    case 0x78:
        BIT8_LOAD(registerAF.hi, registerBC.hi);
        return 4;
    case 0x79:
        BIT8_LOAD(registerAF.hi, registerBC.lo);
        return 4;
    case 0x7A:
        BIT8_LOAD(registerAF.hi, registerDE.hi);
        return 4;
    case 0x7B:
        BIT8_LOAD(registerAF.hi, registerDE.lo);
        return 4;
    case 0x7C:
        BIT8_LOAD(registerAF.hi, registerHL.hi);
        return 4;
    case 0x7D:
        BIT8_LOAD(registerAF.hi, registerHL.lo);
        return 4;
    case 0x7E:
        BIT8_LOAD_MEM(registerAF.hi, registerHL.reg);
        return 8;
    case 0x40:
        BIT8_LOAD(registerBC.hi, registerBC.hi);
        return 4;
    case 0x41:
        BIT8_LOAD(registerBC.hi, registerBC.lo);
        return 4;
    case 0x42:
        BIT8_LOAD(registerBC.hi, registerDE.hi);
        return 4;
    case 0x43:
        BIT8_LOAD(registerBC.hi, registerDE.lo);
        return 4;
    case 0x44:
        BIT8_LOAD(registerBC.hi, registerHL.hi);
        return 4;
    case 0x45:
        BIT8_LOAD(registerBC.hi, registerHL.lo);
        return 4;
    case 0x46:
        BIT8_LOAD_MEM(registerBC.hi, registerHL.reg);
        return 8;
    case 0x48:
        BIT8_LOAD(registerBC.lo, registerBC.hi);
        return 4;
    case 0x49:
        BIT8_LOAD(registerBC.lo, registerBC.lo);
        return 4;
    case 0x4A:
        BIT8_LOAD(registerBC.lo, registerDE.hi);
        return 4;
    case 0x4B:
        BIT8_LOAD(registerBC.lo, registerDE.lo);
        return 4;
    case 0x4C:
        BIT8_LOAD(registerBC.lo, registerHL.hi);
        return 4;
    case 0x4D:
        BIT8_LOAD(registerBC.lo, registerHL.lo);
        return 4;
    case 0x4E:
        BIT8_LOAD_MEM(registerBC.lo, registerHL.reg);
        return 8;
    case 0x50:
        BIT8_LOAD(registerDE.hi, registerBC.hi);
        return 4;
    case 0x51:
        BIT8_LOAD(registerDE.hi, registerBC.lo);
        return 4;
    case 0x52:
        BIT8_LOAD(registerDE.hi, registerDE.hi);
        return 4;
    case 0x53:
        BIT8_LOAD(registerDE.hi, registerDE.lo);
        return 4;
    case 0x54:
        BIT8_LOAD(registerDE.hi, registerHL.hi);
        return 4;
    case 0x55:
        BIT8_LOAD(registerDE.hi, registerHL.lo);
        return 4;
    case 0x56:
        BIT8_LOAD_MEM(registerDE.hi, registerHL.reg);
        return 8;
    case 0x58:
        BIT8_LOAD(registerDE.lo, registerBC.hi);
        return 4;
    case 0x59:
        BIT8_LOAD(registerDE.lo, registerBC.lo);
        return 4;
    case 0x5A:
        BIT8_LOAD(registerDE.lo, registerDE.hi);
        return 4;
    case 0x5B:
        BIT8_LOAD(registerDE.lo, registerDE.lo);
        return 4;
    case 0x5C:
        BIT8_LOAD(registerDE.lo, registerHL.hi);
        return 4;
    case 0x5D:
        BIT8_LOAD(registerDE.lo, registerHL.lo);
        return 4;
    case 0x5E:
        BIT8_LOAD_MEM(registerDE.lo, registerHL.reg);
        return 8;
    case 0x60:
        BIT8_LOAD(registerHL.hi, registerBC.hi);
        return 4;
    case 0x61:
        BIT8_LOAD(registerHL.hi, registerBC.lo);
        return 4;
    case 0x62:
        BIT8_LOAD(registerHL.hi, registerDE.hi);
        return 4;
    case 0x63:
        BIT8_LOAD(registerHL.hi, registerDE.lo);
        return 4;
    case 0x64:
        BIT8_LOAD(registerHL.hi, registerHL.hi);
        return 4;
    case 0x65:
        BIT8_LOAD(registerHL.hi, registerHL.lo);
        return 4;
    case 0x66:
        BIT8_LOAD_MEM(registerHL.hi, registerHL.reg);
        return 8;
    case 0x3E: {
        n = read(pc);
        pc++;
        registerAF.hi = n;
        return 8;
    }
    case 0x68:
        BIT8_LOAD(registerHL.lo, registerBC.hi);
        return 4;
    case 0x69:
        BIT8_LOAD(registerHL.lo, registerBC.lo);
        return 4;
    case 0x6A:
        BIT8_LOAD(registerHL.lo, registerDE.hi);
        return 4;
    case 0x6B:
        BIT8_LOAD(registerHL.lo, registerDE.lo);
        return 4;
    case 0x6C:
        BIT8_LOAD(registerHL.lo, registerHL.hi);
        return 4;
    case 0x6D:
        BIT8_LOAD(registerHL.lo, registerHL.lo);
        return 4;
    case 0x6E:
        BIT8_LOAD_MEM(registerHL.lo, registerHL.reg);
        return 8;
    case 0x70:
        write(registerHL.reg, registerBC.hi);
        return 8;
    case 0x71:
        write(registerHL.reg, registerBC.lo);
        return 8;
    case 0x72:
        write(registerHL.reg, registerDE.hi);
        return 8;
    case 0x73:
        write(registerHL.reg, registerDE.lo);
        return 8;
    case 0x74:
        write(registerHL.reg, registerHL.hi);
        return 8;
    case 0x75:
        write(registerHL.reg, registerHL.lo);
        return 8;
    case 0x36:
        n = read(pc);
        pc++;
        write(registerHL.reg, n);
        return 12;
    case 0x0A:
        BIT8_LOAD_MEM(registerAF.hi, registerBC.reg);
        return 8;
    case 0x1A:
        BIT8_LOAD_MEM(registerAF.hi, registerDE.reg);
        return 8;
    case 0xFA: {
        nn = read16();
        n = read(nn);
        registerAF.hi = n;
        pc++; pc++;
        return 16;
    }
    case 0x47:
        BIT8_LOAD(registerBC.hi, registerAF.hi);
        return 4;
    case 0x4F:
        BIT8_LOAD(registerBC.lo, registerAF.hi);
        return 4;
    case 0x57:
        BIT8_LOAD(registerDE.hi, registerAF.hi);
        return 4;
    case 0x5F:
        BIT8_LOAD(registerDE.lo, registerAF.hi);
        return 4;
    case 0x67:
        BIT8_LOAD(registerHL.hi, registerAF.hi);
        return 4;
    case 0x6F:
        BIT8_LOAD(registerBC.hi, registerAF.hi);
        return 4;
    case 0x02:
        write(registerBC.reg, registerAF.hi);
        return 8;
    case 0x12:
        write(registerDE.reg, registerAF.hi);
        return 8;
    case 0x77:
        write(registerHL.reg, registerAF.hi);
        return 8;

    case 0xEA:
        nn = read16();
        n = read(nn);
        write(n, registerAF.hi);
        return 16;
    case 0xF2:
        n = 0xFF00;
        n |= registerBC.lo;
        BIT8_LOAD(registerAF.hi, n);
        return 8;
    case 0xE2:
        n = 0xFF00;
        n |= registerBC.lo;
        BIT8_LOAD(n, registerAF.hi);
        return 8;
        //load A into memory, dec/inc HL
    case 0x3A:
        BIT8_LOAD_MEM(registerAF.hi, registerHL.reg);
        BIT16_DEC(registerHL.reg);
        return 8;
    case 0x2A:
        BIT8_LOAD_MEM(registerAF.hi, registerHL.reg);
        BIT16_INC(registerHL.reg);
        return 8;
    case 0x22:
        write(registerHL.reg, registerAF.hi);
        BIT16_INC(registerHL.reg);
        return 8;
    case 0x32:
        write(registerHL.reg, registerAF.hi);
        BIT16_DEC(registerHL.reg);
        //load n + 0xFF00 to A
    case 0xE0:
        n = read(pc);
        n |= 0xFF00;
        BIT8_LOAD(n, registerAF.hi);
        return 12;
    case 0xF0:
        n = read(pc);
        n |= 0xFF00;
        BIT8_LOAD(registerAF.hi, n);
        return 12;
    case 0x01:
        BIT16_LOAD(registerBC.reg);
        return 12;
    case 0x11:
        BIT16_LOAD(registerDE.reg);
        return 12;
    case 0x21:
        BIT16_LOAD(registerHL.reg);
        return 12;
    case 0x31:
        BIT16_LOAD(sp);
        return 12;
    case 0xF9:
        sp = registerHL.reg;
        return 8;
    case 0xF8: {
        n = read(pc);
        pc++;
        registerAF.lo = resetBit(registerAF.lo, Z);
        registerAF.lo = resetBit(registerAF.lo, N);

        uint16_t value = (sp + n) & 0xFFFF;
        registerHL.reg = value;

        uint32_t v = sp + n;
        if (v > 0xFFFF)
            registerAF.lo = setBit(registerAF.lo, C);
        else
            registerAF.lo = resetBit(registerAF.lo, C);
        if (((registerAF.lo & 0xF) + (n & 0xF)) > 0xF)
            registerAF.lo = setBit(registerAF.lo, H);
        else
            registerAF.lo = resetBit(registerAF.lo, H);
        return 12;
    }
    case 0x08: {
        nn = read16();
        pc += 2;
        uint8_t lo = sp & 0x00FF;
        uint8_t hi = (sp & 0xFF00) >>8;

        write(nn, lo);
        nn++;
        write(nn, hi);
        return 20;
    }
    case 0xF5:
        PUSH16(registerAF.reg);
        return 16;
    case 0xC5:
        PUSH16(registerBC.reg);
        return 16;
    case 0xD5:
        PUSH16(registerDE.reg);
        return 16;
    case 0xE5:
        PUSH16(registerHL.reg);
        return 16;
    case 0xF1:
        POP16(registerAF.reg);
        return 12;
    case 0xC1:
        POP16(registerBC.reg);
        return 12;
    case 0xD1:
        POP16(registerDE.reg);
        return 12;
    case 0xE1:
        POP16(registerHL.reg);
        return 12;
    case 0x87:
        ADD_8BIT(registerAF.hi, registerAF.hi, false);
        return 4;
    case 0x80:
        ADD_8BIT(registerAF.hi, registerBC.hi, false);
        return 4;
    case 0x81:
        ADD_8BIT(registerAF.hi, registerBC.lo, false);
        return 4;
    case 0x82:
        ADD_8BIT(registerAF.hi, registerDE.hi, false);
        return 4;
    case 0x83:
        ADD_8BIT(registerAF.hi, registerDE.lo, false);
        return 4;
    case 0x84:
        ADD_8BIT(registerAF.hi, registerHL.hi, false);
        return 4;
    case 0x85:
        ADD_8BIT(registerAF.hi, registerHL.lo, false);
        return 4;
    case 0x86:
        ADD_8BIT(registerAF.hi, read(registerHL.reg), false);
        return 8;
    case 0xC6:
        n = read(pc);
        pc++;
        ADD_8BIT(registerAF.hi, n, false);
        return 8;
    case 0x8F:
        ADD_8BIT(registerAF.hi, registerAF.hi, true);
        return 4;
    case 0x88:
        ADD_8BIT(registerAF.hi, registerBC.hi, true);
        return 4;
    case 0x89:
        ADD_8BIT(registerAF.hi, registerBC.lo, true);
        return 4;
    case 0x8A:
        ADD_8BIT(registerAF.hi, registerDE.hi, true);
        return 4;
    case 0x8B:
        ADD_8BIT(registerAF.hi, registerDE.lo, true);
        return 4;
    case 0x8C:
        ADD_8BIT(registerAF.hi, registerHL.hi, true);
        return 4;
    case 0x8D:
        ADD_8BIT(registerAF.hi, registerHL.lo, true);
        return 4;
    case 0x8E:
        ADD_8BIT(registerAF.hi, read(registerHL.reg), true);
        return 8;
    case 0xCE:
        n = read(pc);
        pc++;
        ADD_8BIT(registerAF.hi, n, true);
        return 8;
    case 0x97:
        SUB_8BIT(registerAF.hi, registerAF.hi, false);
        return 4;
    case 0x90:
        SUB_8BIT(registerAF.hi, registerBC.hi, false);
        return 4;
    case 0x91:
        SUB_8BIT(registerAF.hi, registerBC.lo, false);
        return 4;
    case 0x92:
        SUB_8BIT(registerAF.hi, registerDE.hi, false);
        return 4;
    case 0x93:
        SUB_8BIT(registerAF.hi, registerDE.lo, false);
        return 4;
    case 0x94:
        SUB_8BIT(registerAF.hi, registerHL.hi, false);
        return 4;
    case 0x95:
        SUB_8BIT(registerAF.hi, registerHL.lo, false);
        return 4;
    case 0x96:
        SUB_8BIT(registerAF.hi, read(registerHL.reg), false);
        return 8;
    case 0xD6:
        n = read(pc);
        pc++;
        SUB_8BIT(registerAF.hi, n, false);
        return 8;
    case 0x9F:
        SUB_8BIT(registerAF.hi, registerAF.hi, true);
        return 4;
    case 0x98:
        SUB_8BIT(registerAF.hi, registerBC.hi, true);
        return 4;
    case 0x99:
        SUB_8BIT(registerAF.hi, registerBC.lo, true);
        return 4;
    case 0x9A:
        SUB_8BIT(registerAF.hi, registerDE.hi, true);
        return 4;
    case 0x9B:
        SUB_8BIT(registerAF.hi, registerDE.lo, true);
        return 4;
    case 0x9C:
        SUB_8BIT(registerAF.hi, registerHL.hi, true);
        return 4;
    case 0x9D:
        SUB_8BIT(registerAF.hi, registerHL.lo, true);
        return 4;
    case 0x9E:
        SUB_8BIT(registerAF.hi, read(registerHL.reg), true);
        return 8;
        /*case ??:
            n = read(pc);
            pc++;
            SUB_8BIT(registerAF.hi, n, false);
            return ?;*/
    case 0xDE:
        n = read(pc);
        pc++;
        SUB_8BIT(registerAF.hi, n, true);
        return 8;
    case 0xA7:
        AND_8BIT(registerAF.hi, registerAF.hi);
        return 4;
    case 0xA0:
        AND_8BIT(registerAF.hi, registerBC.hi);
        return 4;
    case 0xA1:
        AND_8BIT(registerAF.hi, registerBC.lo);
        return 4;
    case 0xA2:
        AND_8BIT(registerAF.hi, registerDE.hi);
        return 4;
    case 0xA3:
        AND_8BIT(registerAF.hi, registerDE.lo);
        return 4;
    case 0xA4:
        AND_8BIT(registerAF.hi, registerHL.hi);
        return 4;
    case 0xA5:
        AND_8BIT(registerAF.hi, registerHL.lo);
        return 4;
    case 0xA6:
        AND_8BIT(registerAF.hi, read(registerHL.reg));
        return 8;
    case 0xE6:
        n = read(pc);
        pc++;
        AND_8BIT(registerAF.hi, n);
        return 8;
    case 0xB7:
        OR_8BIT(registerAF.hi, registerAF.hi);
        return 4;
    case 0xB0:
        OR_8BIT(registerAF.hi, registerBC.hi);
        return 4;
    case 0xB1:
        OR_8BIT(registerAF.hi, registerBC.lo);
        return 4;
    case 0xB2:
        OR_8BIT(registerAF.hi, registerDE.hi);
        return 4;
    case 0xB3:
        OR_8BIT(registerAF.hi, registerDE.lo);
        return 4;
    case 0xB4:
        OR_8BIT(registerAF.hi, registerHL.hi);
        return 4;
    case 0xB5:
        OR_8BIT(registerAF.hi, registerHL.lo);
        return 4;
    case 0xB6:
        OR_8BIT(registerAF.hi, read(registerHL.reg));
        return 8;
    case 0xF6:
        n = read(pc);
        pc++;
        OR_8BIT(registerAF.hi, n);
        return 8;
    case 0xAF:
        XOR_8BIT(registerAF.hi, registerAF.hi);
        return 4;
    case 0xA8:
        XOR_8BIT(registerAF.hi, registerBC.hi);
        return 4;
    case 0xA9:
        XOR_8BIT(registerAF.hi, registerBC.lo);
        return 4;
    case 0xAA:
        XOR_8BIT(registerAF.hi, registerDE.hi);
        return 4;
    case 0xAB:
        XOR_8BIT(registerAF.hi, registerDE.lo);
        return 4;
    case 0xAC:
        XOR_8BIT(registerAF.hi, registerHL.hi);
        return 4;
    case 0xAD:
        XOR_8BIT(registerAF.hi, registerHL.lo);
        return 4;
    case 0xAE:
        XOR_8BIT(registerAF.hi, read(registerHL.reg));
        return 8;
    case 0xEE:
        n = read(pc);
        pc++;
        XOR_8BIT(registerAF.hi, n);
        return 8;
    case 0xBF:
        CP_8BIT(registerAF.hi, registerAF.hi);
        return 4;
    case 0xB8:
        CP_8BIT(registerAF.hi, registerBC.hi);
        return 4;
    case 0xB9:
        CP_8BIT(registerAF.hi, registerBC.lo);
        return 4;
    case 0xBA:
        CP_8BIT(registerAF.hi, registerDE.hi);
        return 4;
    case 0xBB:
        CP_8BIT(registerAF.hi, registerDE.lo);
        return 4;
    case 0xBC:
        CP_8BIT(registerAF.hi, registerHL.hi);
        return 4;
    case 0xBD:
        CP_8BIT(registerAF.hi, registerHL.lo);
        return 4;
    case 0xBE:
        CP_8BIT(registerAF.hi, read(registerHL.reg));
        return 8;
    case 0xFE:
        n = read(pc);
        pc++;
        CP_8BIT(registerAF.hi, n);
        return 8;
    case 0x3C:
        INC_8BIT(registerAF.hi);
        return 4;
    case 0x04:
        INC_8BIT(registerBC.hi);
        return 4;
    case 0x0C:
        INC_8BIT(registerBC.lo);
        return 4;
    case 0x14:
        INC_8BIT(registerDE.hi);
        return 4;
    case 0x1C:
        INC_8BIT(registerDE.lo);
        return 4;
    case 0x24:
        INC_8BIT(registerHL.hi);
        return 4;
    case 0x2C:
        INC_8BIT(registerHL.lo);
        return 4;
    case 0x34:
        INC_16BIT(registerHL.reg);
        return 12;
    case 0x3D:
        DEC_8BIT(registerAF.hi);
        return 4;
    case 0x05:
        DEC_8BIT(registerBC.hi);
        return 4;
    case 0x0D:
        DEC_8BIT(registerBC.lo);
        return 4;
    case 0x15:
        DEC_8BIT(registerDE.hi);
        return 4;
    case 0x1D:
        DEC_8BIT(registerDE.lo);
        return 4;
    case 0x25:
        DEC_8BIT(registerHL.hi);
        return 4;
    case 0x2D:
        DEC_8BIT(registerHL.lo);
        return 4;
    case 0x35:
        DEC_16BIT(registerHL.reg);
        return 12;
    case 0x09:
        ADD_16BIT(registerHL.reg, registerBC.reg);
        return 8;
    case 0x19:
        ADD_16BIT(registerHL.reg, registerDE.reg);
        return 8;
    case 0x29:
        ADD_16BIT(registerHL.reg, registerHL.reg);
        return 8;
    case 0x39:
        ADD_16BIT(registerHL.reg, sp);
        return 8;
    case 0xE8: {
        int8_t sn = (int8_t)read(pc);
        pc++;
        ADD_16BIT(sp, sn);
        return 16;
    }
    case 0x03:
        registerBC.reg++;
        return 8;
    case 0x13:
        registerDE.reg++;
        return 8;
    case 0x23:
        registerHL.reg++;
        return 8;
    case 0x33:
        sp++;
        return 8;
    case 0x0B:
        registerBC.reg--;
        return 8;
    case 0x1B:
        registerDE.reg--;
        return 8;
    case 0x2B:
        registerHL.reg--;
        return 8;
    case 0x3B:
        sp--;
        return 8;
    case 0x27:
        DAA();
        return 4;
    case 0x2F:
        CPL();
        return 4;
    case 0x3F:
        CCF();
        return 4;
    case 0x37:
        SCF();
        return 4;
    case 0xF3://interrupt variable might need to change AFTER instruction
        enableInterrupts = false;
        return 4;
    case 0xFB:
        enableInterrupts = true;
        return 4;
    case 0x00:
        return 4;
    case 0x76:
        halted = true;
        return 4;
    case 0x10:
        return 4;
    case 0x07:
        RLC(registerAF.hi);
        return 4;
    case 0x17:
        RL(registerAF.hi);
        return 4;
    case 0x0F:
        RRC(registerAF.hi);
        return 4;
    case 0x1F:
        RR(registerAF.hi);
        return 4;
    case 0xC3:
        JP();
        return 12;
    case 0xC2:
        JP(Z, false);
        return 12;
    case 0xCA:
        JP(Z, true);
        return 12;
    case 0xD2:
        JP(C, false);
        return 12;
    case 0xDA:
        JP(C, true);
        return 12;
    case 0xE9:
        pc = registerHL.reg;
        return 4;
    case 0x18:
        JR();
        return 8;
    case 0x20:
        JR(Z, false);
        return 8;
    case 0x28:
        JR(Z, true);
        return 8;
    case 0x30:
        JR(C, false);
        return 8;
    case 0x38:
        JR(C, false);
        return 8;
    case 0xCD:
        CALL();
        return 12;
    case 0xC4:
        CALL(Z, false);
        return 12;
    case 0xCC:
        CALL(Z, true);
        return 12;
    case 0xD4:
        CALL(C, false);
        return 12;
    case 0xDC:
        CALL(C, true);
        return 12;
    case 0xC9:
        RET();
        return 8;
    case 0xC0:
        RET(Z, false);
        return 8;
    case 0xC8:
        RET(Z, true);
        return 8;
    case 0xD0:
        RET(C, false);
        return 8;
    case 0xD8:
        RET(C, true);
        return 8;
    case 0xC7:
        RST(0x00);
        return 32;
    case 0xCF:
        RST(0x08);
        return 32;
    case 0xD7:
        RST(0x10);
        return 32;
    case 0xDF:
        RST(0x18);
        return 32;
    case 0xE7:
        RST(0x20);
        return 32;
    case 0xEF:
        RST(0x28);
        return 32;
    case 0xF7:
        RST(0x30);
        return 32;
    case 0xFF:
        RST(0x38);
        return 32;
    case 0xD9: {
        //RETI(); used when returning from interupt (gets instruction back from stack)
        pc = popShortFromStack();
        enableInterrupts = true;
        return 8; 
    }






    case 0xCB: {
        pc++;
        uint8_t exOpcode = read(pc);
        return executeExOP(exOpcode);
    }
    default:
        std::cout << "Unknown opcode" << std::endl;
        return 0;
    }//end of switch
}

int z80::executeExOP(uint8_t opcode) {

    switch (opcode) {
    case 0x37:
        SWAP_NIBBLES(registerAF.hi);
        return 8;
    case 0x30:
        SWAP_NIBBLES(registerBC.hi);
        return 8;
    case 0x31:
        SWAP_NIBBLES(registerBC.lo);
        return 8;
    case 0x32:
        SWAP_NIBBLES(registerDE.hi);
        return 8;
    case 0x33:
        SWAP_NIBBLES(registerDE.lo);
        return 8;
    case 0x34:
        SWAP_NIBBLES(registerHL.hi);
        return 8;
    case 0x35:
        SWAP_NIBBLES(registerHL.lo);
        return 8;
    case 0x36:
        SWAP_NIB_MEM(registerHL.reg);
        return 16;
    case 0x07:
        RLC(registerAF.hi);
        return 8;
    case 0x00:
        RLC(registerBC.hi);
        return 8;
    case 0x01:
        RLC(registerBC.lo);
        return 8;
    case 0x02:
        RLC(registerDE.hi);
        return 8;
    case 0x03:
        RLC(registerDE.lo);
        return 8;
    case 0x04:
        RLC(registerHL.hi);
        return 8;
    case 0x05:
        RLC(registerHL.lo);
        return 8;
    case 0x06:
        RLCMem(registerHL.reg);
        return 16;
    case 0x17:
        RL(registerAF.hi);
        return 8;
    case 0x10:
        RL(registerBC.hi);
        return 8;
    case 0x11:
        RL(registerBC.lo);
        return 8;
    case 0x12:
        RL(registerDE.hi);
        return 8;
    case 0x13:
        RL(registerDE.lo);
        return 8;
    case 0x14:
        RL(registerHL.hi);
        return 8;
    case 0x15:
        RL(registerHL.lo);
        return 8;
    case 0x16:
        RLMem(registerHL.reg);
        return 16;
    case 0x0F:
        RRC(registerAF.hi);
        return 8;
    case 0x08:
        RRC(registerBC.hi);
        return 8;
    case 0x09:
        RRC(registerBC.lo);
        return 8;
    case 0x0A:
        RRC(registerDE.hi);
        return 8;
    case 0x0B:
        RRC(registerDE.lo);
        return 8;
    case 0x0C:
        RRC(registerHL.hi);
        return 8;
    case 0x0D:
        RRC(registerHL.lo);
        return 8;
    case 0x0E:
        RRCMem(registerHL.reg);
        return 16;
    case 0x1F:
        RR(registerAF.hi);
        return 8;
    case 0x18:
        RR(registerBC.hi);
        return 8;
    case 0x19:
        RR(registerBC.lo);
        return 8;
    case 0x1A:
        RR(registerDE.hi);
        return 8;
    case 0x1B:
        RR(registerDE.lo);
        return 8;
    case 0x1C:
        RR(registerHL.hi);
        return 8;
    case 0x1D:
        RR(registerHL.lo);
        return 8;
    case 0x1E:
        RRMem(registerHL.reg);
        return 16;
    case 0x27:
        SLA(registerAF.hi);
        return 8;
    case 0x20:
        SLA(registerBC.hi);
        return 8;
    case 0x21:
        SLA(registerBC.lo);
        return 8;
    case 0x22:
        SLA(registerDE.hi);
        return 8;
    case 0x23:
        SLA(registerDE.lo);
        return 8;
    case 0x24:
        SLA(registerHL.hi);
        return 8;
    case 0x25:
        SLA(registerHL.lo);
        return 8;
    case 0x26:
        SLAMem(registerHL.reg);
        return 16;
    case 0x2F:
        SRA(registerAF.hi);
        return 8;
    case 0x28:
        SRA(registerBC.hi);
        return 8;
    case 0x29:
        SRA(registerBC.lo);
        return 8;
    case 0x2A:
        SRA(registerDE.hi);
        return 8;
    case 0x2B:
        SRA(registerDE.lo);
        return 8;
    case 0x2C:
        SRA(registerHL.hi);
        return 8;
    case 0x2D:
        SRA(registerHL.lo);
        return 8;
    case 0x2E:
        SRAMem(registerHL.reg);
        return 16;
    case 0x3F:
        SRL(registerAF.hi);
        return 8;
    case 0x38:
        SRL(registerBC.hi);
        return 8;
    case 0x39:
        SRL(registerBC.lo);
        return 8;
    case 0x3A:
        SRL(registerDE.hi);
        return 8;
    case 0x3B:
        SRL(registerDE.lo);
        return 8;
    case 0x3C:
        SRL(registerHL.hi);
        return 8;
    case 0x3D:
        SRL(registerHL.lo);
        return 8;
    case 0x3E:
        SRLMem(registerHL.reg);
        return 16;
    //test bit 0
    case 0x47:
        testBit(registerAF.hi, 0);
        return 8;
    case 0x40:
        testBit(registerBC.hi, 0);
        return 8;
    case 0x41:
        testBit(registerBC.lo, 0);
        return 8;
    case 0x42:
        testBit(registerDE.hi, 0);
        return 8;
    case 0x43:
        testBit(registerDE.lo, 0);
        return 8;
    case 0x44:
        testBit(registerHL.hi, 0);
        return 8;
    case 0x45:
        testBit(registerHL.lo, 0);
        return 8;
    case 0x46:
        testBitMem(registerHL.reg, 0);
        return 12;
        //test bit 1
    case 0x4F:
        testBit(registerAF.hi, 1);
        return 8;
    case 0x48:
        testBit(registerBC.hi, 1);
        return 8;
    case 0x49:
        testBit(registerBC.lo, 1);
        return 8;
    case 0x4A:
        testBit(registerDE.hi, 1);
        return 8;
    case 0x4B:
        testBit(registerDE.lo, 1);
        return 8;
    case 0x4C:
        testBit(registerHL.hi, 1);
        return 8;
    case 0x4D:
        testBit(registerHL.lo, 1);
        return 8;
    case 0x4E:
        testBitMem(registerHL.reg, 1);
        return 12;
        //test bit 2
    case 0x57:
        testBit(registerAF.hi, 2);
        return 8;
    case 0x50:
        testBit(registerBC.hi, 2);
        return 8;
    case 0x51:
        testBit(registerBC.lo, 2);
        return 8;
    case 0x52:
        testBit(registerDE.hi, 2);
        return 8;
    case 0x53:
        testBit(registerDE.lo, 2);
        return 8;
    case 0x54:
        testBit(registerHL.hi, 2);
        return 8;
    case 0x55:
        testBit(registerHL.lo, 2);
        return 8;
    case 0x56:
        testBitMem(registerHL.reg, 2);
        return 12;
        //test bit 3
    case 0x5F:
        testBit(registerAF.hi, 3);
        return 8;
    case 0x58:
        testBit(registerBC.hi, 3);
        return 8;
    case 0x59:
        testBit(registerBC.lo, 3);
        return 8;
    case 0x5A:
        testBit(registerDE.hi, 3);
        return 8;
    case 0x5B:
        testBit(registerDE.lo, 3);
        return 8;
    case 0x5C:
        testBit(registerHL.hi, 3);
        return 8;
    case 0x5D:
        testBit(registerHL.lo, 3);
        return 8;
    case 0x5E:
        testBitMem(registerHL.reg, 3);
        return 12;
        //test bit 4
    case 0x67:
        testBit(registerAF.hi, 4);
        return 8;
    case 0x60:
        testBit(registerBC.hi, 4);
        return 8;
    case 0x61:
        testBit(registerBC.lo, 4);
        return 8;
    case 0x62:
        testBit(registerDE.hi, 4);
        return 8;
    case 0x63:
        testBit(registerDE.lo, 4);
        return 8;
    case 0x64:
        testBit(registerHL.hi, 4);
        return 8;
    case 0x65:
        testBit(registerHL.lo, 4);
        return 8;
    case 0x66:
        testBitMem(registerHL.reg, 4);
        return 12;
        //test bit 5
    case 0x6F:
        testBit(registerAF.hi, 5);
        return 8;
    case 0x68:
        testBit(registerBC.hi, 5);
        return 8;
    case 0x69:
        testBit(registerBC.lo, 5);
        return 8;
    case 0x6A:
        testBit(registerDE.hi, 5);
        return 8;
    case 0x6B:
        testBit(registerDE.lo, 5);
        return 8;
    case 0x6C:
        testBit(registerHL.hi, 5);
        return 8;
    case 0x6D:
        testBit(registerHL.lo, 5);
        return 8;
    case 0x6E:
        testBitMem(registerHL.reg, 5);
        return 12;
        //test bit 6
    case 0x77:
        testBit(registerAF.hi, 6);
        return 8;
    case 0x70:
        testBit(registerBC.hi, 6);
        return 8;
    case 0x71:
        testBit(registerBC.lo, 6);
        return 8;
    case 0x72:
        testBit(registerDE.hi, 6);
        return 8;
    case 0x73:
        testBit(registerDE.lo, 6);
        return 8;
    case 0x74:
        testBit(registerHL.hi, 6);
        return 8;
    case 0x75:
        testBit(registerHL.lo, 6);
        return 8;
    case 0x76:
        testBitMem(registerHL.reg, 6);
        return 12;
        //test bit 7
    case 0x7F:
        testBit(registerAF.hi, 7);
        return 8;
    case 0x78:
        testBit(registerBC.hi, 7);
        return 8;
    case 0x79:
        testBit(registerBC.lo, 7);
        return 8;
    case 0x7A:
        testBit(registerDE.hi, 7);
        return 8;
    case 0x7B:
        testBit(registerDE.lo, 7);
        return 8;
    case 0x7C:
        testBit(registerHL.hi, 7);
        return 8;
    case 0x7D:
        testBit(registerHL.lo, 7);
        return 8;
    case 0x7E:
        testBitMem(registerHL.reg, 7);
        return 12;
    //Reset bit 0
    case 0x87:
        RES(registerAF.hi, 0);
        return 8;
    case 0x80:
        RES(registerBC.hi, 0);
        return 8;
    case 0x81:
        RES(registerBC.lo, 0);
        return 8;
    case 0x82:
        RES(registerDE.hi, 0);
        return 8;
    case 0x83:
        RES(registerDE.lo, 0);
        return 8;
    case 0x84:
        RES(registerHL.hi, 0);
        return 8;
    case 0x85:
        RES(registerHL.lo, 0);
        return 8;
    case 0x86:
        RESMem(registerHL.reg, 0);
        return 16;
        //Reset bit 1
    case 0x8F:
        RES(registerAF.hi, 1);
        return 8;
    case 0x88:
        RES(registerBC.hi, 1);
        return 8;
    case 0x89:
        RES(registerBC.lo, 1);
        return 8;
    case 0x8A:
        RES(registerDE.hi, 1);
        return 8;
    case 0x8B:
        RES(registerDE.lo, 1);
        return 8;
    case 0x8C:
        RES(registerHL.hi, 1);
        return 8;
    case 0x8D:
        RES(registerHL.lo, 1);
        return 8;
    case 0x8E:
        RESMem(registerHL.reg, 1);
        return 16;
        //Reset bit 2
    case 0x97:
        RES(registerAF.hi, 2);
        return 8;
    case 0x90:
        RES(registerBC.hi, 2);
        return 8;
    case 0x91:
        RES(registerBC.lo, 2);
        return 8;
    case 0x92:
        RES(registerDE.hi, 2);
        return 8;
    case 0x93:
        RES(registerDE.lo, 2);
        return 8;
    case 0x94:
        RES(registerHL.hi, 2);
        return 8;
    case 0x95:
        RES(registerHL.lo, 2);
        return 8;
    case 0x96:
        RESMem(registerHL.reg, 2);
        return 16;
        //Reset bit 3
    case 0x9F:
        RES(registerAF.hi, 3);
        return 8;
    case 0x98:
        RES(registerBC.hi, 3);
        return 8;
    case 0x99:
        RES(registerBC.lo, 3);
        return 8;
    case 0x9A:
        RES(registerDE.hi, 3);
        return 8;
    case 0x9B:
        RES(registerDE.lo, 3);
        return 8;
    case 0x9C:
        RES(registerHL.hi, 3);
        return 8;
    case 0x9D:
        RES(registerHL.lo, 3);
        return 8;
    case 0x9E:
        RESMem(registerHL.reg, 3);
        return 16;
        //Reset bit 4
    case 0xA7:
        RES(registerAF.hi, 4);
        return 8;
    case 0xA0:
        RES(registerBC.hi, 4);
        return 8;
    case 0xA1:
        RES(registerBC.lo, 4);
        return 8;
    case 0xA2:
        RES(registerDE.hi, 4);
        return 8;
    case 0xA3:
        RES(registerDE.lo, 4);
        return 8;
    case 0xA4:
        RES(registerHL.hi, 4);
        return 8;
    case 0xA5:
        RES(registerHL.lo, 4);
        return 8;
    case 0xA6:
        RESMem(registerHL.reg, 4);
        return 16;
        //Reset bit 5
    case 0xAF:
        RES(registerAF.hi, 5);
        return 8;
    case 0xA8:
        RES(registerBC.hi, 5);
        return 8;
    case 0xA9:
        RES(registerBC.lo, 5);
        return 8;
    case 0xAA:
        RES(registerDE.hi, 5);
        return 8;
    case 0xAB:
        RES(registerDE.lo, 5);
        return 8;
    case 0xAC:
        RES(registerHL.hi, 5);
        return 8;
    case 0xAD:
        RES(registerHL.lo, 5);
        return 8;
    case 0xAE:
        RESMem(registerHL.reg, 5);
        return 16;
        //Reset bit 6
    case 0xB7:
        RES(registerAF.hi, 6);
        return 8;
    case 0xB0:
        RES(registerBC.hi, 6);
        return 8;
    case 0xB1:
        RES(registerBC.lo, 6);
        return 8;
    case 0xB2:
        RES(registerDE.hi, 6);
        return 8;
    case 0xB3:
        RES(registerDE.lo, 6);
        return 8;
    case 0xB4:
        RES(registerHL.hi, 6);
        return 8;
    case 0xB5:
        RES(registerHL.lo, 6);
        return 8;
    case 0xB6:
        RESMem(registerHL.reg, 6);
        return 16;
        //Reset bit 7
    case 0xBF:
        RES(registerAF.hi, 7);
        return 8;
    case 0xB8:
        RES(registerBC.hi, 7);
        return 8;
    case 0xB9:
        RES(registerBC.lo, 7);
        return 8;
    case 0xBA:
        RES(registerDE.hi, 7);
        return 8;
    case 0xBB:
        RES(registerDE.lo, 7);
        return 8;
    case 0xBC:
        RES(registerHL.hi, 7);
        return 8;
    case 0xBD:
        RES(registerHL.lo, 7);
        return 8;
    case 0xBE:
        RESMem(registerHL.reg, 7);
        return 16;
        //SET bit at 0
    case 0xC7:
        SET(registerAF.hi, 0);
        return 8;
    case 0xC0:
        SET(registerBC.hi, 0);
        return 8;
    case 0xC1:
        SET(registerBC.lo, 0);
        return 8;
    case 0xC2:
        SET(registerDE.hi, 0);
        return 8;
    case 0xC3:
        SET(registerDE.lo, 0);
        return 8;
    case 0xC4:
        SET(registerHL.hi, 0);
        return 8;
    case 0xC5:
        SET(registerHL.lo, 0);
        return 8;
    case 0xC6:
        SETMem(registerHL.reg, 0);
        return 16;
        //SET bit at 1
    case 0xCF:
        SET(registerAF.hi, 1);
        return 8;
    case 0xC8:
        SET(registerBC.hi, 1);
        return 8;
    case 0xC9:
        SET(registerBC.lo, 1);
        return 8;
    case 0xCA:
        SET(registerDE.hi, 1);
        return 8;
    case 0xCB:
        SET(registerDE.lo, 1);
        return 8;
    case 0xCC:
        SET(registerHL.hi, 1);
        return 8;
    case 0xCD:
        SET(registerHL.lo, 1);
        return 8;
    case 0xCE:
        SETMem(registerHL.reg, 1);
        return 16;
        //SET bit at 2
    case 0xD7:
        SET(registerAF.hi, 2);
        return 8;
    case 0xD0:
        SET(registerBC.hi, 2);
        return 8;
    case 0xD1:
        SET(registerBC.lo, 2);
        return 8;
    case 0xD2:
        SET(registerDE.hi, 2);
        return 8;
    case 0xD3:
        SET(registerDE.lo, 2);
        return 8;
    case 0xD4:
        SET(registerHL.hi, 2);
        return 8;
    case 0xD5:
        SET(registerHL.lo, 2);
        return 8;
    case 0xD6:
        SETMem(registerHL.reg, 2);
        return 16;
        //SET bit at 3
    case 0xDF:
        SET(registerAF.hi, 3);
        return 8;
    case 0xD8:
        SET(registerBC.hi, 3);
        return 8;
    case 0xD9:
        SET(registerBC.lo, 3);
        return 8;
    case 0xDA:
        SET(registerDE.hi, 3);
        return 8;
    case 0xDB:
        SET(registerDE.lo, 3);
        return 8;
    case 0xDC:
        SET(registerHL.hi, 3);
        return 8;
    case 0xDD:
        SET(registerHL.lo, 3);
        return 8;
    case 0xDE:
        SETMem(registerHL.reg, 3);
        return 16;
        //SET bit at 4
    case 0xE7:
        SET(registerAF.hi, 4);
        return 8;
    case 0xE0:
        SET(registerBC.hi, 4);
        return 8;
    case 0xE1:
        SET(registerBC.lo, 4);
        return 8;
    case 0xE2:
        SET(registerDE.hi, 4);
        return 8;
    case 0xE3:
        SET(registerDE.lo, 4);
        return 8;
    case 0xE4:
        SET(registerHL.hi, 4);
        return 8;
    case 0xE5:
        SET(registerHL.lo, 4);
        return 8;
    case 0xE6:
        SETMem(registerHL.reg, 4);
        return 16;
    //SET bit at 5
    case 0xEF:
        SET(registerAF.hi, 5);
        return 8;
    case 0xE8:
        SET(registerBC.hi, 5);
        return 8;
    case 0xE9:
        SET(registerBC.lo, 5);
        return 8;
    case 0xEA:
        SET(registerDE.hi, 5);
        return 8;
    case 0xEB:
        SET(registerDE.lo, 5);
        return 8;
    case 0xEC:
        SET(registerHL.hi, 5);
        return 8;
    case 0xED:
        SET(registerHL.lo, 5);
        return 8;
    case 0xEE:
        SETMem(registerHL.reg, 5);
        return 16;
    //SET bit at 6
    case 0xF7:
        SET(registerAF.hi, 6);
        return 8;
    case 0xF0:
        SET(registerBC.hi, 6);
        return 8;
    case 0xF1:
        SET(registerBC.lo, 6);
        return 8;
    case 0xF2:
        SET(registerDE.hi, 6);
        return 8;
    case 0xF3:
        SET(registerDE.lo, 6);
        return 8;
    case 0xF4:
        SET(registerHL.hi, 6);
        return 8;
    case 0xF5:
        SET(registerHL.lo, 6);
        return 8;
    case 0xF6:
        SETMem(registerHL.reg, 6);
        return 16;
        //SET bit at 7
    case 0xFF:
        SET(registerAF.hi, 7);
        return 8;
    case 0xF8:
        SET(registerBC.hi, 7);
        return 8;
    case 0xF9:
        SET(registerBC.lo, 7);
        return 8;
    case 0xFA:
        SET(registerDE.hi, 7);
        return 8;
    case 0xFB:
        SET(registerDE.lo, 7);
        return 8;
    case 0xFC:
        SET(registerHL.hi, 7);
        return 8;
    case 0xFD:
        SET(registerHL.lo, 7);
        return 8;
    case 0xFE:
        SETMem(registerHL.reg, 7);
        return 16;
    



    default:
        std::cout << "Invalid ExOpcode" << std::endl;
        break;

    }//end of switch
}