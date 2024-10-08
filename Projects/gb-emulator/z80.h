#pragma once
#include <cstdint>
#include <iostream>
//#include "bus.h"
class Bus;

class z80
{
public:
    z80();
    ~z80();
    void ConnectBus(Bus* n);

    enum FLAGSz80 {
        C = (1 << 4), //carry flag
        H = (1 << 5), //half carry flag
        N = (1 << 6), //subtract flag
        Z = (1 << 7) //zero flag
    };
    uint16_t pc = 0x100; //program pointer
    uint16_t sp = 0xFFFE; //stack pointer

    //union of a register for 8bit register pair
    //reg is the register pair, lo is the the first register
    //and hi is the second register
    union Register
    {
        uint16_t reg;
        struct
        {
            uint8_t lo;
            uint8_t hi;
        };
    };
    //example: registerAF.reg = 0xAABB
    //registerAF.hi = 0xAA and lo = 0xBB
    //hi = A and lo = F
    Register registerAF;
    Register registerBC;
    Register registerDE;
    Register registerHL;

    //perform a single clock cycle
    void clock();
    bool complete();


private:
    //initiate the bus class
    Bus* bus = nullptr;
    //temp value
    uint16_t nn = 0x0000;
    uint8_t n = 0x00;
    //read and write for memory addresses
    uint8_t read(uint16_t a);
    uint16_t read16(uint16_t a);
    uint16_t read16();
    //Pushing 16 bit short onto stack
    void pushSortToStack(uint16_t value);
    uint16_t popShortFromStack();
    void write(uint16_t a, uint8_t d);
    void write16(uint16_t a, uint16_t d);
    //Reset a bit at location x
    uint8_t resetBit(uint8_t b, uint8_t x);
    //set bit b at location x
    uint8_t setBit(uint8_t b, uint8_t x);
    //getter and setter for register F (flags)
    uint8_t getFlag(FLAGSz80 f);
    bool checkBit(uint8_t reg, int pos);
    bool checkBit(uint16_t reg, int pos);
    void setFlag(FLAGSz80 f, bool v);

    //misc variables
    bool enableInterrupts;
    bool halted;
    uint8_t opcode = 0x00; // current opcode instruction
    uint8_t cycles = 0; // current instructions remaining cycles
    uint32_t clock_cycles = 0; // keeps track of how many cycles have occured (for debugging)

    int executeOP(uint8_t opcode);
    int executeExOP(uint8_t opcode); //extended op codes 0xCB 

    //all instructions for opcodes
    //LD nn into n
    void BIT8_LOAD(uint8_t& reg);
   
    //overload for LD r1, r2 - both uint8_t
    void BIT8_LOAD(uint8_t& reg1, uint8_t& reg2);
    //overload for LD r1, r2 - reg 1 uint8 and reg2 as uint16
    //void BIT8_LOAD for memory at addr
    void BIT8_LOAD_MEM(uint8_t& reg1, uint16_t addr);
    //overload for LD r1, r2 - reg 1 as uint16 and reg2 as uint8
    //void BIT8_LOAD(uint16_t& reg1, uint8_t& reg2);
    //void BIT8_LOAD(uint16_t& reg1, uint16_t& reg2);
    void BIT16_LOAD(uint16_t& reg);
    void BIT16_DEC(uint16_t& reg);
    void BIT16_INC(uint16_t& reg);
    void PUSH16(uint16_t b);
    void POP16(uint16_t reg);

    //Add functions
    void ADD_8BIT(uint8_t& reg1, uint8_t reg2, bool c);
    void ADD_16BIT(uint16_t& reg1, uint16_t reg2);
    //sub functions
    void SUB_8BIT(uint8_t& reg1, uint8_t reg2, bool c);

    //AND
    void AND_8BIT(uint8_t& reg1, uint8_t reg2);
    //OR
    void OR_8BIT(uint8_t& reg1, uint8_t reg2);
    //XOR
    void XOR_8BIT(uint8_t& reg1, uint8_t reg2);
    //Compare A sub n, results arent saved?
    void CP_8BIT(uint8_t reg1, uint8_t reg2);
    //Increment register
    void INC_8BIT(uint8_t& reg);
    void INC_8BIT_MEM(uint8_t addr);
    void INC_16BIT(uint16_t& reg);
    //Decrement register
    void DEC_8BIT(uint8_t& reg);
    void DEC_8BIT_MEM(uint8_t addr);
    void DEC_16BIT(uint16_t& reg);

    //SWAP nibbles
    void SWAP_NIBBLES(uint8_t& reg);
    void SWAP_NIB_MEM(uint16_t addr);
    //Decimal adjust after addition
    void DAA();
    //complement A register
    void CPL();
    //complement carry flag
    void CCF();
    void SCF(); //set carry flag
    void JP(); //Jump to address nn
    void JP(int flag, bool cond); //conditional Jump (if flag true/false then jump)
    void JR(); 
    void JR(int flag, bool cond); //jump signed immediate depending on flag cond
    void CALL();//push address of next instruction to stack
    void CALL(int flag, bool cond);
    void RET();
    void RET(int flag, bool cond);
    void RST(uint8_t byte);
    //void RETI();

    //Exop instructions
    void RLC(uint8_t& reg);
    void RLCMem(uint16_t addr);
    void RL(uint8_t& reg);
    void RLMem(uint16_t addr);
    void RRC(uint8_t& reg);
    void RRCMem(uint16_t addr);
    void RR(uint8_t& reg);
    void RRMem(uint16_t addr);
    void SLA(uint8_t& reg);
    void SLAMem(uint16_t addr);
    void SRA(uint8_t& reg);
    void SRAMem(uint16_t addr);
    void SRL(uint8_t& reg);
    void SRLMem(uint16_t addr);

    //test bit at location bit
    void testBit(uint8_t reg, int bit);
    void testBitMem(uint16_t addr, int bit);

    //set bit at location bit
    void SET(uint8_t& reg, int bit);
    void SETMem(uint16_t addr, int bit);

    //reset bit at location bit
    void RES(uint8_t& reg, int bit);
    void RESMem(uint16_t addr, int bit);
};