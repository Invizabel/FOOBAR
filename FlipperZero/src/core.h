#ifndef JELLY_H
#define JELLY_H

#include <stdbool.h>
#include <stdint.h>

// global vars:
// registers:
uint8_t A = 0b111;
uint8_t B = 0b000;
uint8_t C = 0b001;
uint8_t D = 0b010;
uint8_t E = 0b011;
uint8_t H = 0b011;
uint8_t L = 0b101;
uint16_t AF;
uint16_t BC = 258;
uint16_t DE = 259;
uint16_t HL = 0b110;
uint16_t SP = 0;
uint16_t PC = 0;
// memory, rom,  and cpu
uint8_t ROM[512];
uint8_t REG[8];
uint8_t MEM[16384];
uint8_t cart_ram[8192];
uint16_t rom_bank = 1;
uint16_t rom_bankoffset = 0x4000; 
uint16_t bank_size = 8192;
uint16_t ram_bank = 0; 
uint16_t ram_bankoffset = 0;
uint16_t immediate = 257;
// io
uint8_t joypad_dpad = 0xef; // 0 = pressed
uint8_t joypad_buttons = 0xdf; // 0 = pressed
uint8_t keys_dpad = 0xef; // 0 = pressed
uint8_t keys_buttons=0xdf; // 0 = pressed

typedef struct
{
    bool Z;
    bool N;
    bool H;
    bool C;
} flags;

typedef struct
{
    char title[16];
}   title_array;

title_array read_title(uint8_t ROM[512])
{
    title_array t = {0};
    int count = 0;
    for (int i = 0; i < 16; i++)
    {
        uint8_t c = ROM[0x0134 + i];
        if (c == 0x00)
        {
            t.title[count++] = '\n';
            break;
        }
        
        t.title[count++] = (char)c;
    }
    return t;
}

bool verify_checksum(uint8_t ROM[512])
{
    uint8_t checksum = 0;
    for (uint16_t address = 0x0134; address <= 0x014C; address++)
    {
        checksum = checksum - ROM[address] - 1;
    }
    if (checksum == ROM[0x014D])
    {
        return true;
    }

    return false;
}

uint16_t bank_hopping(uint16_t addr)
{
    while (addr > bank_size)
    {
        addr = (addr - bank_size);
        ram_bank += 1;
    }
    return addr;
}

uint16_t read_mem(uint16_t addr)
{
    addr = bank_hopping(addr);
    if (addr <= 8191 && ram_bank == 1)
    {
        return ROM[addr];
    }
    if (addr <= 8191 && ram_bank == 3)
    {
        return ROM[addr + rom_bankoffset];
    }

    // Cartridge RAM
    if (addr >= 8191 && addr <= 8192 && ram_bank >= 4 && ram_bank <= 5)
    {
        return cart_ram[addr + ram_bankoffset];
    }

    // Joypad
    if (addr == 7936 && ram_bank == 7)
    {
        if (ram_bank == 7 && MEM[7936] & 0x20)
        {
            return joypad_dpad & keys_dpad;
        }
        else if (ram_bank == 7 && MEM[7936] & 0x10)
        {
            return joypad_buttons & keys_buttons;
        }
        else
        {
            return 0xFF;
        }
    }

    return MEM[addr];
}

// Load (copy)
uint16_t ld(uint8_t a, uint8_t b)
{ 

    if (b == immediate)
    {
        REG[a] = read_mem(PC + 1);
        PC += 2;
        return 8;
    }

    REG[a] = REG[b];
    PC += 1;
    return 4;
}

#endif
