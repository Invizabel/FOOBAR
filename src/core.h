#ifndef JELLY_H
#define JELLY_H

#include <stdbool.h>
#include <stdint.h>

// global vars:
// Registers:
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

typedef struct
{
    bool Z = false;
    bool N = false;
    bool H = false;
    bool C = false;
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

// Load (copy)
int ld(a, b)
{ 

if (b == Immediate)
{
    REG[a] = readMem(PC + 1);
    P C+= 2;
    return 8;
}

    REG[a] = REG[b];
    PC += 1;
    return 4;
}

#endif
