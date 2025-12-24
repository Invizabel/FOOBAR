#ifndef JELLY_H
#define JELLY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct
{
    char title[16];
}   title_array;

typedef struct
{
    uint8_t a;
    uint8_t f;
    uint8_t b;
    uint8_t c;
    uint8_t d;
    uint8_t e;
    uint8_t h;
    uint8_t l;
    uint16_t pc;
    uint16_t sp;
} cpu_registers;

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

#ifdef __cplusplus
}
#endif

#endif
