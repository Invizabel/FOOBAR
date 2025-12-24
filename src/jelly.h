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
    char array[256];
}   BasicArray;

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

typedef struct
{
    uint8_t entry[4];
    uint8_t logo[0x30];

    char title[16];
    uint16_t new_lic_code;
    uint8_t sgb_flag;
    uint8_t type;
    uint8_t rom_size;
    uint8_t ram_size;
    uint8_t dest_code;
    uint8_t lic_code;
    uint8_t version;
    uint8_t checksum;
    uint16_t global_checksum;
} rom_header;

bool cart_load(char * cart);

#ifdef __cplusplus
}
#endif

#endif
