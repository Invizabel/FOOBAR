#include <cmath>
#include <cstdint>
#include <string.h>
#include <furi.h>
#include <furi_hal.h>
#include <storage/storage.h>

int PC = 0;
int SP = 0;

uint8_t cart_ram[32768];
uint8_t mem[0x2000];
uint8_t ROM[0x200];

uint8_t BC = 228;
uint16_t DE = 259;
uint8_t HL = 0b110;
uint16_t SPr = 260;
uint16_t Immediate = 257;

uint8_t A = 0b111;
uint8_t B = 0b000;
uint8_t C = 0b001;
uint8_t D = 0b010;
uint8_t E = 0b011;
uint8_t H = 0b100;
uint8_t L = 0b101;

uint8_t N;
uint8_t Z;
uint8_t REG[8];

uint8_t ram_bank = 0;
uint8_t ram_bank_offset = ram_bank * 0x2000 - 0xA000;
uint8_t ROMbank = 1; 
uint8_t ROMbankoffset = (ROMbank - 1) * 0x4000;

uint8_t joypad_dpad = 0xef;
uint8_t joypad_buttons = 0xdf;
uint8_t keys_dpad = 0xef;
uint8_t keys_buttons = 0xdf;

uint8_t current_mem_bank = 1;

void calc_mem(uint16_t addr)
{
    uint8_t bank = addr / 0x2000;
    if (bank != current_mem_bank)
    {
        current_mem_bank = bank;
        for (int i = 0; i < 8192; i++)
        {
            mem[i] = 0x00;
        }
    }
}

uint8_t read_mem(uint8_t addr)
{
    if (addr % 0x2000 <= 0x3fff)
    {
        return ROM[addr];
    }
    
    if (addr % 0x2000 <= 0x7fff)
    {
        return ROM[addr + ROMbankoffset];
    }

    if (addr % 0x2000 >= 0xA000 && addr % 0x2000 <= 0xBFFF)
    {
        return cart_ram[addr + ram_bank_offset];
    }

    if (addr % 0x2000 == 0xFF00)
    {
        calc_mem(addr);

        if (mem[0xFF00 % 0x2000] & 0x20)
        {
            return joypad_dpad & keys_dpad;
        }
        else if (mem[0xFF00 % 0x2000] & 0x10)
        {
            return joypad_buttons & keys_buttons;
        }
        else
        {
            return 0xFF;
        }
    }

    return mem[addr % 0x2000];
}

uint8_t ld16(uint8_t a, uint8_t b, uint8_t c)
{
    if (b == Immediate)
    {
        if (a == HL)
        {
            uint8_t s = (read_mem(PC + 1) + (read_mem(PC + 2) << 8));
            REG[H] = (s >> 8) & 0xFF;
            REG[L] = s & 0xFF;
            PC += 3;
            return 12;
        }

        SP = read_mem(PC + 1) + (read_mem(PC + 2) << 8);
        PC += 3;
        return 12;
    }

    if (c == Immediate)
    {
        REG[a] = read_mem(PC + 2);
        REG[b] = read_mem(PC + 1);
        PC += 3;
        return 12;
    }
  
    SP = (REG[H] << 8) + REG[L];
    PC += 1;
    return 8;
}

uint8_t opcodes(uint8_t opcode)
{
    if (opcode == 0x00)
    {
        PC += 1;
        return 4;
    }
    
    if (opcode == 0x01)
    {
        return ld16(B,C,Immediate);
    }

    return 0;
}

int main()
{
    Storage* storage = (Storage*)furi_record_open("storage");    
    File* rom_file = storage_file_alloc(storage);

    uint8_t advance = 0;
    uint8_t offset = 0;
    if(storage_file_open(rom_file, "/ext/kirby.gb", FSAM_READ, FSOM_OPEN_ALWAYS))
    {
        while(true)
        {
            storage_file_seek(rom_file, offset, SEEK_SET);
            uint8_t data = storage_file_read(rom_file, ROM, 1);
            advance = opcodes(data);
            offset += advance;
        }
    }
    
    storage_file_close(rom_file);
    storage_file_free(rom_file);
    furi_record_close("storage");

    return 0;
}
