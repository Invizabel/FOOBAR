#include <cmath>
#include <cstdint>
#include <string.h>
#include <furi.h>
#include <furi_hal.h>
#include <storage/storage.h>

uint32_t PC = 0;
uint32_t SP = 0;

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
uint8_t rom_bank = 1; 
uint8_t rom_bank_offset = (rom_bank - 1) * 0x4000;

uint8_t joypad_dpad = 0xef;
uint8_t joypad_buttons = 0xdf;
uint8_t keys_dpad = 0xef;
uint8_t keys_buttons = 0xdf;

uint8_t read_rom_now(uint32_t addr)
{
    uint8_t data[1];
    
    Storage * storage = (Storage*)furi_record_open("storage");    
    File * file = storage_file_alloc(storage);

    if(storage_file_open(file, "/ext/kirby.gb", FSAM_READ, FSOM_OPEN_ALWAYS))
    {
        storage_file_seek(file, addr, SEEK_SET);
        storage_file_read(file, data, 1);
    }
    
    storage_file_close(file);
    storage_file_free(file);

    return data[0];
}

uint8_t read_mem_now(uint32_t addr)
{
    
    uint8_t data[1];
    
    Storage * storage = (Storage*)furi_record_open("storage");    
    File * file = storage_file_alloc(storage);

    if(storage_file_open(file, "/ext/cache_system.txt", FSAM_READ, FSOM_OPEN_ALWAYS))
    {
        storage_file_seek(file, addr, SEEK_SET);
        storage_file_read(file, data, 1);
    }
    
    storage_file_close(file);
    storage_file_free(file);

    return data[0];
}

uint8_t read_cart_now(uint32_t addr)
{
    uint8_t data[1];
    
    Storage * storage = (Storage*)furi_record_open("storage");    
    File * file = storage_file_alloc(storage);

    if(storage_file_open(file, "/ext/cache_cart.txt", FSAM_READ, FSOM_OPEN_ALWAYS))
    {
        storage_file_seek(file, addr, SEEK_SET);
        storage_file_read(file, data, 1);
    }
    
    storage_file_close(file);
    storage_file_free(file);

    return data[0];
}

uint8_t read_mem(uint32_t addr)
{
    if (addr <= 0x3fff)
    {
        return read_rom_now(addr);
    }
    
    if (addr <= 0x7fff)
    {
        return read_rom_now(addr + rom_bank_offset);
    }

    if (addr >= 0xA000 && addr <= 0xBFFF)
    {
        return read_cart_now(addr + ram_bank_offset);
    }

    if (addr == 0xFF00)
    {
        if (read_mem_now(0xFF00) & 0x20)
        {
            return joypad_dpad & keys_dpad;
        }
        else if (read_mem_now(0xFF00) & 0x10)
        {
            return joypad_buttons & keys_buttons;
        }
        else
        {
            return 0xFF;
        }
    }

    return read_mem_now(addr);
}

uint8_t ld16(uint8_t a, uint8_t b, uint8_t c)
{
    if (b == Immediate)
    {
        if (a == HL)
        {
            uint16_t s = (read_mem(PC + 1) + (read_mem(PC + 2) << 8));
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
    uint8_t clear_data = 0x00;

    Storage * storage = (Storage*)furi_record_open("storage");    
    File * rom_file = storage_file_alloc(storage);
    File * system_rom = storage_file_alloc(storage);
    File * cart_rom = storage_file_alloc(storage);

    if (storage_file_exists(storage, "/ext/cache_system.txt"))
    {
        storage_common_remove(storage, "/ext/cache_system.txt");
    }

    if (storage_file_exists(storage, "/ext/cache_cart.txt"))
    {
        storage_common_remove(storage, "/ext/cache_cart.txt");
    }

    if(storage_file_open(system_rom, "/ext/cache_system.txt", FSAM_WRITE, FSOM_OPEN_APPEND))
    {
        for (uint16_t i = 0; i < 32768; i++)
        {
            storage_file_write(system_rom, &clear_data, sizeof(clear_data));
        }
    }

    if(storage_file_open(cart_rom, "/ext/cache_cart.txt", FSAM_WRITE, FSOM_OPEN_APPEND))
    {
        for (uint32_t i = 0; i < 65536; i++)
        {
            storage_file_write(cart_rom, &clear_data, sizeof(clear_data));
        }
    }

    uint8_t advance = 0;
    uint8_t offset = 0;
    uint8_t data[1];
    
    if(storage_file_open(rom_file, "/ext/kirby.gb", FSAM_READ, FSOM_OPEN_ALWAYS))
    {
        while(true)
        {
            storage_file_seek(rom_file, offset, SEEK_SET);
            storage_file_read(rom_file, data, 1);
            advance = opcodes(data[0]);
            offset += advance;
            break;
        }
    }
    
    storage_file_close(rom_file);
    storage_file_free(rom_file);
    storage_file_close(system_rom);
    storage_file_free(system_rom);
    storage_file_close(cart_rom);
    storage_file_free(cart_rom);
    furi_record_close("storage");

    return 0;
}
