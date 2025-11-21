#include <cmath>
#include <cstdint>
#include <string.h>
#include <furi.h>
#include <furi_hal.h>
#include <storage/storage.h>

int PC = 0;
int SP = 0;

int BC = 228;
int DE = 259;
int HL = 0b110;
int SPr = 260;
int Immediate = 257;

int A = 0b111;
int B = 0b000;
int C = 0b001;
int D = 0b010;
int E = 0b011;
int H = 0b100;
int L = 0b101;

int N;
int Z;
int REG[8];

int ram_bank = 0;
int ram_bank_offset = ram_bank * 0x2000 - 0xA000;
int rom_bank = 1; 
int rom_bank_offset = (rom_bank - 1) * 0x4000;

int joypad_dpad = 0xef;
int joypad_buttons = 0xdf;
int keys_dpad = 0xef;
int keys_buttons = 0xdf;

uint8_t opcodes(uint8_t opcode)
{
    if (opcode == 0x00)
    {
        PC += 1;
        return 4;
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
        for (int i = 0; i < 32768; i++)
        {
            storage_file_write(system_rom, &clear_data, sizeof(clear_data));
        }
    }

    if(storage_file_open(cart_rom, "/ext/cache_cart.txt", FSAM_WRITE, FSOM_OPEN_APPEND))
    {
        for (int i = 0; i < 65536; i++)
        {
            storage_file_write(cart_rom, &clear_data, sizeof(clear_data));
        }
    }

    uint8_t advance = 0;
    uint8_t offset = 0;
    uint8_t ROM[1];
    
    if(storage_file_open(rom_file, "/ext/kirby.gb", FSAM_READ, FSOM_OPEN_ALWAYS))
    {
        while(true)
        {
            storage_file_seek(rom_file, offset, SEEK_SET);
            uint8_t data = storage_file_read(rom_file, ROM, 1);
            advance = opcodes(data);
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
