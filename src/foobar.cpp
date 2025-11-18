#include <cstdint>
#include <string.h>
#include <furi.h>
#include <furi_hal.h>
#include <storage/storage.h>

int PC = 0;
int SP = 0;

uint8_t cart_ram[32768];
uint8_t mem[65536];
uint8_t ROM[0x4000];

uint16_t BC = 228;
uint16_t DE = 259;
uint16_t HL = 0b110;
uint16_t SPr = 260;
uint16_t Immediate = 257;

uint8_t A = 0b111;
uint8_t B = 0b000;
uint8_t C = 0b001;
uint8_t D = 0b010;
uint8_t E = 0b011;
uint16_t H = 0b100;
uint16_t L = 0b101;

uint8_t N;
uint8_t Z;
uint8_t REG[8];

uint8_t joypad_dpad = 0xef;
uint8_t joypad_buttons = 0xdf;
uint8_t keys_dpad = 0xef;
uint8_t keys_buttons = 0xdf;

void write(uint16_t addr, uint8_t value)
{
    cart_ram[addr] = value;
}

uint8_t read_mem(uint8_t addr)
{
    return ROM[addr];
}

int opcodes(uint8_t opcode)
{
    if (opcode == 0x00)
    {
        PC += 1;
        return 4;
    }
    
    if (opcode == 0x01)
    {
        uint8_t a = B;
        uint8_t b = C;
        uint16_t c = Immediate;
        
        if (B == Immediate)
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

            PC += 12;
            return 12;
        }

        SP = (REG[H] << 8) + REG[L];
        PC += 1;
        return 8;
    }

    return 0;
}

int main()
{
    Storage* storage = (Storage*)furi_record_open("storage");
    
    File* rom_file = storage_file_alloc(storage);

    size_t offset = 0;
    while(true)
    {
        if(storage_file_open(rom_file, "/ext/kirby.gb", FSAM_READ, FSOM_OPEN_ALWAYS))
        {
            if(!storage_file_seek(rom_file, offset, SEEK_SET))
            {
                break;
            }

        
            if(!storage_file_read(rom_file, ROM, 16384))
            {
                break;
            }

            if (sizeof(ROM) <= 0)
            {
                break;
            }

            for (int i = 0; i < 16384; i++)
            {
                opcodes(ROM[i+offset]);
            }

            offset += 16384;
        }
    }
    
    storage_file_close(rom_file);
    storage_file_free(rom_file);
    furi_record_close("storage");
}
