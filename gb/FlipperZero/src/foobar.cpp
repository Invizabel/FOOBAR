#include <furi.h>
#include <storage/storage.h>

Storage * storage = (Storage*)furi_record_open("storage");  

uint8_t PC = 0;
uint8_t SP = 0;

uint8_t rom_size = 0;

const uint8_t RLC = 1;
const uint8_t RRC = 2;
const uint8_t RL  = 3;
const uint8_t RR  = 4;
const uint8_t SLA = 5;
const uint8_t SRA = 6;
const uint8_t SRL = 7;

bool lcd_enabled = false;
uint8_t lcd_lastmode = 1;
uint8_t lcd_scan = 0;

uint8_t BC = 228;
uint16_t DE = 259;
uint8_t HL = 0b110;
uint16_t SPr = 260;
uint16_t Immediate = 257;

const uint8_t A = 0b111;
const uint8_t B = 0b000;
uint8_t C = 0b001;
const uint8_t D = 0b010;
const uint8_t E = 0b011;
uint8_t H = 0b100;
const uint8_t L = 0b101;

uint8_t N;
uint8_t Z;
uint8_t REG[8];

uint8_t timer_prescaler = 0;
uint8_t timer_length = 1;
uint8_t timer_enable = false;

uint8_t ram_bank = 0;
uint8_t ram_bank_offset = ram_bank * 0x2000 - 0xA000;
uint8_t rom_bank = 1; 
uint8_t rom_bank_offset = (rom_bank - 1) * 0x4000;

uint8_t joypad_dpad = 0xef;
uint8_t joypad_buttons = 0xdf;
uint8_t keys_dpad = 0xef;
uint8_t keys_buttons = 0xdf;

bool ram_enabled = false;
bool sound_enabled = false;
uint8_t mbc_ram_mode = 0;

uint8_t read_rom_now(uint32_t addr)
{
    uint8_t data[1];
      
    File * file = storage_file_alloc(storage);

    if(storage_file_open(file, "/ext/kirby.gb", FSAM_READ, FSOM_OPEN_EXISTING))
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
    
    File * file = storage_file_alloc(storage);

    if(storage_file_open(file, "/ext/cache_system.txt", FSAM_READ, FSOM_OPEN_EXISTING))
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
    
    File * file = storage_file_alloc(storage);

    if(storage_file_open(file, "/ext/cache_cart.txt", FSAM_READ, FSOM_OPEN_EXISTING))
    {
        storage_file_seek(file, addr, SEEK_SET);
        storage_file_read(file, data, 1);
    }
    
    storage_file_close(file);
    storage_file_free(file);

    return data[0];
}

uint8_t write_mem_now(uint32_t addr, uint8_t data)
{   
    File * file = storage_file_alloc(storage);

    if(storage_file_open(file, "/ext/cache_system.txt", FSAM_WRITE, FSOM_OPEN_EXISTING))
    {
        storage_file_seek(file, addr, SEEK_SET);
        storage_file_write(file, &data, sizeof(data));
    }
    
    storage_file_close(file);
    storage_file_free(file);

    return data;
}

uint8_t write_cart_now(uint32_t addr, uint8_t data)
{
    File * file = storage_file_alloc(storage);

    if(storage_file_open(file, "/ext/cache_cart.txt", FSAM_WRITE, FSOM_OPEN_EXISTING))
    {
        storage_file_seek(file, addr, SEEK_SET);
        storage_file_write(file, &data, sizeof(data));
    }
    
    storage_file_close(file);
    storage_file_free(file);

    return data;
}

void write_mem(uint32_t addr, uint8_t data);

void reset_sound_registers()
{
    write_mem_now(0xFF10, 0x80);  // NR10
    write_mem_now(0xFF11, 0xBF);  // NR11
    write_mem_now(0xFF12, 0xF3);  // NR12
    write_mem_now(0xFF13, 0x00);
    write_mem_now(0xFF14, 0xBF);  // NR14
    write_mem_now(0xFF15, 0xFF);  // NA
    write_mem_now(0xFF16, 0x3F);  // NR21
    write_mem_now(0xFF17, 0x00);  // NR22
    write_mem_now(0xFF18, 0x00);
    write_mem_now(0xFF19, 0xBF);  // NR24
    write_mem_now(0xFF1A, 0x7F);  // NR30
    write_mem_now(0xFF1B, 0xFF);  // NR31
    write_mem_now(0xFF1C, 0x9F);  // NR32
    write_mem_now(0xFF1D, 0x00);
    write_mem_now(0xFF1E, 0xBF);  // NR33
    write_mem_now(0xFF1F, 0xFF);  // NA
    write_mem_now(0xFF20, 0xFF);  // NR41
    write_mem_now(0xFF21, 0x00);  // NR42
    write_mem_now(0xFF22, 0x00);  // NR43
    write_mem_now(0xFF23, 0xBF);  // NR30
    write_mem_now(0xFF24, 0x77);  // NR50
    write_mem(0xFF25, 0xF3);      // NR51
    write_mem_now(0xFF26, 0xF1);  // NR52
}

uint8_t read_mem(uint16_t addr)
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

void do_mbc(uint32_t addr, uint8_t data) 
{
    switch (read_rom_now(0x147))   // Cartridge type
    {
        case 0x00: // ROM ONLY
            break;

        case 0x01: // MBC1
        case 0x02: // MBC1+RAM
        case 0x03: // MBC1+RAM+BATTERY
            if (addr <= 0x1FFF)
            {
                ram_enabled = ((data & 0x0F) == 0x0A);
            }

            else if (addr <= 0x3FFF)
            {
                data &= 0x1F;
                if (data == 0)
                {
                    data = 1;
                }

                // Set low 5 bits of ROM bank
                rom_bank = (rom_bank & 0xE0) | data;
                rom_bank_offset = ((rom_bank - 1) * 0x4000) % rom_size;
            }

            else if (addr <= 0x5FFF)
            {
                data &= 0x03;

                if (mbc_ram_mode == 0)
                {
                    // Upper bits of ROM bank
                    rom_bank = (rom_bank & 0x1F) | (data << 5);
                    rom_bank_offset = ((rom_bank - 1) * 0x4000) % rom_size;
                }

                else
                {
                    ram_bank = data;
                    ram_bank_offset = ram_bank * 0x2000 - 0xA000;
                }
            }

            else
            {
                mbc_ram_mode = data & 1;

                if (mbc_ram_mode == 0)
                {
                    ram_bank = 0;
                    ram_bank_offset = ram_bank * 0x2000 - 0xA000;
                }

                else
                {
                    rom_bank &= 0x1F;
                    rom_bank_offset = ((rom_bank - 1) * 0x4000) % rom_size;
                }
            }
            break;

        // MBC2
        case 0x05:
        case 0x06:
            if (addr <= 0x1FFF)
            {
                if ((addr & 0x0100) == 0)
                {
                    ram_enabled = ((data & 0x0F) == 0x0A);
                }
            }
        
            else if (addr <= 0x3FFF)
            {
                data &= 0x0F;
                if (data == 0)
                {
                    data = 1;
                }

                rom_bank = data;
                rom_bank_offset = ((rom_bank - 1) * 0x4000) % rom_size;
            }
            break;

        // MBC3
        case 0x11:
        case 0x12:
        case 0x13:
            if (addr <= 0x1FFF)
            {
                ram_enabled = ((data & 0x0F) == 0x0A);
            }
        
            else if (addr <= 0x3FFF)
            {
                if (data == 0)
                {
                    data = 1;
                }
                
                rom_bank = data & 0x7F;
                rom_bank_offset = ((rom_bank - 1) * 0x4000) % rom_size;
            }
        
            else if (addr <= 0x5FFF)
            {
                if (data < 8)
                {
                    ram_bank = data;
                    ram_bank_offset = ram_bank * 0x2000 - 0xA000;
                }

                else
                {
                    // TODO: RTC registers
                }
            }
        
            else
            {
                // TODO: RTC latch
            }
        
            break;

        // MBC5
        case 0x19:
        case 0x1A:
        case 0x1B:
            if (addr <= 0x1FFF)
            {
                ram_enabled = ((data & 0x0F) == 0x0A);
            }
        
            else if (addr <= 0x2FFF)
            {
                rom_bank &= 0x100;
                rom_bank |= data;

                rom_bank_offset = (rom_bank - 1) * 0x4000;
                while (rom_bank_offset > rom_size)
                {
                    rom_bank_offset -= rom_size;
                }
            }
        
            else if (addr <= 0x3FFF)
            {
                rom_bank &= 0xFF;
                if (data & 1)
                {
                    rom_bank += 0x100;
                }

                rom_bank_offset = (rom_bank - 1) * 0x4000;
                while (rom_bank_offset > rom_size)
                {
                    rom_bank_offset -= rom_size;
                }
            }

            else if (addr <= 0x5FFF)
            {
                ram_bank = data & 0x0F;
                ram_bank_offset = ram_bank * 0x2000 - 0xA000;
            }
            break;
    }
}


void write_mem(uint32_t addr, uint8_t data)
{
    if (addr <= 0x7FFF)
    {
        do_mbc(addr, data);
        return;
    }

    if (addr >= 0xA000 && addr <= 0xBFFF && ram_enabled)
    {
        write_cart_now(addr + ram_bank_offset, data);
        return;
    }

    if (addr == 0xFF04)
    {
        write_mem_now(0xFF04, 0x00);
        return;
    }

    if (addr == 0xFF07)
    {
        timer_enable = ((data & (1 << 2)) != 0);

        uint16_t timer_table[4] = {1024, 16, 64, 256};
        timer_length = timer_table[data & 0x03];

        timer_prescaler = timer_length;

        write_mem_now(addr, 0xF8 | data);
        return;
    }

    //LCD control
    if (addr == 0xFF40)
    {
        uint8_t cc = data & (1 << 7);
        if (lcd_enabled != cc)
        {
            lcd_enabled = !!cc;
             // Disabling the display sets it to mode 1
            if (!lcd_enabled)
            {
                // this should also probably set all pixels to white
                lcd_scan = 0;
                write_mem_now(0xFF41,(read_mem_now(0xFF41) & 0xFC) + 1);
            }
        }
    }

    if (addr == 0xFF41)
    {
        //don't overwrite the lowest two bits (mode)
        write_mem_now(0xFF41, read_mem_now(0xFF41) & 0x3);
        data &= 0xFC;
        write_mem_now(0xFF41, read_mem_now(0xFF41) | 0x80 | data); // BGB has highest bit always set
        return;
    }

    // LY - write causes reset
    if (addr == 0xFF44)
    {
        write_mem_now(0xFF44, 0x00);
        return;
    }

    // FF46 - DMA - DMA Transfer and Start Address (W)
    if (addr == 0xFF46)
    {
        uint8_t st = data << 8;
        for (uint8_t i = 0; i <= 0x9F; i++)
        {
            write_mem_now(0xFE00 + i, read_mem(st + i)); 
        }
        return;
    }
}

int ld16(uint8_t a, uint8_t b, uint8_t c)
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

uint8_t shift_process(uint8_t op, uint8_t a)
{

    uint8_t bit7 = a >> 7;
    uint8_t bit0 = a&1;

    switch (op)
    {
        case RLC: // Rotate byte left, save carry
            a = ((a << 1) & 0xff) + bit7;
            C = !!bit7;
            break;
        
        case RRC: // Rotate byte right, save carry
            a = ((a >> 1) & 0xff) + (bit0 << 7);
            C = !!bit0;
            break;
        
        case RL : // Rotate left through carry
            a = ((a << 1) & 0xff) + C;
            C = !!bit7;
            break;

        case RR : // Rotate right through carry
            a = ((a >> 1) & 0xff) + (C << 7);
            C = !!bit0;
            break;

        case SLA: // Shift left
            a = ((a << 1) & 0xff);
            C = !!bit7;
            break;

        case SRA: // Shift right arithmetic
            a = ((a >> 1) & 0xff) + (bit7 << 7);
            C = !!bit0;
            break;

        case SRL: //Shift right logical
            a = ((a >> 1) & 0xff);
            C = !!bit0;
            break;
    }

    N = false;
    H = false;
    Z = (a & 0xFF) == 0;
    return a;
}

int shift(uint8_t op, uint8_t a)
{
    if (a == HL)
    {
        uint32_t addr = (REG[H] << 8) + REG[L];
        write_mem(addr, shift_process(op, read_mem(addr)));
        PC += 1;
        return 16;
    }

    REG[a] = shift_process(op, REG[a]);
    PC += 1;
    return 8;
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
        return ld16(B,C,Immediate);
    }

    if (opcode == 0x02)
    {
        shift(RLC, E);
    }

    return 0;
}

int main()
{
    uint8_t clear_data = 0x00;
  
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
    
    storage_file_close(system_rom);
    storage_file_free(system_rom);

    if(storage_file_open(cart_rom, "/ext/cache_cart.txt", FSAM_WRITE, FSOM_OPEN_APPEND))
    {
        for (uint32_t i = 0; i < 65536; i++)
        {
            storage_file_write(cart_rom, &clear_data, sizeof(clear_data));
        }
    }
    
    storage_file_close(cart_rom);
    storage_file_free(cart_rom);

    uint8_t offset = 0;
    uint8_t data[1];
    
    if(storage_file_open(rom_file, "/ext/kirby.gb", FSAM_READ, FSOM_OPEN_EXISTING))
    {
        rom_size = storage_file_size(rom_file);
        while(true)
        {
            storage_file_seek(rom_file, PC, SEEK_SET);
            storage_file_read(rom_file, &data, 1);
            opcodes(data[0]);
            if (PC == offset)
            {
                break;
            }

            offset = PC;
        }
    }
    
    storage_file_close(rom_file);
    storage_file_free(rom_file);
    
    furi_record_close("storage");

    return 0;
}
