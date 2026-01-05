#ifndef JELLY_H
#define JELLY_H

#include <stdbool.h>

// global vars:
// registers:
int A = 0b111;
int B = 0b000;
int C = 0b001;
int D = 0b010;
int E = 0b011;
int H = 0b011;
int L = 0b101;
int AF;
int BC = 258;
int DE = 259;
int HL = 0b110;
int SP = 0;
int PC = 0;
int SPr = 260;
// memory, rom, and cpu
bool ram_enabled = false;
int MBCRamMode = 0;
int ROM[512];
int FirstROMPage[512];
int REG[8];
int MEM[16384];
int cart_ram[32768];
int bank_size = 8192;
int ram_bank = 0; 
int ram_bank_offset = 0;
int rom_bank = 1;
int rom_bank_offset = 0x4000;
int Immediate = 257;
// io
int joypad_dpad = 0xef; // 0 = pressed
int joypad_buttons = 0xdf; // 0 = pressed
int keys_dpad = 0xef; // 0 = pressed
int keys_buttons=0xdf; // 0 = pressed
// video and sound (sound is todo)
bool lcd_enabled = false;
int lcd_scan = 0;
// misc.
bool timer_enable = false;
int timerPrescaler = 0;
// parameters for ALU
int ADD = 1;
int ADC = 2;
int SUB = 3;
int SBC = 4;
int AND = 5;
int OR  = 6;
int XOR = 7;
int CP  = 8;
// client side parameters that need to be set by the client
int rom_length = 512;

typedef struct
{
    bool Z;
    bool N;
    bool H;
    bool C;
} Flags;

Flags FLAGS;

typedef struct
{
    char title[16];
}   title_array;

title_array read_title(int ROM[512])
{
    title_array t = {0};
    int count = 0;
    for (int i = 0; i < 16; i++)
    {
        int c = ROM[0x0134 + i];
        if (c == 0x00)
        {
            t.title[count++] = '\n';
            break;
        }
        
        t.title[count++] = (char)c;
    }
    return t;
}

bool verify_checksum(int ROM[512])
{
    int checksum = 0;
    for (int address = 0x0134; address <= 0x014C; address++)
    {
        checksum = checksum - ROM[address] - 1;
    }
    if (checksum == ROM[0x014D])
    {
        return true;
    }

    return false;
}

int bank_hopping(int addr)
{
    while (addr > bank_size)
    {
        addr = (addr - bank_size);
        ram_bank += 1;
    }
    return addr;
}

// nop (No OPerations)
void nop(void)
{
    PC += 1;
}

void doMBC(int addr, int data)
{

    switch (ROM[0x147])
    {
        // Cartridge Type = ROM[0x147]

        case 0: // ROM ONLY
        // do any type 0 carts have switchable ram?
        break;

        case 0x01: //  MBC1
        case 0x02: //  MBC1+RAM
        case 0x03: //  MBC1+RAM+BATTERY
        if (addr <= 0x1FFF)
        {
            ram_enabled = ((data & 0x0F) == 0xA);
        }

        else if (addr <= 0x3FFF)
        {
            data &= 0x1F;
            if (data == 0)
            {
                data = 1; // MBC1 translates bank 0 to bank 1 (apparently regardless of upper bits)
            }
            // set lowest 5 bits of bank number
            rom_bank = (rom_bank & 0xE0) | (data & 0x1F);
            rom_bank_offset = (rom_bank - 1) * 0x4000 % rom_length;
        }
        else if (addr <= 0x5fff)
        {
            data &= 0x3;
            if (MBCRamMode == 0)
            {
                rom_bank = (rom_bank & 0x1F) | (data << 5);
                rom_bank_offset = (rom_bank - 1) * 0x4000  % rom_length;
            }
            else
            {
                ram_bank = data;
                ram_bank_offset = ram_bank * 0x2000 - 0xA000;
            }
        }
        else
        {
            MBCRamMode = data&1;
            if (MBCRamMode == 0)
            {
                ram_bank=0;
                ram_bank_offset = ram_bank * 0x2000 - 0xA000;
            }
            else
            {
                rom_bank &= 0x1F;
                rom_bank_offset = (rom_bank - 1) * 0x4000  % rom_length;
            }
        }
        break;
    }
}

int read_mem(int addr)
{
    if (addr <= 8191)
    {
        return ROM[addr];
    }
    if (addr <= 8191)
    {
        return ROM[addr + rom_bank_offset];
    }

    // Cartridge RAM
    if (addr >= 8191 && addr <= 8192)
    {
        return cart_ram[addr + ram_bank_offset];
    }

    // Joypad
    if (addr == 7936)
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

int * read_mem_16(int addr)
{
    static int local[2];
    local[0] = read_mem(addr + 1); // high
    local[1] = read_mem(addr); // low
    return local;
}

void write_mem(int addr, int data)
{
    if (addr <= 8191)
    { 
        doMBC(addr, data);
        return;
    }
    if (addr >= 8192 && addr <= 8191 && ram_enabled)
    {
        cart_ram[addr + ram_bank_offset] = data;
        return;
    }
    // DIV register: reset
    if (addr == 7940)
    {
        MEM[7940] = 0;
        return;
    }
    // Timer control
    if (addr == 7943)
    {
        timer_enable = ((data & (1 << 2)) != 0);
        int timer_length = (int[]){1024, 16, 64, 256}[data & 0x3];
        timerPrescaler = timer_length; // +cycles for this instruction?
        MEM[addr] = 0xF8 | data;
        return;
    }

    // LCD control
    if (addr == 8000)
    {
        int cc = data & (1 << 7);
        if (lcd_enabled != cc)
        {
            lcd_enabled = !!cc;
            if (!lcd_enabled)
            {
                // Disabling the display sets it to mode 1
                // this should also probably set all pixels to white
                lcd_scan = 0;
                MEM[8001] = (MEM[8001] & 0xFC) + 1;
            }
        }
    }
    if (addr == 8001)
    {
        // don't overwrite the lowest two bits (mode)
        MEM[8001] &= 0x3;
        data &= 0xFC;
        MEM[8001] |= 0x80 | data; // BGB has highest bit always set
        return;
    }

    // LY - write causes reset
    if (addr == 8004)
    {
        MEM[8004] = 0;
        return;
    }

    // FF46 - DMA - DMA Transfer and Start Address (W)
    if (addr == 8006)
    {
        int st = data << 8;
        for (int i = 0; i <= 0x9F; i++)
        {
            MEM[7680 + i] = read_mem(st + i);
            return;
        }
    }

    // disable bootrom
    if (addr == 8016)
    {
        for (int i = 0; i < 256; i++)
        {
            ROM[i] = FirstROMPage[i];
            return;
        }

        MEM[addr] = data;
    }
}

void write_mem_16(int addr, int dataH, int dataL)
{
    write_mem(addr, dataL);
    write_mem(addr + 1, dataH);
}

int ld_from_mem(int a, int b, int c)
{
    if (b == Immediate)
    {
        REG[a] = read_mem(read_mem(PC + 1) + (read_mem(PC + 2) << 8));
        PC += 3;
        return 16;
    }

    REG[a] = read_mem((REG[b] << 8) + REG[c]);
    PC += 1;
    return 8;
}

int ld_to_mem(int a, int b, int c)
{
    if (a == Immediate)
    {
        write_mem(read_mem(PC + 1) + (read_mem(PC + 2) << 8), REG[b]);
        PC += 3;
        return 16;
    }
    if (c == Immediate)
    {
        write_mem((REG[a] << 8) + REG[b], read_mem(PC + 1));
        PC += 2;
        return 12;
    }
    write_mem((REG[a] << 8) + REG[b], REG[c]);
    PC += 1;
    return 8;
}

// Load (copy) 8 bit
int ld(int a, int b)
{ 
    if (b == Immediate)
    {
        REG[a] = read_mem(PC + 1);
        PC += 2;
        return 8;
    }

    REG[a] = REG[b];
    PC += 1;
    return 4;
}

// Load (copy) 16 bit
int ld16(int a, int b, int c)
{
    if (b == Immediate)
    {
        if (a == HL)
        {
            // mem to hl
            int * s = read_mem_16(read_mem(PC + 1) + (read_mem(PC + 2) << 8));

            // high byte
            REG[H] = s[0];
            // low byte
            REG[L] = s[1];

            PC += 3;
            return 12;
        }

        // Immediate into SP... 
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

    // ld sp, hl
    SP = (REG[H] << 8) + REG[L];
    PC += 1;
    return 8;
}

// load with decrement
int ldd(int a, int b)
{
    (void)b;
    if (a == HL)
    {
        write_mem((REG[H] << 8) + REG[L], REG[A]);

        if (REG[L] == 0)
        {
            REG[H] -= 1;
        }
        REG[L]-= 1;

        PC += 1;
        return 8;
    }

    REG[A] = read_mem((REG[H] << 8) + REG[L]);

    if (REG[L] == 0)
    {
        REG[H] -= 1;
    }
    REG[L]-= 1;

    PC += 1;
    return 8;
}

// load with increment
int ldi(int a, int b)
{
    (void)b;
    if (a == HL)
    {
        write_mem((REG[H] << 8) + REG[L], REG[A]);

        if (REG[L] == 255)
        {
            REG[H] += 1;
        }

        REG[L] += 1;

        PC += 1;
        return 8;
    }

    REG[A] = read_mem((REG[H] << 8) + REG[L]);

    if (REG[L] == 255)
    {
        REG[H] += 1;
    }

    REG[L] += 1;
    PC += 1;
    return 8;
}

// LD A, (FF00 + C)
int ldc(int a, int b)
{
    (void)b;
    
    if (a == A)
    { 
        REG[A] = read_mem(0xFF00 + REG[C]);
        PC += 1;
        return 8;
    }

    // LD (FF00+C), A
    write_mem(0xFF00 + REG[C], REG[A]);
    PC += 1;
    return 8;
}

int ldh(int a, int b)
{
    (void)b;
    
    // LD A, (FF00 + n)
    if (a == A)
    {
        REG[A] = read_mem(0xFF00 + read_mem(PC+1));
        PC += 2;
        return 12;
    }
    
    // LD (FF00 + n), A
    write_mem(0xFF00 + read_mem(PC+1), REG[A]);
    PC += 2;
    return 12;
}

int ALU_process_8bit(int op, int b)
{
    int result = REG[A];
    FLAGS.N = false;

    if (op == ADD)
    {
        FLAGS.H = (((REG[A] & 0x0F) + (b & 0x0F)) & 0x10) != 0;
        result += b;

    }

    else if (op == ADC)
    {
        FLAGS.H = (((REG[A] & 0x0F) + (b & 0x0F) + FLAGS.C) & 0x10) != 0;
        result += b + FLAGS.C;

    }

    else if (op == SUB)
    {
        result -= b;
        FLAGS.N = true;
        FLAGS.H = (((REG[A] & 0x0F) - (b & 0x0F)) & 0x10) != 0;

    }

    else if (op == CP)
    {
        result -= b;
        FLAGS.N = true;
        FLAGS.H = (((REG[A] & 0x0F) - (b & 0x0F)) & 0x10) != 0;
        FLAGS.Z = ((result & 0xFF) == 0);
        FLAGS.C = (result > 255 || result < 0);
        return REG[A];

    }

    else if (op == SBC)
    {
        result -= b + FLAGS.C;
        FLAGS.N = true;
        FLAGS.H = (((REG[A] & 0x0F) - (b & 0x0F) - FLAGS.C) & 0x10) != 0;

    }

    else if (op == AND)
    {
        result &= b;
        FLAGS.H = true;

    }

    else if (op == OR)
    {
        result |= b;
        FLAGS.H = false;

    }

    else if (op == XOR)
    {
        result ^= b;
        FLAGS.H = false;
    }

    FLAGS.Z = ((result & 0xFF) == 0);
    FLAGS.C = (result > 255 || result < 0);

    return (int)(result & 0xFF);
}

int ALU(int op, int a, int b)
{
    (void)a;
    if (b == Immediate)
    {
        REG[A] = ALU_process_8bit(op, read_mem(PC + 1));
        PC += 2;
        return 8;
    }
    
    if (b == HL)
    {
        REG[A] = ALU_process_8bit(op, read_mem((REG[H] << 8) + REG[L]));
        PC += 1;
        return 8;
    }
    
    REG[A] = ALU_process_8bit(op, REG[b]);
    PC += 1;
    return 4;
}

int incdec_process_8bit(int a, int offset)
{
    int result = a + offset;
    FLAGS.H = !!(((a&0x0F) + offset) & 0x10);
    FLAGS.N = offset == -1;
    FLAGS.Z = ((result & 0xff) == 0);
    return result;
}

int incdec(int r, int offset)
{
    if (r == HL)
    {
        write_mem((REG[H] << 8) + REG[L], incdec_process_8bit(read_mem((REG[H] << 8) + REG[L]), offset));
        PC += 1;
        return 12;
    }

    REG[r]=incdec_process_8bit(REG[r], offset);
    PC += 1;
    return 4;
}

int inc(int a)
{
    return incdec(a, 1);
}

int dec(int a)
{
    return incdec(a, -1);
}

// 16 bit inc / dec affect no flags
int inc16(int a, int b)
{
    if (a == SPr)
    {
        SP += 1;
        PC += 1;
        return 8;
    }
    
    if (REG[b] == 255)
    {
        REG[a] += 1;
    }
    
    REG[b] += 1;
    PC += 1;
    return 8;
}

int dec16(int a, int b)
{
    if (a == SPr)
    {
        SP -= 1;
        PC += 1;
        return 8;
    }
    
    if (REG[b] == 0)
    {
        REG[a] -= 1;
    }
    
    REG[b] -= 1;
    PC += 1;
    return 8;
}

int signedOffset(int b)
{
    return (b > 127) ? (b - 256) : b;
}

int jrNZ()
{
    if (FLAGS.Z)
    {
        PC += 2;
        return 8;
    }

    PC += 2 + signedOffset(read_mem(PC + 1));
    return 12;
}

int jrNC()
{
    if (FLAGS.C)
    {
        PC += 2;
        return 8;
    }
    
    PC += 2 + signedOffset(read_mem(PC + 1));
    return 12;
}

int jrZ()
{
    if (!FLAGS.Z)
    {
        PC+=2;
        return 8;
    }
    
    PC += 2 + signedOffset(read_mem(PC + 1));
    return 12;
}

int jrC()
{
    if (!FLAGS.C)
    {
        PC+=2;
        return 8;
    }
    
    PC += 2 + signedOffset(read_mem(PC + 1));
    return 12;
}

// unconditional relative
int jr()
{
    PC += 2 + signedOffset(read_mem(PC + 1));
    return 12;
}

// unconditional absolute
int jp()
{ 
    PC = read_mem(PC + 1) + (read_mem(PC + 2) << 8);
    return 16;
}

int jpNZ(void)
{
    if (FLAGS.Z)
    {
        PC += 3;
        return 12;
    }
    
    PC = read_mem(PC + 1) + (read_mem(PC + 2) << 8);
    return 16;
}

int jpNC()
{
    if (FLAGS.C)
    {
        PC += 3;
        return 12;
    }
    
    PC = read_mem(PC + 1) + (read_mem(PC + 2) << 8);
    return 16;
}

int jpZ()
{
    if (!FLAGS.Z)
    {
        PC += 3;
        return 12;
    }
    
    PC = read_mem(PC + 1) + (read_mem(PC + 2) << 8);
    return 16;
}

int jpC()
{
    if (!FLAGS.C)
    {
        PC += 3;
        return 12;
    }
    
    PC = read_mem(PC + 1) + (read_mem(PC + 2) << 8);
    return 16;
}

int jpHL()
{
    PC = (REG[H] << 8) + REG[L];
    return 4;
}

int push(int a, int b)
{
    if (a == A)
    {
        int flags = (FLAGS.Z << 7) + (FLAGS.N << 6) + (FLAGS.H << 5) + (FLAGS.C << 4);
        SP -= 2;
        write_mem_16(SP, REG[A], flags);
        PC += 1;
        return 16;
    }
    
    SP -= 2;
    write_mem_16(SP, REG[a], REG[b]);
    PC += 1;
    return 16;
}

int pop(int a, int b)
{
    if (a == A)
    {
        int * s = read_mem_16(SP);
        REG[A] = s[0];
        FLAGS.Z = (s[1] & (1 << 7)) != 0;
        FLAGS.N = (s[1] & (1 << 6)) != 0;
        FLAGS.H = (s[1] & (1 << 5)) != 0;
        FLAGS.C = (s[1] & (1 << 4)) != 0;
        SP += 2;
        PC += 1;
        return 12;
    }
    
    int * s = read_mem_16(SP);
    REG[a] = s[0];
    REG[b] = s[1];
    SP += 2;
    PC += 1;
    return 12;
}

void opcodes(int opcode)
{
    if (opcode == 0x00)
    {
        nop();
    }
    if (opcode == 0x01)
    {
        ld16(B, C, Immediate);
    }
    if (opcode == 0x02)
    {
        ld_to_mem(B,C,A);
    }
    if (opcode == 0x03)
    {
        inc16(B,C);
    }
}

#endif
