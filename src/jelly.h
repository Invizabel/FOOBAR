#ifndef JELLY_H
#define JELLY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <cstddef>
#include <stdint.h>

int REG[8];

struct {
    bool Z;
    bool N;
    bool H;
    bool C;
} FLAGS = { false, false, false, false };

int PC = 0;
int SP = 0;

bool IME = false; // Interrupt master enable
bool cpu_halted = false;

uint8_t A = 0b111;
uint8_t B = 0b000;
uint8_t C = 0b001;
uint8_t D = 0b010;
uint8_t E = 0b011;
uint8_t H = 0b100;
uint8_t L = 0b101;

uint8_t MEM[0x10000];
uint8_t *FirstROMPage = NULL;
uint8_t ROM[512];
int ROMbank = 1;
uint32_t ROMbankoffset = (ROMbank-1)*0x4000;
uint8_t cartRAM[0x8000];
int RAMbank = 0;
int RAMbankoffset = RAMbank*0x2000 - 0xA000;
bool RAMenabled = false;
int MBCRamMode = 0;
int divPrescaler = 0;
int timerPrescaler = 0;
int timerLength = 1;
bool timerEnable = false;
bool LCD_enabled = false;
int LCD_lastmode = 1;
int LCD_scan = 0;
uint8_t joypad_dpad = 0xef;
uint8_t joypad_buttons = 0xdf;
uint8_t keys_dpad = 0xef;
uint8_t keys_buttons = 0xdf;

uint8_t HL = 0b110;
uint16_t Immediate = 257;
const int BC  = 258;
const int DE  = 259;
const int SPr = 260;

const int RLC = 1;
const int RRC = 2;
const int RL  = 3;
const int RR  = 4;
const int SLA = 5;
const int SRA = 6;
const int SRL = 7;

static int (*opcodes[256])(void);

// actual functions
int nop(void)
{
    PC++;
    return 4;
}

void doMBC(unsigned int addr, uint8_t data)
{
    if (ROM[0x147] == 0)
    {
    }
    else if (ROM[0x147] == 0x01 || ROM[0x147] == 0x02 || ROM[0x147] == 0x03)
    {
        if (addr <= 0x1FFF)
        {
            RAMenabled = ((data & 0x0F) == 0xA);
        }
        else if (addr <= 0x3FFF)
        {
            data &= 0x1F;
            if (data == 0) data = 1;
            ROMbank = (ROMbank & 0xE0) | (data & 0x1F);
            ROMbankoffset = ((ROMbank - 1) * 0x4000) % sizeof(ROM) / sizeof(ROM[0]);
        }
        else if (addr <= 0x5FFF)
        {
            data &= 0x03;
            if (MBCRamMode == 0)
            {
                ROMbank = (ROMbank & 0x1F) | (data << 5);
                ROMbankoffset = ((ROMbank - 1) * 0x4000) % sizeof(ROM) / sizeof(ROM[0]);
            }
            else
            {
                RAMbank = data;
                RAMbankoffset = (RAMbank * 0x2000) - 0xA000;
            }
        }
        else
        {
            MBCRamMode = data & 1;
            if (MBCRamMode == 0)
            {
                RAMbank = 0;
                RAMbankoffset = (RAMbank * 0x2000) - 0xA000;
            }
            else
            {
                ROMbank &= 0x1F;
                ROMbankoffset = ((ROMbank - 1) * 0x4000) % sizeof(ROM) / sizeof(ROM[0]);
            }
        }
    }
    else if (ROM[0x147] == 0x05 || ROM[0x147] == 0x06)
    {
        if (addr <= 0x1FFF)
        {
            if ((addr & 0x0100) == 0)
                RAMenabled = ((data & 0x0F) == 0xA);
        }
        else if (addr <= 0x3FFF)
        {
            data &= 0x0F;
            if (data == 0) data = 1;
            ROMbank = data;
            ROMbankoffset = ((ROMbank - 1) * 0x4000) % sizeof(ROM) / sizeof(ROM[0]);
        }
    }
    else if (ROM[0x147] == 0x11 || ROM[0x147] == 0x12 || ROM[0x147] == 0x13)
    {
        if (addr <= 0x1FFF)
        {
            RAMenabled = ((data & 0x0F) == 0xA);
        }
        else if (addr <= 0x3FFF)
        {
            if (data == 0) data = 1;
            ROMbank = data & 0x7F;
            ROMbankoffset = ((ROMbank - 1) * 0x4000) % sizeof(ROM) / sizeof(ROM[0]);
        }
        else if (addr <= 0x5FFF)
        {
            if (data < 8)
            {
                RAMbank = data;
                RAMbankoffset = (RAMbank * 0x2000) - 0xA000;
            }
        }
        else
        {
        }
    }
    else if (ROM[0x147] == 0x19 || ROM[0x147] == 0x1A || ROM[0x147] == 0x1B)
    {
        if (addr <= 0x1FFF)
        {
            RAMenabled = ((data & 0x0F) == 0xA);
        }
        else if (addr <= 0x2FFF)
        {
            ROMbank &= 0x100;
            ROMbank |= data;
            ROMbankoffset = ((ROMbank - 1) * 0x4000);
            while (ROMbankoffset > sizeof(ROM) / sizeof(ROM[0])) ROMbankoffset -= sizeof(ROM) / sizeof(ROM[0]);
        }
        else if (addr <= 0x3FFF)
        {
            ROMbank &= 0xFF;
            if (data & 1) ROMbank += 0x100;
            ROMbankoffset = ((ROMbank - 1) * 0x4000);
            while (ROMbankoffset > sizeof(ROM) / sizeof(ROM[0])) ROMbankoffset -= sizeof(ROM) / sizeof(ROM[0]);
        }
        else if (addr <= 0x5FFF)
        {
            RAMbank = data & 0x0F;
            RAMbankoffset = (RAMbank * 0x2000) - 0xA000;
        }
    }
}

int readMem(int addr)
{
    if (addr <= 0x3FFF) return ROM[addr];
    if (addr <= 0x7FFF) return ROM[addr + ROMbankoffset];
    if (addr >= 0xA000 && addr <= 0xBFFF) return cartRAM[addr + RAMbankoffset];
    if (addr == 0xFF00)
    {
        if (MEM[0xFF00] & 0x20) return joypad_dpad & keys_dpad;
        if (MEM[0xFF00] & 0x10) return joypad_buttons & keys_buttons;
        return 0xFF;
    }
    return MEM[addr];
}

void writeMem(uint16_t addr, uint16_t data)
{
    if (addr <= 0x7fff)
    {
        doMBC(addr, data);
        return;
    }
    if (addr >= 0xA000 && addr <= 0xBFFF && RAMenabled)
    {
        cartRAM[addr + RAMbankoffset] = data;
        return;
    }
    if (addr == 0xFF04)
    {
        MEM[0xFF04] = 0;
        return;
    }
    if (addr == 0xFF07)
    {
        timerEnable = (data & (1 << 2)) != 0;
        static const unsigned int lengths[4] = {1024, 16, 64, 256};
        timerLength = lengths[data & 0x3];
        timerPrescaler = timerLength;
        MEM[addr] = 0xF8 | data;
        return;
    }

    // LCD control

    if (addr <= 0x7fff)
    {
        doMBC(addr, data);
        return;
    }
    if (addr >= 0xA000 && addr <= 0xBFFF && RAMenabled)
    {
        cartRAM[addr + RAMbankoffset] = data;
        return;
    }
    if (addr == 0xFF04)
    {
        MEM[0xFF04] = 0;
        return;
    }
    if (addr == 0xFF07)
    {
        timerEnable = (data & (1 << 2)) != 0;
        static const unsigned int lengths[4] = {1024, 16, 64, 256};
        timerLength = lengths[data & 0x3];
        timerPrescaler = timerLength;
        MEM[addr] = 0xF8 | data;
        return;
    }
    if (addr == 0xFF40)
    {
        uint8_t cc = data & (1 << 7);
        if (LCD_enabled != cc)
        {
            LCD_enabled = !!cc;
            if (!LCD_enabled)
            {
                LCD_scan = 0;
                MEM[0xFF41] = (MEM[0xFF41] & 0xFC) + 1;
            }
        }
    }
    if (addr == 0xFF41)
    {
        MEM[0xFF41] &= 0x03;
        data &= 0xFC;
        MEM[0xFF41] |= 0x80 | data;
        return;
    }
    if (addr == 0xFF44)
    {
        MEM[0xFF44] = 0;
        return;
    }
    if (addr == 0xFF46)
    {
        uint16_t st = (uint16_t)data << 8;
        for (uint16_t i = 0; i <= 0x9F; ++i)
        {
            MEM[0xFE00 + i] = readMem(st + i);
        }
        return;
    }
    if (addr == 0xFF50)
    {
        for (int i = 0; i < 256; ++i)
        {
            ROM[i] = FirstROMPage[i];
        }
        return;
    }
    MEM[addr] = data;

}


void writeMem16(int addr, int dataH, int dataL)
{
    writeMem(addr, dataL);
    writeMem(addr + 1, dataH);
}

int readMem16(int addr)
{
    uint8_t low  = readMem(addr);
    uint8_t high = readMem(addr + 1);
    return ((uint16_t)high << 8) | low;
}


int ld16(int a, int b, int c = 0)
{
    if (b == Immediate)
    {
        if (a == HL)
        {
            uint16_t addr = readMem(PC + 1) | (readMem(PC + 2) << 8);
            uint16_t s = readMem16(addr);
            REG[H] = (s >> 8) & 0xFF;
            REG[L] = s & 0xFF;
            PC += 3;
            return 12;
        }

        SP = readMem(PC + 1) + (readMem(PC + 2) << 8);
        PC += 3;
        return 12;
    }

    if (c == Immediate)
    {

        REG[a] = readMem(PC + 2);
        REG[b] = readMem(PC + 1);

        PC += 3;
        return 12;
    }

    SP = (REG[H]<<8) + REG[L];
    PC++;
    return 8;
}

int ld_to_mem(int a, int b, int c)
{
    if (a == Immediate)
    {
        writeMem(readMem(PC + 1) + (readMem(PC + 2) << 8), REG[b]);
        PC += 3;
        return 16;
    }
    if (c == Immediate)
    {
        writeMem((REG[a] << 8) + REG[b], readMem(PC + 1));
        PC += 2;
        return 12;
    }
    writeMem((REG[a] << 8) + REG[b], REG[c]);
    PC++;
    return 8;
}

int inc16(int a, int b = 0)
{
    if (a == SPr)
    {
        SP++;
        PC++;
        return 8;
    }
    if (REG[b] == 255) REG[a]++;
    REG[b]++;
    PC++;
    return 8;
}

int shift_process(int op, int a)
{
    int bit7 = a >> 7;
    int bit0 = a & 1;

    if (op == RLC)
    {
        a = ((a << 1) & 0xFF) + bit7;
        FLAGS.C = !!bit7;
    } else if (op == RRC)
    {
        a = ((a >> 1) & 0xFF) + (bit0 << 7);
        FLAGS.C = !!bit0;
    }
    else if (op == RL)
    {
        a = ((a << 1) & 0xFF) + FLAGS.C;
        FLAGS.C = !!bit7;
    } else if (op == RR)
    {
        a = ((a >> 1) & 0xFF) + (FLAGS.C << 7);
        FLAGS.C = !!bit0;
    }
    else if (op == SLA)
    {
        a = ((a << 1) & 0xFF);
        FLAGS.C = !!bit7;
    }
    else if (op == SRA)
    {
        a = ((a >> 1) & 0xFF) + (bit7 << 7);
        FLAGS.C = !!bit0;
    }
    else if (op == SRL)
    {
        a = ((a >> 1) & 0xFF);
        FLAGS.C = !!bit0;
    }

    FLAGS.N = false;
    FLAGS.H = false;
    FLAGS.Z = (a & 0xFF) == 0;
    return a;
}

int incdec_process_8bit(uint8_t a, int offset)
{
    int result = a + offset;
    FLAGS.H = (((a & 0x0F) + offset) & 0x10) != 0;
    FLAGS.N = offset == -1;
    FLAGS.Z = ((result & 0xFF) == 0);
    return (uint8_t)(result & 0xFF);
}

int incdec(int r, int8_t offset)
{
    if (r == HL)
    {
        uint16_t addr = (REG[H] << 8) | REG[L];
        uint8_t val = readMem(addr);
        uint8_t newVal = incdec_process_8bit(val, offset);
        writeMem(addr, newVal);
        PC++;
        return 12;
    }
    else
    {
        REG[r] = incdec_process_8bit(REG[r], offset);
        PC++;
        return 4;
    }
}

int inc(int r)
{
    return incdec(r, 1);
}

int dec(int r)
{
    return incdec(r, -1);
}

int ld(int a, int b)
{
    if (b == Immediate)
    {
        REG[a] = readMem(PC + 1);
        PC += 2;
        return 8;
    }
    REG[a] = REG[b];
    PC += 1;
    return 4;
}

int shift_fast(int op, int a)
{
    REG[a] = shift_process(op, REG[a]);
    FLAGS.Z = 0;
    PC++;
    return 4;
}

int ld_imm_sp(void)
{
    writeMem16(readMem(PC+1) + (readMem(PC+2) << 8), SP >> 8, SP & 0xFF);
    PC += 3;
    return 20;
}

int addHL(int a, int b) {
    if (a == SPr) {
        unsigned int c = (REG[L] + (SP & 0xFF)) > 255 ? 1 : 0;
        unsigned int h = REG[H] + (SP >> 8) + c;
        FLAGS.H = ((REG[H] & 0x0F) + ((SP >> 8) & 0x0F) + c) & 0x10 ? 1 : 0;
        REG[H] = h & 0xFF;
        FLAGS.C = h > 255 ? 1 : 0;
        FLAGS.N = 0;
        PC++;
        return 8;
    } else {
        unsigned int c = (REG[L] + REG[b]) > 255 ? 1 : 0;
        unsigned int h = REG[H] + REG[a] + c;
        FLAGS.H = ((REG[H] & 0x0F) + (REG[a] & 0x0F) + c) & 0x10 ? 1 : 0;
        REG[H] = h & 0xFF;
        FLAGS.C = h > 255 ? 1 : 0;
        FLAGS.N = 0;
        PC++;
        return 8;
    }
}

int ld_from_mem(int a, int b, int c)
{
    if (b == Immediate)
    {
        REG[a] = readMem(readMem(PC + 1) + (readMem(PC + 2) << 8));
        PC += 3;
        return 16;
    }
    else
    {
        REG[a] = readMem((REG[b] << 8) + REG[c]);
        PC++;
        return 8;
    }
}

int dec16(int a, int b)
{
    if (a == SPr)
    {
        SP--;
        PC++;
        return 8;
    }
    else
    {
        if (REG[b] == 0) REG[a]--;
        REG[b]--;
        PC++;
        return 8;
    }
}

int signedOffset(unsigned char b)
{
    return b > 127 ? b - 256 : b;
}

int jrNZ(void)
{
    if (FLAGS.Z)
    {
        PC += 2;
        return 8;
    }
    PC += 2 + signedOffset(readMem(PC + 1));
    return 12;
}

int jr(void)
{
    PC += 2 + signedOffset(readMem(PC + 1));
    return 12;
}

int ldi(int a, int b)
{
    if (a == HL)
    {
        writeMem((REG[H] << 8) + REG[L], REG[A]);
        if (REG[L] == 255) REG[H]++;
        REG[L]++;
        PC++;
        return 8;
    }
    REG[A] = readMem((REG[H] << 8) + REG[L]);
    if (REG[L] == 255) REG[H]++;
    REG[L]++;
    PC++;
    return 8;
}

int daa(void)
{
    if (FLAGS.N)
    {
        if (FLAGS.C) A -= 0x60;
        if (FLAGS.H) A -= 0x06;
    }
    else
    {
        if (A > 0x99 || FLAGS.C)
        {
            A += 0x60;
            FLAGS.C = 1;
        }
        if ((A & 0x0F) > 0x09 || FLAGS.H)
        {
            A += 0x06;
        }
    }
    FLAGS.Z = (A == 0);
    FLAGS.H = 0;
    PC++;
    return 4;
}

int jrZ(void) {
    if (!FLAGS.Z) {
        PC += 2;
        return 8;
    }
    PC += 2 + signedOffset(readMem(PC + 1));
    return 12;
}

int cpl(void)
{
    REG[A] = ~REG[A];
    FLAGS.N = 1;
    FLAGS.H = 1;
    PC++;
    return 4;
}

int jrNC(void)
{
    if (FLAGS.C)
    {
        PC += 2; return 8;
    }
    PC += 2 + signedOffset(readMem(PC + 1));
    return 12;
}

int ldd(int a, int b)
{
    uint16_t addr = (REG[H] << 8) | REG[L];
    if (a == HL)
    {
        writeMem(addr, REG[A]);
        if (REG[L] == 0) REG[H]--;
        REG[L]--;
    }
    else
    {
        REG[A] = readMem(addr);
        if (REG[L] == 0) REG[H]--;
        REG[L]--;
    }
    PC++;
    return 8;
}

int stop()
{
  // TODO
  PC += 2;
  return 4;
}

// helper functions:
int ld16_b_c_immediate(void)
{
    return ld16(B, C, Immediate);
}

int ld_to_mem_b_c_a(void)
{
    return ld_to_mem(B, C, A);
}

int inc16_b_c(void)
{
    return inc16(B, C);
}

int inc_b(void)
{
    return inc(B);
}

int dec_b(void)
{
    return dec(B);
}

int ld_b_immediate(void)
{
    return ld(B, Immediate);
}

int shift_fast_rlc_a(void)
{
    return shift_fast(RLC, A);
}

int addhl_b_c(void)
{
    return addHL(B, C);
}

int ld_from_mem_a_b_c(void)
{
    return ld_from_mem(A, B, C);
}

int dec16_b_c(void)
{
    return dec16(B, C);
}

int inc_c(void)
{
    return inc(C);
}

int dec_c(void)
{
    return dec(C);
}

int ld_c_immediate(void)
{
    return ld(C, Immediate);
}

int shift_fast_rrc_a(void)
{
    return shift_fast(RRC, A);
}

int ld16_d_e_immediate(void)
{
    return ld16(D, E, Immediate);
}

int ld_to_mem_d_e_a(void)
{
    return ld_to_mem(D, E, A);
}

int inc16_d_e(void)
{
    return inc(D);
}

int inc_d(void)
{
    return inc16(D, E);
}

int dec_d(void)
{
    return dec(D);
}

int ld_d_immediate(void)
{
    return ld(D, Immediate);
}

int shift_fast_rl_a(void)
{
    return shift_fast(RL, A);
}

int addhl_d_e(void)
{
    return addHL(D, E);
}

int ld_from_mem_a_d_e(void)
{
    return ld_from_mem(A, D, E);
}

int dec16_d_e(void)
{
    return dec16(D, E);
}

int inc_e(void)
{
    return inc(E);
}

int dec_e(void)
{
    return dec(E);
}

int ld_e_immediate(void)
{
    return ld(E, Immediate);
}

int shift_fast_rr_a(void)
{
    return shift_fast(RR, A);
}

int ld16_h_l_immediate(void)
{
    return ld16(H, L, Immediate);
}

int ldi_hl_a(void)
{
    return ldi(HL, A);
}

int inc16_h_l(void)
{
    return inc16(H, L);
}

int inc_h(void)
{
    return inc(H);
}

int dec_h(void)
{
    return dec(H);
}

int ld_h_immediate(void)
{
    return ld(H, Immediate);
}

int addhl_h_l(void)
{
    return addHL(H, L);
}

int ldi_a_hl(void)
{
    return ldi(A, HL);
}

int dec16_h_l(void)
{
    return dec16(H, L);
}

int inc_l(void)
{
    return inc(L);
}

int dec_l(void)
{
    return dec(L);
}

int ld_l_immediate(void)
{
    return ld(L, Immediate);
}

int ld16_spr_immediate(void)
{
    return ld16(SPr, Immediate);
}

int ldd_hl_a(void)
{
    return ldd(HL, A);
}

int inc16_spr(void)
{
    return inc16(SPr);
}

// main opcodes:
void run_opcode(void)
{
    opcodes[0x00] = nop;
    opcodes[0x01] = ld16_b_c_immediate;
    opcodes[0x02] = ld_to_mem_b_c_a;
    opcodes[0x03] = inc16_b_c;
    opcodes[0x04] = inc_b;
    opcodes[0x05] = dec_b;
    opcodes[0x06] = ld_b_immediate;
    opcodes[0x07] = shift_fast_rlc_a;
    opcodes[0x08] = ld_imm_sp;
    opcodes[0x09] = addhl_b_c;
    opcodes[0x0A] = ld_from_mem_a_b_c;
    opcodes[0x0B] = dec16_b_c;
    opcodes[0x0C] = inc_c;
    opcodes[0x0D] = dec_c;
    opcodes[0x0E] = ld_c_immediate;
    opcodes[0x0F] = shift_fast_rrc_a;

    opcodes[0x10] = stop;
    opcodes[0x11] = ld16_d_e_immediate;
    opcodes[0x12] = ld_to_mem_d_e_a;
    opcodes[0x13] = inc16_d_e;
    opcodes[0x14] = inc_d;
    opcodes[0x15] = dec_d;
    opcodes[0x16] = ld_d_immediate;
    opcodes[0x17] = shift_fast_rl_a;
    opcodes[0x18] = jr;
    opcodes[0x19] = addhl_d_e;
    opcodes[0x1A] = ld_from_mem_a_d_e;
    opcodes[0x1B] = dec16_d_e;
    opcodes[0x1C] = inc_e;
    opcodes[0x1D] = dec_e;
    opcodes[0x1E] = ld_e_immediate;
    opcodes[0x1F] = shift_fast_rr_a;

    opcodes[0x20] = jrNZ;
    opcodes[0x21] = ld16_h_l_immediate;
    opcodes[0x22] = ldi_hl_a;
    opcodes[0x23] = inc16_h_l;
    opcodes[0x24] = inc_h;
    opcodes[0x25] = dec_h;
    opcodes[0x26] = ld_h_immediate;
    opcodes[0x27] = daa;
    opcodes[0x28] = jrZ;
    opcodes[0x29] = addhl_h_l;
    opcodes[0x2A] = ldi_a_hl;
    opcodes[0x2B] = dec16_h_l;
    opcodes[0x2C] = inc_l;
    opcodes[0x2D] = dec_l;
    opcodes[0x2E] = ld_l_immediate;
    opcodes[0x2F] = cpl;

    opcodes[0x30] = jrNC;
    opcodes[0x31] = ld16_spr_immediate;
    opcodes[0x32] = ldd_hl_a;
    opcodes[0x33] = inc16_spr;
}

#ifdef __cplusplus
}
#endif

#endif
