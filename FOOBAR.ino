// Registers and Pointers
uint8_t REG[8];
uint16_t PC = 0;
uint16_t SP = 0;
// Flags and State
bool IME = false; // Interrupt master enable
bool cpu_halted = false;
// Memory and ROM
uint8_t MEM[0x10000];
uint8_t ROM[512];
uint16_t ROMbank = 1; 
int ROMbankoffset = (ROMbank-1) * 0x4000;
uint8_t RAMbank = 0; 
uint16_t RAMbankoffset = RAMbank * 0x2000 - 0xA000;
uint8_t cartRAM[0x8000];
// Joypad Input
uint8_t joypad_dpad = 0xef,joypad_buttons = 0xdf; // 0=pressed
uint8_t keys_dpad = 0xef, keys_buttons = 0xdf; // 0=pressed
// Opcode Constants
const uint8_t A = 0b111;
const uint8_t B = 0b000;
const uint8_t C = 0b001;
const uint8_t D = 0b010;
const uint8_t E = 0b011;
const uint8_t H = 0b100;
const uint8_t L = 0b101;
const uint8_t HL = 0b110;
const uint16_t Immediate = 257;
const uint16_t BC = 258;
const uint16_t DE = 259;
const uint16_t SPr = 260;

typedef uint8_t (*OpcodeFunc)();
OpcodeFunc opcodes[256];

struct Pair { uint8_t hi, lo; };

uint16_t readMem(uint16_t addr)
{
  if (addr <= 0x3fff) return ROM[addr];
  if (addr <= 0x7fff) return ROM[addr + ROMbankoffset];

  // Cartridge RAM
  if (addr >= 0xA000 && addr <=0xBFFF) return cartRAM[addr + RAMbankoffset];

  // Joypad
  if (addr == 0xFF00)
  {
    if (MEM[0xFF00]&0x20)
    {
      return joypad_dpad & keys_dpad;
    }
    else if (MEM[0xFF00]&0x10)
    {
      return joypad_buttons & keys_buttons;
    }
    else return 0xFF;
  }

  return MEM[addr];
}

Pair readMem16(uint8_t addr)
{
  Pair p;
  p.hi = readMem(addr+1);
  p.lo = readMem(addr);
  return p;
}

// Messy...
uint8_t ld16(uint8_t a, uint8_t b, uint8_t c)
{
  if (b == Immediate)
  {
    if (a == HL)
    {
      // mem to hl
      Pair s = readMem16(readMem(PC+1) + (readMem( PC+2 )<<8));
      REG[H] = s.hi;
      REG[L] = s.lo;
      PC+=3;
      return 12;
    }

    // immediate into SP... 
    SP = readMem(PC+1) + (readMem(PC+2) << 8);
    PC += 3;
    return 12;
  }
  if (c == Immediate)
  {
    
    REG[a] = readMem(PC+2);
    REG[b] = readMem(PC+1);

    PC += 3;
    return 12;
  }

  // ld sp, hl
  SP = (REG[H] << 8) + REG[L];
  PC++;
  return 8;
}

uint8_t nop()
{
  PC++;
  return 4;
}

uint8_t ld16_bc_immediate()
{
  return ld16(B,C,Immediate);
}

void setup()
{
  opcodes[0x00] = nop;
  opcodes[0x01] = ld16_bc_immediate;
}

void loop()
{
  // put your main code here, to run repeatedly:
}
