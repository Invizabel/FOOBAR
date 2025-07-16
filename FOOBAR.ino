// Registers and Pointers
uint8_t REG[8];
uint16_t PC = 0;
uint16_t SP = 0;
// Flags and State
uint16_t dpixels[128*64];
uint16_t divPrescaler = 0;
uint16_t timerPrescaler = 0;
bool timerEnable = false;
bool LCD_enabled = false;
uint16_t timerLength = 1;
uint8_t LCD_lastmode = 1;
uint8_t LCD_scan = 0;
bool IME = false; // Interrupt master enable
bool cpu_halted = false;
// Memory and ROM
uint8_t MEM[0x10000];
uint8_t FirstROMPage[512];
uint8_t ROM[512];
uint16_t ROMbank = 1; 
int32_t ROMbankoffset = ((int32_t)ROMbank - 1) * 0x4000;
uint8_t cartRAM[0x8000]; // some carts have up to 128K of ram?
uint16_t RAMbank = 0;
int32_t RAMbankoffset = ((int32_t)RAMbank * 0x2000) - 0xA000;
bool RAMenabled=false;
uint8_t MBCRamMode = 0; //for MBC1
// Joypad Input
uint8_t joypad_dpad = 0xef,joypad_buttons = 0xdf; // 0 = pressed
uint8_t keys_dpad = 0xef, keys_buttons = 0xdf; // 0 = pressed
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
const uint16_t RLC = 1;
const uint16_t RRC = 2;
const uint16_t RL  = 3;
const uint16_t RR  = 4;
const uint16_t SLA = 5;
const uint16_t SRA = 6;
const uint16_t SRL = 7;

typedef uint16_t (*OpcodeFunc)();
OpcodeFunc opcodes[256];

struct Pair { uint16_t hi, lo; };

struct Flags {
  bool Z = false; // Zero flag
  bool N = false; // Subtract flag
  bool H = false; // Half-carry flag
  bool C = false; // Carry flag
};

Flags FLAGS;

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

Pair readMem16(uint16_t addr)
{
  Pair p;
  p.hi = readMem(addr+1);
  p.lo = readMem(addr);
  return p;
}

uint16_t doMBC(uint16_t addr, uint16_t data)
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
      RAMenabled = ((data & 0x0F) == 0xA);
    }
    else if (addr <= 0x3FFF)
    {
      data &= 0x1F;
      if (data==0) data = 1; // MBC1 translates bank 0 to bank 1 (apparently regardless of upper bits)
      // set lowest 5 bits of bank number
      ROMbank = (ROMbank & 0xE0) | (data & 0x1F);
      ROMbankoffset = (ROMbank-1) * 0x4000 % sizeof(ROM);
    }
    else if (addr <= 0x5fff)
    {
      data &= 0x3;
      if (MBCRamMode == 0)
      {
        ROMbank = (ROMbank & 0x1F) | (data << 5);
        ROMbankoffset = (ROMbank - 1) * 0x4000  % sizeof(ROM);
      }
      else
      {
        RAMbank = data;
        RAMbankoffset = RAMbank * 0x2000 - 0xA000;
      }
    }
    else
    {
      MBCRamMode = data & 1;
      if (MBCRamMode == 0)
      {
        RAMbank = 0;
        RAMbankoffset = RAMbank*0x2000 - 0xA000;
      }
      else
      {
        ROMbank &= 0x1F;
        ROMbankoffset = (ROMbank - 1) * 0x4000  % sizeof(ROM);
      }
    }
    
  break;

  case 0x05: //  MBC2
  case 0x06: //  MBC2+BATTERY
    
    if (addr <= 0x1FFF)
    {
      if ((addr&0x0100) ==0)
        RAMenabled = ((data & 0x0F) == 0xA) ;
    }
    else if (addr <= 0x3FFF)
    {
      data &= 0x0F;
      if (data == 0) data = 1;
      ROMbank = data;
      ROMbankoffset = (ROMbank - 1) * 0x4000 % sizeof(ROM);
    }

  break;

  // case 0x08: //  ROM+RAM
  // case 0x09: //  ROM+RAM+BATTERY
  // case 0x0B: //  MMM01
  // case 0x0C: //  MMM01+RAM
  // case 0x0D: //  MMM01+RAM+BATTERY
  // case 0x0F: //  MBC3+TIMER+BATTERY
  // case 0x10: //  MBC3+TIMER+RAM+BATTERY
  case 0x11: //  MBC3
  case 0x12: //  MBC3+RAM
  case 0x13: //  MBC3+RAM+BATTERY

    if (addr <= 0x1FFF)
    {
      RAMenabled = ((data & 0x0F) == 0xA);
    }
    else if (addr <= 0x3FFF)
    {
      if (data == 0) data = 1; // allows access to banks 0x20, 0x40, 0x60
      ROMbank = data & 0x7F;
      ROMbankoffset = (ROMbank - 1) * 0x4000 % sizeof(ROM);
    }
    else if (addr <= 0x5fff)
    {
      if (data < 8)
      {
        RAMbank=data;
        RAMbankoffset = RAMbank*0x2000 - 0xA000;
      }
      else
      {
        // RTC registers here
      }
    }
    else
    {
      // RTC latch
    }
  break;

  case 0x19: //  MBC5
  case 0x1A: //  MBC5+RAM
  case 0x1B: //  MBC5+RAM+BATTERY
  // case 0x1C: //  MBC5+RUMBLE
  // case 0x1D: //  MBC5+RUMBLE+RAM
  // case 0x1E: //  MBC5+RUMBLE+RAM+BATTERY
    if (addr <= 0x1FFF)
    {
      RAMenabled = ((data & 0x0F) == 0xA);
    }
    else if (addr <= 0x2FFF)
    {
      // Allows access to bank 0
      ROMbank &= 0x100;
      ROMbank |= data;
      ROMbankoffset = (ROMbank - 1)*  0x4000;
      while (ROMbankoffset>sizeof(ROM)) ROMbankoffset -= sizeof(ROM);
    }
    else if (addr <= 0x3FFF)
    {
      ROMbank &= 0xFF;
      if (data & 1) ROMbank += 0x100;
      ROMbankoffset = (ROMbank-1)*0x4000;
      while (ROMbankoffset > sizeof(ROM)) ROMbankoffset -= sizeof(ROM);
    }
    else if (addr <= 0x5fff)
    {
      RAMbank = data & 0x0F;
      RAMbankoffset = RAMbank*0x2000 - 0xA000;
    }
  break;

  // case 0x20: //  MBC6
  // case 0x22: //  MBC7+SENSOR+RUMBLE+RAM+BATTERY
  // case 0xFC: //  POCKET CAMERA
  // case 0xFD: //  BANDAI TAMA5
  // case 0xFE: //  HuC3
  // case 0xFF: //  HuC1+RAM+BATTERY

    default: Serial.print("Unimplemented memory controller");

  }
}

uint16_t writeMem(uint16_t addr, uint16_t data)
{
  if (addr <= 0x7fff)
  { 
    doMBC(addr, data);
    return 0;
  }

  if (addr >= 0xA000 && addr <= 0xBFFF && RAMenabled)
  {
    cartRAM[addr + RAMbankoffset] = data;
    return 0;
  }

  //DIV register: reset
  if (addr == 0xFF04)
  {
    MEM[0xFF04] = 0;
    return 0;
  }
  // Timer control
  if (addr == 0xFF07)
  {
    timerEnable = ((data&(1<<2)) != 0);
    timerLength = (uint16_t[]){1024, 16, 64, 256}[data & 0x3];
    timerPrescaler = timerLength; // +cycles for this instruction?
    MEM[addr] = 0xF8|data;
    return 0;
  }

  // Sound Control
  /*
  if (addr==0xFF26)
  {
    if (data & (1 << 7))
    {
      MEM[0xFF26] = data & (1 << 7);
      SoundEnabled = true;
      audioCtx.resume()
    } else {
      SoundEnabled = false;
      // should we set each oscillator to amplitude zero too?
      audioCtx.suspend()
      // Zero all sound registers
      resetSoundRegisters();
    }
    return;
  }
  if (addr>=0xFF10 && addr <=0xFF25) {
    if (!SoundEnabled) return;
    // FF10 - NR10 - Channel 1 Sweep register (R/W)
    if (addr == 0xFF10) {
      sound[1].sweepTime = (data>>4)&0x7;
      sound[1].sweepPrescaler = sound[1].sweepTime;
      sound[1].sweepDir = (data&(1<<3)) ? 0 : 1;
      sound[1].sweepShift = data&0x7;
      MEM[addr] = data &0x80
      return;
    }
    // FF11 - NR11 - Channel 1 Sound length/Wave pattern duty (R/W)
    if (addr == 0xFF11) {
      MEM[addr] = data;
      sound[1].duty(data>>6)
      return;
    }
    // FF12 - NR12 - Channel 1 Volume Envelope (R/W)
    if (addr == 0xFF12) {
      MEM[addr] = data
      sound[1].envDirection = (data&(1<<3)) ? 1: -1;
      sound[1].envSpeed = data&0x7;
      sound[1].envCounter = 0;
      return;
    }
    // FF13 - NR13 - Channel 1 Frequency lo (Write Only)
    if (addr == 0xFF13) {
      sound[1].freqnum=(((MEM[0xFF14]&0x7)<<8)+ data);
      sound[1].freq(  131072/(2048-  sound[1].freqnum  )  )
      MEM[addr] = data
      return;
    }
    // FF14 - NR14 - Channel 1 Frequency hi (R/W)
    if (addr == 0xFF14) {
      //bit 7 is initialize
      sound[1].freqnum=(((data&0x7)<<8)+ MEM[0xFF13]);
      sound[1].freq(  131072/(2048-  sound[1].freqnum  )  )
      if (data&(1<<7)) {
        sound[1].initialized = true
        sound[1].env = MEM[0xFF12]>>4; // default envelope value
        sound[1].envCounter = 0;
        sound[1].amp( sound[1].env/15 )

        sound[1].lengthEnabled = (data&(1<<6)) !=0;
        sound[1].length = (64-(MEM[0xFF11]&0x3F));

        MEM[0xFF26] |= (1<<0) // flag sound 1 as on
        //if (sound[1].sweepShift) {sweepCalculate()}
      }
      MEM[addr] = data
      return;
    }


    // FF16 - NR21 - Channel 2 Sound Length/Wave Pattern Duty (R/W)
        // Bit 7-6 - Wave Pattern Duty (Read/Write)
        // Bit 5-0 - Sound length data (Write Only) (t1: 0-63)
    if (addr == 0xFF16) {
      MEM[addr] = data
      sound[2].duty(data>>6)
      return;
    }

    // FF17 - NR22 - Channel 2 Volume Envelope (R/W)
    if (addr == 0xFF17) {
      MEM[addr] = data
      sound[2].envDirection = (data&(1<<3)) ? 1: -1;
      sound[2].envSpeed = data&0x7;
      sound[2].envCounter = 0;
      return;
    }
    // FF18 - NR23 - Channel 2 Frequency lo data (W)
    if (addr == 0xFF18) {
      sound[2].freq(  131072/(2048-  (((MEM[0xFF19]&0x7)<<8)+ data)  )  )
      MEM[addr] = data
      return;
    }
    // FF19 - NR24 - Channel 2 Frequency hi data (R/W)
    if (addr == 0xFF19) {
      sound[2].freq(  131072/(2048-  (((data&0x7)<<8)+ MEM[0xFF18])  )  )
      //bit 7 is initialize
      if (data&(1<<7)) {
        sound[2].initialized = true
        sound[2].env = MEM[0xFF17]>>4; //Default envelope value
        sound[2].envCounter = 0;
        sound[2].amp( sound[2].env/15 )

        sound[2].lengthEnabled = (data&(1<<6)) !=0;
        sound[2].length = (64-(MEM[0xFF16]&0x3F));
        MEM[0xFF26] |= (1<<1) // flag sound 2 as on
      }
      MEM[addr] = data
      return;
    }

    // Sound 3 - user-defined waveform
    // "it can output a sound while changing its length, frequency, and level"
    // not sure what changing its length means

    // FF1A - NR30 - Channel 3 Sound on/off (R/W)
    if (addr==0xFF1A) {
      if (data&(1<<7)) {
        sound[3].initialized=true;

        // is this the right (only?) place to load the waveform?
        setSound3Waveform()

      } else {
        sound[3].initialized=false;
        sound[3].amp(0)
      }
      return;
    }
    // FF1B - NR31 - Channel 3 Sound Length
    if (addr==0xFF1B) {
      MEM[addr] = data
      return;
    }
    // FF1C - NR32 - Channel 3 Select output level (R/W)
    if (addr==0xFF1C) {
      // Really we ought to bit-crush it, but whatever
      if (sound[3].initialized) sound[3].amp( [ 0,0.5,0.25,0.125 ][((data>>5)&0x3)] )
      MEM[addr] = data
      return;
    }

    // FF1D - NR33 - Channel 3 Frequency's lower data (W)
    if (addr == 0xFF1D) {
      sound[3].freq(   65536/(2048-  (((MEM[0xFF1E]&0x7)<<8)+ data)  )  )
      MEM[addr] = data
      return;
    }
    // FF1E - NR34 - Channel 3 Frequency's higher data (R/W)
    if (addr == 0xFF1E) {
      sound[3].freq(   65536/(2048-  (((data&0x7)<<8)+ MEM[0xFF1D])  )  )
      //bit 7 is initialize
      if (data&(1<<7)) {

        sound[3].initialized = true

        sound[3].amp( [ 0,0.5,0.25,0.15 ][((MEM[0xFF1C]>>5)&0x3)] )

        sound[3].lengthEnabled = (data&(1<<6)) !=0;
        sound[3].length = (256-MEM[0xFF1B]);

        MEM[0xFF26] |= (1<<2) // flag sound 3 as on
      }
      MEM[addr] = data
      return;
    }

    // Sound 4 - Noise
    // FF20 - NR41 - Channel 4 Sound Length (R/W)
    if (addr==0xFF20) {
      MEM[addr] = data
      return;
    }
    // FF21 - NR42 - Channel 4 Volume Envelope (R/W)
    if (addr==0xFF21) {
      MEM[addr] = data
      sound[4].envDirection = (data&(1<<3)) ? 1: -1;
      sound[4].envSpeed = data&0x7;
      sound[4].envCounter = 0;
      return;
    }
    // FF22 - NR43 - Channel 4 Polynomial Counter (R/W)
    if (addr==0xFF22) {
      sound[4].freq(data>>4, data&0x7)
      sound[4].polySteps(data&(1<<3))

      MEM[addr] = data
      return;
    }
    // FF23 - NR44 - Channel 4 Counter/consecutive; Inital (R/W)
    if (addr==0xFF23) {

      sound[4].initialized = true
      sound[4].env = MEM[0xFF21]>>4; //Default envelope value
      sound[4].envCounter = 0;
      sound[4].amp( sound[4].env/15 )
      sound[4].length = (64-(MEM[0xFF20]&0x3F));
      
      MEM[0xFF26] |= (1<<3) // flag sound 4 as on

      sound[4].lengthEnabled = (data&(1<<6)) !=0;
      MEM[addr] = data; 
      return
    }


    // FF24 - NR50 - Channel control / ON-OFF / Volume (R/W)
    if (addr == 0xFF24) {
       //  Bit 7   - Output Vin to SO2 terminal (1=Enable)
       //  Bit 6-4 - SO2 output level (volume)  (0-7)
       //  Bit 3   - Output Vin to SO1 terminal (1=Enable)
       //  Bit 2-0 - SO1 output level (volume)  (0-7)

      // is level zero mute ? "minimum level"
      sound.SO2.gain.setValueAtTime(((data>>4)&0x7)/7, audioCtx.currentTime)
      sound.SO1.gain.setValueAtTime((data&0x7)/7,      audioCtx.currentTime)
      MEM[addr] = data; 
      return
    }

    // FF25 - NR51 - Selection of Sound output terminal (R/W)
    if (addr == 0xff25) {

      var con = (MEM[0xff25]^data) & data;
      var dis = (MEM[0xff25]^data) & (~data);

      for (var i=0;i<4;i++) {
        if (con&(1<<i))
          sound[i+1].gainNode.connect(sound.SO1)
        if (dis&(1<<i))
          sound[i+1].gainNode.disconnect(sound.SO1)
        if (con&(1<<(4+i)))
          sound[i+1].gainNode.connect(sound.SO2)
        if (dis&(1<<(4+i)))
          sound[i+1].gainNode.disconnect(sound.SO2)
      }

      MEM[addr] = data; 
      return
    }

    return;
  }
  if (addr>=0xFF30 && addr<=0xFF3F) sound[3].waveChanged=true;
  */
  //LCD control
  if (addr == 0xFF40)
  {
    uint16_t cc = data&(1 << 7);
    if (LCD_enabled != cc)
    {
      LCD_enabled = !!cc;
      if (!LCD_enabled)
      { // Disabling the display sets it to mode 1
        // this should also probably set all pixels to white
        LCD_scan = 0;
        MEM[0xFF41] = (MEM[0xFF41] & 0xFC) + 1;
      }
    }
  }
  if (addr == 0xFF41)
  {
    //don't overwrite the lowest two bits (mode)
    MEM[0xFF41] &= 0x3;
    data &= 0xFC;
    MEM[0xFF41] |= 0x80 | data; // BGB has highest bit always set
    return 0;
  }

  // LY - write causes reset
  if (addr == 0xFF44)
  {
    MEM[0xFF44] = 0;
    return 0;
  }

  // FF46 - DMA - DMA Transfer and Start Address (W)
  if (addr==0xFF46)
  {
    uint16_t st = data << 8;
    for (uint16_t i = 0;i <= 0x9F; i++)
    {
      MEM[0xFE00 + i] = readMem(st + i);
    }
    return 0;
  }

  // disable bootrom
  if (addr==0xFF50)
  {
    for (uint16_t i = 0; i < 256; i++)
    {
      ROM[i] = FirstROMPage[i];
    }
    return 0;
  }

  MEM[addr] = data;
}

uint16_t writeMem16(uint16_t addr, uint16_t dataH, uint16_t dataL)
{
  writeMem(addr, dataL);
  writeMem(addr+1, dataH);
}

// Messy...
uint16_t ld16(uint16_t a, uint16_t b, uint16_t c)
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

uint16_t ld_to_mem(uint16_t a, uint16_t b, uint16_t c)
{
  if (a == Immediate)
  {
    writeMem(readMem(PC + 1) + (readMem( PC + 2) << 8), REG[b]);
    PC += 3;
    return 16;
  }
  if (c == Immediate)
  {
    writeMem((REG[a] << 8)+REG[b], readMem(PC+1));
    PC += 2;
    return 12;
  }
  writeMem((REG[a] << 8) + REG[b], REG[c]);
  PC++;
  return 8;
}

uint16_t incdec_process_8bit(uint16_t a, uint16_t offset)
{
    uint16_t result = a + offset;
    FLAGS.H = !!(((a & 0x0F) + offset) & 0x10);
    FLAGS.N = offset == -1;
    FLAGS.Z = ((result & 0xff) == 0);
    return result;
}

uint16_t incdec(uint16_t r, uint16_t offset)
{
  if (r == HL)
  {
    writeMem((REG[H] << 8) + REG[L], incdec_process_8bit(readMem((REG[H] << 8) + REG[L]), offset));
    PC++;
    return 12;
  }
  REG[r] = incdec_process_8bit(REG[r], offset);
  PC++;
  return 4;
}

uint16_t dec(uint16_t a)
{
  return incdec(a, -1);
}

uint16_t inc(uint16_t a)
{
  return incdec(a, 1);
}

uint16_t ld(uint16_t a, uint16_t b)
{ 
  if (b == Immediate)
  {
    REG[a] = readMem(PC+1);
    PC += 2;
    return 8;
  }
  REG[a] = REG[b];
  PC++;
  return 4;
}

uint16_t shift_process(uint16_t op, uint16_t a)
{
  uint16_t bit7 = a >> 7, bit0 = a&1;

  switch (op)
  {
    case RLC: // Rotate byte left, save carry
      a = ((a << 1) & 0xff) + bit7;
      FLAGS.C = !!bit7;
    break;
    case RRC: // Rotate byte right, save carry
      a = ((a >> 1) & 0xff) + (bit0 << 7);
      FLAGS.C = !!bit0;
    break;
    case RL : //Rotate left through carry
      a = ((a << 1) & 0xff) + FLAGS.C ;
      FLAGS.C = !!bit7;
    break;
    case RR : //Rotate right through carry
      a = ((a >> 1) & 0xff) + (FLAGS.C << 7);
      FLAGS.C = !!bit0;
    break;
    case SLA: //Shift left
      a = ((a << 1) & 0xff);
      FLAGS.C = !!bit7;
    break;
    case SRA: //Shift right arithmetic
      a = ((a >> 1) & 0xff) + (bit7 << 7);
      FLAGS.C = !!bit0;
    break;
    case SRL: //Shift right logical
      a = ((a >> 1) & 0xff);
      FLAGS.C = !!bit0;
    break;
  }

  FLAGS.N = false;
  FLAGS.H = false;
  FLAGS.Z= (a & 0xFF) == 0;
  return a;
}

uint16_t shift_fast(uint16_t op, uint16_t a)
{
    REG[a] = shift_process(op, REG[a]);
    FLAGS.Z = false; // Bizarre, but correct
    PC++;
    return 4;
}

uint16_t ld_imm_sp()
{
  writeMem16(readMem(PC + 1) + (readMem(PC + 2) << 8), SP >> 8, SP & 0xFF);
  PC += 3;
  return 20;
}

uint16_t addHL(uint16_t a, uint16_t b)
{
  if (a == SPr)
  {
    uint16_t c = (REG[L] += (SP & 0xFF)) > 255 ? 1:0;
    uint16_t h = REG[H] + (SP>>8) + c;
    FLAGS.H = !!(((REG[H] & 0x0F) + ((SP >> 8) & 0x0F) + c) & 0x10);
    REG[H] = h;
    FLAGS.C = (h > 255);
    FLAGS.N = false;
    PC++;
    return 8;
  }
    uint16_t c = (REG[L]+= REG[b])>255?1:0;
    uint16_t h = REG[H] + REG[a] + c;
    FLAGS.H = !!(((REG[H] & 0x0F) + (REG[a] & 0x0F) + c) & 0x10);
    REG[H] = h;
    FLAGS.C = (h > 255);
    FLAGS.N = false;
    PC++;
    return 8;
}

uint16_t ld_from_mem(uint16_t a, uint16_t b, uint16_t c)
{
  if (b == Immediate)
  {
    REG[a] = readMem(readMem(PC + 1) + (readMem(PC + 2) << 8));
    PC+=3;
    return 16;
  }
  REG[a] = readMem((REG[b] << 8) + REG[c]);
  PC++;
  return 8;
}

uint16_t dec16(uint16_t a, uint16_t b)
{
  if (a == SPr)
  {
    SP--;
    PC++;
    return 8;
  }
  if (REG[b] ==0) REG[a]--;
  REG[b]--;
  PC++;
  return 8;
}

uint16_t func_stop()
{
  //TODO
  PC+=2;
  return 4;
}

uint16_t ld_e_immediate()
{
  return ld(E, Immediate);
}

// 16 bit inc / dec affect no flags
uint16_t inc16(uint16_t a, uint16_t b)
{
  if (a == SPr)
  {
    SP++;
    PC++;
    return 8;
  }
  if (REG[b] ==255) REG[a]++;
  REG[b]++;
  PC++;
  return 8;
}

uint16_t signedOffset(uint16_t b)
{
  return (b > 127) ? (b - 256) : b;
}

uint16_t jr()  // unconditional relative
{
  PC += 2 + signedOffset(readMem(PC + 1));
  return 12;
}

uint16_t jrNZ()
{
  if (FLAGS.Z)
  {
    PC+=2;
    return 8;
  }
  
  PC += 2 + signedOffset(readMem(PC + 1));
  return 12;
}

uint16_t ldi(uint16_t a, uint16_t b) //load with increment
{
  if (a == HL)
  {
    writeMem((REG[H] << 8) + REG[L], REG[A]);
    if (REG[L]==255) REG[H]++;
    REG[L]++;
    
    PC++;
    return 8;
  }
  
  REG[A] = readMem((REG[H] << 8) + REG[L]);
  if (REG[L]==255) REG[H]++;
  REG[L]++;

  PC++;
  return 8;
}

uint16_t daa()
{
  //http://gbdev.gg8.se/wiki/articles/DAA

  if (FLAGS.N)
  {
    if (FLAGS.C) REG[A] -= 0x60;
    if (FLAGS.H) REG[A] -= 0x06;
  }
  else 
  {
    if (REG[A] > 0x99 || FLAGS.C)
    {
      REG[A] += 0x60;
      FLAGS.C = true;
    }
    if ((REG[A] & 0x0f)> 0x09 || FLAGS.H) REG[A] += 0x06;
  }

  FLAGS.Z = REG[A] == 0;
  FLAGS.H = false;

  PC++;
  return 4;
}

uint16_t jrZ()
{
  if (!FLAGS.Z)
  {
    PC += 2;
    return 8;
  }
 
  PC += 2 + signedOffset(readMem(PC + 1));
  return 12;
}

uint16_t cpl()
{
  REG[A] = ~REG[A];
  FLAGS.N = true;
  FLAGS.H = true;
  PC++;
  return 4;
}

uint16_t jrNC()
{
  if (FLAGS.C)
  {
    PC += 2;
    return 8;
  }
  
  PC += 2 + signedOffset(readMem(PC + 1));
  return 12;
}

uint16_t ldd(uint16_t a, uint16_t b) // load with decrement
{ 
  if (a == HL)
  {
    writeMem((REG[H] << 8) + REG[L], REG[A]);
    if (REG[L]==0) REG[H]--;
    REG[L]--;
    PC++;
    return 8;
  }
  
  REG[A] = readMem((REG[H] << 8) + REG[L]);
  if (REG[L]==0) REG[H]--;
  REG[L]--;
  PC++;
  return 8;
}

uint16_t scf()
{
  FLAGS.N = false;
  FLAGS.H = false;
  FLAGS.C = true;
  PC++;
  return 4;
}

uint16_t jrC()
{
  if (!FLAGS.C)
  {
    PC += 2;
    return 8;
  }
  
  PC += 2 + signedOffset(readMem(PC + 1));
  return 12;
}

uint16_t ccf()
{
  FLAGS.N = false;
  FLAGS.H = false;
  FLAGS.C = !FLAGS.C;
  PC++;
  return 4;
}

uint16_t nop()
{
  PC++;
  return 4;
}

uint16_t ld16_bc_immediate()
{
  return ld16(B,C,Immediate);
}

uint16_t ld_to_mem_bca()
{
  return ld_to_mem(B,C,A);
}

uint16_t inc16_b_c()
{
  return inc16(B,C);
}

uint16_t inc_b()
{
  return inc(B);
}

uint16_t dec_b()
{
  return dec(B);
}

uint16_t ld_b_immediate()
{
  return ld(B, Immediate);
}

uint16_t shift_fast_rlc_a()
{
  return shift_fast(RLC, A);
}

uint16_t add_hl_b_c()
{
  return addHL(B, C);
}

uint16_t ld_from_mem_a_b_c()
{
  return ld_from_mem(A, B, C);
}

uint16_t dec16_b_c()
{
  return dec16(B,C);
}

uint16_t inc_c()
{
  return inc(C);
}

uint16_t dec_c()
{
  dec(C);
}

uint16_t ld_c_immediate()
{
  return ld(C, Immediate);
}

uint16_t shift_fast_rrc_a()
{
  return shift_fast(RRC, A);
}

uint16_t ld16_d_e_immediate()
{
  return ld16(D,E,Immediate);
}

uint16_t ld_to_mem_d_e_a()
{
  return ld_to_mem(D,E,A);
}

uint16_t inc16_d_e()
{
  return inc16(D,E);
}

uint16_t inc_d()
{
  return inc(D);
}

uint16_t dec_d()
{
  return dec(D);
}

uint16_t ld_d_immediate()
{
  return ld(D, Immediate);
}

uint16_t shift_fast_rl_a()
{
  return shift_fast(RL, A);
}

uint16_t addhl_d_e()
{
  return addHL(D,E); //ADD HL, DE;
}

uint16_t ld_from_mem_a_d_e()
{
  return ld_from_mem(A, D, E);
}

uint16_t dec16_d_e()
{
  return dec16(D,E);
}

uint16_t inc_e()
{
  return inc(E);
}

uint16_t dec_e()
{
  return dec(E);
}

uint16_t shift_fast_rr_a()
{
  return shift_fast(RR, A);
}

uint16_t ld16_h_l_immediate()
{
  return ld16(H,L,Immediate);
}

uint16_t ldi_hl_a()
{
  return ldi(HL, A);
}

uint16_t inc16_h_l()
{
  return inc16(H, L);
}

uint16_t inc_h()
{
  return inc(H);
}

uint16_t dec_h()
{
  return dec(H);
}

uint16_t ld_h_immediate()
{
  return ld(H, Immediate);
}

uint16_t addhl_h_l()
{
  return addHL(H, L);
}

uint16_t ldi_a_hl()
{
  return ldi(A, HL);
}

uint16_t dec16_h_l()
{
  return dec16(H, L);
}

uint16_t inc_l()
{
  return inc(L);
}

uint16_t dec_l()
{
  return dec(L);
}

uint16_t ld_l_immediate()
{
  return ld(L, Immediate);
}

uint16_t ld16_spr_immediate()
{
  return ld16(SPr, Immediate, 0);
}

uint16_t ldd_hl_a()
{
  return ldd(HL, A);
}
uint16_t inc16_spr()
{
  return inc16(SPr, 0);
}

uint16_t inc_hl()
{
  return inc(HL);
}

uint16_t dec_hl()
{
  return dec(HL);
}

uint16_t ld_to_mem_h_l_immediate()
{
  return ld_to_mem(H, L, Immediate);
}

uint16_t addhl_spr()
{
  return addHL(SPr, 0);
}

uint16_t ldd_a_hl()
{
  return ldd(A, HL);
}

uint16_t dec16_spr()
{
  return dec16(SPr, 0);
}

uint16_t inc_a()
{
  return inc(A);
}

uint16_t dec_a()
{
  return dec(A);
}

uint16_t ld_a_immediate()
{
  return ld(A, Immediate);
}

void setup()
{
  opcodes[0x00] = nop;
  opcodes[0x01] = ld16_bc_immediate;
  opcodes[0x02] = ld_to_mem_bca;
  opcodes[0x03] = inc16_b_c;
  opcodes[0x04] = inc_b;
  opcodes[0x05] = dec_b;
  opcodes[0x06] = ld_b_immediate;
  opcodes[0x07] = shift_fast_rlc_a; // rlca
  opcodes[0x08] = ld_imm_sp; // LD (nn), SP
  opcodes[0x09] = add_hl_b_c;
  opcodes[0x0A] = ld_from_mem_a_b_c;
  opcodes[0x0B] = dec16_b_c;
  opcodes[0x0C] = inc_c;
  opcodes[0x0D] = dec_c;
  opcodes[0x0E] = ld_c_immediate;
  opcodes[0x0F] = shift_fast_rrc_a;

  opcodes[0x10] = func_stop;
  opcodes[0x11] = ld16_d_e_immediate;
  opcodes[0x12] = ld_to_mem_d_e_a;
  opcodes[0x13] = inc16_d_e;
  opcodes[0x14] = inc_d;
  opcodes[0x15] = dec_d;
  opcodes[0x16] = ld_d_immediate;
  opcodes[0x17] = shift_fast_rl_a;
  opcodes[0x18] = jr;
  opcodes[0x19] = addhl_d_e; //ADD HL, DE
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
  opcodes[0x34] = inc_hl;
  opcodes[0x35] = dec_hl;
  opcodes[0x36] = ld_to_mem_h_l_immediate;
  opcodes[0x37] = scf;
  opcodes[0x38] = jrC;
  opcodes[0x39] = addhl_spr;
  opcodes[0x3A] = ldd_a_hl;
  opcodes[0x3B] = dec16_spr;
  opcodes[0x3C] = inc_a;
  opcodes[0x3D] = dec_a;
  opcodes[0x3E] = ld_a_immediate;
  opcodes[0x3F] = ccf;
}

void loop()
{
  uint16_t smaller_img[1024];
  uint16_t dst_index = 0;
  for (uint16_t y = 0; y < sizeof(dpixels); y += 3)
  {
    for (uint16_t x = 0; x < sizeof(dpixels[y]); x += 3)
    {
      uint16_t src_index = y * 64 + x;
      smaller_img[dst_index++] = dpixels[src_index];
    }
  }
  
  memcpy(dpixels, smaller_img, sizeof(smaller_img));
  // put your main code here, to run repeatedly:
}
