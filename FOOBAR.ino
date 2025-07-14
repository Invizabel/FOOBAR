// Registers and Pointers
uint8_t REG[8];
uint16_t PC = 0;
uint16_t SP = 0;
// Flags and State
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

typedef uint16_t (*OpcodeFunc)();
OpcodeFunc opcodes[256];

struct Pair { uint16_t hi, lo; };

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

uint16_t writeMem(uint16_t addr, uint16_t data)
{
  /*
  if (addr <= 0x7fff)
  { 
    doMBC(addr, data);
    return;
  }*/

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
  ld_to_mem(B,C,A);
}

void setup()
{
  opcodes[0x00] = nop;
  opcodes[0x01] = ld16_bc_immediate;
  opcodes[0x03] = ld_to_mem_bca;
}

void loop()
{
  // put your main code here, to run repeatedly:
}
