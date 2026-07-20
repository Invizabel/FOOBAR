lastSampleTime = 0
frame = 0
frameRate = 0
sampleFrames = 5

MEM = []
FirstROMPage = []
ROM = []
ROMbank = 1
ROMbankoffset = (ROMbank - 1) * 0x4000

cartRAM = []
RAMbank = 0
RAMbankoffset = RAMbank * 0x2000 - 0xA000
RAMenabled = False
MBCRamMode = 0

divPrescale = 0
timerPrescaler = 0
timerLength = 1
timerEnable = False
LCD_enabled = False
LCD_lastmode = 1
LCD_scan = 0
joypad_dpad = 0xef # 0 = pressed
joypad_buttons = 0xdf # 0 = pressed
keys_dpad = 0xef # 0 = pressed
keys_buttons = 0xdf # 0 = pressed

REG = []

FLAGS = {
    "Z": False,
    "N": False,
    "H": False,
    "C": False,
}

PC = 0

SP = 0

IME = False # Interrupt master enable
cpu_halted = False

A = 0b111
B = 0b000
C = 0b001
D = 0b010
E = 0b011
H = 0b100
L = 0b101

HL = 0b110

Immediate = 257
BC = 258
DE = 259
SPr = 260

opcodes = []

ADD = 1
ADC = 2
SUB = 3
SBC = 4
AND = 5
OR  = 6
XOR = 7
CP  = 8

def readMem(addr):
    if addr <= 0x3fff:
        return ROM[addr]
    
    if addr <= 0x7fff:
        return ROM[addr + ROMbankoffset]
    
    # Cartridge RAM
    if addr >= 0xA000 and addr <=0xBFFF:
        return cartRAM[addr + RAMbankoffset]
    
    # Joypad
    if addr == 0xFF00:
        if MEM[0xFF00] & 0x20:
            return joypad_dpad & keys_dpad
        elif MEM[0xFF00] & 0x10:
            return joypad_buttons & keys_buttons
        else:
            return 0xFF
        
    return MEM[addr]

def readMem16(addr):
    # just presuming that some peripherals will need to hook on 16 bits
    return [readMem(addr + 1), readMem(addr)]

def  writeMem(addr, data):

    if addr <= 0x7fff: 
        doMBC(addr, data)
        return
    
    if addr >= 0xA000 and addr <= 0xBFFF and RAMenabled:
        cartRAM[addr + RAMbankoffset] = data
        return
    
    # DIV register: reset
    if addr == 0xFF04:
        MEM[0xFF04] = 0
        return
    
    # Timer control
    if addr == 0xFF07:
        timerEnable = ((data&(1<<2))!=0)
        timerLength = [1024, 16, 64, 256][data & 0x3]
        timerPrescaler = timerLength # +cycles for this instruction?
        MEM[addr] = 0xF8|data
        return
    
    # Sound Control
    if addr == 0xFF26:
        if data & 1 << 7:
            MEM[0xFF26] = data & (1 << 7)
            SoundEnabled = True
            # <todo> audioCtx.resume()
        
        else:
            SoundEnabled = False
            # should we set each oscillator to amplitude zero too?
            # <todo> audioCtx.suspend()
            # Zero all sound registers
            resetSoundRegisters()
    
        return
    
    if addr >= 0xFF10 and addr <= 0xFF25:
        if not SoundEnabled:
            return
         
        # FF10 - NR10 - Channel 1 Sweep register (R/W)
        if addr == 0xFF10:
            ''' <todo>
            sound[1].sweepTime = (data>>4)&0x7;
            sound[1].sweepPrescaler = sound[1].sweepTime;
            sound[1].sweepDir = (data&(1<<3)) ? 0 : 1;
            sound[1].sweepShift = data&0x7;
            '''

            MEM[addr] = data & 0x80
            return
        
        # FF11 - NR11 - Channel 1 Sound length/Wave pattern duty (R/W)
        if addr == 0xFF11:
            MEM[addr] = data
            # <todo> sound[1].duty(data>>6)
            return
        
        # FF12 - NR12 - Channel 1 Volume Envelope (R/W)
        if addr == 0xFF12:
            MEM[addr] = data
            ''' <todo>
            sound[1].envDirection = (data&(1<<3)) ? 1: -1;
            sound[1].envSpeed = data&0x7;
            sound[1].envCounter = 0;
            '''
            return
        
        # FF13 - NR13 - Channel 1 Frequency lo (Write Only)
        if addr == 0xFF13:
            ''' <todo>
            sound[1].freqnum=(((MEM[0xFF14]&0x7)<<8)+ data);
            sound[1].freq(  131072/(2048-  sound[1].freqnum  )  )
            '''
            MEM[addr] = data
            return
        
        # FF14 - NR14 - Channel 1 Frequency hi (R/W)
        if addr == 0xFF14:
            # bit 7 is initialize
            ''' <todo>
            sound[1].freqnum=(((data&0x7)<<8)+ MEM[0xFF13]);
            sound[1].freq(  131072/(2048-  sound[1].freqnum  )  )
            '''

            if data & (1 << 7):
                ''' <todo>
                sound[1].initialized = true
                sound[1].env = MEM[0xFF12]>>4; // default envelope value
                sound[1].envCounter = 0;
                sound[1].amp( sound[1].env/15 )

                sound[1].lengthEnabled = (data&(1<<6)) !=0;
                sound[1].length = (64-(MEM[0xFF11]&0x3F));
                '''

                MEM[0xFF26] |= (1<<0) # flag sound 1 as on

            MEM[addr] = data
            return
        
        # FF16 - NR21 - Channel 2 Sound Length/Wave Pattern Duty (R/W)
        # it 7-6 - Wave Pattern Duty (Read/Write)
        # Bit 5-0 - Sound length data (Write Only) (t1: 0-63)
        if addr == 0xFF16:
            MEM[addr] = data
            # <todo> sound[2].duty(data>>6)
            return
        
        # FF17 - NR22 - Channel 2 Volume Envelope (R/W)
        if addr == 0xFF17:
            MEM[addr] = data
            ''' <todo>
            sound[2].envDirection = (data&(1<<3)) ? 1: -1;
            sound[2].envSpeed = data&0x7;
            sound[2].envCounter = 0;'''
            return
        
        # FF18 - NR23 - Channel 2 Frequency lo data (W)
        if addr == 0xFF18:
            # <todo> sound[2].freq(  131072/(2048-  (((MEM[0xFF19]&0x7)<<8)+ data)  )  )
            MEM[addr] = data
            return
        
        # FF19 - NR24 - Channel 2 Frequency hi data (R/W)
        if addr == 0xFF19:
            # <todo> sound[2].freq(  131072/(2048-  (((data&0x7)<<8)+ MEM[0xFF18])  )  )
            # bit 7 is initialize
            if data & (1 << 7):
                ''' <todo>
                sound[2].initialized = true
                sound[2].env = MEM[0xFF17]>>4; //Default envelope value
                sound[2].envCounter = 0;
                sound[2].amp( sound[2].env/15 )

                sound[2].lengthEnabled = (data&(1<<6)) !=0;
                sound[2].length = (64-(MEM[0xFF16]&0x3F));
                '''

                MEM[0xFF26] |= (1 << 1) # flag sound 2 as on

            MEM[addr] = data
            return
        
        # Sound 3 - user-defined waveform
        # "it can output a sound while changing its length, frequency, and level"
        # not sure what changing its length means

        # FF1A - NR30 - Channel 3 Sound on/off (R/W)
        
        if addr == 0xFF1A:
            if data & (1 << 7):
                # sound[3].initialized=true;
                # is this the right (only?) place to load the waveform?
                setSound3Waveform()

            else:
                ''' <todo>
                sound[3].initialized=false;
                sound[3].amp(0)
                '''
            return

        # FF1B - NR31 - Channel 3 Sound Length
        if addr == 0xFF1B:
            MEM[addr] = data
            return
        
        # FF1C - NR32 - Channel 3 Select output level (R/W)
        if addr == 0xFF1C:
            # Really we ought to bit-crush it, but whatever   
            # <todo> if (sound[3].initialized) sound[3].amp( [ 0,0.5,0.25,0.125 ][((data>>5)&0x3)] )
            MEM[addr] = data
            return
        
        # FF1D - NR33 - Channel 3 Frequency's lower data (W)
        if addr == 0xFF1D:
            # <todo> sound[3].freq(   65536/(2048-  (((MEM[0xFF1E]&0x7)<<8)+ data)  )  )
            MEM[addr] = data
            return

        # FF1E - NR34 - Channel 3 Frequency's higher data (R/W)
        if addr == 0xFF1E:
            # <todo> sound[3].freq(   65536/(2048-  (((data&0x7)<<8)+ MEM[0xFF1D])  )  )
            # bit 7 is initialize
            if data & (1 << 7):
                ''' <todo>
                sound[3].initialized = true

                sound[3].amp( [ 0,0.5,0.25,0.15 ][((MEM[0xFF1C]>>5)&0x3)] )

                sound[3].lengthEnabled = (data&(1<<6)) !=0;
                sound[3].length = (256-MEM[0xFF1B]);
                '''

                MEM[0xFF26] |= (1 << 2) # flag sound 3 as on

            MEM[addr] = data
            return
        
        # Sound 4 - Noise
        # FF20 - NR41 - Channel 4 Sound Length (R/W)
        if addr == 0xFF20:
            MEM[addr] = data
            return
        
        # FF21 - NR42 - Channel 4 Volume Envelope (R/W)
        if addr == 0xFF21:
            MEM[addr] = data
            ''' <todo>
            sound[4].envDirection = (data&(1<<3)) ? 1: -1;
            sound[4].envSpeed = data&0x7;
            sound[4].envCounter = 0;'''
            return
        
        # FF22 - NR43 - Channel 4 Polynomial Counter (R/W)
        if addr == 0xFF22:
            ''' <todo>
            sound[4].freq(data>>4, data&0x7)
            sound[4].polySteps(data&(1<<3))'''

            MEM[addr] = data
            return
        
        # FF23 - NR44 - Channel 4 Counter/consecutive; Inital (R/W)
        if addr == 0xFF23:
            ''' <todo>
            sound[4].initialized = true
            sound[4].env = MEM[0xFF21]>>4; //Default envelope value
            sound[4].envCounter = 0;
            sound[4].amp( sound[4].env/15 )
            sound[4].length = (64-(MEM[0xFF20]&0x3F));'''

            MEM[0xFF26] |= (1 << 3) # flag sound 4 as on
            
            # <todo> sound[4].lengthEnabled = (data&(1<<6)) !=0;
            MEM[addr] = data
            return
        
        # FF24 - NR50 - Channel control / ON-OFF / Volume (R/W)
        if addr == 0xFF24:
            # Bit 7   - Output Vin to SO2 terminal (1=Enable)
            # Bit 6-4 - SO2 output level (volume)  (0-7)
            # Bit 3   - Output Vin to SO1 terminal (1=Enable)
            # Bit 2-0 - SO1 output level (volume)  (0-7)

            # is level zero mute ? "minimum level"
            ''' <todo> sound.SO2.gain.setValueAtTime(((data>>4)&0x7)/7, audioCtx.currentTime)
            sound.SO1.gain.setValueAtTime((data&0x7)/7,      audioCtx.currentTime)'''
            MEM[addr] = data 
            return
        
        # FF25 - NR51 - Selection of Sound output terminal (R/W)
        if addr == 0xff25:
            con = (MEM[0xff25] ^ data) & data
            dis = (MEM[0xff25] ^ data) & (~ data)
            
            for i in range(4):
                if con & (1<<i):
                    # <todo> sound[i+1].gainNode.connect(sound.SO1)
                    pass
                
                if dis & (1 << i):
                    # <todo> sound[i+1].gainNode.disconnect(sound.SO1)
                    pass
                
                if con & (1 << (4 + i)):
                    # <todo> sound[i+1].gainNode.connect(sound.SO2)
                    pass
                
                if dis & (1 << (4 + i)):
                    # <todo> sound[i+1].gainNode.disconnect(sound.SO2)
                    pass

            MEM[addr] = data
            return
        
        return
    
    if addr >= 0xFF30 and addr <= 0xFF3F:
        # <todo> sound[3].waveChanged=true;
        pass

    # LCD control
    if addr == 0xFF40:
        cc = data & (1 << 7)
        if (LCD_enabled != cc):
            LCD_enabled = bool(cc)
            if not LCD_enabled:
                # this should also probably set all pixels to white
                LCD_scan = 0
                MEM[0xFF41] = (MEM[0xFF41] & 0xFC) +1

    if addr == 0xFF41:
        # don't overwrite the lowest two bits (mode)
        MEM[0xFF41] &= 0x3
        data &= 0xFC
        MEM[0xFF41] |= 0x80 | data; # BGB has highest bit always set
        return
    
    # LY - write causes reset
    if addr == 0xFF44:
        MEM[0xFF44] = 0
        return
    
    # FF46 - DMA - DMA Transfer and Start Address (W)
    if addr == 0xFF46:
        st= data << 8
        for i in range(0x9F): 
            MEM[0xFE00 + i] = readMem(st + i)
        return
    
    # disable bootrom
    if addr == 0xFF50:
        for i in range(256):
            ROM[i] = FirstROMPage[i]
        return
    
    MEM[addr] = data

def writeMem16(addr, dataH, dataL):
    writeMem(addr, dataL)
    writeMem(addr + 1, dataH)

def doMBC(addr, data):
    global RAMenabled
    global ROMbankoffset
    global RAMbankoffset
  
    # Cartridge Type = ROM[0x147]
    if ROM[0x147] == 0: # ROM ONLY
        # do any type 0 carts have switchable ram?
        pass
    
    elif ROM[0x147] == 0x01 | ROM[0x147] == 0x02 | ROM[0x147] == 0x03:
        if addr <= 0x1FFF:
            RAMenabled = ((data & 0x0F) == 0xA)

        elif addr <= 0x3FFF:
            data &= 0x1F
            if (data == 0):
                data = 1  # MBC1 translates bank 0 to bank 1 (apparently regardless of upper bits)
            # set lowest 5 bits of bank number
            ROMbank = (ROMbank & 0xE0) | (data & 0x1F)
            ROMbankoffset = (ROMbank - 1) * 0x4000 % len(ROM)

        elif addr <= 0x5fff:
            data &= 0x3
            if MBCRamMode == 0:
                ROMbank = (ROMbank & 0x1F) | (data << 5)
                ROMbankoffset = (ROMbank-1) * 0x4000  % len(ROM)
            else:
                RAMbank = data
                RAMbankoffset = RAMbank * 0x2000 - 0xA000

        else:
            MBCRamMode = data & 1
            if MBCRamMode == 0:
                RAMbank = 0
                RAMbankoffset = RAMbank * 0x2000 - 0xA000
            else:
                ROMbank &= 0x1F
                ROMbankoffset = (ROMbank - 1) * 0x4000  % len(ROM)

    elif ROM[0x147] == 0x05 | ROM[0x147] == 0x06:
        if addr <= 0x1FFF:
            if (addr & 0x0100) == 0:
                RAMenabled = ((data & 0x0F) == 0xA) 
            elif addr <= 0x3FFF:
                data &= 0x0F
            
            if data == 0:
                data = 1
            ROMbank = data
            ROMbankoffset = (ROMbank - 1) * 0x4000 % len(ROM)

    # case 0x08: #  ROM+RAM
    # case 0x09: #  ROM+RAM+BATTERY
    # case 0x0B: #  MMM01
    # case 0x0C: #  MMM01+RAM
    # case 0x0D: #  MMM01+RAM+BATTERY
    # case 0x0F: #  MBC3+TIMER+BATTERY
    # case 0x10: #  MBC3+TIMER+RAM+BATTERY
    elif ROM[0x147] == 0x11 | ROM[0x147] == 0x12 | ROM[0x147] == 0x13:
        if addr <= 0x1FFF:
            RAMenabled = ((data & 0x0F) == 0xA)
        elif addr <= 0x3FFF:
            if data == 0:
                data = 1 # allows access to banks 0x20, 0x40, 0x60
            ROMbank = data & 0x7F
            ROMbankoffset = (ROMbank - 1) * 0x4000 % len(ROM)
        elif addr <= 0x5fff:
            if data < 8:
                RAMbank = data;
                RAMbankoffset = RAMbank * 0x2000 - 0xA000
            else:
                # RTC registers here
                pass
        else:
            # RTC latch
            pass

    elif ROM[0x147] == 0x19 | ROM[0x147] == 0x1A | ROM[0x147] == 0x1B:
        if addr <= 0x1FFF:
            RAMenabled = ((data & 0x0F) == 0xA)

        elif addr <= 0x2FFF:
            # Allows access to bank 0
            ROMbank &= 0x100
            ROMbank |= data
            ROMbankoffset = (ROMbank - 1) * 0x4000
            while ROMbankoffset > len(rom):
                ROMbankoffset -= len(rom)
            
        elif addr <= 0x3FFF:
            ROMbank &= 0xFF
            if data & 1:
                ROMbank += 0x100
            
            ROMbankoffset = (ROMbank - 1) * 0x4000
            while ROMbankoffset > len(ROM):
                ROMbankoffset -= len(ROM)

        elif addr <= 0x5fff:
            RAMbank = data & 0x0F
            RAMbankoffset = RAMbank * 0x2000 - 0xA000

    else:
        raise NotImplementedError("Unimplemented memory controller")
      
def resetSoundRegisters():
   MEM[0xFF10] = 0x80 # NR10
   MEM[0xFF11] = 0xBF # NR11
   MEM[0xFF12] = 0xF3 # NR12
   MEM[0xFF13] = 0
   MEM[0xFF14] = 0xBF # NR14
   MEM[0xFF15] = 0xFF # NA
   MEM[0xFF16] = 0x3F # NR21
   MEM[0xFF17] = 0x00 #  NR22
   MEM[0xFF18] = 0
   MEM[0xFF19] = 0xBF # NR24
   MEM[0xFF1A] = 0x7F # NR30
   MEM[0xFF1B] = 0xFF # NR31
   MEM[0xFF1C] = 0x9F # NR32
   MEM[0xFF1D] = 0
   MEM[0xFF1E] = 0xBF # NR33
   MEM[0xFF1F] = 0xFF # NA
   MEM[0xFF20] = 0xFF # NR41
   MEM[0xFF21] = 0x00 # NR42
   MEM[0xFF22] = 0x00 # NR43
   MEM[0xFF23] = 0xBF # NR30
   MEM[0xFF24] = 0x77 # NR50
   writeMem(0xFF25,0xF3) # NR51
   MEM[0xFF26] = 0xF1 # NR52

def ld(a,b):
    if b == Immediate:
        REG[a] = readMem(PC + 1)
        PC += 2
        return 8

    REG[a] = REG[b]
    PC += 1
    return 4

def ld_from_mem(a, b, c):
    if b == Immediate:
        REG[a] = readMem(readMem(PC + 1) + (readMem(PC + 2) << 8))
        PC += 3
        return 16

    REG[a] = readMem((REG[b] << 8) + REG[c])
    PC += 1
    return 8

def ld_to_mem(a, b, c):
    if a == Immediate:
        writeMem(readMem(PC + 1) + (readMem(PC + 2) << 8), REG[b])
        PC += 3
        return 16
    
    if c == Immediate:
        writeMem((REG[a] << 8) + REG[b], readMem(PC + 1))
        PC += 2
        return 12
    
    writeMem((REG[a] << 8) + REG[b], REG[c])
    PC += 1
    return 8

def ld16(a, b, c):
    global SP

    if b == Immediate:
        if (a == HL):
            # mem to hl
            s = readMem16(readMem(PC + 1) + (readMem(PC + 2 ) <<8 ))

            REG[H] = s[0]
            REG[L] = s[1]

            PC += 3
            return 12

        # immediate into SP... 
        SP = readMem(PC + 1) + (readMem(PC + 2) << 8)
        PC += 3
        return 12

    if c == Immediate:
        REG[a] = readMem( PC+2 )
        REG[b] = readMem( PC+1 )

        PC += 3
        return 12

    # ld sp, hl
    SP = (REG[H] << 8) + REG[L]
    PC += 1
    return 8

def ldd(a, b):
    if a == HL:
        writeMem((REG[H] << 8) + REG[L], REG[A])

        if REG[L] == 0:
            REG[H] -= 1
        REG[L] -= 1

        PC += 1
        return 8

    REG[A] = readMem((REG[H] << 8) + REG[L])

    if (REG[L]==0):
        REG[H] -= 1
    
    REG[L] -= 1

    PC += 1
    return 8

def ldi(a, b):
    if a == HL:
        writeMem((REG[H] << 8) + REG[L], REG[A])
        if (REG[L] == 255):
            REG[H] += 1
        
        REG[L] += 1
        
        PC += 1
        return 8
    
    REG[A] =  readMem((REG[H] << 8) + REG[L])
    if REG[L] == 255:
        REG[H] += 1
    
    REG[L] += 1

    PC += 1
    return 8

def ldc(a, b):
    if a == A:
        REG[A] = readMem(0xFF00 + REG[C])
        P += 1
        return 8
    
    writeMem(0xFF00 + REG[C], REG[A])
    PC += 1
    return 8

def ldh(a, b):
    if a == A:
        REG[A] = readMem(0xFF00 + readMem(PC + 1))
        PC += 2
        return 12

    writeMem(0xFF00 + readMem(PC + 1), REG[A])
    PC += 2
    return 12

def ALU(op, a, b):
    if b == Immediate:
        REG[A] = ALU_process_8bit(op, readMem(PC + 1))
        PC += 2
        return 8

    if b == HL:
        REG[A] = ALU_process_8bit(op, readMem((REG[H] << 8) + REG[L]))
        PC += 1
        return 8
    
    REG[A] = ALU_process_8bit(op, REG[b])
    PC += 1
    return 4

def ALU_process_8bit(op, b):
    result = REG[A]
    FLAGS["N"] = False
    if (op):
        if op == ADD:
            FLAGS["H"] == bool((((REG[A] & 0x0F) + (b & 0x0F)) & 0x10))
            result += B

        elif op == ADC:
            FLAGS.H = bool(((REG[A] & 0x0F) + (b & 0x0F) + FLAGS["C"]) & 0x10)
            result += b + FLAGS["C"]

        elif op == SUB:
            result -= B
            FLAGS["N"] = True
            FLAGS["H"] = bool(((REG[A] & 0x0F) - (b & 0x0F)) & 0x10)

        elif op == CP:
            result -= b
            FLAGS["N"] = True
            FLAGS["H"] = bool(((REG[A] & 0x0F) - (b & 0x0F)) & 0x10)
            FLAGS["Z"] = ((result & 0xff) == 0)
            FLAGS["C"] = result > 255 or result < 0

        elif op == SBC:
            result -= b + FLAGS["C"]
            FLAGS["N"] = True
            FLAGS["H"] = bool(((REG[A] & 0x0F) - (b & 0x0F) - FLAGS["C"]) & 0x10)

        elif op == AND:
            result &= b
            FLAGS["H"] = True

        elif op == OR:
            result |= b
            FLAGS["H"] = False

        elif op == XOR:
            result ^= b
            FLAGS["H"] = False

    FLAGS["Z"] = ((result & 0xff) == 0)
    FLAGS["C"] = result > 255 or result < 0

    return result & 0xFF

def inc(a):
    return incdec(a, 1)

def dec(a):
    return incdec(a, -1)

def incdec(r, offset):
    if r == HL:
        writeMem((REG[H] << 8) + REG[L], incdec_process_8bit(readMem((REG[H] << 8) + REG[L]), offset))
        PC += 1
        return 12

    REG[r] = incdec_process_8bit(REG[r], offset)
    PC += 1
    return 4

def incdec_process_8bit(a, offset):
    result = a + offset
    FLAGS["H"] = bool(((a & 0x0F) + offset) & 0x10)
    FLAGS["N"] = offset == -1
    FLAGS["Z"] = ((result & 0xff) == 0)
    return result

def inc16(a, b):
    if a == SPr:
        SP += 1
        PC += 1
        return 8
    
    if REG[b] == 255:
        REG[a] += 1
    
    REG[b] += 1
    PC += 1
    return 8

def dec16(a, b):
    if a == SPr
        SP -= 1
        PC += 1
        return 8
        
    if (REG[b] == 0):
        REG[a] -= 1
    
    REG[b] -= 1
    PC += 1
    return 8

def signedOffset(b):
    return (b - 256) if b > 127 else b

def jrNZ():
    if FLAGS["Z"]:
        PC += 2
        return 8
    
    PC += 2 + signedOffset(readMem(PC + 1))
    return 12

def jrNC():
    if FLAGS["C"]:
        PC += 2
        return 8
    
    PC += 2 + signedOffset(readMem(PC + 1))
    return 12

def jrZ():
    if not FLAGS["Z"]:
        PC += 2
        return 8

    PC += 2 + signedOffset(readMem(PC + 1))
    return 12

def jrC():
    if not FLAGS["C"]:
        PC += 2
        return 8

    PC += 2 + signedOffset(readMem(PC + 1))
    return 12

def jr():
    PC += 2 + signedOffset(readMem(PC + 1))
    return 12

def jp():
    PC = readMem(PC + 1) + (readMem(PC + 2) << 8)
    return 16