import pygame
import sys

lastSampleTime = 0
frame = 0
frameRate = 0
sampleFrames = 5

MEM = [0] * 0x10000
FirstROMPage = [0] * 512
ROM = [0] * 32768
ROMbank = 1
ROMbankoffset = (ROMbank - 1) * 0x4000

cartRAM = [0] * 0x8000 # some carts have up to 128K of ram?
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

REG = [0] * 8

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

opcodes = [0] * 256
CBcodes = [0] * 256

ADD = 1
ADC = 2
SUB = 3
SBC = 4
AND = 5
OR  = 6
XOR = 7
CP  = 8

RLC = 1
RRC = 2
RL  = 3
RR  = 4
SLA = 5
SRA = 6
SRL = 7

unused = 4 # GMB locks when called

pygame.init()

screen = pygame.display.set_mode((160*4, 144*4))
pygame.display.set_caption("FoobarGB")
clock = pygame.time.Clock()

def Game():
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            pygame.display.flip()
            clock.tick(60)

def readMem(addr = 0):
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

def readMem16(addr = 0):
    # just presuming that some peripherals will need to hook on 16 bits
    return [readMem(addr + 1), readMem(addr)]

def  writeMem(addr = 0, data = 0):
    global timerEnable
    global timerPrescaler

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
            sound[1].freq( 131072/(2048-  sound[1].freqnum  )  )
            '''
            MEM[addr] = data
            return
        
        # FF14 - NR14 - Channel 1 Frequency hi (R/W)
        if addr == 0xFF14:
            # bit 7 is initialize
            ''' <todo>
            sound[1].freqnum=(((data&0x7)<<8)+ MEM[0xFF13]);
            sound[1].freq( 131072/(2048-  sound[1].freqnum  )  )
            '''

            if data & (1 << 7):
                ''' <todo>
                sound[1].initialized = true
                sound[1].env = MEM[0xFF12]>>4; #  default envelope value
                sound[1].envCounter = 0;
                sound[1].amp(sound[1].env/15 )

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
            # <todo> sound[2].freq( 131072/(2048-  (((MEM[0xFF19]&0x7)<<8)+ data)  )  )
            MEM[addr] = data
            return
        
        # FF19 - NR24 - Channel 2 Frequency hi data (R/W)
        if addr == 0xFF19:
            # <todo> sound[2].freq( 131072/(2048-  (((data&0x7)<<8)+ MEM[0xFF18])  )  )
            # bit 7 is initialize
            if data & (1 << 7):
                ''' <todo>
                sound[2].initialized = true
                sound[2].env = MEM[0xFF17]>>4; # Default envelope value
                sound[2].envCounter = 0;
                sound[2].amp(sound[2].env/15 )

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
                ''' <todo>
                setSound3Waveform()
                '''
                pass

            else:
                ''' <todo>
                sound[3].initialized=false;
                sound[3].amp(0)
                '''
                pass
            return

        # FF1B - NR31 - Channel 3 Sound Length
        if addr == 0xFF1B:
            MEM[addr] = data
            return
        
        # FF1C - NR32 - Channel 3 Select output level (R/W)
        if addr == 0xFF1C:
            # Really we ought to bit-crush it, but whatever   
            # <todo> if (sound[3].initialized) sound[3].amp([0, 0.5, 0.25, 0.125][((data>>5)&0x3)] )
            MEM[addr] = data
            return
        
        # FF1D - NR33 - Channel 3 Frequency's lower data (W)
        if addr == 0xFF1D:
            # <todo> sound[3].freq(  65536/(2048-  (((MEM[0xFF1E]&0x7)<<8)+ data)  )  )
            MEM[addr] = data
            return

        # FF1E - NR34 - Channel 3 Frequency's higher data (R/W)
        if addr == 0xFF1E:
            # <todo> sound[3].freq(  65536/(2048-  (((data&0x7)<<8)+ MEM[0xFF1D])  )  )
            # bit 7 is initialize
            if data & (1 << 7):
                ''' <todo>
                sound[3].initialized = true

                sound[3].amp([0, 0.5, 0.25, 0.15][((MEM[0xFF1C]>>5)&0x3)] )

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
            sound[4].env = MEM[0xFF21]>>4; # Default envelope value
            sound[4].envCounter = 0;
            sound[4].amp(sound[4].env/15 )
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

def writeMem16(addr = 0, dataH = 0, dataL = 0):
    writeMem(addr, dataL)
    writeMem(addr + 1, dataH)

def doMBC(addr = 0, data = 0):
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
    writeMem(0xFF25, 0xF3) # NR51
    MEM[0xFF26] = 0xF1 # NR52

def ld(a = 0, b = 0):
    global PC

    if b == Immediate:
        REG[a] = readMem(PC + 1)
        PC += 2
        return 8

    REG[a] = REG[b]
    PC += 1
    return 4

def ld_from_mem(a = 0, b = 0, c = 0):
    global PC

    if b == Immediate:
        REG[a] = readMem(readMem(PC + 1) + (readMem(PC + 2) << 8))
        PC += 3
        return 16

    REG[a] = readMem((REG[b] << 8) + REG[c])
    PC += 1
    return 8

def ld_to_mem(a = 0, b = 0, c = 0):
    global PC

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

def ld16(a = 0, b = 0, c = 0):
    global PC
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
        REG[a] = readMem(PC + 2)
        REG[b] = readMem(PC + 1)

        PC += 3
        return 12

    # ld sp, hl
    SP = (REG[H] << 8) + REG[L]
    PC += 1
    return 8

def ldd(a = 0, b = 0):
    global PC

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

def ldi(a = 0, b = 0):
    global PC

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

def ldc(a = 0, b = 0):
    global PC

    if a == A:
        REG[A] = readMem(0xFF00 + REG[C])
        PC += 1
        return 8
    
    writeMem(0xFF00 + REG[C], REG[A])
    PC += 1
    return 8

def ldh(a = 0, b = 0):
    global PC

    if a == A:
        REG[A] = readMem(0xFF00 + readMem(PC + 1))
        PC += 2
        return 12

    writeMem(0xFF00 + readMem(PC + 1), REG[A])
    PC += 2
    return 12

def ALU(op = 0, a = 0, b = 0):
    global PC

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

def ALU_process_8bit(op = 0, b = 0):
    result = REG[A]
    FLAGS["N"] = False
    if (op):
        if op == ADD:
            FLAGS["H"] == bool((((REG[A] & 0x0F) + (b & 0x0F)) & 0x10))
            result += B

        elif op == ADC:
            FLAGS["H"] = bool(((REG[A] & 0x0F) + (b & 0x0F) + FLAGS["C"]) & 0x10)
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

def inc(a = 0):
    return incdec(a, 1)

def dec(a = 0):
    return incdec(a, -1)

def incdec(r = 0, offset = 0):
    global PC

    if r == HL:
        writeMem((REG[H] << 8) + REG[L], incdec_process_8bit(readMem((REG[H] << 8) + REG[L]), offset))
        PC += 1
        return 12

    REG[r] = incdec_process_8bit(REG[r], offset)
    PC += 1
    return 4

def incdec_process_8bit(a = 0, offset = 0):
    result = a + offset
    FLAGS["H"] = bool(((a & 0x0F) + offset) & 0x10)
    FLAGS["N"] = offset == -1
    FLAGS["Z"] = ((result & 0xff) == 0)
    return result

def inc16(a = 0, b = 0):
    global PC
    global SP

    if a == SPr:
        SP += 1
        PC += 1
        return 8
    
    if REG[b] == 255:
        REG[a] += 1
    
    REG[b] += 1
    PC += 1
    return 8

def dec16(a = 0, b = 0):
    global PC
    global SP

    if a == SPr:
        SP -= 1
        PC += 1
        return 8
        
    if (REG[b] == 0):
        REG[a] -= 1
    
    REG[b] -= 1
    PC += 1
    return 8

def signedOffset(b = 0):
    return (b - 256) if b > 127 else b

def jrNZ():
    global PC

    if FLAGS["Z"]:
        PC += 2
        return 8
    
    PC += 2 + signedOffset(readMem(PC + 1))
    return 12

def jrNC():
    global PC

    if FLAGS["C"]:
        PC += 2
        return 8
    
    PC += 2 + signedOffset(readMem(PC + 1))
    return 12

def jrZ():
    global PC

    if not FLAGS["Z"]:
        PC += 2
        return 8

    PC += 2 + signedOffset(readMem(PC + 1))
    return 12

def jrC():
    global PC

    if not FLAGS["C"]:
        PC += 2
        return 8

    PC += 2 + signedOffset(readMem(PC + 1))
    return 12

def jr():
    PC += 2 + signedOffset(readMem(PC + 1))
    return 12

def jp():
    global PC
    
    PC = readMem(PC + 1) + (readMem(PC + 2) << 8)
    return 16

def jpNZ():
    global PC

    if FLAGS["Z"]:
        PC += 3
        return 12
    
    PC = readMem(PC + 1) + (readMem(PC + 2) << 8)
    return 16

def jpNC():
    global PC

    if FLAGS["C"]:
        PC += 3
        return 12
    
    PC = readMem(PC+1)+(readMem(PC+2)<<8)
    return 16

def jpZ():
    global PC

    if not FLAGS["Z"]:
        PC += 3
        return 12
    
    PC = readMem(PC + 1) + (readMem(PC + 2) << 8)
    return 16


def jpC():
    global PC

    if not FLAGS["C"]:
        PC += 3
        return 12
    
    PC = readMem(PC + 1) + (readMem(PC + 2) << 8)
    return 16

def jpHL():
    global PC

    PC = (REG[H] << 8) + REG[L]
    return 4

def push(a = 0, b = 0):
    global PC
    global SP

    if a == A:
        flags = (FLAGS["Z"] << 7) + (FLAGS["N"] << 6) + (FLAGS["H"] << 5) + (FLAGS["C"] << 4)
        SP -= 2
        writeMem16(SP, REG[A], flags)
        PC += 1
        return 16
    
    SP -= 2
    writeMem16(SP, REG[a], REG[b])
    PC += 1
    return 16

def pop(a = 0, b = 0):
    global PC
    global SP

    if a == A:
        s = readMem16(SP)
        REG[A] = s[0]
        FLAGS["Z"] = (s[1] & (1 << 7)) != 0
        FLAGS["N"] = (s[1] & (1 << 6)) != 0
        FLAGS["H"] = (s[1] & (1 << 5)) != 0
        FLAGS["C"] = (s[1] & (1 << 4)) != 0
        SP += 2
        PC += 1
        return 12
    
    s = readMem16(SP)
    REG[a] = s[0]
    REG[b] = s[1]
    SP += 2
    PC += 1
    return 12

def call():
    global PC
    global SP

    SP -= 2
    npc = PC + 3
    writeMem16(SP, npc >> 8, npc & 0xFF)
    PC = readMem(PC + 1) + (readMem(PC + 2) << 8)
    return 24

def callNZ():
    global PC

    if FLAGS["Z"]:
        PC += 3
        return 12
    
    return call()

def callNC():
    global PC

    if FLAGS["C"]:
        PC += 3
        return 12
    
    return call()

def callZ():
    global PC

    if not FLAGS["Z"]:
        PC += 3
        return 12
    
    return call()

def callC():
    global PC

    if not FLAGS["C"]:
        PC += 3
        return 12
    
    return call()

def ret():
    global PC

    s = readMem16(SP)
    SP += 2
    PC = (s[0] << 8) + s[1]
    return 16

def retNZ():
    global PC

    if FLAGS["Z"]:
        PC += 1
        return 8
    
    ret()
    return 20

def retNC():
    global PC

    if FLAGS["C"]:
        PC += 1
        return 8
    
    ret()
    return 20

def retZ():
    global PC

    if not FLAGS["Z"]:
        PC += 1
        return 8
    
    ret()
    return 20

def retC():
    global PC

    if not FLAGS["C"]:
        PC += 1
        return 8
    
    ret()
    return 20

def reti():
    global IME
    IME = True
    return ret()

def ei():
    global PC

    global IME
    IME = True
    PC += 1
    return 4

def di():
    global PC
    global IME

    IME = False
    PC += 1
    return 4

def rst(a = 0):
    global PC
    global SP

    SP -= 2
    npc = PC + 1
    PC = a
    return 16

def shift_fast(op = 0, a = 0):
    global PC

    REG[a] = shift_process(op, REG[a])
    FLAGS["Z"] = False
    PC += 1
    return 4

def shift(op = 0, a = 0):
    global PC

    if a == HL:
        addr = (REG[H] << 8) + REG[L]
        writeMem(addr, shift_process(op, readMem(addr)))
        PC += 1
        return 16

    REG[a] = shift_process(op, REG[a])
    PC += 1
    return 8

def shift_process(op = 0, a = 0):
    bit7 = a >> 7
    bit0 = a & 1

    # Rotate byte left, save carry
    if op == RLC:
        a = ((a << 1) & 0xff) + bit7
        FLAGS["C"] = bool(bit7)

    # Rotate byte right, save carry
    elif op == RRC:
        a = ((a >> 1) & 0xff) + (bit0 << 7)
        FLAGS["C"] = bool(bit0)
    
    # Rotate left through carry
    elif op == RL:
        a = ((a << 1) & 0xff) + FLAGS["C"] 
        FLAGS["C"] = bool(bit7)
    
    # Rotate right through carry
    elif op == RR:
        a = ((a >> 1 ) & 0xff) + (FLAGS["C"] << 7)
        FLAGS["C"] = bool(bit0)
    
    # Shift left
    elif SLA:
        a = ((a << 1 )& 0xff)
        FLAGS["C"] = bool(bit7)

    # Shift right arithmetic 
    elif SRA:
        a = ((a >> 1) & 0xff) + (bit7 << 7)
        FLAGS["C"] = bool(bit0)

    # Shift right logical
    elif op == SRL:
        a = ((a >> 1) & 0xff)
        FLAGS["C"] = bool(bit0)

    FLAGS["N"] = False
    FLAGS["H"] = False
    FLAGS["Z"] = (a & 0xFF) == 0
    return a

def ccf():
    global PC

    FLAGS["N"] = False
    FLAGS["H"] = False
    FLAGS["C"] = not FLAGS["C"]
    PC += 1
    return 4

def scf():
    global PC

    FLAGS["N"] = False
    FLAGS["H"] = False
    FLAGS["C"] = True
    PC += 1
    return 4

def cpl():
    global PC

    REG[A] = ~REG[A]
    FLAGS["N"] = True
    FLAGS["H"] = True
    PC += 1
    return 4

def addHL(a = 0, b = 0):
    global PC

    if a == SPr:
        REG[L] += (SP & 0xFF)
        c = 1 if REG[L] > 255 else 0
        h = REG[H] + (SP >> 8) + c
        FLAGS["H"] = bool(((REG[H] & 0x0F) + ((SP >> 8) & 0x0F) + c) & 0x10)
        REG[H] = h
        FLAGS["C"] = (h > 255)
        FLAGS["N"] = False
        PC += 1
        return 8
    
    REG[L] += REG[b]
    c = 1 if REG[L] > 255 else 0
    h = REG[H] + REG[a] + c
    FLAGS["H"] = bool(((REG[H] & 0x0F) + (REG[a] & 0x0F) + c) & 0x10)
    REG[H] = h
    FLAGS["C"] = (h > 255)
    FLAGS["N"] = False
    PC += 1
    return 8


# http:# gbdev.gg8.se/wiki/articles/DAA
def daa():
    global PC
    
    if FLAGS["N"]:
        if FLAGS["C"]:
            REG[A] -= 0x60
        
        if FLAGS["H"]:
            REG[A] -= 0x06
    else:
        if REG[A] > 0x99 or FLAGS["C"]:
            REG[A] += 0x60
            FLAGS["C"] = True
        
        if (REG[A] & 0x0f) > 0x09 or FLAGS["H"]:
            REG[A] += 0x06

    FLAGS["Z"] = REG[A] == 0
    FLAGS["H"] = False

    PC += 1
    return 4

def ld_imm_sp():
    global PC

    writeMem16(readMem(PC + 1) + (readMem(PC + 2) << 8), SP >> 8, SP & 0xFF)
    PC += 3
    return 20

def ld_hl_spdd():
    global PC

    b = signedOffset(readMem(PC + 1))

    FLAGS["H"] = bool(((SP & 0x0F) + (b & 0x0F)) & 0x010)
    FLAGS["C"]= bool(((SP & 0xFF) + (b & 0xFF)) & 0x100)

    n = SP + b
    REG[H] =(n >> 8)
    REG[L] = n&0xFF

    FLAGS["N"] = False
    FLAGS["Z"] = False
    PC += 2
    return 12

def add_sp_n():
    global PC

    b = signedOffset(readMem(PC + 1))

    FLAGS["H"]= bool(((SP & 0x0F) + (b & 0x0F)) & 0x010)
    FLAGS["C"] = bool(((SP & 0xFF) + (b & 0xFF)) & 0x100)

    SP += b
    FLAGS["N"] = False
    FLAGS["Z"] = False

    SP &= 0xFFFF
    PC += 2
    return 16

def halt():
    global PC

    # if interrupts disabled, stall 1 cycle, skip next instruction and continue
    if (IME):
        cpu_halted = True
    
    PC += 1
    return 4

def stop():
    global PC

    # TODO
    PC += 2
    return 4

def nop():
    global PC

    PC += 1
    return 4

def opcode_cb():
    global PC
    
    return CBcodes[readMem(PC := PC + 1)]()

opcodes[0x00] = nop()
opcodes[0x01] = ld16(B, C, Immediate)
opcodes[0x02] = ld_to_mem(B, C, A)
opcodes[0x03] = inc16(B, C)
opcodes[0x04] = inc(B)
opcodes[0x05] = dec(B)
opcodes[0x06] = ld(B, Immediate)
opcodes[0x07] = shift_fast(RLC, A)
opcodes[0x08] = ld_imm_sp
opcodes[0x09] = addHL(B, C)
opcodes[0x0A] = ld_from_mem(A, B, C)
opcodes[0x0B] = dec16(B, C)
opcodes[0x0C] = inc(C)
opcodes[0x0D] = dec(C)
opcodes[0x0E] = ld(C, Immediate)
opcodes[0x0F] = shift_fast(RRC, A)

opcodes[0x10] = stop
opcodes[0x11] = ld16(D, E, Immediate)
opcodes[0x12] = ld_to_mem(D, E, A)
opcodes[0x13] = inc16(D, E)
opcodes[0x14] = inc(D)
opcodes[0x15] = dec(D)
opcodes[0x16] = ld(D, Immediate)
opcodes[0x17] = shift_fast(RL, A) 
opcodes[0x18] = jr
opcodes[0x19] = addHL(D, E)
opcodes[0x1A] = ld_from_mem(A, D, E)
opcodes[0x1B] = dec16(D, E)
opcodes[0x1C] = inc(E)
opcodes[0x1D] = dec(E)
opcodes[0x1E] = ld(E, Immediate)
opcodes[0x1F] = shift_fast(RR, A)

opcodes[0x20] = jrNZ
opcodes[0x21] = ld16(H, L, Immediate)
opcodes[0x22] = ldi(HL, A)
opcodes[0x23] = inc16(H, L)
opcodes[0x24] = inc(H)
opcodes[0x25] = dec(H)
opcodes[0x26] = ld(H, Immediate)
opcodes[0x27] = daa
opcodes[0x28] = jrZ
opcodes[0x29] = addHL(H, L)
opcodes[0x2A] = ldi(A, HL)
opcodes[0x2B] = dec16(H, L)
opcodes[0x2C] = inc(L)
opcodes[0x2D] = dec(L)
opcodes[0x2E] = ld(L, Immediate)
opcodes[0x2F] = cpl

opcodes[0x30] = jrNC
opcodes[0x31] = ld16(SPr, Immediate)
opcodes[0x32] = ldd(HL, A)
opcodes[0x33] = inc16(SPr)
opcodes[0x34] = inc(HL)
opcodes[0x35] = dec(HL)
opcodes[0x36] = ld_to_mem(H, L, Immediate)
opcodes[0x37] = scf
opcodes[0x38] = jrC
opcodes[0x39] = addHL(SPr)
opcodes[0x3A] = ldd(A, HL)
opcodes[0x3B] = dec16(SPr)
opcodes[0x3C] = inc(A)
opcodes[0x3D] = dec(A)
opcodes[0x3E] = ld(A, Immediate)
opcodes[0x3F] = ccf

opcodes[0x40] = ld(B, B)
opcodes[0x41] = ld(B, C)
opcodes[0x42] = ld(B, D)
opcodes[0x43] = ld(B, E)
opcodes[0x44] = ld(B, H)
opcodes[0x45] = ld(B, L)
opcodes[0x46] = ld_from_mem(B, H, L)
opcodes[0x47] = ld(B, A)

opcodes[0x48] = ld(C, B)
opcodes[0x49] = ld(C, C)
opcodes[0x4A] = ld(C, D)
opcodes[0x4B] = ld(C, E)
opcodes[0x4C] = ld(C, H)
opcodes[0x4D] = ld(C, L)
opcodes[0x4E] = ld_from_mem(C, H, L)
opcodes[0x4F] = ld(C, A)

opcodes[0x50] = ld(D, B)
opcodes[0x51] = ld(D, C)
opcodes[0x52] = ld(D, D)
opcodes[0x53] = ld(D, E)
opcodes[0x54] = ld(D, H)
opcodes[0x55] = ld(D, L)
opcodes[0x56] = ld_from_mem(D, H, L)
opcodes[0x57] = ld(D, A)

opcodes[0x58] = ld(E, B)
opcodes[0x59] = ld(E, C)
opcodes[0x5A] = ld(E, D)
opcodes[0x5B] = ld(E, E)
opcodes[0x5C] = ld(E, H)
opcodes[0x5D] = ld(E, L)
opcodes[0x5E] = ld_from_mem(E, H, L)
opcodes[0x5F] = ld(E, A)

opcodes[0x60] = ld(H, B)
opcodes[0x61] = ld(H, C)
opcodes[0x62] = ld(H, D)
opcodes[0x63] = ld(H, E)
opcodes[0x64] = ld(H, H)
opcodes[0x65] = ld(H, L)
opcodes[0x66] = ld_from_mem(H, H, L)
opcodes[0x67] = ld(H, A)

opcodes[0x68] = ld(L, B)
opcodes[0x69] = ld(L, C)
opcodes[0x6A] = ld(L, D)
opcodes[0x6B] = ld(L, E)
opcodes[0x6C] = ld(L, H)
opcodes[0x6D] = ld(L, L)
opcodes[0x6E] = ld_from_mem(L, H, L)
opcodes[0x6F] = ld(L, A)

opcodes[0x70] = ld_to_mem(H, L, B)
opcodes[0x71] = ld_to_mem(H, L, C)
opcodes[0x72] = ld_to_mem(H, L, D)
opcodes[0x73] = ld_to_mem(H, L, E)
opcodes[0x74] = ld_to_mem(H, L, H)
opcodes[0x75] = ld_to_mem(H, L, L)
opcodes[0x76] = halt
opcodes[0x77] = ld_to_mem(H, L, A)

opcodes[0x78] = ld(A, B)
opcodes[0x79] = ld(A, C)
opcodes[0x7A] = ld(A, D)
opcodes[0x7B] = ld(A, E)
opcodes[0x7C] = ld(A, H)
opcodes[0x7D] = ld(A, L)
opcodes[0x7E] = ld_from_mem(A, H, L)
opcodes[0x7F] = ld(A, A)

opcodes[0x80] = ALU(ADD, A, B)
opcodes[0x81] = ALU(ADD, A, C)
opcodes[0x82] = ALU(ADD, A, D)
opcodes[0x83] = ALU(ADD, A, E)
opcodes[0x84] = ALU(ADD, A, H)
opcodes[0x85] = ALU(ADD, A, L)
opcodes[0x86] = ALU(ADD, A, HL)
opcodes[0x87] = ALU(ADD, A, A)

opcodes[0x88] = ALU(ADC, A, B)
opcodes[0x89] = ALU(ADC, A, C)
opcodes[0x8A] = ALU(ADC, A, D)
opcodes[0x8B] = ALU(ADC, A, E)
opcodes[0x8C] = ALU(ADC, A, H)
opcodes[0x8D] = ALU(ADC, A, L)
opcodes[0x8E] = ALU(ADC, A, HL)
opcodes[0x8F] = ALU(ADC, A, A)

opcodes[0x90] = ALU(SUB, A, B)
opcodes[0x91] = ALU(SUB, A, C)
opcodes[0x92] = ALU(SUB, A, D)
opcodes[0x93] = ALU(SUB, A, E)
opcodes[0x94] = ALU(SUB, A, H)
opcodes[0x95] = ALU(SUB, A, L)
opcodes[0x96] = ALU(SUB, A, HL)
opcodes[0x97] = ALU(SUB, A, A)

opcodes[0x98] = ALU(SBC, A, B)
opcodes[0x99] = ALU(SBC, A, C)
opcodes[0x9A] = ALU(SBC, A, D)
opcodes[0x9B] = ALU(SBC, A, E)
opcodes[0x9C] = ALU(SBC, A, H)
opcodes[0x9D] = ALU(SBC, A, L)
opcodes[0x9E] = ALU(SBC, A, HL)
opcodes[0x9F] = ALU(SBC, A, A)

opcodes[0xA0] = ALU(AND, A, B)
opcodes[0xA1] = ALU(AND, A, C)
opcodes[0xA2] = ALU(AND, A, D)
opcodes[0xA3] = ALU(AND, A, E)
opcodes[0xA4] = ALU(AND, A, H)
opcodes[0xA5] = ALU(AND, A, L)
opcodes[0xA6] = ALU(AND, A, HL)
opcodes[0xA7] = ALU(AND, A, A)

opcodes[0xA8] = ALU(XOR, A, B)
opcodes[0xA9] = ALU(XOR, A, C)
opcodes[0xAA] = ALU(XOR, A, D)
opcodes[0xAB] = ALU(XOR, A, E)
opcodes[0xAC] = ALU(XOR, A, H)
opcodes[0xAD] = ALU(XOR, A, L)
opcodes[0xAE] = ALU(XOR, A, HL)
opcodes[0xAF] = ALU(XOR, A, A)

opcodes[0xB0] = ALU(OR, A, B)
opcodes[0xB1] = ALU(OR, A, C)
opcodes[0xB2] = ALU(OR, A, D)
opcodes[0xB3] = ALU(OR, A, E)
opcodes[0xB4] = ALU(OR, A, H)
opcodes[0xB5] = ALU(OR, A, L)
opcodes[0xB6] = ALU(OR, A, HL)
opcodes[0xB7] = ALU(OR, A, A)

opcodes[0xB8] = ALU(CP, A, B)
opcodes[0xB9] = ALU(CP, A, C)
opcodes[0xBA] = ALU(CP, A, D)
opcodes[0xBB] = ALU(CP, A, E)
opcodes[0xBC] = ALU(CP, A, H)
opcodes[0xBD] = ALU(CP, A, L)
opcodes[0xBE] = ALU(CP, A, HL)
opcodes[0xBF] = ALU(CP, A, A)

opcodes[0xC0] = retNZ
opcodes[0xC1] = pop(B, C)
opcodes[0xC2] = jpNZ
opcodes[0xC3] = jp
opcodes[0xC4] = callNZ
opcodes[0xC5] = push(B, C)
opcodes[0xC6] = ALU(ADD, A, Immediate)
opcodes[0xC7] = rst(0x00)
opcodes[0xC8] = retZ
opcodes[0xC9] = ret
opcodes[0xCA] = jpZ
opcodes[0xCB] = opcode_cb
opcodes[0xCC] = callZ
opcodes[0xCD] = call
opcodes[0xCE] = ALU(ADC, A, Immediate)
opcodes[0xCF] = rst(0x08)

opcodes[0xD0] = retNC
opcodes[0xD1] = pop(D, E)
opcodes[0xD2] = jpNC
opcodes[0xD3] = unused
opcodes[0xD4] = callNC
opcodes[0xD5] = push(D, E)
opcodes[0xD6] = ALU(SUB, A, Immediate)
opcodes[0xD7] = rst(0x10)
opcodes[0xD8] = retC
opcodes[0xD9] = Game
opcodes[0xDA] = jpC
opcodes[0xDB] = unused
opcodes[0xDC] = callC
opcodes[0xDD] = unused
opcodes[0xDE] = ALU(SBC, A, Immediate)
opcodes[0xDF] = rst(0x18)

opcodes[0xE0] = ldh(Immediate, A)
opcodes[0xE1] = pop(H, L)
opcodes[0xE2] = ldc(C, A)
opcodes[0xE3] = unused
opcodes[0xE4] = unused
opcodes[0xE5] = push(H, L)
opcodes[0xE6] = ALU(AND, A, Immediate)
opcodes[0xE7] = rst(0x20)
opcodes[0xE8] = add_sp_n
opcodes[0xE9] = jpHL
opcodes[0xEA] = ld_to_mem(Immediate, A)
opcodes[0xEB] = unused
opcodes[0xEC] = unused
opcodes[0xED] = unused
opcodes[0xEE] = ALU(XOR, A, Immediate)
opcodes[0xEF] = rst(0x28)

opcodes[0xF0] = ldh(A, Immediate)
opcodes[0xF1] = pop(A, FLAGS)
opcodes[0xF2] =  ldc(A, C)
opcodes[0xF3] = di
opcodes[0xF4] = unused
opcodes[0xF5] = push(A, FLAGS)
opcodes[0xF6] = ALU(OR, A, Immediate)
opcodes[0xF7] = rst(0x30)
opcodes[0xF8] = ld_hl_spdd
opcodes[0xF9] = ld16()
opcodes[0xFA] = ld_from_mem(A, Immediate)
opcodes[0xFB] = ei
opcodes[0xFC] = unused
opcodes[0xFD] = unused
opcodes[0xFE] = ALU(CP, A, Immediate)
opcodes[0xFF] = rst(0x38)

CBcodes[0x00] = shift(RLC, B)
CBcodes[0x01] = shift(RLC, C)
CBcodes[0x02] = shift(RLC, D)
CBcodes[0x03] = shift(RLC, E)
CBcodes[0x04] = shift(RLC, H)
CBcodes[0x05] = shift(RLC, L)
CBcodes[0x06] = shift(RLC, HL)
CBcodes[0x07] = shift(RLC, A)
CBcodes[0x08] = shift(RRC, B)
CBcodes[0x09] = shift(RRC, C)
CBcodes[0x0A] = shift(RRC, D)
CBcodes[0x0B] = shift(RRC, E)
CBcodes[0x0C] = shift(RRC, H)
CBcodes[0x0D] = shift(RRC, L)
CBcodes[0x0E] = shift(RRC, HL)
CBcodes[0x0F] = shift(RRC, A)

CBcodes[0x10] = shift(RL, B)
CBcodes[0x11] = shift(RL, C)
CBcodes[0x12] = shift(RL, D)
CBcodes[0x13] = shift(RL, E)
CBcodes[0x14] = shift(RL, H)
CBcodes[0x15] = shift(RL, L)
CBcodes[0x16] = shift(RL, HL)
CBcodes[0x17] = shift(RL, A)
CBcodes[0x18] = shift(RR, B)
CBcodes[0x19] = shift(RR, C)
CBcodes[0x1A] = shift(RR, D)
CBcodes[0x1B] = shift(RR, E)
CBcodes[0x1C] = shift(RR, H)
CBcodes[0x1D] = shift(RR, L)
CBcodes[0x1E] = shift(RR, HL)
CBcodes[0x1F] = shift(RR, A)

CBcodes[0x20] = shift(SLA, B)
CBcodes[0x21] = shift(SLA, C)
CBcodes[0x22] = shift(SLA, D)
CBcodes[0x23] = shift(SLA, E)
CBcodes[0x24] = shift(SLA, H)
CBcodes[0x25] = shift(SLA, L)
CBcodes[0x26] = shift(SLA, HL)
CBcodes[0x27] = shift(SLA, A)
CBcodes[0x28] = shift(SRA, B)
CBcodes[0x29] = shift(SRA, C)
CBcodes[0x2A] = shift(SRA, D)
CBcodes[0x2B] = shift(SRA, E)
CBcodes[0x2C] = shift(SRA, H)
CBcodes[0x2D] = shift(SRA, L)
CBcodes[0x2E] = shift(SRA, HL)
CBcodes[0x2F] = shift(SRA, A)

CBcodes[0x38] = shift(SRL, B)
CBcodes[0x39] = shift(SRL, C)
CBcodes[0x3A] = shift(SRL, D)
CBcodes[0x3B] = shift(SRL, E)
CBcodes[0x3C] = shift(SRL, H)
CBcodes[0x3D] = shift(SRL, L)
CBcodes[0x3E] = shift(SRL, HL)
CBcodes[0x3F] = shift(SRL, A)

def swap(r = 0):
    global PC

    if r == HL:
        a = readMem( (REG[H] << 8) + REG[L])
        a = (a >> 4) + (( a<< 4) & 0xFF)
        writeMem((REG[H] << 8) + REG[L], a)
        FLAGS["Z"] = (a == 0)
        FLAGS["N"] = False
        FLAGS["H"] = False
        FLAGS["C"] = False
        PC += 1
        return 16
    
    REG[r] = (REG[r] >> 4) + ((REG[r] << 4) & 0xFF)
    FLAGS["Z"] = (REG[r] == 0)
    FLAGS["N"] = False
    FLAGS["H"] = False
    FLAGS["C"] = False
    PC += 1
    return 8

CBcodes[0x30] = swap(B)
CBcodes[0x31] = swap(C)
CBcodes[0x32] = swap(D)
CBcodes[0x33] = swap(E)
CBcodes[0x34] = swap(H)
CBcodes[0x35] = swap(L)
CBcodes[0x36] = swap(HL)
CBcodes[0x37] = swap(A)

def bit(b = 0, r = 0):
    global PC

    b = (1 << b)

    if r == HL:
        FLAGS["Z"] = ((readMem((REG[H] << 8) + REG[L]) & b) == 0)
        FLAGS["H"] = True
        FLAGS["N"] = False
        PC += 1
        return 12
    
    FLAGS["Z"] = ((REG[r] & b) == 0)
    FLAGS["H"] = True
    FLAGS["N"] = False
    PC += 1
    return 8

def set(b = 0, r = 0):
    global PC

    b = (1 << b)

    if r == HL:
        writeMem((REG[H] << 8) + REG[L], readMem((REG[H] << 8) + REG[L]) | b)
        PC += 1
        return 16
    
    REG[r] |= b
    PC += 1
    return 8

def res(b = 0, r = 0):
    global PC

    b = ~(1 << b)

    if r == HL:
        writeMem((REG[H] << 8) + REG[L], readMem((REG[H] << 8) + REG[L]) & b)
        PC += 1
        return 16
    
    REG[r] &= b
    PC += 1
    return 8


for i in range(8):
    for j in range(8):
        CBcodes[0x40 + i * 8 + j] = bit(i, j)
        CBcodes[0x80 + i * 8 + j] = res(i, j)
        CBcodes[0xC0 + i * 8 + j] = set(i, j)

bootCode = [
    int(x, 16)
    for x in (
        "31 FE FF AF 21 FF 9F 32 CB 7C 20 FB 21 26 FF 0E 11 3E 80 32 E2 "
        "0C 3E F3 E2 32 3E 77 77 3E FC E0 47 11 A8 00 21 10 80 1A CD 95 "
        "00 CD 96 00 13 7B FE 34 20 F3 11 D8 00 06 08 1A 13 22 23 05 20 "
        "F9 3E 19 EA 10 99 21 2F 99 0E 0C 3D 28 08 32 0D 20 F9 2E 0F 18 "
        "F3 67 3E 64 57 E0 42 3E 91 E0 40 04 1E 02 0E 0C F0 44 FE 90 20 "
        "FA 0D 20 F7 1D 20 F2 0E 13 24 7C 1E 83 FE 62 28 06 1E C1 FE 64 "
        "20 06 7B E2 0C 3E 87 E2 F0 42 90 E0 42 15 20 D2 05 20 4F 16 20 "
        "18 CB 4F 06 04 C5 CB 11 17 C1 CB 11 17 05 20 F5 22 23 22 23 C9 "
        "00 00 00 0D 00 09 11 09 89 39 08 C9 00 0B 00 03 00 0C CC CC 00 "
        "0F 00 00 00 00 EC CC EC CC DD DD 99 99 98 89 EE FB 67 63 6E 0E "
        "CC DD 1F 9F 88 88 00 00 00 00 00 00 00 00 21 A8 00 11 A8 00 1A "
        "13 BE 20 FE 23 7D FE 34 20 F5 06 19 78 86 23 05 20 FB 86 20 FE "
        "3E 01 E0 50"
    ).split()
]

Game()