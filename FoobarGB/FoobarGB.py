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
    global MEM
    global timerEnable
    global timerPrescaler
    global SoundEnabled
    global LCD_scan

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
  
  match ROM[0x147]:
    # Cartridge Type = ROM[0x147]
    case 0: # ROM ONLY
        # do any type 0 carts have switchable ram?
        pass
    
    case 0x01 | 0x02 | 0x03:
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

    case 0x05 | 0x06:
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
    case 0x11 | 0x12 | 0x13:
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
            # RTC registers here
            pass