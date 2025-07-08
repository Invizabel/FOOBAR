let storage = require("storage");
let malloc = ffi("void* malloc(int)");
let free = ffi("void free(void*)");

let MEM = [];
let FirstROMPage = malloc(512);
let ROM = malloc(512);
let REG = malloc(8);
let opcodes = Uint8Array(256);

let FLAGS = {Z:false,N:false,H:false,C:false};

let joypad_dpad = 0xef;
let joypad_buttons=0xdf;
let keys_dpad = 0xef;
let keys_buttons = 0xdf;

let ROMbank = 1; 
let ROMbankoffset = (ROMbank-1)*16384;
let PC = 0;
let SP = 0;
let IME = false;
let cpu_halted=false;
let A = 0x0111;
let B = 0x0000;
let C = 0x0001;
let D = 0x0010;
let E = 0x0011;
let H = 0x0100;
let L = 0x0101;
let HL = 0x0110;
let Immediate = 257;
let BC = 258;
let DE = 259;
let SPr = 260;

function readMem(addr)
{
    if (addr <= 0x3fff) return ROM[ addr ];
    if (addr <= 0x7fff) return ROM[ addr + ROMbankoffset ];
    if (addr >= 0xA000 && addr <=0xBFFF) return cartRAM[ addr + RAMbankoffset ];
    if (addr===0xFF00)
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

function readMem16(addr)
{
    return [readMem(addr+1),readMem(addr)]
}

function ld16(a,b,c)
{
    if (b===Immediate)
    {
        if (a===HL) return function()
        {
            let s = readMem16(readMem( PC+1 ) + (readMem( PC+2 )<<8));

            REG[H] = s[0];
            REG[L] = s[1];

            PC+=3;
            return 12;
        }
        return function()
        {
            SP = readMem( PC+1 ) + (readMem( PC+2 ) << 8);
            PC+=3;
            return 12;
        }
    }
    if (c===Immediate) return function()
    {
        REG[a] = readMem( PC+2 );
        REG[b] = readMem( PC+1 );

        PC+=3;
        return 12;
    }
    return function()
    {
        SP = (REG[H]<<8) + REG[L];
        PC++;
        return 8;
    }
}

opcodes[0x00] = function nop(){ PC++; return 4 };
opcodes[0x01] = ld16(B,C,Immediate);
