let storage = require("storage");

function opcode()
{
    let hits = {};
    hits[0x00] = function nop(){ PC++; return 4 };
    hits[0x01] = ld16(B,C,Immediate);
    hits[0x02] = ld_to_mem(B,C,A);
    hits[0x03] = inc16(B,C);
    hits[ 0x04 ] = inc(B);
    return hits;
}

opcode();
