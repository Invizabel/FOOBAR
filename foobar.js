let storage = require("storage");

function op_00()
{
    return function nop(){ PC++; return 4 };
}

opcode();
