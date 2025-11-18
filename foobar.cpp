#include <cstdint>
#include <string.h>

int PC;
uint8_t MEMORY[64 * 1024];
uint16_t BC;

const uint8_t A = 0b111;

void write(uint16_t addr, uint8_t value)
{
    MEMORY[addr] = value;
}

int opcodes(char * opcode)
{
    if (strcmp(opcode, "00"))
    {
        return 0;
    }
    
    if (strcmp(opcode, "01"))
    {
        uint8_t low = PC;
        PC += 1;
        uint8_t high = PC;
        PC += 1;
        BC = (high << 8) | low;
    }

    if (strcmp(opcode, "02") == 0)
    {
        write(BC, A);
    }

    return 0;
}

int main()
{
    char opcode[] = "02";
    opcodes(opcode);
}
