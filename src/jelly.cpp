#include <furi.h>
#include <furi_hal.h>
#include <storage/storage.h>

extern "C"
{
    #include "jelly.h"
}

int main()
{
    Storage * storage = (Storage*)furi_record_open("storage");    
    File* rom_file = storage_file_alloc(storage);
    uint8_t ROM[512];
    title_array t = {0};
    if(storage_file_open(rom_file, "/ext/kirby.gb", FSAM_READ, FSOM_OPEN_EXISTING))
    {
        storage_file_read(rom_file, ROM, 512);
        t = read_title(ROM);
    }
    
    
    if(storage_file_open(rom_file, "/ext/kirby.txt", FSAM_WRITE, FSOM_OPEN_ALWAYS))
    {
        storage_file_write(rom_file, &t, sizeof(t));
    }

    storage_file_close(rom_file);
    storage_file_free(rom_file);
    furi_record_close("storage");
    
    return 0;
}
