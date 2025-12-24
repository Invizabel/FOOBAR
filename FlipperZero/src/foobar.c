#include "dolphin/dolphin.h"
#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <stdbool.h>
#include <stdint.h>
#include <storage/storage.h>
#include "core.h"

bool is_title = false;
char title_buffer[16];

title_array fetch_title()
{
    Storage * storage = (Storage*)furi_record_open("storage");
    File* rom_file = storage_file_alloc(storage);
    uint8_t ROM[512];
    title_array t = {0};
    if(storage_file_open(rom_file, "/ext/roms/Kirby_s Dream Land.gb", FSAM_READ, FSOM_OPEN_EXISTING))
    {
        storage_file_read(rom_file, ROM, 512);
        bool is_valid = verify_checksum(ROM);
        if (is_valid)
        {
            t = read_title(ROM);
        }
    }

    storage_file_close(rom_file);
    storage_file_free(rom_file);
    furi_record_close("storage");

    return t;
}

static void input_callback(InputEvent* event, void* context)
{
    FuriMessageQueue* queue = (FuriMessageQueue*)context;
    if(event->type == InputTypeShort || event->type == InputTypeRepeat || event->type == InputTypePress)
    {
    }

    if(event->type == InputTypeRelease)
    {
    }
    
    
    furi_message_queue_put(queue, event, FuriWaitForever);
}

void draw_callback(Canvas* canvas, void* context)
{
    UNUSED(context);

    if (!is_title)
    {
        title_array t = fetch_title();
        for (int i = 0; i < 16; i++)
        {
            title_buffer[i] = t.title[i];
        }
        is_title = true;
    }
    
    canvas_clear(canvas);
    canvas_draw_str(canvas,2,8,title_buffer);

    canvas_commit(canvas);
}

int main()
{
    FuriMessageQueue* queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    ViewPort* view_port = view_port_alloc();
    view_port_draw_callback_set(view_port, draw_callback, NULL);
    view_port_input_callback_set(view_port, input_callback, queue);
    Gui* gui = (Gui*)furi_record_open("gui");
    gui_add_view_port(gui, view_port, GuiLayerFullscreen);
    dolphin_deed(DolphinDeedPluginGameStart);
    InputEvent event;
    bool running = true;
    while(running)
    {
        if(furi_message_queue_get(queue, &event, FuriWaitForever) == FuriStatusOk)
        {
            if(event.type == InputTypeShort && event.key == InputKeyBack)
            {
                running = false;
            }
        }
        view_port_update(view_port);
    }

    view_port_enabled_set(view_port, false);
    furi_message_queue_free(queue);
    gui_remove_view_port(gui, view_port);
    view_port_free(view_port);
    furi_record_close(RECORD_GUI);
    return 0;
    
    return 0;
}
