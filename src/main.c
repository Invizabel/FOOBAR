// this bit is required to get peanut_gb to compile
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wredundant-decls"
#define PEANUT_GB_IMPLEMENTATION
#include "core.h"
#pragma GCC diagnostic pop

// c imports
#include <stdint.h>
#include <stdlib.h>

// Flipper imports
#include "dolphin/dolphin.h"
#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <storage/storage.h>

struct priv_t
{
    // Pointer to allocated memory holding GB file
    uint8_t * rom;
    // Pointer to allocated memory holding save file
    uint8_t * cart_ram;
    // Size of the cart_ram in bytes
    size_t save_size;
    // Pointer to boot ROM binary if available
    uint8_t * bootrom;
    // Colour palette for each BG, OBJ0, and OBJ1
    uint16_t selected_palette[3][4];
	uint16_t fb[LCD_HEIGHT][LCD_WIDTH];
};

// Returns a byte from the ROM file at the given address
uint8_t gb_rom_read(struct gb_s * gb, const uint_fast32_t addr)
{
	const struct priv_t * const p = gb->direct.priv;
	return p->rom[addr];
}

// Returns a byte from the cartridge RAM at the given address
uint8_t gb_cart_ram_read(struct gb_s * gb, const uint_fast32_t addr)
{
	const struct priv_t * const p = gb->direct.priv;
	return p->cart_ram[addr];
}

// Writes a given byte to the cartridge RAM at the given address
void gb_cart_ram_write(struct gb_s * gb, const uint_fast32_t addr,
		       const uint8_t val)
{
	const struct priv_t * const p = gb->direct.priv;
	p->cart_ram[addr] = val;
}

uint8_t gb_bootrom_read(struct gb_s * gb, const uint_fast16_t addr)
{
	const struct priv_t * const p = gb->direct.priv;
	return p->bootrom[addr];
}

void lcd_draw_line(struct gb_s * gb, const uint8_t pixels[160], const uint_fast8_t line)
{
	struct priv_t * priv = gb->direct.priv;

	for(unsigned int x = 0; x < LCD_WIDTH; x++)
	{
		priv->fb[line][x] = priv->selected_palette[(pixels[x] & LCD_PALETTE_ALL) >> 4][pixels[x] & 3];
	}
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
    canvas_clear(canvas);

    // draw here

    canvas_commit(canvas);
}

int main()
{
    FuriMessageQueue * queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    ViewPort * view_port = view_port_alloc();
    view_port_draw_callback_set(view_port, draw_callback, NULL);
    view_port_input_callback_set(view_port, input_callback, queue);
    Gui * gui = (Gui *)furi_record_open("gui");
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
}
