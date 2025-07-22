#include <Arduino.h>
#include "peanut_gb.h"
#define ENABLE_SOUND 0
#define ENABLE_LCD 1

struct priv_t
{
  uint8_t * rom;
  uint8_t * cart_ram;
};

static struct gb_s gb;
enum gb_init_error_e ret;
static struct priv_t priv;

uint8_t gb_rom_read(struct gb_s * gb, const uint_fast32_t addr)
{
  const struct priv_t * const p = (const struct priv_t *)gb->direct.priv;
  return p->rom[addr];
}

uint8_t gb_cart_ram_read(struct gb_s * gb, const uint_fast32_t addr)
{
  const struct priv_t * const p = (const struct priv_t * const)gb->direct.priv;
  return p->cart_ram[addr];
}

void gb_cart_ram_write(struct gb_s * gb, const uint_fast32_t addr,const uint8_t new_val)
{
  const struct priv_t * const p = (const struct priv_t * const)gb->direct.priv;
  p->cart_ram[addr] = new_val;
}

void gb_error(struct gb_s *gb, const enum gb_error_e gb_err, const uint16_t new_val)
{
  const char * gb_err_str[GB_INVALID_MAX] = {
    "UNKNOWN",
    "INVALID OPCODE",
    "INVALID READ",
    "INVALID WRITE",
    "HALT FOREVER"
  };
  struct priv_t * priv = (struct priv_t *)gb->direct.priv;

  free(priv->cart_ram);
  free(priv->rom);
}


void setup()
{
  Serial1.begin(230400);
  char * rom_file_name = NULL;
  static struct gb_s gb;
  static struct priv_t priv;
  enum gb_init_error_e ret;

  /*
  ret = gb_init(&gb, &gb_rom_read, &gb_cart_ram_read, &gb_cart_ram_write, &gb_error, &priv);
  if(ret != GB_INIT_NO_ERROR)
  {
    Serial1.write(-1);
    exit(0);
  }
  */
}

void loop()
{
  char recv[256];
  uint16_t index = 0;
  while (Serial1.available())
  {
    char c = Serial1.read();
    if (c == 'Z')
    {
      Serial1.write("A");
      index = 0;
    }
    
    else
    {
        recv[index++] = c;
    }
  }
}
