#ifndef FONT_H
#define FONT_H
#include <stdint.h>

typedef struct {
  uint8_t code, adv;
  int8_t xoff;
  uint8_t y0, w, h;
  uint16_t off;
} Glyph;

typedef struct {
  uint8_t h, base, cap_top, n;
  const Glyph *g;
  const uint8_t *data;
} Font;

typedef struct {
  uint8_t w, h;
  int8_t xoff, y0;
  uint8_t gw, gh;
  const uint8_t *data;
} Icon;

extern const Font font_s, font_m, font_l;
extern const Icon ic_pause_o, ic_pause_f, ic_play, ic_restart, ic_back, ic_next, ic_left, ic_right, ic_lock;

#endif
