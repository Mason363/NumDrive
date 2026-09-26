#ifndef GFX_H
#define GFX_H
#include <stdint.h>
#include "font.h"

/* Drawing happens into horizontal strips of the screen (rows [y0, y1), full width). */
typedef struct {
  uint16_t *px;
  int y0, y1;
} Canvas;
extern Canvas gc;

#define RGB(r, g, b) ((uint16_t)((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3)))
#define C_WHITE 0xFFFF
#define C_BLACK 0x0000

static inline uint16_t mix565(uint16_t d, uint16_t s, int a) { /* a: 0..32 */
  uint32_t dd = (d | (uint32_t)d << 16) & 0x07E0F81Fu;
  uint32_t ss = (s | (uint32_t)s << 16) & 0x07E0F81Fu;
  uint32_t r = (dd + (((ss - dd) * (uint32_t)a) >> 5)) & 0x07E0F81Fu;
  return (uint16_t)(r | r >> 16);
}

void g_begin(uint16_t *px, int y0, int n);
void g_rect(int x, int y, int w, int h, uint16_t c, int a);
void g_rrect(int x, int y, int w, int h, int r, uint16_t c, int a);
/* rounded rectangle with a darker bottom band, black outline and optional white halo */
void g_button(int x, int y, int w, int h, int r, uint16_t top, uint16_t bottom, int halo);
int g_text_w(const Font *f, const char *s);
void g_text(const Font *f, int x, int y, const char *s, uint16_t c, int a);
void g_text_c(const Font *f, int cx, int y, const char *s, uint16_t c, int a);
void g_icon(const Icon *ic, int x, int y, uint16_t c, int a);
int glyph_alpha(const Font *f, const Glyph *g, int x, int y); /* 0..3 */
const Glyph *font_glyph(const Font *f, int ch);

#endif
