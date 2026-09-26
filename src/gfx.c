/* Minimal anti-aliased 2D drawing into screen strips (UI, menus, HUD). */
#include <math.h>
#include "gfx.h"
#include "platform.h"

Canvas gc;

void g_begin(uint16_t *px, int y0, int n) {
  gc.px = px;
  gc.y0 = y0;
  gc.y1 = y0 + n;
}

static inline void put(int x, int y, uint16_t c, int a) {
  if (x < 0 || x >= SCREEN_W || y < gc.y0 || y >= gc.y1 || a <= 0) return;
  uint16_t *p = gc.px + (y - gc.y0) * SCREEN_W + x;
  *p = a >= 32 ? c : mix565(*p, c, a);
}

static void span(int x0, int x1, int y, uint16_t c, int a) {
  if (y < gc.y0 || y >= gc.y1) return;
  if (x0 < 0) x0 = 0;
  if (x1 > SCREEN_W) x1 = SCREEN_W;
  uint16_t *p = gc.px + (y - gc.y0) * SCREEN_W;
  if (a >= 32)
    for (int x = x0; x < x1; x++) p[x] = c;
  else
    for (int x = x0; x < x1; x++) p[x] = mix565(p[x], c, a);
}

void g_rect(int x, int y, int w, int h, uint16_t c, int a) {
  int ya = y < gc.y0 ? gc.y0 : y, yb = y + h > gc.y1 ? gc.y1 : y + h;
  for (int yy = ya; yy < yb; yy++) span(x, x + w, yy, c, a);
}

void g_rrect(int x, int y, int w, int h, int r, uint16_t c, int a) {
  if (r * 2 > h) r = h / 2;
  if (r * 2 > w) r = w / 2;
  int ya = y < gc.y0 ? gc.y0 : y, yb = y + h > gc.y1 ? gc.y1 : y + h;
  for (int yy = ya; yy < yb; yy++) {
    int dy = yy < y + r ? y + r - yy : (yy >= y + h - r ? yy - (y + h - r) + 1 : 0);
    if (!dy) {
      span(x, x + w, yy, c, a);
      continue;
    }
    float cy = dy - 0.5f;
    /* corner pixels with coverage, straight middle */
    for (int dx = r; dx > 0; dx--) {
      float cx = dx - 0.5f;
      float d = sqrtf(cx * cx + cy * cy);
      float cov = (float)r - d + 0.5f;
      if (cov <= 0) continue;
      int aa = cov >= 1 ? a : (int)(a * cov);
      put(x + r - dx, yy, c, aa);
      put(x + w - r + dx - 1, yy, c, aa);
    }
    span(x + r, x + w - r, yy, c, a);
  }
}

void g_button(int x, int y, int w, int h, int r, uint16_t top, uint16_t bottom, int halo) {
  if (halo) g_rrect(x - 2, y - 2, w + 4, h + 4, r + 2, C_WHITE, 32);
  g_rrect(x - 1, y - 1, w + 2, h + 2, r + 1, C_BLACK, 32);
  g_rrect(x, y, w, h, r, bottom, 32);
  g_rrect(x, y, w, h - 3, r, top, 32);
}

const Glyph *font_glyph(const Font *f, int ch) {
  /* glyphs are sorted by code; the small and medium fonts are a contiguous range */
  int d = ch - f->g[0].code;
  if (d >= 0 && d < f->n && f->g[d].code == ch) return &f->g[d];
  int lo = 0, hi = f->n - 1;
  while (lo <= hi) {
    int m = (lo + hi) >> 1, c = f->g[m].code;
    if (c == ch) return &f->g[m];
    if (c < ch) lo = m + 1;
    else hi = m - 1;
  }
  return 0;
}

int glyph_alpha(const Font *f, const Glyph *g, int x, int y) {
  x -= g->xoff;
  y -= g->y0;
  if (x < 0 || y < 0 || x >= g->w || y >= g->h) return 0;
  int i = y * g->w + x;
  return (f->data[g->off + (i >> 2)] >> ((i & 3) * 2)) & 3;
}

int g_text_w(const Font *f, const char *s) {
  int w = 0;
  for (; *s; s++) {
    const Glyph *g = font_glyph(f, (unsigned char)*s);
    if (g) w += g->adv;
  }
  return w;
}

static void blit2(const uint8_t *data, int gw, int gh, int x, int y, uint16_t c, int a) {
  int ya = y < gc.y0 ? gc.y0 : y, yb = y + gh > gc.y1 ? gc.y1 : y + gh;
  for (int yy = ya; yy < yb; yy++) {
    for (int xx = 0; xx < gw; xx++) {
      int i = (yy - y) * gw + xx;
      int v = (data[i >> 2] >> ((i & 3) * 2)) & 3;
      if (v) put(x + xx, yy, c, v == 3 ? a : a * v / 3);
    }
  }
}

void g_text(const Font *f, int x, int y, const char *s, uint16_t c, int a) {
  if (y >= gc.y1 || y + f->h <= gc.y0) return;
  for (; *s; s++) {
    const Glyph *g = font_glyph(f, (unsigned char)*s);
    if (!g) continue;
    if (g->w) blit2(f->data + g->off, g->w, g->h, x + g->xoff, y + g->y0, c, a);
    x += g->adv;
  }
}

void g_text_c(const Font *f, int cx, int y, const char *s, uint16_t c, int a) {
  g_text(f, cx - g_text_w(f, s) / 2, y, s, c, a);
}

void g_icon(const Icon *ic, int x, int y, uint16_t c, int a) {
  if (y >= gc.y1 || y + ic->h <= gc.y0) return;
  blit2(ic->data, ic->gw, ic->gh, x + ic->xoff, y + ic->y0, c, a);
}
