/* In-game HUD, photo cards (pause / win / lose) and the level list, drawn like Fancade's UI. */
#include <math.h>
#include <string.h>
#include "ui.h"
#include "gfx.h"
#include "render.h"
#include "world.h"
#include "save.h"
#include "platform.h"

/* palette sampled from the original */
#define C_OUTLINE RGB(0x39, 0x39, 0x4c)
#define C_PILL_TXT RGB(0xb1, 0xc0, 0xd3)
#define C_CAPTION RGB(0xa4, 0xb2, 0xc4)
#define C_BLUE RGB(0x00, 0x8c, 0xff)
#define C_BLUE_D RGB(0x00, 0x6a, 0xd8)
#define C_GRAY RGB(0xb2, 0xb7, 0xcb)
#define C_GRAY_D RGB(0x7a, 0x81, 0x99)
#define C_PANEL RGB(0x00, 0xb9, 0xff)
#define C_PANEL_D RGB(0x00, 0x8b, 0xbf)
#define C_PANEL_TXT RGB(0xb4, 0xe4, 0xff)
#define C_GREEN RGB(0x39, 0xc7, 0x46)
#define C_GREEN_D RGB(0x26, 0x8f, 0x34)
#define C_TILE_B RGB(0x00, 0x7a, 0xee)
#define C_TILE_BD RGB(0x00, 0x58, 0xc0)
#define C_LOCK RGB(0x68, 0x76, 0x95)
#define C_LOCK_D RGB(0x56, 0x62, 0x7e)
#define C_SHADOW RGB(0x10, 0x1c, 0x58)
#define C_TITLE_O RGB(0x19, 0x1a, 0x1b)

/* ------------------------------------------------------------------- HUD */
static int hud_t;
static uint8_t hud_btn, hud_used;
static int fade_l, fade_r;
float hud_score, hud_coins;

void hud_reset(void) {
  hud_t = 0;
  hud_score = hud_coins = -1;
  hud_btn = hud_used = 0;
  fade_l = fade_r = 32;
}

void hud_tick(uint8_t buttons) {
  hud_t++;
  hud_btn = buttons;
  hud_used |= buttons;
  if ((hud_used & 1) && fade_l > 0) fade_l -= 4;
  if ((hud_used & 2) && fade_r > 0) fade_r -= 4;
}

static void hint_box(int x, int y, const Icon *ic, bool on) {
  if (on) {
    g_rect(x, y, 18, 18, C_WHITE, 29);
    g_icon(ic, x + 6, y + 6, sky565, 32);
  } else {
    g_rect(x + 2, y + 2, 14, 14, C_WHITE, 10);
    g_rect(x, y, 18, 2, C_WHITE, 20);
    g_rect(x, y + 16, 18, 2, C_WHITE, 20);
    g_rect(x, y + 2, 2, 14, C_WHITE, 20);
    g_rect(x + 16, y + 2, 2, 14, C_WHITE, 20);
    g_icon(ic, x + 6, y + 6, C_WHITE, 24);
  }
}

/* Set Score counter text: white halo, dark outline and a face with a darker extruded bottom */
static void counter_text(const Font *f, int xr, int y, const char *s, uint16_t face, uint16_t side, uint16_t outl,
                         int depth) {
  int x = xr - g_text_w(f, s);
  for (int dy = -2; dy <= depth + 2; dy++)
    for (int dx = -2; dx <= 2; dx++)
      if ((dx != -2 && dx != 2) || (dy != -2 && dy != depth + 2)) g_text(f, x + dx, y + dy, s, C_WHITE, 32);
  for (int dy = -1; dy <= depth + 1; dy++)
    for (int dx = -1; dx <= 1; dx++) g_text(f, x + dx, y + dy, s, outl, 32);
  for (int dy = depth; dy > 0; dy--) g_text(f, x, y + dy, s, side, 32);
  g_text(f, x, y, s, face, 32);
}

/* the gold coin next to the coin count: a diamond with a raised rim and a thick lower edge */
static void coin_icon(int cx, int cy) {
  enum { R = 5, E = 2 };
  for (int y = cy - R - 2; y <= cy + R + E + 2; y++) {
    if (y < gc.y0 || y >= gc.y1) continue;
    for (int x = cx - R - 2; x <= cx + R + 2; x++) {
      int dx = x - cx, dy = y - cy, ax = dx < 0 ? -dx : dx;
      int d = ax + (dy < 0 ? -dy : dy);
      int de = ax + (dy < 0 ? -dy : dy > E ? dy - E : 0);
      uint16_t c;
      int a = 32;
      if (de > R + 2) continue;
      if (de == R + 2) c = C_WHITE, a = 24;
      else if (de == R + 1) c = C_OUTLINE;
      else if (d > R) c = RGB(0xca, 0x84, 0x00);
      else if (d == R) c = dx + dy < 0 ? RGB(0xff, 0xff, 0x80) : RGB(0xff, 0xd8, 0x00);
      else if (d > 2) c = RGB(0xff, 0xd8, 0x00);
      else if (d == 2) c = dx + dy < 0 ? RGB(0xca, 0x84, 0x00) : RGB(0xff, 0xe0, 0x00);
      else c = RGB(0xed, 0xaf, 0x00);
      g_rect(x, y, 1, 1, c, a);
    }
  }
}

static char *fmt_uint(char *p, unsigned v) {
  char t[10];
  int n = 0;
  do t[n++] = (char)('0' + v % 10), v /= 10;
  while (v && n < 10);
  while (n) *p++ = t[--n];
  return p;
}

static void draw_counters(void) {
  char buf[16];
  if (hud_score >= 0 && gc.y0 < 38) {
    unsigned t = hud_score < 9e7f ? (unsigned)(hud_score * 10 + 0.5f) : 999999999u;
    char *p = fmt_uint(buf, t / 10);
    *p++ = '.';
    *p++ = (char)('0' + t % 10);
    *p = 0;
    counter_text(&font_l, 312, 18 - font_l.cap_top, buf, C_WHITE, RGB(0xb8, 0xbe, 0xcc), C_OUTLINE, 2);
  }
  if (hud_coins >= 0 && gc.y1 > 28 && gc.y0 < 48) {
    *fmt_uint(buf, hud_coins < 4e9f ? (unsigned)(hud_coins + 0.5f) : 0) = 0;
    counter_text(&font_s, 300, 34 - font_s.cap_top, buf, RGB(0xff, 0xd8, 0x00), RGB(0xc0, 0x99, 0x00),
                 RGB(0x3f, 0x30, 0x00), 1);
    coin_icon(306, 38);
  }
}

static void draw_pill(const char *txt, int y) {
  int w = g_text_w(&font_s, txt) + 22;
  int x = 160 - w / 2;
  g_rrect(x - 1, y - 1, w + 2, 14, 7, C_BLACK, 32);
  g_rrect(x, y, w, 12, 6, C_WHITE, 32);
  g_text_c(&font_s, 160, y + 6 - (font_s.cap_top + font_s.base) / 2, txt, C_PILL_TXT, 32);
}

void hud_draw(uint16_t *px, int y, int n) {
  g_begin(px, y, n);
  if (y < 30) {
    g_icon(&ic_pause_o, 7, 6, C_OUTLINE, 32);
    g_icon(&ic_pause_f, 7, 6, C_WHITE, 32);
    if (hud_t < 150 + 12) {
      int py = 7 - (hud_t > 150 ? (hud_t - 150) * 2 : 0);
      static char buf[40];
      const char *nm = level.name;
      int i = 0;
      const char *pre = "Level ";
      while (*pre) buf[i++] = *pre++;
      int num = level_index + 1;
      if (num >= 100) buf[i++] = (char)('0' + num / 100);
      if (num >= 10) buf[i++] = (char)('0' + num / 10 % 10);
      buf[i++] = (char)('0' + num % 10);
      buf[i++] = ':';
      buf[i++] = ' ';
      while (*nm && i < 38) buf[i++] = *nm++;
      buf[i] = 0;
      draw_pill(buf, py);
    }
  }
  draw_counters();
  if (y + n > 104 && y < 122) {
    hint_box(16, 104, &ic_left, hud_btn & 1);
    hint_box(286, 104, &ic_right, (hud_btn & 2) != 0);
  }
  if (y + n > 180 && y < 216) {
    if (fade_l > 0) {
      g_icon(&ic_back, 36, 184, C_WHITE, fade_l * 14 / 32);
      g_icon(&ic_back, 37, 184, C_WHITE, fade_l * 14 / 32);
    }
    if (fade_r > 0) {
      g_icon(&ic_next, 274, 184, C_WHITE, fade_r * 14 / 32);
      g_icon(&ic_next, 275, 184, C_WHITE, fade_r * 14 / 32);
    }
  }
}

/* viewfinder brackets shown just before the photo is taken */
void viewfinder_draw(uint16_t *px, int y, int n, int t) {
  g_begin(px, y, n);
  int m = 6 - t / 2; /* brackets tighten slightly */
  if (m < 0) m = 0;
  int x0 = 40 + m, x1 = 280 - m, y0 = m, y1 = 240 - m, L = 34, T = 3;
  g_rect(x0, y0, L, T, C_WHITE, 32);
  g_rect(x0, y0, T, L, C_WHITE, 32);
  g_rect(x1 - L, y0, L, T, C_WHITE, 32);
  g_rect(x1 - T, y0, T, L, C_WHITE, 32);
  g_rect(x0, y1 - T, L, T, C_WHITE, 32);
  g_rect(x0, y1 - L, T, L, C_WHITE, 32);
  g_rect(x1 - L, y1 - T, L, T, C_WHITE, 32);
  g_rect(x1 - T, y1 - L, T, L, C_WHITE, 32);
  /* focus arc */
  const float R = 28;
  for (int yy = y; yy < y + n; yy++) {
    float dy = yy + 0.5f - 120;
    if (fabsf(dy) > R + 2) continue;
    for (int xx = 130; xx < 190; xx++) {
      float dx = xx + 0.5f - 160, d = sqrtf(dx * dx + dy * dy), e = fabsf(d - R);
      if (e > 1.2f || dy > 0 || dx < -6) continue;
      int a = (int)((1.2f - e) * 14);
      uint16_t *p = px + (yy - y) * SCREEN_W + xx;
      *p = mix565(*p, C_WHITE, a);
    }
  }
}

/* ------------------------------------------------------------ background */
#define BW 40
#define BH 30
static uint8_t (*blur)[BW][3]; /* allocated from the level arena when a menu first opens */

void ui_capture_background(void) {
  static uint16_t acc[BW][3];
  static int blur_level = -1;
  if (!blur || blur_level != level_serial) {
    blur = arena_keep_alloc(sizeof(uint8_t) * BH * BW * 3);
    blur_level = level_serial;
  }
  if (!blur) return;
  cam_view(SCREEN_W * 0.5f, SCREEN_H * 0.5f, 1, 0);
  render_prepare(0, SCREEN_W);
  for (int sy = 0; sy < SCREEN_H; sy += STRIP_H) {
    int n = SCREEN_H - sy < STRIP_H ? SCREEN_H - sy : STRIP_H;
    uint16_t *b = render_strip(sy, n, 0, SCREEN_W);
    for (int by = 0; by < n / 8; by++) { /* STRIP_H is a multiple of 8 */
      memset(acc, 0, sizeof acc);
      for (int r = 0; r < 8; r++) {
        const uint16_t *row = b + (by * 8 + r) * SCREEN_W;
        for (int x = 0; x < SCREEN_W; x++) {
          uint16_t c = row[x];
          acc[x >> 3][0] += c >> 11;
          acc[x >> 3][1] += (c >> 5) & 63;
          acc[x >> 3][2] += c & 31;
        }
      }
      uint8_t(*o)[3] = blur[sy / 8 + by];
      for (int i = 0; i < BW; i++) {
        o[i][0] = (uint8_t)(acc[i][0] * 255 / (64 * 31));
        o[i][1] = (uint8_t)(acc[i][1] * 255 / (64 * 63));
        o[i][2] = (uint8_t)(acc[i][2] * 255 / (64 * 31));
      }
    }
  }
  /* soften: two separable box passes */
  uint8_t(*t)[BW][3] = render_scratch();
  for (int pass = 0; pass < 2; pass++) {
    for (int y = 0; y < BH; y++)
      for (int x = 0; x < BW; x++)
        for (int k = 0; k < 3; k++) {
          int a = x > 0 ? x - 1 : 0, b2 = x < BW - 1 ? x + 1 : BW - 1;
          t[y][x][k] = (uint8_t)((blur[y][a][k] + 2 * blur[y][x][k] + blur[y][b2][k]) >> 2);
        }
    for (int y = 0; y < BH; y++)
      for (int x = 0; x < BW; x++)
        for (int k = 0; k < 3; k++) {
          int a = y > 0 ? y - 1 : 0, b2 = y < BH - 1 ? y + 1 : BH - 1;
          blur[y][x][k] = (uint8_t)((t[a][x][k] + 2 * t[y][x][k] + t[b2][x][k]) >> 2);
        }
  }
}

/* one screen row of the blurred background, columns [x0, x1) */
static void bg_row(uint16_t *dst, int y, int x0, int x1) {
  static uint16_t row[BW][3];
  if (!blur) {
    for (int x = x0; x < x1; x++) dst[x] = sky565;
    return;
  }
  int ty = 2 * y + 1 - 8, iy = ty >> 4, wy = ty & 15;
  if (iy < 0) iy = 0, wy = 0;
  if (iy >= BH - 1) iy = BH - 2, wy = 16;
  for (int i = 0; i < BW; i++)
    for (int k = 0; k < 3; k++) row[i][k] = (uint16_t)(blur[iy][i][k] * (16 - wy) + blur[iy + 1][i][k] * wy);
  for (int x = x0; x < x1; x++) {
    int tx = 2 * x + 1 - 8, ix = tx >> 4, wx = tx & 15;
    if (ix < 0) ix = 0, wx = 0;
    if (ix >= BW - 1) ix = BW - 2, wx = 16;
    int r = (row[ix][0] * (16 - wx) + row[ix + 1][0] * wx) >> 8;
    int g = (row[ix][1] * (16 - wx) + row[ix + 1][1] * wx) >> 8;
    int b = (row[ix][2] * (16 - wx) + row[ix + 1][2] * wx) >> 8;
    dst[x] = RGB(r, g, b);
  }
}

/* ------------------------------------------------------------------ cards */
typedef struct {
  int16_t x0, y0, x1, y1;
  uint16_t c;
} Box;
typedef struct {
  const Font *f;
  int16_t x, y, w;
  uint16_t c;
  const char *s;
} Run;

static struct {
  int kind, level;
  float w, h, ang;
  float cx, cy;
  Box photo;
  Box box[3];
  int nbox;
  Run run[12];
  int nrun;
  char txt[4][24];
} cd;

static void add_run(const Font *f, int x, int baseline, const char *s, uint16_t c, bool center) {
  Run *r = &cd.run[cd.nrun++];
  r->f = f;
  r->w = (int16_t)g_text_w(f, s);
  r->x = (int16_t)(center ? x - r->w / 2 : x);
  r->y = (int16_t)(baseline - f->base);
  r->c = c;
  r->s = s;
}

static const char *const desc[] = {"Drive silly cars at high", "speeds in dangerous", "terrain. Oh, and you",
                                   "can't steer. Good luck!"};

void card_setup(int kind, int lvl) {
  memset(&cd, 0, sizeof cd);
  cd.kind = kind;
  cd.level = lvl;
  if (kind == CARD_PAUSE) {
    cd.w = 206;
    cd.h = 166;
    cd.ang = -5.4f * 3.14159265f / 180;
    cd.cx = 160;
    cd.cy = 92;
    /* content: x in [-91, 91], y in [-71, 47] */
    cd.box[0] = (Box){-91, -71, 44, 12, C_PANEL};
    cd.box[1] = (Box){-91, 12, 44, 47, C_PANEL_D};
    cd.nbox = 2;
    cd.photo = (Box){44, -71, 91, 47, 0};
    for (int i = 0; i < 4; i++) add_run(&font_s, -86, -58 + i * 10, desc[i], C_WHITE, false);
    add_run(&font_s, -86, 4, "Inspired by Drive Mad", C_WHITE, false);
    add_run(&font_s, -86, 24, "Made by Mason Chen", C_WHITE, false);
    char *t = cd.txt[0];
    int n = lvl + 1, i = 0;
    const char *pre = "Level ";
    while (*pre) t[i++] = *pre++;
    if (n >= 100) t[i++] = (char)('0' + n / 100);
    if (n >= 10) t[i++] = (char)('0' + n / 10 % 10);
    t[i++] = (char)('0' + n % 10);
    t[i] = 0;
    add_run(&font_s, -86, 34, t, C_PANEL_TXT, false);
    strcpy(cd.txt[1], level_name(lvl));
    add_run(&font_s, -86, 43, cd.txt[1], C_PANEL_TXT, false);
    add_run(&font_m, 0, 66, "NumDrive", C_CAPTION, true);
  } else {
    cd.w = 143;
    cd.h = 156;
    cd.ang = -10.5f * 3.14159265f / 180;
    cd.cx = 160;
    cd.cy = 91;
    cd.photo = (Box){-59, -66, 59, 52, 0};
    add_run(&font_m, 0, 69, kind == CARD_WIN ? "Good game" : "Terminated", C_CAPTION, true);
  }
}

int card_buttons(void) { return cd.kind == CARD_LOSE ? 1 : 2; }

/* 2x2 bilinear sample of a text run, returns alpha 0..32 */
static int run_texel(const Run *r, int x, int y) {
  if (y < 0 || y >= r->f->h) return 0;
  int pos = 0;
  for (const char *s = r->s; *s; s++) {
    const Glyph *g = font_glyph(r->f, (unsigned char)*s);
    if (!g) continue;
    if (x >= pos + g->xoff && x < pos + g->xoff + g->w) {
      int a = glyph_alpha(r->f, g, x - pos, y);
      if (a) return a;
    }
    pos += g->adv;
    if (pos + 4 < x) continue;
    if (pos > x + 4) break;
  }
  return 0;
}

static int run_alpha(const Run *r, float u, float v) {
  float tx = u - r->x - 0.5f, ty = v - r->y - 0.5f;
  if (tx < -2 || tx > r->w + 2 || ty < -1 || ty > r->f->h) return 0;
  int ix = (int)floorf(tx), iy = (int)floorf(ty);
  float fx = tx - ix, fy = ty - iy;
  float a = run_texel(r, ix, iy) * (1 - fx) * (1 - fy) + run_texel(r, ix + 1, iy) * fx * (1 - fy) +
            run_texel(r, ix, iy + 1) * (1 - fx) * fy + run_texel(r, ix + 1, iy + 1) * fx * fy;
  return (int)(a * 32 / 3 + 0.5f);
}

static float ease_back(float t) {
  if (t >= 1) return 1;
  float s = 1.9f;
  t -= 1;
  return t * t * ((s + 1) * t + s) + 1;
}

/* buttons below the card */
static void card_btns(int sel, int yb) {
  const int x0 = 89, w = 142, h = 30, r = 6;
  if (cd.kind == CARD_LOSE) {
    g_button(x0, yb, w, h, r, C_BLUE, C_BLUE_D, 0);
    g_icon(&ic_play, 160 - 6, yb + 3, C_WHITE, 32);
    g_text_c(&font_s, 160, yb + 25 - font_s.base, "Retry", C_WHITE, 32);
  } else {
    const int ws = 46;
    g_rrect(x0 - 1, yb - 1, w + 2, h + 2, r + 1, C_BLACK, 32);
    g_rrect(x0, yb, w, h, r, C_BLUE_D, 32);
    g_rrect(x0, yb, w, h - 3, r, C_BLUE, 32);
    /* grey restart segment on the left */
    for (int yy = yb; yy < yb + h; yy++) {
      if (yy < gc.y0 || yy >= gc.y1) continue;
      uint16_t *p = gc.px + (yy - gc.y0) * SCREEN_W;
      for (int x = x0; x < x0 + ws; x++) {
        uint16_t c = p[x];
        if (c == C_BLUE) p[x] = C_GRAY;
        else if (c == C_BLUE_D) p[x] = C_GRAY_D;
        else if (c != C_BLACK) p[x] = mix565(c, C_GRAY, 20);
      }
    }
    g_icon(&ic_restart, x0 + ws / 2 - 7, yb + 3, C_WHITE, 32);
    g_text_c(&font_s, x0 + ws / 2, yb + 25 - font_s.base, "Restart", C_WHITE, 32);
    int cx = x0 + ws + (w - ws) / 2;
    g_icon(&ic_play, cx - 7, yb + 3, C_WHITE, 32);
    g_text_c(&font_s, cx, yb + 25 - font_s.base, cd.kind == CARD_PAUSE ? "Resume" : "Next Level", C_WHITE, 32);
    /* focus ring */
    int fx = sel == 0 ? x0 : x0 + ws, fw = sel == 0 ? ws : w - ws;
    g_rect(fx + 1, yb + 1, fw - 2, 1, C_WHITE, 26);
    g_rect(fx + 1, yb + h - 5, fw - 2, 1, C_WHITE, 26);
    g_rect(fx + 1, yb + 1, 1, h - 5, C_WHITE, 26);
    g_rect(fx + fw - 2, yb + 1, 1, h - 5, C_WHITE, 26);
  }
}

void card_draw(int anim, int sel, bool full) {
  float t = anim / 12.0f;
  float s = 0.55f + 0.45f * ease_back(t);
  float ang = cd.ang * (t >= 1 ? 1 : 0.3f + 0.7f * ease_back(t));
  float cs = cosf(ang), sn = sinf(ang);
  float cx = cd.cx, cy = cd.cy;
  float hw = cd.w * 0.5f, hh = cd.h * 0.5f;
  /* screen bbox of card and shadow */
  float ex = (fabsf(cs) * hw + fabsf(sn) * hh) * s, ey = (fabsf(sn) * hw + fabsf(cs) * hh) * s;
  int bx0 = (int)(cx - ex) - 1, bx1 = (int)(cx + ex) + 6, by0 = (int)(cy - ey) - 1, by1 = (int)(cy + ey) + 7;
  if (bx0 < 0) bx0 = 0;
  if (bx1 > SCREEN_W) bx1 = SCREEN_W;
  if (by0 < 0) by0 = 0;
  if (by1 > SCREEN_H) by1 = SCREEN_H;
  /* photo camera: the photo shows the centre square of the game view */
  float pu = (cd.photo.x0 + cd.photo.x1) * 0.5f, pv = (cd.photo.y0 + cd.photo.y1) * 0.5f;
  float ph = (float)(cd.photo.y1 - cd.photo.y0);
  float pcx = cx + s * (pu * cs - pv * sn), pcy = cy + s * (pu * sn + pv * cs);
  float pex = (fabsf(cs) * (cd.photo.x1 - cd.photo.x0) + fabsf(sn) * ph) * 0.5f * s;
  float pey = (fabsf(sn) * (cd.photo.x1 - cd.photo.x0) + fabsf(cs) * ph) * 0.5f * s;
  int px0 = (int)(pcx - pex), px1 = (int)(pcx + pex) + 2, py0 = (int)(pcy - pey), py1 = (int)(pcy + pey) + 2;
  if (px0 < 0) px0 = 0;
  if (px1 > SCREEN_W) px1 = SCREEN_W;
  Camera saved = cam;
  cam_view(pcx, pcy, s * ph / 240.0f, -ang);
  render_prepare(px0, px1);
  int ybtn = 190;
  int y_lo = full ? 0 : by0, y_hi = full ? SCREEN_H : by1;
  int cx0 = full ? 0 : bx0, cx1 = full ? SCREEN_W : bx1;
  for (int sy = y_lo - y_lo % STRIP_H; sy < y_hi; sy += STRIP_H) {
    int n = SCREEN_H - sy < STRIP_H ? SCREEN_H - sy : STRIP_H;
    uint16_t *buf = render_buffer();
    bool photo_here = sy < py1 && sy + n > py0;
    if (photo_here) render_strip(sy, n, px0, px1);
    for (int r = 0; r < n; r++) {
      int y = sy + r;
      uint16_t *row = buf + r * SCREEN_W;
      static uint16_t world[SCREEN_W];
      bool in_rows = y >= by0 && y < by1;
      if (photo_here && y >= py0 && y < py1) memcpy(world + px0, row + px0, (size_t)(px1 - px0) * 2);
      bg_row(row, y, cx0, cx1);
      if (!in_rows) continue;
      float dy = y + 0.5f - cy;
      for (int x = bx0; x < bx1; x++) {
        float dx = x + 0.5f - cx;
        float u = (dx * cs + dy * sn) / s, v = (-dx * sn + dy * cs) / s;
        float e = fminf(hw - fabsf(u), hh - fabsf(v)) * s;
        uint16_t bgc = row[x];
        if (e < 0.5f) {
          /* shadow */
          float dx2 = dx - 3, dy2 = dy - 4;
          float u2 = (dx2 * cs + dy2 * sn) / s, v2 = (-dx2 * sn + dy2 * cs) / s;
          float e2 = fminf(hw - fabsf(u2), hh - fabsf(v2)) * s;
          if (e2 > -0.5f) bgc = mix565(bgc, C_SHADOW, (int)(13 * fminf(1, e2 + 0.5f)));
          if (e <= -0.5f) {
            row[x] = bgc;
            continue;
          }
        }
        uint16_t c = C_WHITE;
        if (u >= cd.photo.x0 && u < cd.photo.x1 && v >= cd.photo.y0 && v < cd.photo.y1 && photo_here && x >= px0 &&
            x < px1) {
          c = world[x];
        } else {
          for (int i = 0; i < cd.nbox; i++)
            if (u >= cd.box[i].x0 && u < cd.box[i].x1 && v >= cd.box[i].y0 && v < cd.box[i].y1) c = cd.box[i].c;
          for (int i = 0; i < cd.nrun; i++) {
            const Run *rn = &cd.run[i];
            if (v < rn->y - 1 || v > rn->y + rn->f->h + 1 || u < rn->x - 2 || u > rn->x + rn->w + 2) continue;
            int a = run_alpha(rn, u, v);
            if (a) c = mix565(c, rn->c, a > 32 ? 32 : a);
          }
        }
        if (e < 0.5f) c = mix565(bgc, c, (int)((e + 0.5f) * 32));
        row[x] = c;
      }
    }
    g_begin(buf, sy, n);
    if (full || (ybtn - 3 < sy + n && ybtn + 33 > sy)) card_btns(sel, ybtn);
    if (full) {
      g_icon(&ic_back, 34, 84, C_WHITE, 32);
      g_icon(&ic_back, 35, 84, C_WHITE, 32);
    }
    if (full)
      plat_push(0, sy, SCREEN_W, n, buf);
    else {
      int r0 = sy < by0 ? by0 - sy : 0, r1 = sy + n > by1 ? by1 - sy : n;
      for (int r = r0; r < r1; r++) plat_push(bx0, sy + r, bx1 - bx0, 1, buf + r * SCREEN_W + bx0);
    }
  }
  cam = saved;
  cam_update();
}

/* ---------------------------------------------------------------- levels */
#define TILE 28
#define PITCH 34
#define GX (160 - (5 * PITCH - (PITCH - TILE)) / 2)
#define GY 46

bool level_unlocked(int i) { return i == 0 || lvl_done(i) || lvl_done(i - 1); }

void levels_draw(int sel, int scroll) {
  for (int sy = 0; sy < SCREEN_H; sy += STRIP_H) {
    int n = SCREEN_H - sy < STRIP_H ? SCREEN_H - sy : STRIP_H;
    uint16_t *buf = render_buffer();
    for (int r = 0; r < n; r++) bg_row(buf + r * SCREEN_W, sy + r, 0, SCREEN_W);
    g_begin(buf, sy, n);
    if (sy < 40) {
      int ty = 22 - font_l.base;
      for (int d = 0; d < 8; d++) {
        static const int8_t o[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, 1}, {-1, 1}, {1, -1}};
        g_text_c(&font_l, 160 + o[d][0], ty + o[d][1] + 1, "Levels", C_TITLE_O, 32);
      }
      g_text_c(&font_l, 160, ty, "Levels", C_WHITE, 32);
    }
    for (int row = 0; row < 7; row++) {
      int ly = GY + row * PITCH;
      if (ly >= sy + n || ly + TILE + 3 <= sy) continue;
      int fade = ly > 190 ? 32 - (ly - 190) * 32 / 40 : 32;
      if (fade <= 0) continue;
      for (int col = 0; col < 5; col++) {
        int i = (scroll + row) * 5 + col;
        if (i >= nlevels) break;
        int lx = GX + col * PITCH;
        uint16_t top, bot;
        bool unl = level_unlocked(i);
        if (lvl_done(i)) top = C_GREEN, bot = C_GREEN_D;
        else if (unl) top = C_TILE_B, bot = C_TILE_BD;
        else top = C_LOCK, bot = C_LOCK_D;
        if (fade < 32 || !unl) {
          /* translucent tiles over the background */
          int a = unl ? fade : fade * 24 / 32;
          g_rrect(lx - 1, ly - 1, TILE + 2, TILE + 2, 6, C_BLACK, a);
          g_rrect(lx, ly, TILE, TILE, 5, bot, a);
          g_rrect(lx, ly, TILE, TILE - 3, 5, top, a);
        } else {
          g_button(lx, ly, TILE, TILE, 5, top, bot, i == sel);
        }
        char num[4];
        int k = 0, v = i + 1;
        if (v >= 100) num[k++] = (char)('0' + v / 100);
        if (v >= 10) num[k++] = (char)('0' + v / 10 % 10);
        num[k++] = (char)('0' + v % 10);
        num[k] = 0;
        g_text_c(&font_m, lx + TILE / 2, ly + (TILE - 3) / 2 - (font_m.cap_top + font_m.base) / 2 + 1, num, C_WHITE,
                 unl ? fade : fade * 20 / 32);
      }
    }
    if (sy < 110 && sy + n > 84) {
      g_icon(&ic_back, 34, 84, C_WHITE, 32);
      g_icon(&ic_back, 35, 84, C_WHITE, 32);
    }
    plat_push(0, sy, SCREEN_W, n, buf);
  }
}
