#ifndef RENDER_H
#define RENDER_H
#include <stdint.h>
#include <stdbool.h>
#include "fmath.h"

#define STRIP_H 16

typedef struct {
  vec3 focus;
  quat rot;
  float range;
  int perspective;
  vec3 right, up, fwd;
  float scale; /* pixels per world unit (ortho) */
  vec3 shake;
  float cx, cy;      /* screen position of the focus */
  float zoom_mul;    /* extra magnification (photos) */
  float roll;        /* extra roll in radians (photos) */
} Camera;

extern Camera cam;
extern uint16_t sky565;

bool render_init_level(void); /* per level renderer data (arena) */
void render_frame(void (*post)(uint16_t *px, int y, int n)); /* full screen, post draws overlays */
void render_prepare(int x0, int x1);            /* per-view object culling */
uint16_t *render_strip(int y, int n, int x0, int x1); /* world into the strip buffer */
uint16_t *render_buffer(void);
void *render_scratch(void); /* SCREEN_W * STRIP_H * 2 bytes, free outside rendering */
void cam_default(void);
void cam_view(float cx, float cy, float zoom_mul, float roll);
void cam_update(void);
uint16_t rgb565(int r, int g, int b);
void shade_color(int pal, vec3 n, int *r, int *g, int *b);

extern const uint8_t palette_rgb[34][3];

#endif
