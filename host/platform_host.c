/* Headless host platform for automated testing.
 * Env vars: ND_FRAMES (stop after N frames), ND_SHOTS ("1,20,50" or "all"), ND_OUT (dir),
 * ND_KEYS ("RIGHT:0:120,LEFT:200:260,OK:5:6") key holds by frame range. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/platform.h"

static uint16_t fb[SCREEN_W * SCREEN_H];
static int frame;
static int max_frames = 600;
static char shots[4096];
static char keys[4096];
static const char *outdir = "out";
static int inited;

static void init(void) {
  if (inited) return;
  inited = 1;
  const char *s;
  if ((s = getenv("ND_FRAMES"))) max_frames = atoi(s);
  if ((s = getenv("ND_SHOTS"))) snprintf(shots, sizeof shots, ",%s,", s);
  if ((s = getenv("ND_KEYS"))) snprintf(keys, sizeof keys, "%s", s);
  if ((s = getenv("ND_OUT"))) outdir = s;
}

int host_frame(void) { return frame; }

void plat_push(int x, int y, int w, int h, const uint16_t *px) {
  for (int j = 0; j < h; j++)
    for (int i = 0; i < w; i++)
      if (x + i >= 0 && x + i < SCREEN_W && y + j >= 0 && y + j < SCREEN_H) fb[(y + j) * SCREEN_W + x + i] = px[j * w + i];
}

void plat_fill(int x, int y, int w, int h, uint16_t c) {
  for (int j = 0; j < h; j++)
    for (int i = 0; i < w; i++)
      if (x + i >= 0 && x + i < SCREEN_W && y + j >= 0 && y + j < SCREEN_H) fb[(y + j) * SCREEN_W + x + i] = c;
}

void plat_pull(int x, int y, int w, int h, uint16_t *px) {
  for (int j = 0; j < h; j++)
    for (int i = 0; i < w; i++) px[j * w + i] = fb[(y + j) * SCREEN_W + x + i];
}

static int keycode(const char *n) {
  if (!strncmp(n, "RIGHT", 5)) return K_RIGHT;
  if (!strncmp(n, "LEFT", 4)) return K_LEFT;
  if (!strncmp(n, "UP", 2)) return K_UP;
  if (!strncmp(n, "DOWN", 4)) return K_DOWN;
  if (!strncmp(n, "OK", 2)) return K_OK;
  if (!strncmp(n, "BACK", 4)) return K_BACK;
  if (!strncmp(n, "EXE", 3)) return K_EXE;
  return -1;
}

uint64_t plat_keys(void) {
  init();
  uint64_t k = 0;
  char buf[4096];
  snprintf(buf, sizeof buf, "%s", keys);
  for (char *tok = strtok(buf, ","); tok; tok = strtok(NULL, ",")) {
    int a = 0, b = 0;
    char name[16] = {0};
    if (sscanf(tok, "%15[A-Z]:%d:%d", name, &a, &b) == 3 && frame >= a && frame < b) {
      int c = keycode(name);
      if (c >= 0) k |= KEY(c);
    }
  }
  return k;
}

uint32_t plat_millis(void) { return (uint32_t)(frame * 1000 / 60); }
void plat_vblank(void) {}
void plat_sleep(int ms) { (void)ms; }
uint32_t plat_random(void) {
  static uint32_t s = 12345;
  s = s * 1103515245u + 12345u;
  return s >> 1;
}

static void save(void) {
  char path[512];
  snprintf(path, sizeof path, "%s/f%05d.ppm", outdir, frame);
  FILE *f = fopen(path, "wb");
  if (!f) return;
  fprintf(f, "P6\n%d %d\n255\n", SCREEN_W, SCREEN_H);
  for (int i = 0; i < SCREEN_W * SCREEN_H; i++) {
    uint16_t c = fb[i];
    unsigned char rgb[3] = {(unsigned char)(((c >> 11) & 31) * 255 / 31), (unsigned char)(((c >> 5) & 63) * 255 / 63),
                            (unsigned char)((c & 31) * 255 / 31)};
    fwrite(rgb, 1, 3, f);
  }
  fclose(f);
}

void plat_frame_done(void) {
  init();
  char tag[32];
  snprintf(tag, sizeof tag, ",%d,", frame);
  if (strstr(shots, ",all,") || strstr(shots, tag)) save();
  frame++;
  if (frame >= max_frames) exit(0);
}
