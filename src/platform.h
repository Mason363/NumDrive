#ifndef PLATFORM_H
#define PLATFORM_H
#include <stdint.h>
#include <stdbool.h>

#define SCREEN_W 320
#define SCREEN_H 240

/* key bits follow eadk_key_t */
enum {
  K_LEFT = 0, K_UP = 1, K_DOWN = 2, K_RIGHT = 3, K_OK = 4, K_BACK = 5, K_HOME = 6,
  K_SHIFT = 12, K_ALPHA = 13, K_BACKSPACE = 17, K_FOUR = 36, K_SIX = 38, K_ONE = 42,
  K_TWO = 43, K_THREE = 44, K_PLUS = 45, K_MINUS = 46, K_ZERO = 48, K_EXE = 52
};
#define KEY(k) ((uint64_t)1 << (k))

void plat_push(int x, int y, int w, int h, const uint16_t *px);
void plat_fill(int x, int y, int w, int h, uint16_t c);
void plat_pull(int x, int y, int w, int h, uint16_t *px);
uint64_t plat_keys(void);
uint32_t plat_millis(void);
void plat_vblank(void);
void plat_sleep(int ms);
uint32_t plat_random(void);
void plat_frame_done(void); /* host harness hook (captures frames) */

#endif
