#include <eadk.h>
#include "platform.h"

void plat_push(int x, int y, int w, int h, const uint16_t *px) {
  eadk_rect_t r = {(uint16_t)x, (uint16_t)y, (uint16_t)w, (uint16_t)h};
  eadk_display_push_rect(r, px);
}

void plat_fill(int x, int y, int w, int h, uint16_t c) {
  eadk_rect_t r = {(uint16_t)x, (uint16_t)y, (uint16_t)w, (uint16_t)h};
  eadk_display_push_rect_uniform(r, c);
}

void plat_pull(int x, int y, int w, int h, uint16_t *px) {
  eadk_rect_t r = {(uint16_t)x, (uint16_t)y, (uint16_t)w, (uint16_t)h};
  eadk_display_pull_rect(r, px);
}

uint64_t plat_keys(void) { return eadk_keyboard_scan(); }
uint32_t plat_millis(void) { return (uint32_t)eadk_timing_millis(); }
void plat_vblank(void) { eadk_display_wait_for_vblank(); }
void plat_sleep(int ms) { eadk_timing_msleep(ms); }
uint32_t plat_random(void) { return eadk_random(); }
void plat_frame_done(void) {}
