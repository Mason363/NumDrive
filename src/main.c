#include <stdlib.h>
#include <string.h>
#ifdef HOST
#include <stdio.h>
#endif
#include "platform.h"
#include "world.h"
#include "vm.h"
#include "physics.h"
#include "render.h"
#include "ui.h"
#include "save.h"

#ifndef HOST
#include <eadk.h>
const char eadk_app_name[] __attribute__((section(".rodata.eadk_app_name"))) = "NumDrive";
const uint32_t eadk_api_level __attribute__((section(".rodata.eadk_api_level"))) = 0;
#endif

enum { ST_PLAY, ST_CARD, ST_LEVELS };
#define SHOT_FRAMES 10 /* viewfinder shown before the photo */

static int win_timer = -1, lose_timer = -1;
#ifdef HOST
void host_outcome(int won);
#endif
void game_win(int delay) {
#ifdef HOST
  if (win_timer < 0 && lose_timer < 0) host_outcome(1);
#endif
  if (win_timer < 0 && lose_timer < 0) win_timer = delay > SHOT_FRAMES ? delay : SHOT_FRAMES;
}
void game_lose(int delay) {
#ifdef HOST
  if (getenv("ND_WDBG") && win_timer < 0 && lose_timer < 0) fprintf(stderr, "lose at frame %d\n", vm_frame_count);
  if (win_timer < 0 && lose_timer < 0) host_outcome(2);
#endif
  if (win_timer < 0 && lose_timer < 0) lose_timer = delay > SHOT_FRAMES ? delay : SHOT_FRAMES;
}

static int cur;
static bool start_level(int i) {
  if (i < 0) i = 0;
  if (i >= nlevels) i = nlevels - 1;
  cur = i;
  win_timer = lose_timer = -1;
  phys_reset();
  cam_default();
  if (!world_load_level(i)) return false;
  phys_start();
  if (!render_init_level()) return false;
  hud_reset();
  if (progress.last != i) {
    progress.last = (uint8_t)i;
    save_store();
  }
  return true;
}

static void sim_step(uint8_t buttons) {
  vm_buttons = buttons;
  vm_frame();
#ifdef HOST
  if (getenv("ND_ODBG") && vm_frame_count <= 1) {
    const char *l = getenv("ND_ODBG");
    while (*l) {
      int o = atoi(l);
      if (o >= 0 && o < nobj) fprintf(stderr, "f%d obj%d pos=(%.2f %.2f %.2f) rot=(%.3f %.3f %.3f %.3f) flags=%x\n", vm_frame_count, o, objs[o].pos.x, objs[o].pos.y, objs[o].pos.z, objs[o].rot.x, objs[o].rot.y, objs[o].rot.z, objs[o].rot.w, objs[o].flags);
      while (*l && *l != ',') l++;
      if (*l) l++;
    }
  }
#endif
  phys_step();
#ifdef HOST
  if (getenv("ND_OBJHIST") && vm_frame_count == atoi(getenv("ND_OBJHIST"))) {
    for (int i = 0; i < nobj; i++)
      fprintf(stderr, "OBJ %d src %d np %d flags %x pos %.1f %.1f %.1f\n", i, objs[i].src, objs[i].shape->np, objs[i].flags, objs[i].pos.x, objs[i].pos.y, objs[i].pos.z);
  }
#endif
#ifdef HOST
  {
    extern void phys_debug(int);
    phys_debug(vm_frame_count);
  }
#endif
  vm_late();
}

static int shot_t;
static void hud_post(uint16_t *px, int y, int n) {
  if (shot_t > 0) viewfinder_draw(px, y, n, SHOT_FRAMES - shot_t);
  else hud_draw(px, y, n);
}

extern bool level_unlocked(int i);

#ifdef ARMTEST
extern char **environ;
#endif

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
#ifdef ARMTEST
  environ = argv + 1; /* semihosting has no environment: pass KEY=VALUE arguments */
#endif
  if (!world_init()) return 1;
  save_load();
  int lvl = progress.last < nlevels ? progress.last : 0;
#ifdef HOST
  if (getenv("ND_LEVEL")) lvl = atoi(getenv("ND_LEVEL"));
#endif
  if (!start_level(lvl)) {
    plat_fill(0, 0, SCREEN_W, SCREEN_H, 0xF800);
    plat_sleep(2000);
    return 1;
  }
  int state = ST_PLAY, kind = CARD_PAUSE, anim = 0, sel = 1, lsel = 0, lscroll = 0;
  bool dirty = true;
  uint64_t prev = 0;
  uint32_t last = plat_millis();
  int acc = 0;
  for (;;) {
    uint64_t k = plat_keys();
    uint64_t hit = k & ~prev;
    prev = k;
    if (k & KEY(K_HOME)) break;
    /* frame pacing: fixed 60 Hz simulation, catching up when rendering is slow */
    int steps = 1;
#ifndef HOST
    uint32_t now = plat_millis();
    if (now - last < 16) {
      plat_sleep((int)(16 - (now - last)));
      now = plat_millis();
    }
    acc += (int)(now - last) * 3;
    last = now;
    steps = acc / 50;
    if (steps < 1) steps = 1;
    if (steps > 3) steps = 3, acc = 0;
    acc -= steps * 50;
    if (acc < 0) acc = 0;
#else
    (void)last;
    (void)acc;
#endif
    if (state == ST_PLAY) {
      if (hit & (KEY(K_BACK) | KEY(K_OK))) {
        shot_t = 0;
        ui_capture_background();
        card_setup(CARD_PAUSE, cur);
        kind = CARD_PAUSE;
        state = ST_CARD;
        anim = 0;
        sel = 1;
        continue;
      }
      uint8_t b = ((k & KEY(K_LEFT)) ? 1 : 0) | ((k & KEY(K_RIGHT)) ? 2 : 0);
      bool ending = false;
      for (int s = 0; s < steps && !ending; s++) {
        sim_step(win_timer >= 0 || lose_timer >= 0 ? 0 : b);
        hud_tick(b);
        if (win_timer > 0) win_timer--;
        if (lose_timer > 0) lose_timer--;
        ending = win_timer == 0 || lose_timer == 0;
      }
      int t = win_timer >= 0 ? win_timer : lose_timer;
      shot_t = t > 0 && t <= SHOT_FRAMES ? t : 0;
      if (ending) {
        kind = win_timer == 0 ? CARD_WIN : CARD_LOSE;
        if (kind == CARD_WIN && !lvl_done(cur)) {
          lvl_set_done(cur);
          save_store();
        }
        ui_capture_background();
        card_setup(kind, cur);
        state = ST_CARD;
        anim = 0;
        sel = 1;
        win_timer = lose_timer = -1;
        continue;
      }
#ifdef HOST
      static int norender = -1;
      if (norender < 0) norender = getenv("ND_NORENDER") != 0;
      if (!norender)
#endif
        render_frame(hud_post);
    } else if (state == ST_CARD) {
      if (card_buttons() == 2) {
        if ((hit & KEY(K_LEFT)) && sel) sel = 0, dirty = true;
        if ((hit & KEY(K_RIGHT)) && !sel) sel = 1, dirty = true;
      } else {
        sel = 1;
      }
      if (hit & (KEY(K_OK) | KEY(K_EXE))) {
        if (kind == CARD_PAUSE && sel == 1) {
          state = ST_PLAY;
        } else {
          int next = kind == CARD_WIN && sel == 1 ? cur + 1 : cur;
          start_level(next < nlevels ? next : cur);
          state = ST_PLAY;
        }
        continue;
      }
      if (hit & KEY(K_BACK)) { /* the "<" arrow: level list */
        state = ST_LEVELS;
        lsel = cur;
        lscroll = lsel / 5 - 2 < 0 ? 0 : lsel / 5 - 2;
        dirty = true;
        continue;
      }
      if (kind != CARD_PAUSE)
        for (int s = 0; s < steps; s++) sim_step(0);
      bool full = dirty || anim <= 13;
      if (kind != CARD_PAUSE || full) card_draw(anim, sel, full);
      dirty = false;
      anim++;
    } else {
      int old = lsel;
      if (hit & KEY(K_LEFT)) lsel--;
      if (hit & KEY(K_RIGHT)) lsel++;
      if (hit & KEY(K_UP)) lsel -= 5;
      if (hit & KEY(K_DOWN)) lsel += 5;
      if (lsel < 0 || lsel >= nlevels || !level_unlocked(lsel)) lsel = old;
      if (lsel != old) dirty = true;
      if (lsel / 5 < lscroll) lscroll = lsel / 5, dirty = true;
      if (lsel / 5 > lscroll + 3) lscroll = lsel / 5 - 3, dirty = true;
      if (hit & (KEY(K_OK) | KEY(K_EXE))) {
        start_level(lsel);
        state = ST_PLAY;
        continue;
      }
      if (hit & KEY(K_BACK)) {
        state = ST_CARD;
        dirty = true;
        continue;
      }
      if (dirty) levels_draw(lsel, lscroll);
      dirty = false;
    }
    plat_frame_done();
  }
  return 0;
}
