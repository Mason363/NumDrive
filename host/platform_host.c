/* Headless host platform for automated testing.
 * Env vars: ND_FRAMES (stop after N frames), ND_SHOTS ("1,20,50" or "all"), ND_OUT (dir),
 * ND_KEYS ("RIGHT:0:120,LEFT:200:260,OK:5:6") key holds by frame range. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/platform.h"
#include "../src/render.h"
#ifndef ARMTEST
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <sys/prctl.h>
#include <math.h>
#include <semaphore.h>
#endif

static uint16_t fb[SCREEN_W * SCREEN_H];
static int frame;
static int max_frames = 600;
static char shots[4096];
static char keys[4096];
static const char *outdir = "out";
static int inited;

#ifndef ARMTEST
/* ND_SEARCH="seg,beam,frames": beam search over held keys (none / right / left per segment) for a win.
   Workers are forked processes paused at segment ends; the first process coordinates. */
#define SW 512
#define SPATH 256
typedef struct {
  volatile int cmd[SW], arg[SW][3], done[SW], status[SW];
  volatile float score[SW], cy[SW];
  volatile unsigned char path[SW][SPATH];
  volatile int plen[SW];
  sem_t go[SW], done_sem; /* a worker sleeps on go[id]; the coordinator on done_sem */
} Search;
static Search *sr;
static int s_seg, s_beam, s_frames, s_id = -1, s_act, s_outcome;
static void s_checkpoint(void);
#endif
void host_outcome(int won) {
#ifndef ARMTEST
  if (!s_outcome) s_outcome = won;
#else
  (void)won;
#endif
}

static void init(void) {
  if (inited) return;
  inited = 1;
  const char *s;
  if ((s = getenv("ND_FRAMES"))) max_frames = atoi(s);
  if ((s = getenv("ND_SHOTS"))) snprintf(shots, sizeof shots, ",%s,", s);
  if ((s = getenv("ND_KEYS"))) snprintf(keys, sizeof keys, "%s", s);
  if ((s = getenv("ND_OUT"))) outdir = s;
#ifndef ARMTEST
  if ((s = getenv("ND_SEARCH"))) {
    s_seg = 20, s_beam = 8, s_frames = 1800;
    sscanf(s, "%d,%d,%d", &s_seg, &s_beam, &s_frames);
    max_frames = 1 << 30;
    sr = mmap(0, sizeof(Search), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    memset((void *)sr, 0, sizeof(Search));
    for (int i = 0; i < SW; i++) sem_init(&sr->go[i], 1, 0);
    sem_init(&sr->done_sem, 1, 0);
  }
#endif
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
#ifndef ARMTEST
  if (sr) {
    if (s_id < 0) s_checkpoint(); /* frame 0: become the coordinator and the first worker */
    return s_act == 1 ? KEY(K_RIGHT) : s_act == 2 ? KEY(K_LEFT) : 0;
  }
#endif
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
#ifndef ARMTEST
  if (sr && (frame % s_seg == 0 || s_outcome)) s_checkpoint();
#endif
  if (frame >= max_frames) exit(0);
}
#ifndef ARMTEST

static void s_wait_cmd(void) {
  for (;;) {
    while (sem_wait(&sr->go[s_id]) != 0) {
    }
    int c = sr->cmd[s_id];
    sr->cmd[s_id] = 0;
    if (c == 2) _exit(0);
    /* expand: a child per action; this process is done */
    for (int a = 0; a < 3; a++) {
      int id = sr->arg[s_id][a];
      if (fork() == 0) {
        memcpy((void *)sr->path[id], (void *)sr->path[s_id], SPATH);
        sr->plen[id] = sr->plen[s_id];
        if (sr->plen[id] < SPATH) sr->path[id][sr->plen[id]++] = (unsigned char)a;
        s_id = id;
        s_act = a;
        return;
      }
    }
    _exit(0);
  }
}

static void s_report(void) {
  static float s_dir;
  if (!s_dir) s_dir = getenv("ND_SDIR") ? atof(getenv("ND_SDIR")) : 1;
  sr->score[s_id] = s_outcome == 1 ? 1e6f - frame : s_outcome == 2 ? -1e6f : cam.focus.x * s_dir;
  sr->cy[s_id] = cam.focus.y;
  sr->status[s_id] = s_outcome;
  __sync_synchronize();
  sr->done[s_id] = 1;
  sem_post(&sr->done_sem);
}

static void s_coordinate(void) {
  int beam[64], nb = 1, next = 1;
  if (s_beam > 64) s_beam = 64;
  beam[0] = 0;
  while (sem_wait(&sr->done_sem) != 0) {
  }
  for (int gen = 0; gen * s_seg < s_frames; gen++) {
    int kids[192], nk = 0;
    for (int i = 0; i < nb; i++) {
      for (int a = 0; a < 3; a++) {
        int id = next++ % SW;
        if (id == 0) id = next++ % SW;
        sr->done[id] = 0;
        sr->arg[beam[i]][a] = id;
        kids[nk++] = id;
      }
      __sync_synchronize();
      sr->cmd[beam[i]] = 1;
      sem_post(&sr->go[beam[i]]);
    }
    for (int i = 0; i < nk; i++)
      while (sem_wait(&sr->done_sem) != 0) {
      }
    /* best first; a win ends the search */
    for (int i = 0; i < nk; i++)
      for (int j = i + 1; j < nk; j++)
        if (sr->score[kids[j]] > sr->score[kids[i]]) {
          int t = kids[i];
          kids[i] = kids[j];
          kids[j] = t;
        }
    int best = kids[0];
    if (sr->status[best] == 1 || (gen + 1) * s_seg >= s_frames || sr->status[best] == 2) {
      printf("search %s gen %d score %.2f path ", sr->status[best] == 1 ? "win" : "none", gen, sr->score[best]);
      for (int i = 0; i < sr->plen[best]; i++) putchar(".RL"[sr->path[best][i]]);
      putchar('\n');
      fflush(stdout);
      for (int i = 0; i < nk; i++) sr->cmd[kids[i]] = 2, sem_post(&sr->go[kids[i]]);
      usleep(100000); /* let the workers go before their reaper does */
      exit(0);
    }
    /* keep the best, but at most two per spot so the beam spreads over where the car can get */
    nb = 0;
    for (int i = 0; i < nk; i++) {
      int k = kids[i], same = 0;
      for (int j = 0; j < nb; j++)
        same += (int)floorf(sr->score[beam[j]]) == (int)floorf(sr->score[k]) && (int)floorf(sr->cy[beam[j]]) == (int)floorf(sr->cy[k]);
      if (nb < s_beam && sr->status[k] == 0 && same < 2) beam[nb++] = k;
      else sr->cmd[k] = 2, sem_post(&sr->go[k]);
    }
  }
  exit(0);
}

static void s_checkpoint(void) {
  if (s_id < 0) {
    /* orphaned workers are adopted and reaped here */
    signal(SIGCHLD, SIG_IGN);
    prctl(PR_SET_CHILD_SUBREAPER, 1);
    s_id = 0;
    if (fork() != 0) s_coordinate();
    s_act = 0;
  }
  s_report();
  s_wait_cmd();
  sr->done[s_id] = 0;
}
#endif
