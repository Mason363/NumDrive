#ifndef SAVE_H
#define SAVE_H
#include <stdint.h>
#include <stdbool.h>

/* progress: one bit per completed level, plus the last played level */
typedef struct {
  uint8_t done[25];
  uint8_t last;
  uint8_t pad[2];
} Progress;

extern Progress progress;
void save_load(void);
void save_store(void);

static inline bool lvl_done(int i) { return (progress.done[i >> 3] >> (i & 7)) & 1; }
static inline void lvl_set_done(int i) { progress.done[i >> 3] |= (uint8_t)(1 << (i & 7)); }

#endif
