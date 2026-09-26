/* Progress persistence. On the calculator the progress lives in a small record
 * ("drivemad.sav") of the Epsilon file storage, located through the userland header. */
#include <string.h>
#include "save.h"

Progress progress;


#if defined(HOST) || defined(NO_STORAGE)
#ifdef HOST
#include <stdio.h>
#include <stdlib.h>
#endif
void save_load(void) {
  memset(&progress, 0, sizeof progress);
#ifdef HOST
  const char *f = getenv("ND_SAVE");
  FILE *fp = f ? fopen(f, "rb") : 0;
  if (fp) {
    if (fread(&progress, sizeof progress, 1, fp) != 1) memset(&progress, 0, sizeof progress);
    fclose(fp);
  }
#endif
}
void save_store(void) {
#ifdef HOST
  const char *f = getenv("ND_SAVE");
  FILE *fp = f ? fopen(f, "wb") : 0;
  if (fp) {
    fwrite(&progress, sizeof progress, 1, fp);
    fclose(fp);
  }
#endif
}
#else

static const char rec_name[] = "drivemad.sav";
#define SAVE_MAGIC 0x444Du
#define USERLAND_MAGIC 0xDEC0EDFEu
#define STORAGE_MAGIC 0xEE0BDDBAu

static uint8_t *storage_buf;
static uint32_t storage_size;

static bool find_storage(void) {
  static const uint32_t cand[] = {0x90010000u, 0x90410000u};
  for (int i = 0; i < 2; i++) {
    const uint32_t *h = (const uint32_t *)cand[i];
    if (h[0] != USERLAND_MAGIC || h[11] != USERLAND_MAGIC) continue;
    uint32_t addr = h[3], size = h[4];
    if (addr < 0x20000000u || addr >= 0x24080000u || size < 1024 || size > 0x20000) continue;
    const uint32_t *fs = (const uint32_t *)addr;
    if (fs[0] != STORAGE_MAGIC) continue;
    uint32_t foot;
    memcpy(&foot, (const uint8_t *)addr + 4 + size, 4);
    if (foot != STORAGE_MAGIC) continue;
    storage_buf = (uint8_t *)addr + 4;
    storage_size = size;
    return true;
  }
  return false;
}

static inline uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static inline void wr16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

/* walk records; returns our record (or 0) and the end of used space */
static uint8_t *find_record(uint8_t **end) {
  uint8_t *p = storage_buf, *lim = storage_buf + storage_size, *found = 0;
  while (p + 2 <= lim) {
    uint16_t sz = rd16(p);
    if (sz == 0) break;
    if (sz < 3 || p + sz > lim) return *end = 0, (uint8_t *)0; /* corrupt: do not touch */
    if (!found && strcmp((const char *)p + 2, rec_name) == 0) found = p;
    p += sz;
  }
  *end = p;
  return found;
}

#define REC_SIZE (2 + sizeof rec_name + 2 + sizeof(Progress))

void save_load(void) {
  memset(&progress, 0, sizeof progress);
  if (!storage_buf && !find_storage()) return;
  uint8_t *end, *r = find_record(&end);
  if (!r || rd16(r) != REC_SIZE) return;
  const uint8_t *d = r + 2 + sizeof rec_name;
  if (rd16(d) != SAVE_MAGIC) return;
  memcpy(&progress, d + 2, sizeof progress);
}

void save_store(void) {
  if (!storage_buf && !find_storage()) return;
  uint8_t *end, *r = find_record(&end);
  if (!end) return;
  if (r && rd16(r) != REC_SIZE) return;
  if (!r) {
    /* append, keeping a comfortable margin and the terminating zero size */
    if ((uint32_t)(end - storage_buf) + REC_SIZE + 2 + 256 > storage_size) return;
    r = end;
    wr16(r, REC_SIZE);
    memcpy(r + 2, rec_name, sizeof rec_name);
    wr16(r + REC_SIZE, 0);
  }
  uint8_t *d = r + 2 + sizeof rec_name;
  wr16(d, SAVE_MAGIC);
  memcpy(d + 2, &progress, sizeof progress);
}
#endif
