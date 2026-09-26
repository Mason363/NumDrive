#include <stdlib.h>
#include <string.h>
#define ARENA_IMPL
#include "world.h"
#include "inflate.h"
#include "ops.h"
#ifdef HOST
#include <stdio.h>
#define FAIL(msg) do { fprintf(stderr, "world: %s (line %d)\n", msg, __LINE__); return false; } while (0)
#else
#define FAIL(msg) return false
#endif

extern const uint8_t pack_data[];

/* ------------------------------------------------------------------ arena */
#ifndef ARENA_SIZE
#define ARENA_SIZE (122 * 1024)
#endif
static uint8_t arena[ARENA_SIZE] __attribute__((aligned(8)));
/* persistent data grows up from the bottom; the object table sits at the very top (arena_hi) and
 * setup-only data grows down below it */
static uint32_t arena_pos, arena_tmp = ARENA_SIZE, arena_hi = ARENA_SIZE;
uint32_t arena_peak;

static inline void arena_track(void) {
  uint32_t used = arena_pos + (ARENA_SIZE - arena_tmp);
  if (used > arena_peak) arena_peak = used;
}

void *arena_alloc(uint32_t size) {
  size = (size + 7) & ~7u;
  if (arena_pos + size > arena_tmp) {
    OOM("arena");
    return 0;
  }
  void *p = arena + arena_pos;
  arena_pos += size;
  arena_track();
  return p;
}
void *arena_tmp_alloc(uint32_t size) {
  size = (size + 7) & ~7u;
  if (arena_tmp < arena_pos + size) return 0;
  arena_tmp -= size;
  arena_track();
  return arena + arena_tmp;
}
void arena_tmp_reset(void) { arena_tmp = arena_hi; }
uint32_t arena_tmp_mark(void) { return arena_tmp; }
void arena_tmp_release(uint32_t m) { arena_tmp = m; }
static void *arena_high_alloc(uint32_t size) {
  size = (size + 7) & ~7u;
  if (arena_tmp < arena_pos + size || arena_tmp != arena_hi) return 0;
  arena_hi -= size;
  arena_tmp = arena_hi;
  arena_track();
  return arena + arena_hi;
}
/* long lived allocation from the top (below the object table) after setup; kept until level reset */
void *arena_keep_alloc(uint32_t size) {
  if (arena_tmp != arena_hi) return 0;
  return arena_high_alloc(size);
}
int level_serial; /* incremented on every level load */

void arena_reset(void) {
  level_serial++;
  arena_pos = 0;
  arena_tmp = arena_hi = ARENA_SIZE;
}
#if defined(HOST) && defined(ARENA_PROFILE)
#include <stdio.h>
static struct { const char *f; int l; uint32_t n; } prof[64];
static int nprof;
/* live allocations (bump order) and a snapshot of them at the peak */
static struct { uint32_t off, n; const char *f; int l; } live[8192];
static int nlive;
static char peak_snap[4096];
static uint32_t snap_peak;
static void prof_dump(void) {
  for (int i = 0; i < nprof; i++) printf("PROF %s:%d %u\n", prof[i].f, prof[i].l, prof[i].n);
  printf("PEAK %u\n%s", snap_peak, peak_snap);
}
static void prof_snapshot(void) {
  uint32_t used = arena_pos + (ARENA_SIZE - arena_tmp);
  if (used <= snap_peak) return;
  snap_peak = used;
  int n = snprintf(peak_snap, sizeof peak_snap, "  pos %u tmp %u\n", arena_pos, ARENA_SIZE - arena_tmp);
  for (int i = 0; i < nlive && n < (int)sizeof peak_snap - 64; i++) {
    bool seen = false;
    for (int k = 0; k < i && !seen; k++) seen = live[k].l == live[i].l && live[k].f == live[i].f;
    if (seen) continue;
    uint32_t tot = 0;
    int cnt = 0;
    for (int j = i; j < nlive; j++)
      if (live[j].l == live[i].l && live[j].f == live[i].f) tot += live[j].n, cnt++;
    n += snprintf(peak_snap + n, sizeof peak_snap - n, "  %s:%d x%d %u\n", live[i].f, live[i].l, cnt, tot);
  }
}
void *arena_alloc_dbg(uint32_t size, const char *f, int l) {
  if (!nprof) atexit(prof_dump);
  int i;
  for (i = 0; i < nprof; i++)
    if (prof[i].l == l && prof[i].f == f) break;
  if (i == nprof && nprof < 64) prof[nprof++] = (typeof(prof[0])){f, l, 0};
  if (i < 64) prof[i].n += (size + 7) & ~7u;
  while (nlive && live[nlive - 1].off >= arena_pos) nlive--;
  uint32_t off = arena_pos;
  void *p = arena_alloc(size);
  if (p && nlive < 8192) live[nlive++] = (typeof(live[0])){off, (size + 7) & ~7u, f, l};
  prof_snapshot();
  return p;
}
#define arena_alloc(n) arena_alloc_dbg((n), __FILE__, __LINE__)
#endif
uint32_t arena_used(void) { return arena_pos; }
uint32_t arena_free(void) { return arena_tmp - arena_pos; }
uint32_t arena_mark(void) { return arena_pos; }
void arena_release(uint32_t m) { arena_pos = m; }
bool arena_release_top(void *p, uint32_t size) {
  size = (size + 7) & ~7u;
  if ((uint8_t *)p + size != arena + arena_pos) return false;
  arena_pos -= size;
  return true;
}
bool arena_extend(void *p, uint32_t old_size, uint32_t new_size) {
  old_size = (old_size + 7) & ~7u;
  new_size = (new_size + 7) & ~7u;
  if ((uint8_t *)p + old_size != arena + arena_pos) return false;
  uint32_t start = (uint32_t)((uint8_t *)p - arena);
  if (start + new_size > arena_tmp) return false;
  arena_pos = start + new_size;
  arena_track();
  return true;
}
void *arena_top(uint32_t *avail) {
  *avail = arena_tmp - arena_pos;
  return arena + arena_pos;
}

/* ------------------------------------------------------------------- pack */
static uint16_t rd16(const uint8_t *p) { return p[0] | p[1] << 8; }
static uint32_t rd32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static float rdf(const uint8_t *p) {
  uint32_t v = rd32(p);
  float f;
  memcpy(&f, &v, 4);
  return f;
}

int pack_records(void) { return rd16(pack_data + 4); }
static uint32_t pack_size_word(int rec) {
  int n = pack_records();
  return rd32(pack_data + 6 + 4 * (n + 1) + 4 * rec);
}
uint32_t pack_rawsize(int rec) { return pack_size_word(rec) & 0x7FFFFFFFu; }
/* records stored uncompressed are used in place (no RAM copy) */
static const uint8_t *pack_inplace(int rec) {
  return (pack_size_word(rec) & 0x80000000u) ? pack_data + rd32(pack_data + 6 + 4 * rec) : 0;
}
int pack_load(int rec, uint8_t *dst, uint32_t cap) {
  const uint8_t *o = pack_data + 6 + 4 * rec;
  uint32_t a = rd32(o), b = rd32(o + 4), n = pack_rawsize(rec);
  if (n > cap) return -1;
  if (pack_inplace(rec)) {
    memcpy(dst, pack_data + a, n);
    return (int)n;
  }
  return inflate_raw(pack_data + a, b - a, dst, cap);
}

static const uint8_t *load_rec(int rec) {
  const uint8_t *in = pack_inplace(rec);
  if (in) return in;
  uint32_t n = pack_rawsize(rec);
  uint8_t *p = arena_alloc(n);
  if (!p || pack_load(rec, p, n) != (int)n) return 0;
  return p;
}

/* -------------------------------------------------------------- directory */
uint16_t nglobals;
const uint8_t *global_types;
int nlevels;
static const uint8_t *level_dir; /* per level: u16 record, name (u8 length + chars) */
static uint16_t nblocklib, block_group, block_rec0;
static const uint8_t *dir_buf; /* the directory record is stored uncompressed */

bool world_init(void) {
  dir_buf = pack_inplace(0);
  if (!dir_buf) FAIL("world_init");
  const uint8_t *p = dir_buf;
  nblocklib = rd16(p);
  block_group = p[2];
  block_rec0 = rd16(p + 3);
  p += 5;
  nglobals = rd16(p);
  p += 2;
  global_types = p;
  p += nglobals;
  nlevels = rd16(p);
  level_dir = p + 2;
  return true;
}

static const uint8_t *level_entry(int i) {
  const uint8_t *p = level_dir;
  while (i-- > 0) p += 3 + p[2];
  return p;
}

const char *level_name(int i) {
  static char buf[24];
  const uint8_t *q = level_entry(i) + 2;
  int l = q[0] < 23 ? q[0] : 23;
  memcpy(buf, q + 1, l);
  buf[l] = 0;
  return buf;
}

/* ----------------------------------------------------------------- blocks */
Block *blocks[256];
static uint8_t need_blk[256];

/* block record: u8 flags, u8 ncomp, per component (u16 voxel count, f32 sums[3], u8 bounds[6]),
 * then the part kept in RAM: u8 nbox, boxes (3 bytes each), u8 full faces, per face u8 count and quads (3 bytes) */
#define COMP_STATS 6
static uint32_t block_tail(const uint8_t *t) {
  const uint8_t *p = t + 1 + 3 * t[0] + 1;
  for (int f = 0; f < 6; f++) p += 1 + 3 * p[0];
  return (uint32_t)(p - t);
}

static bool load_blocks(void) {
  for (int g0 = 0; g0 < nblocklib; g0 += block_group) {
    bool any = false;
    for (int i = g0; i < g0 + block_group && i < nblocklib; i++)
      if (need_blk[i] && !blocks[i]) any = true;
    if (!any) continue;
    int rec = block_rec0 + g0 / block_group;
    uint32_t n = pack_rawsize(rec);
    uint32_t tm = arena_tmp_mark();
    uint8_t *base = arena_tmp_alloc(n);
    if (!base || pack_load(rec, base, n) != (int)n) FAIL("load_blocks");
    const uint8_t *p = base;
    for (int i = g0; i < g0 + block_group && i < nblocklib; i++) {
      int ncomp = p[1];
      const uint8_t *st = p + 2, *t = st + COMP_STATS * ncomp;
      uint32_t tl = block_tail(t);
      if (need_blk[i] && !blocks[i]) {
        Block *b = arena_alloc(sizeof(Block));
        uint8_t *keep = arena_alloc(tl);
        if (!b || !keep) FAIL("load_blocks");
        memset(b, 0, sizeof *b);
        b->flags = p[0];
        b->ncomp = (uint8_t)ncomp;
        b->bb = arena_alloc(6 * ncomp);
        if (!b->bb) FAIL("load_blocks");
        memcpy(b->bb, st, 6 * ncomp);
        memcpy(keep, t, tl);
        b->nbox = keep[0];
        b->boxes = keep + 1;
        const uint8_t *r = keep + 1 + 3 * b->nbox;
        b->full = *r++;
        for (int f = 0; f < 6; f++) {
          b->nq[f] = r[0];
          b->fq[f] = r + 1;
          r += 1 + 3 * r[0];
        }
        blocks[i] = b;
      }
      p = t + tl;
    }
    arena_tmp_release(tm);
  }
  return true;
}

/* ---------------------------------------------------------------- objects */
/* Grids are shipped as ready-made objects (grouped by the pack tool with Fancade's glue rules):
 * u16 count, per object: varint parts, per part: u8 block, varint key delta. */
static uint32_t rdvar(const uint8_t **pp) {
  const uint8_t *p = *pp;
  uint32_t v = 0;
  int sh = 0;
  while (*p & 0x80) {
    v |= (uint32_t)(*p++ & 0x7F) << sh;
    sh += 7;
  }
  v |= (uint32_t)(*p++) << sh;
  *pp = p;
  return v;
}

/* first pass: note the blocks used, return the end of the data */
static const uint8_t *scan_objects(const uint8_t *p) {
  uint16_t n = rd16(p);
  p += 2;
  uint32_t total = 0;
  for (int o = 0; o < n; o++) total += rdvar(&p);
  for (uint32_t i = 0; i < total; i++) need_blk[*p++] = 1;
  for (uint32_t i = 0; i < total; i++) rdvar(&p);
  return p;
}

int shape_find(const Shape *s, int x, int y, int z) {
  if (x < 0 || y < 0 || z < 0 || x > 1023 || y > 127 || z > 1023) return -1;
  uint32_t k = (uint32_t)x << 20 | (uint32_t)y << 13 | (uint32_t)z << 3;
  int lo = 0, hi = s->np - 1, r = -1;
  while (lo <= hi) {
    int m = (lo + hi) >> 1;
    if (s->key[m] < k) lo = m + 1;
    else {
      r = m;
      hi = m - 1;
    }
  }
  if (r >= 0 && (s->key[r] & ~7u) == k) return r;
  return -1;
}

/* position of one block of an object: its cell for a stock block, the centre of its voxels for a
   custom one */
vec3 shape_block_center(const Shape *s, int x, int y, int z) {
  int v0[3] = {8, 8, 8}, v1[3] = {-1, -1, -1};
  for (int i = 0; i < s->np; i++) {
    uint32_t k = s->key[i];
    if (PK_X(k) != x || PK_Y(k) != y || PK_Z(k) != z) continue;
    const Block *b = blocks[s->blk[i]];
    if (b->flags & 8) return v3(x + 0.5f, y + 0.5f, z + 0.5f);
    const uint8_t *bb = b->bb + PK_C(k) * 6;
    for (int a = 0; a < 3; a++) {
      if (bb[a] < v0[a]) v0[a] = bb[a];
      if (bb[a + 3] > v1[a]) v1[a] = bb[a + 3];
    }
  }
  if (v1[0] < v0[0]) return v3(x + 0.5f, y + 0.5f, z + 0.5f);
  return v3(x + (v0[0] + v1[0] + 1) / 16.0f, y + (v0[1] + v1[1] + 1) / 16.0f, z + (v0[2] + v1[2] + 1) / 16.0f);
}

static void shape_finish(Shape *s) {
  /* occlusion masks */
  for (int i = 0; i < s->np; i++) {
    uint32_t k = s->key[i];
    int x = PK_X(k), y = PK_Y(k), z = PK_Z(k);
    uint8_t m = 0;
    static const int8_t D[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (int f = 0; f < 6; f++) {
      int j = shape_find(s, x + D[f][0], y + D[f][1], z + D[f][2]);
      if (j >= 0 && (blocks[s->blk[j]]->full & (1 << (f ^ 1)))) m |= 1 << f;
    }
    s->occ[i] = m;
  }
  /* mass: the volume of the parts' voxel bounds (Fancade's colliders); com: the centre of the cells
     holding colliders (a hinged plank balances over a pivot in its column, as in the original); bounds */
  float v = 0;
  float bx0 = 1e9f, by0 = 1e9f, bz0 = 1e9f, bx1 = -1e9f, by1 = -1e9f, bz1 = -1e9f;
  int clo[2][3] = {{1 << 20, 1 << 20, 1 << 20}, {1 << 20, 1 << 20, 1 << 20}}, chi[2][3] = {{-1, -1, -1}, {-1, -1, -1}};
  uint8_t coll = 0;
  for (int i = 0; i < s->np; i++) {
    Block *b = blocks[s->blk[i]];
    uint32_t k = s->key[i];
    int c = PK_C(k);
    int cell[3] = {PK_X(k), PK_Y(k), PK_Z(k)};
    for (int t = 0; t < 2; t++)
      if (t == 0 || (b->flags & 3))
        for (int a = 0; a < 3; a++) {
          if (cell[a] < clo[t][a]) clo[t][a] = cell[a];
          if (cell[a] > chi[t][a]) chi[t][a] = cell[a];
        }
    float cx = cell[0] * 8.0f, cy = cell[1] * 8.0f, cz = cell[2] * 8.0f;
    const uint8_t *bb = b->bb + c * 6;
    if (bb[0] <= bb[3]) v += (bb[3] - bb[0] + 1) * (bb[4] - bb[1] + 1) * (bb[5] - bb[2] + 1);
    if (cx + bb[0] < bx0) bx0 = cx + bb[0];
    if (cy + bb[1] < by0) by0 = cy + bb[1];
    if (cz + bb[2] < bz0) bz0 = cz + bb[2];
    if (cx + bb[3] + 1 > bx1) bx1 = cx + bb[3] + 1;
    if (cy + bb[4] + 1 > by1) by1 = cy + bb[4] + 1;
    if (cz + bb[5] + 1 > bz1) bz1 = cz + bb[5] + 1;
    if (b->flags & 3) coll = 1;
  }
  s->mass = v / 512.0f;
  {
    int t = chi[1][0] >= 0 ? 1 : 0;
    s->com = s->np ? v3((clo[t][0] + chi[t][0] + 1) * 0.5f, (clo[t][1] + chi[t][1] + 1) * 0.5f, (clo[t][2] + chi[t][2] + 1) * 0.5f)
                   : v3(0, 0, 0);
  }
  {
    /* Fancade's object position: the centre of its bounds, where a stock block counts its whole cell
       and a custom block its voxels */
    float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
    for (int i = 0; i < s->np; i++) {
      const Block *b = blocks[s->blk[i]];
      uint32_t k = s->key[i];
      const uint8_t *bb = b->bb + PK_C(k) * 6;
      int c[3] = {PK_X(k), PK_Y(k), PK_Z(k)};
      for (int a = 0; a < 3; a++) {
        float l = (b->flags & 8) ? c[a] : c[a] + bb[a] / 8.0f, h = (b->flags & 8) ? c[a] + 1 : c[a] + (bb[a + 3] + 1) / 8.0f;
        if (l < lo[a]) lo[a] = l;
        if (h > hi[a]) hi[a] = h;
      }
    }
    s->origin = s->np ? v3((lo[0] + hi[0]) * 0.5f, (lo[1] + hi[1]) * 0.5f, (lo[2] + hi[2]) * 0.5f) : s->com;
    /* an object of one block sits at the centre of its cell */
    bool one = true;
    for (int i = 1; i < s->np && one; i++) one = (s->key[i] & ~7u) == (s->key[0] & ~7u);
    if (s->np && one) s->origin = v3(PK_X(s->key[0]) + 0.5f, PK_Y(s->key[0]) + 0.5f, PK_Z(s->key[0]) + 0.5f);
  }
  s->bmin = v3(bx0 / 8, by0 / 8, bz0 / 8);
  s->bmax = v3(bx1 / 8, by1 / 8, bz1 / 8);
  s->coll = coll;
  if (s->np == 1 && (blocks[s->blk[0]]->flags & 3) == 2) {
    Block *b = blocks[s->blk[0]];
    const uint8_t *bb = b->bb + PK_C(s->key[0]) * 6;
    s->sphere = 1;
    float ex = (bb[3] - bb[0] + 1) * 0.5f, ey = (bb[4] - bb[1] + 1) * 0.5f;
    s->radius = (ex > ey ? ex : ey) / 8.0f;
  }
}

/* second pass (blocks loaded): build the shapes; the list of shapes is setup-only data */
static Shape **build_shapes(const uint8_t *p, uint16_t *nout) {
  uint16_t n = rd16(p);
  p += 2;
  Shape **shapes = arena_tmp_alloc(sizeof(Shape *) * (n ? n : 1));
  if (!shapes) return 0;
  const uint8_t *cnt = p;
  uint32_t total = 0;
  for (int o = 0; o < n; o++) total += rdvar(&p);
  const uint8_t *bp = p, *dp = p + total; /* block stream, key delta stream */
  for (int o = 0; o < n; o++) {
    uint32_t np = rdvar(&cnt);
    Shape *s = arena_alloc(sizeof(Shape));
    if (!s) return 0;
    memset(s, 0, sizeof *s);
    s->np = (uint16_t)np;
    s->key = arena_alloc(4 * np);
    s->blk = arena_alloc(np);
    s->occ = arena_alloc(np);
    if (!s->key || !s->blk || !s->occ) return 0;
    uint32_t key = 0;
    for (uint32_t i = 0; i < np; i++) {
      s->blk[i] = *bp++;
      key += rdvar(&dp);
      s->key[i] = key;
    }
    shape_finish(s);
    shapes[o] = s;
  }
  *nout = n;
  return shapes;
}

/* --------------------------------------------------------------- programs */
Obj *objs;
int obj_cap;
int nobj, nlevelobj;
Level level;
/* programs of the level (with their custom block children) */
#define NPROG 48
static Prog *progs[NPROG];
static int nprogs;
static Prog *prog_of(int rec) {
  for (int i = 0; i < nprogs; i++)
    if (progs[i]->rec == rec) return progs[i];
  return 0;
}

int op_data_size(int op) {
  switch (op) {
    case OP_NUMBER: return 4;
    case OP_VECTOR: case OP_ROTATION: return 12;
    case OP_GET_VAR_NUM: case OP_GET_VAR_VEC: case OP_GET_VAR_ROT: case OP_GET_VAR_TRU: case OP_GET_VAR_OBJ:
    case OP_GET_VAR_CON: case OP_SET_VAR_NUM: case OP_SET_VAR_VEC: case OP_SET_VAR_ROT: case OP_SET_VAR_TRU:
    case OP_SET_VAR_OBJ: case OP_SET_VAR_CON: return 2;
    case OP_WIN: case OP_LOSE: case OP_BUTTON: case OP_SET_CAMERA: case OP_JOYSTICK: case OP_PLAY_SOUND: return 1;
    default: return 0;
  }
}


static bool load_children(Prog *p);
static Prog *load_prog(int rec) {
  Prog *old = prog_of(rec);
  if (old) return old;
  if (nprogs == NPROG) return 0;
  const uint8_t *d = load_rec(rec);
  if (!d) return 0;
  Prog *p = arena_alloc(sizeof(Prog));
  memset(p, 0, sizeof *p);
  progs[nprogs++] = p;
  p->rec = rec;
  const uint8_t *q = d;
  p->is_level = q[0] & 1;
  p->yc = (q[0] >> 1) & 15;
  bool tables = q[0] & 0x80;
  p->nnodes = rd16(q + 1);
  p->nentries = rd16(q + 3);
  p->entries = q + 5;
  q += 5 + 2 * p->nentries;
  p->nlocals = rd16(q);
  p->ltypes = q + 2;
  q += 2 + p->nlocals;
  p->ninputs = q[0];
  p->noutputs = q[1];
  p->outrefs = q + 2;
  q += 2 + 2 * p->noutputs;
  p->nself = q[0];
  p->selfvox = q + 1;
  q += 1 + 3 * p->nself;
  /* template objects */
  p->objdata = q;
  q = scan_objects(q);
  if (tables) {
    /* records read in place from flash carry their node offset and output slot tables */
    p->nslots = rd16(q);
    q += 2 + ((q + 2 - d) & 1);
    p->off = (const uint16_t *)q;
    p->slot = (const uint16_t *)(q + 2 * p->nnodes);
    p->data = q + 4 * p->nnodes;
    return load_children(p) ? p : 0;
  }
  p->data = q;
  /* node offsets */
  uint16_t *off = arena_alloc(2 * p->nnodes + 2), *slot = arena_alloc(2 * p->nnodes + 2);
  if (!off || !slot) return 0;
  p->off = off;
  p->slot = slot;
  uint16_t slots = 0;
  const uint8_t *n = q;
  for (int i = 0; i < p->nnodes; i++) {
    off[i] = (uint16_t)(n - q);
    int op = n[0];
    slot[i] = NONE16;
    if (op == OP_CUSTOM) {
      n += 10 + 2 * n[9]; /* header, inputs */
      n += 3 + 2 * n[2];  /* anchor object, self objects */
      continue;
    }
    const unsigned char *sh = op_shape[op];
    if (sh[3] && sh[1]) {
      slot[i] = slots;
      slots += sh[1];
    }
    n += 1 + 2 * sh[0];
    for (int e = 0; e < sh[2]; e++) n += 1 + 2 * n[0];
    n += 1 + 2 * n[0];
    n += op_data_size(op);
  }
  p->nslots = slots;
  return load_children(p) ? p : 0;
}

static bool load_children(Prog *p) {
  for (int i = 0; i < p->nnodes; i++) {
    const uint8_t *nd = p->data + p->off[i];
    if (nd[0] == OP_CUSTOM && !load_prog(rd16(nd + 1))) return false;
  }
  return true;
}

Prog *prog_child(const Prog *p, int i) {
  const uint8_t *nd = p->data + p->off[i];
  return nd[0] == OP_CUSTOM ? prog_of(rd16(nd + 1)) : 0;
}

float mat_tab[NMAT][2];
int nmat;
uint8_t mat_find(float friction, float bounce) {
  int best = 0;
  float bd = 1e30f;
  for (int i = 0; i < nmat; i++) {
    float df = mat_tab[i][0] - friction, db = mat_tab[i][1] - bounce, d = df * df + db * db;
    if (d == 0) return (uint8_t)i;
    if (d < bd) bd = d, best = i;
  }
  if (nmat == NMAT) return (uint8_t)best; /* full: the closest pair */
  mat_tab[nmat][0] = friction;
  mat_tab[nmat][1] = bounce;
  return (uint8_t)nmat++;
}

static void obj_init(Obj *o, Shape *s) {
  memset(o, 0, sizeof *o);
  o->shape = s;
  o->pos = s->origin;
  o->rot = qident();
  o->flags = OF_VISIBLE | (s->coll ? OF_COLLIDE : 0);
  for (int i = 0; i < s->np; i++)
    if (blocks[s->blk[i]]->flags & 4) o->flags |= OF_PHYSICS;
  o->mat = 0;
  o->mass = s->mass > 0 ? s->mass : 1;
#ifdef HOST
  if (getenv("ND_DEFMASS")) o->mass *= atof(getenv("ND_DEFMASS"));
#endif
  o->lockp = 7;
  o->lockr = 7;
  o->body = -1;
}

int obj_clone(int src) {
  if (src < 0) return -1;
  if (nobj >= obj_cap) {
    OOM("objects");
    return -1;
  }
  Obj *o = &objs[nobj];
  *o = objs[src];
  o->flags &= ~(OF_TEMPLATE | OF_DEAD | OF_DYNAMIC);
  o->flags |= OF_VISIBLE;
  if (o->shape->coll) o->flags |= OF_COLLIDE;
  o->body = -1;
  o->src = objs[src].src;
  return nobj++;
}

void obj_destroy(int id) {
  if (id < 0 || id >= nobj) return;
  objs[id].flags |= OF_DEAD;
  objs[id].flags &= ~(OF_VISIBLE | OF_COLLIDE);
}

/* ------------------------------------------------------------------ level */
extern bool vm_setup_envs(void); /* vm.c */

int level_index;
#ifdef HOST
int st_obj_setup;
uint32_t st_arena_setup;
#endif

bool world_load_level(int index) {
  level_index = index;
  arena_reset();
  nmat = 0;
  mat_find(0.5f, 0.0f); /* index 0: Fancade's defaults */
#ifdef HOST
  if (getenv("ND_OBJFRIC")) mat_tab[0][0] = atof(getenv("ND_OBJFRIC"));
#endif
  memset(blocks, 0, sizeof blocks);
  memset(need_blk, 0, sizeof need_blk);
  nobj = 0;
  nprogs = 0;
  /* the level record is only needed while loading: inflate it into free space, read the object
   * capacity, reserve the object table at the top, then keep the record just below it */
  uint32_t avail;
  uint8_t *d = arena_top(&avail);
  int lrec = rd16(level_entry(index));
  uint32_t rn = pack_rawsize(lrec);
  if (rn > avail || pack_load(lrec, d, rn) != (int)rn) FAIL("world_load_level");
  const uint8_t *q = d;
  int l = q[0];
  memcpy(level.name, q + 1, l < 39 ? l : 39);
  level.name[l < 39 ? l : 39] = 0;
  q += 1 + l;
  level.sx = rd16(q);
  level.sy = rd16(q + 2);
  level.sz = rd16(q + 4);
  level.bg = q[6];
  int prec = rd16(q + 7);
  obj_cap = rd16(q + 9);
  level.body_cap = rd16(q + 11);
  level.joint_cap = rd16(q + 13);
  q += 15;
#ifdef HOST
  if (getenv("ND_NOCAPS")) obj_cap = 3000, level.body_cap = 600, level.joint_cap = 800;
#endif
  uint32_t head = (uint32_t)(q - d);
  objs = arena_high_alloc(sizeof(Obj) * obj_cap);
  uint8_t *rec = arena_tmp_alloc(rn);
  if (!objs || !rec) FAIL("world_load_level");
  memmove(rec, d, rn);
  const uint8_t *objdata = rec + head;
  scan_objects(objdata);
  level.prog = load_prog(prec);
  if (!level.prog) FAIL("world_load_level");
  level.index = index;
  if (!load_blocks()) FAIL("world_load_level");
  /* level objects first, then the template shapes of every program */
  uint16_t n;
  Shape **shapes = build_shapes(objdata, &n);
  if (!shapes) FAIL("world_load_level");
  for (int i = 0; i < n; i++) {
    obj_init(&objs[nobj], shapes[i]);
    objs[nobj].src = nobj;
    nobj++;
  }
  nlevelobj = nobj;
  for (int r = 0; r < nprogs; r++) {
    Prog *p = progs[r];
    p->tmpl = build_shapes(p->objdata, &p->ntmpl);
    if (!p->tmpl) FAIL("world_load_level");
  }
#ifdef HOST
  if (getenv("ND_SHDBG")) {
    for (int r = -1; r < nprogs; r++) {
      Shape **ss = r < 0 ? shapes : progs[r]->tmpl;
      int ns = r < 0 ? n : progs[r]->ntmpl;
      for (int i = 0; i < ns; i++) {
        Shape *sh = ss[i];
        uint32_t h = 2166136261u, k0 = sh->np ? sh->key[0] & ~7u : 0;
        uint32_t x0 = k0 >> 20, y0 = (k0 >> 13) & 127, z0 = (k0 >> 3) & 1023;
        for (int j = 0; j < sh->np; j++) {
          uint32_t k = sh->key[j], d = ((k >> 20) - x0) << 20 | (((k >> 13) & 127) - y0) << 13 | (((k >> 3) & 1023) - z0) << 3 | (k & 7);
          h = (h ^ d) * 16777619u;
          h = (h ^ sh->blk[j]) * 16777619u;
        }
        fprintf(stderr, "SHAPE r%d i%d np %d hash %08x\n", r, i, sh->np, h);
        if (r < 0 && getenv("ND_SHPARTS") && atoi(getenv("ND_SHPARTS")) == i)
          for (int j = 0; j < sh->np; j++) {
            const Block *b = blocks[sh->blk[j]];
            int c = PK_C(sh->key[j]);
            const uint8_t *bb = b->bb + c * 6;
            fprintf(stderr, "  part blk%d fl%x cell(%d,%d,%d) comp %d coll %d bb[%d-%d,%d-%d,%d-%d]\n", sh->blk[j], b->flags, PK_X(sh->key[j]), PK_Y(sh->key[j]), PK_Z(sh->key[j]), c,
                    b->flags & 3, bb[0], bb[3], bb[1], bb[4], bb[2], bb[5]);
          }
      }
    }
  }
#endif
  if (!vm_setup_envs()) FAIL("world_load_level");
  arena_tmp_reset(); /* the level record and template lists are only needed while instantiating */
#ifdef HOST
  {
    extern int st_obj_setup;
    extern uint32_t st_arena_setup;
    if (nobj > st_obj_setup) st_obj_setup = nobj;
    if (arena_pos > st_arena_setup) st_arena_setup = arena_pos;
  }
#endif
  return true;
}

/* template objects are created by the VM when environments are instantiated */
int world_add_template(Shape *s) {
  if (nobj >= obj_cap) return -1;
  obj_init(&objs[nobj], s);
  objs[nobj].flags &= ~(OF_VISIBLE | OF_COLLIDE);
  objs[nobj].flags |= OF_TEMPLATE;
  objs[nobj].src = nobj;
  return nobj++;
}
