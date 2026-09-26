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
#define ARENA_SIZE (100 * 1024)
#endif
static uint8_t arena[ARENA_SIZE] __attribute__((aligned(8)));
static uint32_t arena_pos;
uint32_t arena_peak;

void *arena_alloc(uint32_t size) {
  size = (size + 7) & ~7u;
  if (arena_pos + size > ARENA_SIZE) return 0;
  void *p = arena + arena_pos;
  arena_pos += size;
  if (arena_pos > arena_peak) arena_peak = arena_pos;
  return p;
}
void arena_reset(void) { arena_pos = 0; }
#if defined(HOST) && defined(ARENA_PROFILE)
#include <stdio.h>
static struct { const char *f; int l; uint32_t n; } prof[64];
static int nprof;
static void prof_dump(void) {
  for (int i = 0; i < nprof; i++) printf("PROF %s:%d %u\n", prof[i].f, prof[i].l, prof[i].n);
}
void *arena_alloc_dbg(uint32_t size, const char *f, int l) {
  if (!nprof) atexit(prof_dump);
  int i;
  for (i = 0; i < nprof; i++)
    if (prof[i].l == l && prof[i].f == f) break;
  if (i == nprof && nprof < 64) prof[nprof++] = (typeof(prof[0])){f, l, 0};
  if (i < 64) prof[i].n += (size + 7) & ~7u;
  return arena_alloc(size);
}
#define arena_alloc(n) arena_alloc_dbg((n), __FILE__, __LINE__)
#endif
uint32_t arena_used(void) { return arena_pos; }
uint32_t arena_mark(void) { return arena_pos; }
void arena_release(uint32_t m) { arena_pos = m; }
void *arena_top(uint32_t *avail) {
  *avail = ARENA_SIZE - arena_pos;
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
uint32_t pack_rawsize(int rec) {
  int n = pack_records();
  return rd32(pack_data + 6 + 4 * (n + 1) + 4 * rec);
}
int pack_load(int rec, uint8_t *dst, uint32_t cap) {
  const uint8_t *o = pack_data + 6 + 4 * rec;
  uint32_t a = rd32(o), b = rd32(o + 4);
  if (pack_rawsize(rec) > cap) return -1;
  return inflate_raw(pack_data + a, b - a, dst, cap);
}

static uint8_t *load_rec(int rec) {
  uint32_t n = pack_rawsize(rec);
  uint8_t *p = arena_alloc(n);
  if (!p || pack_load(rec, p, n) != (int)n) return 0;
  return p;
}

/* -------------------------------------------------------------- directory */
uint16_t nglobals;
const uint8_t *global_types;
int nlevels;
static uint16_t level_rec[200], level_nm[200];
static uint16_t nblocklib, block_group, block_rec0;
static uint8_t dir_buf[3200];
static uint8_t gtypes[256];

bool world_init(void) {
  uint32_t n = pack_rawsize(0);
  if (n > sizeof dir_buf || pack_load(0, dir_buf, sizeof dir_buf) != (int)n) FAIL("world_init");
  const uint8_t *p = dir_buf;
  nblocklib = rd16(p);
  block_group = p[2];
  block_rec0 = rd16(p + 3);
  p += 5;
  nglobals = rd16(p);
  p += 2;
  memcpy(gtypes, p, nglobals);
  global_types = gtypes;
  p += nglobals;
  nlevels = rd16(p);
  p += 2;
  for (int i = 0; i < nlevels; i++) {
    level_rec[i] = rd16(p);
    level_nm[i] = (uint16_t)(p + 2 - dir_buf);
    p += 3 + p[2];
  }
  return true;
}

const char *level_name(int i) {
  static char buf[24];
  const uint8_t *q = dir_buf + level_nm[i];
  int l = q[0] < 23 ? q[0] : 23;
  memcpy(buf, q + 1, l);
  buf[l] = 0;
  return buf;
}

/* ----------------------------------------------------------------- blocks */
Block *blocks[256];
static uint8_t need_blk[256];

static const uint8_t *parse_block(const uint8_t *p, Block *b) {
  b->flags = p[0];
  b->ncomp = p[1];
  p += 2;
  b->solid = p;
  p += 64;
  b->comp = 0;
  if (b->ncomp > 1) {
    b->comp = p;
    p += 256;
  }
  b->nbox = p[0];
  b->boxes = p + 1;
  p += 1 + 3 * b->nbox;
  b->glue = p;
  p += 48 * b->ncomp;
  b->full = p[0];
  p++;
  for (int f = 0; f < 6; f++) {
    b->nq[f] = p[0];
    b->fq[f] = p + 1;
    p += 1 + 3 * b->nq[f];
  }
  return p;
}

static bool load_blocks(void) {
  for (int g0 = 0; g0 < nblocklib; g0 += block_group) {
    bool any = false;
    for (int i = g0; i < g0 + block_group && i < nblocklib; i++)
      if (need_blk[i] && !blocks[i]) any = true;
    if (!any) continue;
    int rec = block_rec0 + g0 / block_group;
    uint32_t n = pack_rawsize(rec);
    uint32_t mark = arena_mark();
    uint8_t *base = arena_alloc(n);
    if (!base || pack_load(rec, base, n) != (int)n) FAIL("load_blocks");
    /* compact: keep only needed block records at the start of base */
    const uint8_t *p = base;
    uint32_t keep = 0;
    for (int i = g0; i < g0 + block_group && i < nblocklib; i++) {
      Block tb;
      const uint8_t *next = parse_block(p, &tb);
      uint32_t sz = (uint32_t)(next - p);
      if (need_blk[i] && !blocks[i]) {
        memmove(base + keep, p, sz);
        keep += sz;
      }
      p = next;
    }
    arena_release(mark);
    arena_alloc(keep);
    const uint8_t *q = base;
    for (int i = g0; i < g0 + block_group && i < nblocklib; i++) {
      if (need_blk[i] && !blocks[i]) {
        Block *bk = arena_alloc(sizeof(Block));
        if (!bk) FAIL("load_blocks");
        memset(bk, 0, sizeof *bk);
        q = parse_block(q, bk);
        blocks[i] = bk;
      }
    }
  }
  /* per-block component statistics */
  for (int i = 0; i < 256; i++) {
    Block *b = blocks[i];
    if (!b || b->cnt) continue;
    b->cnt = arena_alloc(2 * b->ncomp);
    b->sum = arena_alloc(12 * b->ncomp);
    b->bb = arena_alloc(6 * b->ncomp);
    if (!b->cnt || !b->sum || !b->bb) FAIL("load_blocks");
    for (int c = 0; c < b->ncomp; c++) {
      b->cnt[c] = 0;
      b->sum[c * 3] = b->sum[c * 3 + 1] = b->sum[c * 3 + 2] = 0;
      b->bb[c * 6] = b->bb[c * 6 + 1] = b->bb[c * 6 + 2] = 8;
      b->bb[c * 6 + 3] = b->bb[c * 6 + 4] = b->bb[c * 6 + 5] = 0;
    }
    for (int z = 0; z < 8; z++)
      for (int y = 0; y < 8; y++)
        for (int x = 0; x < 8; x++) {
          if (!blk_solid(b, x, y, z)) continue;
          int c = blk_comp(b, x, y, z);
          b->cnt[c]++;
          b->sum[c * 3] += x + 0.5f;
          b->sum[c * 3 + 1] += y + 0.5f;
          b->sum[c * 3 + 2] += z + 0.5f;
          uint8_t *bb = b->bb + c * 6;
          if (x < bb[0]) bb[0] = x;
          if (y < bb[1]) bb[1] = y;
          if (z < bb[2]) bb[2] = z;
          if (x > bb[3]) bb[3] = x;
          if (y > bb[4]) bb[4] = y;
          if (z > bb[5]) bb[5] = z;
        }
  }
  return true;
}

/* ------------------------------------------------------------------- grid */
typedef struct {
  uint16_t ncells;
  uint32_t *key; /* z<<20 | y<<10 | x */
  uint8_t *blk;
  uint16_t *nodebase;
  uint16_t nnodes;
  uint16_t *nodeobj;
  uint16_t nobj;
} Grid;

static const uint8_t *decode_grid(const uint8_t *p, Grid *g, bool pass1) {
  uint16_t x0 = rd16(p);
  if (x0 == 0xFFFF) {
    g->ncells = 0;
    return p + 2;
  }
  uint16_t y0 = rd16(p + 2), z0 = rd16(p + 4), dx = rd16(p + 6), dy = rd16(p + 8), dz = rd16(p + 10);
  p += 12;
  uint32_t total = (uint32_t)dx * dy * dz, i = 0;
  uint16_t n = 0;
  const uint8_t *start = p;
  while (i < total) {
    i += p[0];
    int run = p[1];
    p += 2;
    for (int k = 0; k < run; k++) {
      uint8_t b = p[k];
      if (pass1) {
        need_blk[b] = 1;
      } else {
        uint32_t li = i + k;
        uint32_t x = x0 + li % dx, y = y0 + (li / dx) % dy, z = z0 + li / ((uint32_t)dx * dy);
        g->key[n] = z << 20 | y << 10 | x;
        g->blk[n] = b;
      }
      n++;
    }
    p += run;
    i += run;
  }
  (void)start;
  g->ncells = n;
  return p;
}

static int grid_find(const Grid *g, uint32_t key) {
  int lo = 0, hi = g->ncells - 1;
  while (lo <= hi) {
    int m = (lo + hi) >> 1;
    if (g->key[m] < key) lo = m + 1;
    else if (g->key[m] > key) hi = m - 1;
    else return m;
  }
  return -1;
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

static uint16_t *uf;
static uint16_t uf_find(uint16_t a) {
  while (uf[a] != a) {
    uf[a] = uf[uf[a]];
    a = uf[a];
  }
  return a;
}

static uint64_t rd64(const uint8_t *p) { return (uint64_t)rd32(p) | (uint64_t)rd32(p + 4) << 32; }

/* Build objects of a grid; fills g->nodeobj and returns shapes (arena). */
static Shape **form_objects(Grid *g) {
  g->nodebase = arena_alloc(2 * (g->ncells + 1));
  uint16_t nn = 0;
  for (int i = 0; i < g->ncells; i++) {
    g->nodebase[i] = nn;
    nn += blocks[g->blk[i]]->ncomp;
  }
  g->nodebase[g->ncells] = nn;
  g->nnodes = nn;
  g->nodeobj = arena_alloc(2 * nn + 2);
  uint32_t mark = arena_mark();
  uf = arena_alloc(2 * nn + 2);
  if (!g->nodeobj || !uf) return 0;
  for (int i = 0; i < nn; i++) uf[i] = i;
  static const uint32_t step[3] = {1, 1 << 10, 1 << 20};
  for (int i = 0; i < g->ncells; i++) {
    Block *A = blocks[g->blk[i]];
    for (int axis = 0; axis < 3; axis++) {
      int j;
      uint32_t k = g->key[i];
      if (axis == 0 && (k & 1023) == 1023) continue;
      if (axis == 1 && ((k >> 10) & 1023) == 1023) continue;
      if (axis == 0 && i + 1 < g->ncells && g->key[i + 1] == k + 1) j = i + 1;
      else j = grid_find(g, k + step[axis]);
      if (j < 0) continue;
      Block *B = blocks[g->blk[j]];
      for (int ca = 0; ca < A->ncomp; ca++) {
        uint64_t ma = rd64(A->glue + (ca * 6 + axis * 2) * 8);
        if (!ma) continue;
        for (int cb = 0; cb < B->ncomp; cb++) {
          uint64_t mb = rd64(B->glue + (cb * 6 + axis * 2 + 1) * 8);
          if (ma & mb) {
            uint16_t ra = uf_find(g->nodebase[i] + ca), rb = uf_find(g->nodebase[j] + cb);
            if (ra != rb) uf[rb] = ra;
          }
        }
      }
    }
  }
  /* assign object ids in parse order: z desc, y desc, x asc, comp asc */
  for (int i = 0; i < nn; i++) g->nodeobj[i] = NONE16;
  uint16_t *rootobj = arena_alloc(2 * nn + 2);
  for (int i = 0; i < nn; i++) rootobj[i] = NONE16;
  uint16_t nobjs = 0;
  int end = g->ncells;
  while (end > 0) {
    int start = end - 1;
    uint32_t row = g->key[start] >> 10;
    while (start > 0 && (g->key[start - 1] >> 10) == row) start--;
    for (int i = start; i < end; i++)
      for (int c = 0; c < blocks[g->blk[i]]->ncomp; c++) {
        uint16_t nd = g->nodebase[i] + c;
        uint16_t r = uf_find(nd);
        if (rootobj[r] == NONE16) rootobj[r] = nobjs++;
        g->nodeobj[nd] = rootobj[r];
      }
    end = start;
  }
  g->nobj = nobjs;
  /* count parts per object */
  uint16_t *cnt = arena_alloc(2 * nobjs + 2);
  memset(cnt, 0, 2 * nobjs + 2);
  for (int i = 0; i < nn; i++) cnt[g->nodeobj[i]]++;
  arena_release(mark); /* drop union-find scratch (cnt is recomputed below) */
  Shape **shapes = arena_alloc(sizeof(Shape *) * (nobjs ? nobjs : 1));
  uint16_t *pc = arena_alloc(2 * nobjs + 2);
  memset(pc, 0, 2 * nobjs + 2);
  for (int i = 0; i < nn; i++) pc[g->nodeobj[i]]++;
  for (int o = 0; o < nobjs; o++) {
    Shape *s = arena_alloc(sizeof(Shape));
    memset(s, 0, sizeof *s);
    s->key = arena_alloc(4 * pc[o]);
    s->blk = arena_alloc(pc[o]);
    shapes[o] = s;
  }
  if (!shapes[nobjs ? nobjs - 1 : 0] && nobjs) return 0;
  /* fill parts in grid order, then sort by x,y,z */
  for (int i = 0; i < g->ncells; i++) {
    uint32_t k = g->key[i];
    uint32_t x = k & 1023, y = (k >> 10) & 1023, z = k >> 20;
    for (int c = 0; c < blocks[g->blk[i]]->ncomp; c++) {
      Shape *s = shapes[g->nodeobj[g->nodebase[i] + c]];
      s->key[s->np] = x << 20 | y << 13 | z << 3 | c;
      s->blk[s->np] = g->blk[i];
      s->np++;
    }
  }
  for (int o = 0; o < nobjs; o++) {
    Shape *s = shapes[o];
    /* insertion sort (parts arrive mostly sorted by z,y,x; sort to x,y,z) - use shell sort */
    for (int gap = s->np / 2; gap > 0; gap /= 2)
      for (int i = gap; i < s->np; i++) {
        uint32_t k = s->key[i];
        uint8_t b = s->blk[i];
        int j = i;
        while (j >= gap && s->key[j - gap] > k) {
          s->key[j] = s->key[j - gap];
          s->blk[j] = s->blk[j - gap];
          j -= gap;
        }
        s->key[j] = k;
        s->blk[j] = b;
      }
    /* occlusion masks */
    s->occ = arena_alloc(s->np);
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
    /* mass, com, bounds */
    float m = 0, sx = 0, sy = 0, sz = 0;
    float bx0 = 1e9f, by0 = 1e9f, bz0 = 1e9f, bx1 = -1e9f, by1 = -1e9f, bz1 = -1e9f;
    uint8_t coll = 0;
    for (int i = 0; i < s->np; i++) {
      Block *b = blocks[s->blk[i]];
      uint32_t k = s->key[i];
      int c = PK_C(k);
      float cx = PK_X(k) * 8.0f, cy = PK_Y(k) * 8.0f, cz = PK_Z(k) * 8.0f;
      m += b->cnt[c];
      sx += b->sum[c * 3] + cx * b->cnt[c];
      sy += b->sum[c * 3 + 1] + cy * b->cnt[c];
      sz += b->sum[c * 3 + 2] + cz * b->cnt[c];
      const uint8_t *bb = b->bb + c * 6;
      if (cx + bb[0] < bx0) bx0 = cx + bb[0];
      if (cy + bb[1] < by0) by0 = cy + bb[1];
      if (cz + bb[2] < bz0) bz0 = cz + bb[2];
      if (cx + bb[3] + 1 > bx1) bx1 = cx + bb[3] + 1;
      if (cy + bb[4] + 1 > by1) by1 = cy + bb[4] + 1;
      if (cz + bb[5] + 1 > bz1) bz1 = cz + bb[5] + 1;
      if (b->flags & 3) coll = 1;
    }
    s->mass = m / 512.0f;
    s->com = m > 0 ? v3(sx / m / 8, sy / m / 8, sz / m / 8) : v3(0, 0, 0);
    {
      float bd = 1e30f;
      s->origin = s->com;
      for (int i = 0; i < s->np; i++) {
        vec3 c = v3(PK_X(s->key[i]) + 0.5f, PK_Y(s->key[i]) + 0.5f, PK_Z(s->key[i]) + 0.5f);
        vec3 d = vsub(c, s->com);
        float dd = vdot(d, d);
        if (dd < bd - 1e-6f) {
          bd = dd;
          s->origin = c;
        }
      }
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
      s->scen = v3(PK_X(s->key[0]) + (bb[0] + bb[3] + 1) / 16.0f, PK_Y(s->key[0]) + (bb[1] + bb[4] + 1) / 16.0f,
                   PK_Z(s->key[0]) + (bb[2] + bb[5] + 1) / 16.0f);
    }
  }
  return shapes;
}

/* --------------------------------------------------------------- programs */
Obj objs[MAX_OBJ];
int nobj, nlevelobj;
Level level;
static Prog **progs; /* by record */
static Grid level_grid;
static Grid *prog_grid; /* by program: indexed via prog->rec in progs array (parallel) */

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

static Prog *load_prog(int rec, bool pass1);

static Prog *load_prog(int rec, bool pass1) {
  if (progs[rec]) return progs[rec];
  uint8_t *d = load_rec(rec);
  if (!d) return 0;
  Prog *p = arena_alloc(sizeof(Prog));
  memset(p, 0, sizeof *p);
  progs[rec] = p;
  p->rec = rec;
  const uint8_t *q = d;
  p->is_level = q[0];
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
  /* template grid */
  Grid *g = &prog_grid[rec];
  const uint8_t *gstart = q;
  q = decode_grid(q, g, true);
  (void)gstart;
  p->data = q;
  /* keep grid bytes for pass 2 via a pointer stash */
  p->cellkey = (uint32_t *)gstart; /* temporarily: raw grid pointer */
  /* node offsets */
  p->off = arena_alloc(2 * p->nnodes + 2);
  p->slot = arena_alloc(2 * p->nnodes + 2);
  p->child = arena_alloc(sizeof(Prog *) * (p->nnodes ? p->nnodes : 1));
  if (!p->off || !p->slot || !p->child) return 0;
  uint16_t slots = 0;
  const uint8_t *n = q;
  for (int i = 0; i < p->nnodes; i++) {
    p->off[i] = (uint16_t)(n - q);
    int op = n[0];
    p->child[i] = 0;
    p->slot[i] = NONE16;
    if (op == OP_CUSTOM) {
      int crec = rd16(n + 1);
      int nin = n[9];
      n += 10 + 2 * nin;
      p->child[i] = (Prog *)(uintptr_t)(crec + 1); /* resolved after */
      continue;
    }
    const unsigned char *sh = op_shape[op];
    if (sh[3] && sh[1]) {
      p->slot[i] = slots;
      slots += sh[1];
    }
    n += 1 + 2 * sh[0];
    for (int e = 0; e < sh[2]; e++) n += 1 + 2 * n[0];
    n += 1 + 2 * n[0];
    n += op_data_size(op);
  }
  p->nslots = slots;
  for (int i = 0; i < p->nnodes; i++) {
    if (p->child[i]) {
      int crec = (int)(uintptr_t)p->child[i] - 1;
      p->child[i] = load_prog(crec, pass1);
      if (!p->child[i]) return 0;
    }
  }
  return p;
}

int world_obj_at(Prog *p, uint16_t tbase, int cx, int cy, int cz, int vx, int vy, int vz) {
  Grid *g = p ? &prog_grid[p->rec] : &level_grid;
  cx += vx >> 3;
  cy += vy >> 3;
  cz += vz >> 3;
  vx &= 7;
  vy &= 7;
  vz &= 7;
  int i = grid_find(g, (uint32_t)cz << 20 | (uint32_t)cy << 10 | (uint32_t)cx);
  if (i < 0) return -1;
  Block *b = blocks[g->blk[i]];
  int c = blk_solid(b, vx, vy, vz) ? blk_comp(b, vx, vy, vz) : 0;
  return tbase + g->nodeobj[g->nodebase[i] + c];
}

static void obj_init(Obj *o, Shape *s) {
  memset(o, 0, sizeof *o);
  o->shape = s;
  o->pos = s->origin;
  o->rot = qident();
  o->flags = OF_VISIBLE | (s->coll ? OF_COLLIDE : 0);
  for (int i = 0; i < s->np; i++)
    if (blocks[s->blk[i]]->flags & 4) o->flags |= OF_PHYSICS;
  o->friction = 0.5f;
  o->bounce = 0.0f;
  o->mass = s->mass > 0 ? s->mass : 1;
#ifdef HOST
  if (getenv("ND_DEFMASS")) o->mass *= atof(getenv("ND_DEFMASS"));
#endif
  o->lockp = 7;
  o->lockr = 7;
}

int obj_clone(int src) {
  if (nobj >= MAX_OBJ || src < 0) return -1;
  Obj *o = &objs[nobj];
  *o = objs[src];
  o->flags &= ~(OF_TEMPLATE | OF_DEAD | OF_DYNAMIC);
  o->flags |= OF_VISIBLE;
  if (o->shape->coll) o->flags |= OF_COLLIDE;
  o->body = 0;
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

static void finish_grid(Prog *p) {
  /* second pass: decode template grid, form template shapes */
  Grid *g = &prog_grid[p->rec];
  const uint8_t *raw = (const uint8_t *)p->cellkey;
  p->cellkey = 0;
  if (!raw) return;
  Grid tmp = *g;
  if (tmp.ncells) {
    g->key = arena_alloc(4 * tmp.ncells);
    g->blk = arena_alloc(tmp.ncells);
    decode_grid(raw, g, false);
    p->tmpl = form_objects(g);
    p->ntmpl = g->nobj;
  }
}

int level_index;

bool world_load_level(int index) {
  level_index = index;
  arena_reset();
  memset(blocks, 0, sizeof blocks);
  memset(need_blk, 0, sizeof need_blk);
  nobj = 0;
  int np = pack_records();
  progs = arena_alloc(sizeof(Prog *) * np);
  prog_grid = arena_alloc(sizeof(Grid) * np);
  if (!progs || !prog_grid) FAIL("world_load_level");
  memset(progs, 0, sizeof(Prog *) * np);
  memset(prog_grid, 0, sizeof(Grid) * np);
  uint32_t mark = arena_mark();
  uint8_t *d = load_rec(level_rec[index]);
  if (!d) FAIL("world_load_level");
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
  q += 9;
  const uint8_t *graw = q;
  decode_grid(graw, &level_grid, true);
  uint16_t ncells = level_grid.ncells;
  /* copy the raw grid bytes to scratch-safe place: we re-decode after programs are loaded */
  const uint8_t *after = decode_grid(graw, &level_grid, true);
  uint16_t expect_obj = rd16(after);
  uint32_t graw_len = (uint32_t)(after - graw);
  uint8_t *gcopy = arena_alloc(graw_len);
  memcpy(gcopy, graw, graw_len);
  (void)mark;
  level.prog = load_prog(prec, true);
  if (!level.prog) FAIL("world_load_level");
  level.index = index;
  if (!load_blocks()) FAIL("world_load_level");
  /* level objects */
  level_grid.key = arena_alloc(4 * ncells + 4);
  level_grid.blk = arena_alloc(ncells + 1);
  decode_grid(gcopy, &level_grid, false);
  Shape **shapes = form_objects(&level_grid);
  if (!shapes && level_grid.nobj) FAIL("world_load_level");
  if (level_grid.nobj != expect_obj) FAIL("world_load_level");
  for (int i = 0; i < level_grid.nobj; i++) {
    obj_init(&objs[nobj], shapes[i]);
    objs[nobj].src = nobj;
    nobj++;
  }
  nlevelobj = nobj;
  for (int r = 0; r < np; r++)
    if (progs[r]) finish_grid(progs[r]);
  return vm_setup_envs();
}

/* template objects are created by the VM when environments are instantiated */
int world_add_template(Shape *s) {
  if (nobj >= MAX_OBJ) return -1;
  obj_init(&objs[nobj], s);
  objs[nobj].flags &= ~(OF_VISIBLE | OF_COLLIDE);
  objs[nobj].flags |= OF_TEMPLATE;
  objs[nobj].src = nobj;
  return nobj++;
}
