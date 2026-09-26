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
#define ARENA_SIZE (121 * 1024)
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
static uint16_t level_rec[200], level_nm[200];
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

/* Fancade's position of a block in an object is the centre of that block's voxels (a thin plate
   lying at the bottom of its block has its position near the bottom) */
vec3 shape_block_center(const Shape *s, int x, int y, int z) {
  int v0[3] = {8, 8, 8}, v1[3] = {-1, -1, -1};
  for (int i = 0; i < s->np; i++) {
    uint32_t k = s->key[i];
    if (PK_X(k) != x || PK_Y(k) != y || PK_Z(k) != z) continue;
    const uint8_t *bb = blocks[s->blk[i]]->bb + PK_C(k) * 6;
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
    s->origin = shape_block_center(s, (int)s->origin.x, (int)s->origin.y, (int)s->origin.z);
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
static Prog **progs; /* by record */

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


static Prog *load_prog(int rec) {
  if (progs[rec]) return progs[rec];
  const uint8_t *d = load_rec(rec);
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
  /* template objects */
  p->objdata = q;
  q = scan_objects(q);
  p->data = q;
  /* node offsets */
  p->off = arena_alloc(2 * p->nnodes + 2);
  p->slot = arena_alloc(2 * p->nnodes + 2);
  if (!p->off || !p->slot) return 0;
  uint16_t slots = 0;
  const uint8_t *n = q;
  for (int i = 0; i < p->nnodes; i++) {
    p->off[i] = (uint16_t)(n - q);
    int op = n[0];
    p->slot[i] = NONE16;
    if (op == OP_CUSTOM) {
      n += 10 + 2 * n[9]; /* header, inputs */
      n += 3 + 2 * n[2];  /* anchor object, self objects */
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
    const uint8_t *nd = p->data + p->off[i];
    if (nd[0] == OP_CUSTOM && !load_prog(rd16(nd + 1))) return 0;
  }
  return p;
}

Prog *prog_child(const Prog *p, int i) {
  const uint8_t *nd = p->data + p->off[i];
  return nd[0] == OP_CUSTOM ? progs[rd16(nd + 1)] : 0;
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
  memset(blocks, 0, sizeof blocks);
  memset(need_blk, 0, sizeof need_blk);
  nobj = 0;
  int np = pack_records();
  progs = arena_alloc(sizeof(Prog *) * np);
  if (!progs) FAIL("world_load_level");
  memset(progs, 0, sizeof(Prog *) * np);
  /* the level record is only needed while loading: inflate it into free space, read the object
   * capacity, reserve the object table at the top, then keep the record just below it */
  uint32_t avail;
  uint8_t *d = arena_top(&avail);
  uint32_t rn = pack_rawsize(level_rec[index]);
  if (rn > avail || pack_load(level_rec[index], d, rn) != (int)rn) FAIL("world_load_level");
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
  for (int r = 0; r < np; r++) {
    Prog *p = progs[r];
    if (!p) continue;
    p->tmpl = build_shapes(p->objdata, &p->ntmpl);
    if (!p->tmpl) FAIL("world_load_level");
  }
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
