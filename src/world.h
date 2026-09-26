#ifndef WORLD_H
#define WORLD_H
#include <stdint.h>
#include <stdbool.h>
#include "fmath.h"

#define NONE16 0xFFFF

/* out-of-memory reporting on the host harness */
#ifdef HOST
#include <stdio.h>
#define OOM(what) do { static int once; if (!once++) fprintf(stderr, "OOM %s\n", what); } while (0)
#else
#define OOM(what) do { } while (0)
#endif

/* ------------------------------------------------------------------ arena */
void *arena_alloc(uint32_t size);
#if defined(HOST) && defined(ARENA_PROFILE) && !defined(ARENA_IMPL)
void *arena_alloc_dbg(uint32_t size, const char *f, int l);
#define arena_alloc(n) arena_alloc_dbg((n), __FILE__, __LINE__)
#endif
void arena_reset(void);
void *arena_tmp_alloc(uint32_t size); /* setup-only data, freed after the level is instantiated */
void arena_tmp_reset(void);
uint32_t arena_tmp_mark(void);
void arena_tmp_release(uint32_t m);
void *arena_keep_alloc(uint32_t size); /* after setup: allocation kept until the next level */
extern int level_serial;
uint32_t arena_used(void);
uint32_t arena_mark(void);
void arena_release(uint32_t mark);
bool arena_release_top(void *p, uint32_t size); /* frees p if it is the last allocation */
bool arena_extend(void *p, uint32_t old_size, uint32_t new_size); /* grow the last allocation in place */
void *arena_top(uint32_t *avail); /* scratch space at the top of the arena */

/* ------------------------------------------------------------------- pack */
int pack_records(void);
uint32_t pack_rawsize(int rec);
int pack_load(int rec, uint8_t *dst, uint32_t cap);

/* ----------------------------------------------------------------- blocks */
typedef struct {
  uint8_t flags;  /* bits 0-1 collider (0 none, 1 box, 2 sphere), bit 2 physics prefab */
  uint8_t ncomp;
  uint8_t full;   /* 6 bits: face fully covered */
  uint8_t nbox;
  const uint8_t *boxes; /* nbox * 3 */
  const uint8_t *fq[6]; /* quads per face, 3 bytes each */
  uint8_t nq[6];
  /* per component stats (voxel units) */
  uint8_t *bb; /* ncomp * 6: min xyz, max xyz (inclusive) */
} Block;

extern Block *blocks[256]; /* loaded blocks by library index (NULL if unused) */

/* ----------------------------------------------------------------- shapes */
/* A set of voxel-block parts forming one object shape (shared by clones). */
typedef struct {
  uint16_t np;
  uint8_t coll;    /* 1 if any part collides */
  uint8_t sphere;  /* shape is a single sphere */
  uint32_t *key;   /* x<<20 | y<<13 | z<<3 | comp  (cell coords), sorted */
  uint8_t *blk;    /* library index per part */
  uint8_t *occ;    /* per part: faces hidden by a full neighbour face of the same shape */
  float mass;
  float radius;    /* sphere radius */
  vec3 com;        /* rest-space centre of mass (world units) */
  vec3 origin;     /* rest-space reference point (centre of the block nearest the com) */
  vec3 bmin, bmax; /* rest-space bounds (world units) */
} Shape;

#define PK_X(k) ((k) >> 20)
#define PK_Y(k) (((k) >> 13) & 127)
#define PK_Z(k) (((k) >> 3) & 1023)
#define PK_C(k) ((k) & 7)

/* ---------------------------------------------------------------- objects */
enum {
  OF_VISIBLE = 1, OF_COLLIDE = 2, OF_PHYSICS = 4, OF_DYNAMIC = 8, OF_TEMPLATE = 16, OF_DEAD = 32, OF_MOVED = 64
};

typedef struct {
  Shape *shape;
  vec3 pos; /* world position of the shape origin */
  quat rot;
  float mass;
  uint8_t flags;
  uint8_t mat;          /* friction and bounciness (index into mat_tab) */
  uint16_t src;         /* object this was cloned from (or itself) */
  int16_t body;         /* physics body index or -1 */
  uint8_t lockp, lockr; /* allowed axes bits (x=1,y=2,z=4) for position / rotation */
} Obj;

/* the distinct friction / bounciness pairs of the level's objects */
#define NMAT 32
extern float mat_tab[NMAT][2];
extern int nmat;
static inline float obj_friction(const Obj *o) { return mat_tab[o->mat][0]; }
static inline float obj_bounce(const Obj *o) { return mat_tab[o->mat][1]; }
uint8_t mat_find(float friction, float bounce);

extern Obj *objs;
extern int obj_cap;
extern int nobj;
extern int nlevelobj;

/* world transform helpers: rest point -> world */
static inline vec3 obj_world(const Obj *o, vec3 rest) {
  return vadd(o->pos, qrot(o->rot, vsub(rest, o->shape->origin)));
}
static inline vec3 obj_local(const Obj *o, vec3 w) {
  return vadd(o->shape->origin, qrot(qconj(o->rot), vsub(w, o->pos)));
}

/* --------------------------------------------------------------- programs */
typedef struct Prog {
  uint8_t is_level;
  uint8_t yc; /* model centre height of a script block, in 1/16 */
  uint16_t rec;
  uint16_t nnodes;
  const uint16_t *off;  /* node offsets into data */
  const uint8_t *data;  /* node records */
  uint16_t nentries;
  const uint8_t *entries;
  uint16_t nlocals;
  const uint8_t *ltypes;
  uint8_t ninputs, noutputs;
  const uint8_t *outrefs;
  uint8_t nself;
  const uint8_t *selfvox;
  uint16_t ntmpl;       /* template shapes formed from the inner grid */
  Shape **tmpl;
  const uint16_t *slot; /* per node output slot (or NONE16) */
  uint16_t nslots;
  const uint8_t *objdata; /* template objects (see scan_objects) */
} Prog;

Prog *prog_child(const Prog *p, int node); /* child program of a CUSTOM node, else NULL */

typedef struct Env Env;

/* ------------------------------------------------------------------ level */
typedef struct {
  char name[40];
  uint16_t sx, sy, sz;
  uint16_t body_cap, joint_cap; /* physics capacities measured for this level */
  uint8_t bg;
  Prog *prog;
  int index;
} Level;

extern Level level;
extern int level_index;
extern int nlevels;
const char *level_name(int i);

bool world_init(void);
bool world_load_level(int index);


/* global variables */
extern uint16_t nglobals;
extern const uint8_t *global_types;

/* clone / destroy */
int obj_clone(int src);
void obj_destroy(int id);

/* find part index of cell (x,y,z) in shape (first component), or -1 */
int shape_find(const Shape *s, int x, int y, int z);
vec3 shape_block_center(const Shape *s, int x, int y, int z);

#endif
