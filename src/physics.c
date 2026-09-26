/* 2D rigid body physics (motion in the xy plane, rotation about z) for Fancade objects.
 * Collision shapes keep their z extents so only overlapping depth ranges interact. */
#include <string.h>
#include "physics.h"
#include "world.h"

vec3 phys_gravity = {0, -9.8f, 0};

#define DT (1.0f / PHYS_HZ)
#define MAX_ELEMS 128   /* collision elements considered per body */
#define MAX_CONTACTS 1024
#define MAX_EVENTS 64
#define ITER 10

/* tunables (calibrated against the original game) */
float k_mass_scale = 1.0f;
float k_default_friction = 0.5f;
float k_motor_scale = 300.0f; /* the original's motors are effectively traction limited */
float k_lin_motor_scale = 300.0f; /* linear motors too */
float k_linear_damping = 0.0f;
float k_angular_damping = 0.0f;

typedef struct {
  uint8_t circle;
  float cx, cy;   /* centre relative to com, in the body base frame (boxes are axis aligned there) */
  float hx, hy;   /* half extents (box) or radius in hx (circle) */
  float z0, z1;   /* z range relative to the com */
} Elem;

typedef struct Body {
  int obj;
  float invm, invi, mass, inertia;
  float x, y, z;  /* com */
  float a;        /* accumulated rotation about z (radians) */
  quat rot;       /* full orientation (object rotation) */
  float m00, m01, m10, m11, m02, m12; /* xy rows of the rotation matrix */
  float m20, m21, m22;                /* z row */
  float vx, vy, w;
  float pvx, pvy, pw; /* split-impulse pseudo velocities (position correction only) */
  float fx, fy, tq;
  float lockx, locky, lockr; /* 1 free, 0 locked */
  vec3 spin3;     /* extra free rotation (deg/s) about x,y for tumbling */
  uint8_t nel;
  uint8_t zmark;  /* scratch: already shifted in z */
  Elem *el;
  float bound;    /* bounding radius around com */
  float zmin, zmax;
} Body;

typedef struct {
  int a, b;               /* body indices; b may be -1 for static */
  int sobj;               /* static object id when b == -1 */
  float px, py;           /* world contact point */
  float nx, ny;           /* normal from a to b (pointing out of b into a) */
  float depth;
  float mu, rest;
  float jn, jt, jp;
  float mn, mt, bias, pbias;
  float rax, ray, rbx, rby;
} Contact;

typedef struct {
  int a, b;              /* body indices (a = base, -1 = static base object sobj), b = part */
  int sobj;              /* static base object (moves with Set Position), -1 = none */
  vec3 la, lb;           /* anchors in rest frames (relative to com) */
  vec3 axx;              /* base frame x axis in the base rest frame */
  float ref;             /* reference relative angle */
  float lo[3], hi[3];    /* x, y, angle limits (lo > hi = free) */
  float k[3], c[3];      /* springs (k == 0: none) */
  float mv[3], mf[3];    /* motor target velocity and max force */
  float acc[3], accm[3], accl[3]; /* accumulated impulses: spring, motor, limit/lock */
  float spt[3], splo[3], sphi[3];   /* spring row target velocity and impulse range (per step) */
  uint8_t spring[3];
} Joint;

/* per level pools (arena) */
static Body *bodies;
static int nbodies, body_cap;
static int nelems;
static Contact *con; /* scratch space, valid during a step */
static int ncon, con_cap;
static Joint *joints;
static int njoints, joint_cap;
static struct { int16_t a, b; float imp; float nx, ny; } events[MAX_EVENTS];
static int nevents;

/* ---------------------------------------------------------------- raycast */
static bool ray_box(vec3 o, vec3 d, vec3 mn, vec3 mx, float *tin) {
  float t0 = -1e30f, t1 = 1e30f;
  float oo[3] = {o.x, o.y, o.z}, dd[3] = {d.x, d.y, d.z}, a[3] = {mn.x, mn.y, mn.z}, b[3] = {mx.x, mx.y, mx.z};
  for (int k = 0; k < 3; k++) {
    if (fabsf(dd[k]) < 1e-12f) {
      if (oo[k] < a[k] || oo[k] > b[k]) return false;
    } else {
      float inv = 1.0f / dd[k];
      float ta = (a[k] - oo[k]) * inv, tb = (b[k] - oo[k]) * inv;
      if (ta > tb) {
        float t = ta;
        ta = tb;
        tb = t;
      }
      if (ta > t0) t0 = ta;
      if (tb < t1) t1 = tb;
      if (t0 > t1) return false;
    }
  }
  if (t1 < 0 || t0 < 0) return false;
  *tin = t0;
  return true;
}

static bool ray_sphere(vec3 o, vec3 d, vec3 c, float r, float *tin) {
  vec3 m = vsub(o, c);
  float a = vdot(d, d), b = vdot(m, d), cc = vdot(m, m) - r * r;
  if (cc < 0 || b > 0) return false;
  float disc = b * b - a * cc;
  if (disc < 0) return false;
  *tin = (-b - sqrtf(disc)) / a;
  return *tin >= 0;
}

static void part_sphere(const Block *b, int comp, uint32_t k, vec3 *c, float *r) {
  const uint8_t *bb = b->bb + comp * 6;
  float ex = (bb[3] - bb[0] + 1) * 0.5f, ey = (bb[4] - bb[1] + 1) * 0.5f;
  *r = (ex > ey ? ex : ey) / 8.0f;
  *c = v3(PK_X(k) + (bb[0] + bb[3] + 1) / 16.0f, PK_Y(k) + (bb[1] + bb[4] + 1) / 16.0f, PK_Z(k) + (bb[2] + bb[5] + 1) / 16.0f);
}

/* is the rest-space point inside a box collider of the shape? */
static bool shape_solid_at(const Shape *sh, vec3 l) {
  int cx = (int)floorf(l.x), cy = (int)floorf(l.y), cz = (int)floorf(l.z);
  int pj = shape_find(sh, cx, cy, cz);
  for (int j = pj; pj >= 0 && j < sh->np && (sh->key[j] & ~7u) == (sh->key[pj] & ~7u); j++) {
    const Block *nb = blocks[sh->blk[j]];
    if ((nb->flags & 3) != 1) continue;
    const uint8_t *bb = nb->bb + PK_C(sh->key[j]) * 6;
    float vx = (l.x - cx) * 8, vy = (l.y - cy) * 8, vz = (l.z - cz) * 8;
    if (vx >= bb[0] && vx <= bb[3] + 1 && vy >= bb[1] && vy <= bb[4] + 1 && vz >= bb[2] && vz <= bb[5] + 1) return true;
  }
  return false;
}

static bool ray_part(const Shape *s, int pi, vec3 o, vec3 d, float *best) {
  const Block *b = blocks[s->blk[pi]];
  int coll = b->flags & 3;
  if (!coll) return false;
  uint32_t k = s->key[pi];
  float cx = PK_X(k), cy = PK_Y(k), cz = PK_Z(k);
  int comp = PK_C(k);
  bool hit = false;
  float t;
  if (coll == 2) {
    vec3 c;
    float r;
    part_sphere(b, comp, k, &c, &r);
    if (ray_sphere(o, d, c, r, &t) && t < *best) {
      *best = t;
      hit = true;
    }
    return hit;
  }
  /* Fancade box colliders are the bounds of the block's voxels */
  const uint8_t *bb = b->bb + comp * 6;
  if (bb[0] > bb[3]) return false;
  vec3 mn = v3(cx + bb[0] / 8.0f, cy + bb[1] / 8.0f, cz + bb[2] / 8.0f);
  vec3 mx = v3(cx + (bb[3] + 1) / 8.0f, cy + (bb[4] + 1) / 8.0f, cz + (bb[5] + 1) / 8.0f);
  if (ray_box(o, d, mn, mx, &t) && t < *best) {
    /* entering through a face shared with another box of the object (Fancade merges them) */
    float dl = sqrtf(vdot(d, d));
    float e = dl > 1e-9f ? 0.01f / dl : 0;
    if (t > e && shape_solid_at(s, vadd(o, vscale(d, t - e)))) return false;
    *best = t;
    hit = true;
  }
  return hit;
}

static bool ray_shape(const Shape *s, vec3 o, vec3 d, float *best) {
  {
    float t0 = 0, t1 = 1;
    float oo[3] = {o.x, o.y, o.z}, dd[3] = {d.x, d.y, d.z}, a[3] = {s->bmin.x, s->bmin.y, s->bmin.z},
          b[3] = {s->bmax.x, s->bmax.y, s->bmax.z};
    for (int k = 0; k < 3; k++) {
      if (fabsf(dd[k]) < 1e-12f) {
        if (oo[k] < a[k] || oo[k] > b[k]) return false;
      } else {
        float inv = 1.0f / dd[k];
        float ta = (a[k] - oo[k]) * inv, tb = (b[k] - oo[k]) * inv;
        if (ta > tb) {
          float tt = ta;
          ta = tb;
          tb = tt;
        }
        if (ta > t0) t0 = ta;
        if (tb < t1) t1 = tb;
        if (t0 > t1) return false;
      }
    }
  }
  bool hit = false;
  if (s->np <= 24) {
    for (int i = 0; i < s->np; i++)
      if (ray_part(s, i, o, d, best)) hit = true;
    return hit;
  }
  int x = (int)floorf(o.x), y = (int)floorf(o.y), z = (int)floorf(o.z);
  int sx = d.x > 0 ? 1 : -1, sy = d.y > 0 ? 1 : -1, sz = d.z > 0 ? 1 : -1;
  float tdx = fabsf(d.x) > 1e-12f ? fabsf(1.0f / d.x) : 1e30f;
  float tdy = fabsf(d.y) > 1e-12f ? fabsf(1.0f / d.y) : 1e30f;
  float tdz = fabsf(d.z) > 1e-12f ? fabsf(1.0f / d.z) : 1e30f;
  float tmx = tdx < 1e29f ? ((sx > 0 ? x + 1 - o.x : o.x - x) * tdx) : 1e30f;
  float tmy = tdy < 1e29f ? ((sy > 0 ? y + 1 - o.y : o.y - y) * tdy) : 1e30f;
  float tmz = tdz < 1e29f ? ((sz > 0 ? z + 1 - o.z : o.z - z) * tdz) : 1e30f;
  float tcur = 0;
  int bz = tdz > 1e29f && fabsf(o.z - roundf(o.z)) < 1e-5f;
  int by = tdy > 1e29f && fabsf(o.y - roundf(o.y)) < 1e-5f;
  for (int guard = 0; guard < 4096; guard++) {
    if (tcur > *best || tcur > 1.0f) break;
    for (int dz = 0; dz <= bz; dz++)
      for (int dy = 0; dy <= by; dy++) {
        int pi = shape_find(s, x, y - dy, z - dz);
        if (pi < 0) continue;
        for (int j = pi; j < s->np && (s->key[j] & ~7u) == (s->key[pi] & ~7u); j++)
          if (ray_part(s, j, o, d, best)) hit = true;
      }
    if (tmx < tmy && tmx < tmz) {
      tcur = tmx;
      tmx += tdx;
      x += sx;
    } else if (tmy < tmz) {
      tcur = tmy;
      tmy += tdy;
      y += sy;
    } else {
      tcur = tmz;
      tmz += tdz;
      z += sz;
    }
    if (tcur > 1e29f) break;
  }
  return hit;
}

bool phys_raycast_ex(vec3 from, vec3 to, vec3 *hit, int *obj, int ignore) {
  float best = 1.0f;
  int bo = -1;
  for (int i = 0; i < nobj; i++) {
    Obj *ob = &objs[i];
    if (!(ob->flags & OF_COLLIDE) || (ob->flags & OF_DEAD) || i == ignore) continue;
    vec3 o = obj_local(ob, from), t = obj_local(ob, to);
    float b = best;
    if (ray_shape(ob->shape, o, vsub(t, o), &b) && b < best) {
      best = b;
      bo = i;
    }
  }
  if (bo < 0) return false;
  *hit = vadd(from, vscale(vsub(to, from), best));
  *obj = bo;
  return true;
}

bool phys_raycast(vec3 from, vec3 to, vec3 *hit, int *obj) { return phys_raycast_ex(from, to, hit, obj, -1); }

/* ----------------------------------------------------------------- bodies */
static inline float cross2(float ax, float ay, float bx, float by) { return ax * by - ay * bx; }

static void body_matrix(Body *b) {
  quat q = b->rot;
  float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z, xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
  float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
  b->m00 = 1 - 2 * (yy + zz);
  b->m01 = 2 * (xy - wz);
  b->m02 = 2 * (xz + wy);
  b->m10 = 2 * (xy + wz);
  b->m11 = 1 - 2 * (xx + zz);
  b->m12 = 2 * (yz - wx);
  b->m20 = 2 * (xz - wy);
  b->m21 = 2 * (yz + wx);
  b->m22 = 1 - 2 * (xx + yy);
}

static void sync_obj(Body *b) {
  Obj *o = &objs[b->obj];
  o->rot = b->rot;
  vec3 comw = v3(b->x, b->y, b->z);
  o->pos = vsub(comw, qrot(o->rot, vsub(o->shape->com, o->shape->origin)));
}

static void sync_body(Body *b) {
  Obj *o = &objs[b->obj];
  b->rot = o->rot;
  body_matrix(b);
  vec3 c = obj_world(o, o->shape->com);
  b->x = c.x;
  b->y = c.y;
  b->z = c.z;
}

/* add collision elements for a shape, expressed in the body base frame relative to the com */
static bool build_elems(Body *b) {
  Obj *o = &objs[b->obj];
  const Shape *s = o->shape;
  int cap = 0;
  for (int i = 0; i < s->np; i++) {
    const Block *bk = blocks[s->blk[i]];
    if (bk->flags & 3) cap++;
  }
  if (cap > 255) cap = 255;
  uint32_t mark = arena_mark();
  b->el = arena_alloc(sizeof(Elem) * (cap ? cap : 1));
  if (!b->el) return false;
  b->nel = 0;
  for (int i = 0; i < s->np && b->nel < cap; i++) {
    const Block *bk = blocks[s->blk[i]];
    int coll = bk->flags & 3;
    if (!coll) continue;
    uint32_t k = s->key[i];
    int comp = PK_C(k);
    if (coll == 2) {
      vec3 c;
      float rad;
      part_sphere(bk, comp, k, &c, &rad);
      Elem *e = &b->el[b->nel++];
      e->circle = 1;
      e->cx = c.x - s->com.x;
      e->cy = c.y - s->com.y;
      e->hx = rad;
      e->hy = rad;
      e->z0 = c.z - s->com.z - rad;
      e->z1 = c.z - s->com.z + rad;
      continue;
    }
    const uint8_t *bb = bk->bb + comp * 6;
    if (bb[0] > bb[3]) continue;
    float mnx = PK_X(k) + bb[0] / 8.0f, mny = PK_Y(k) + bb[1] / 8.0f, mnz = PK_Z(k) + bb[2] / 8.0f;
    float mxx = PK_X(k) + (bb[3] + 1) / 8.0f, mxy = PK_Y(k) + (bb[4] + 1) / 8.0f, mxz = PK_Z(k) + (bb[5] + 1) / 8.0f;
    Elem *e = &b->el[b->nel++];
    e->circle = 0;
    e->cx = (mnx + mxx) * 0.5f - s->com.x;
    e->cy = (mny + mxy) * 0.5f - s->com.y;
    e->hx = (mxx - mnx) * 0.5f;
    e->hy = (mxy - mny) * 0.5f;
    e->z0 = mnz - s->com.z;
    e->z1 = mxz - s->com.z;
  }
  /* merge boxes sharing an edge */
  for (int i = 0; i < b->nel; i++) {
    Elem *e = &b->el[i];
    if (e->circle) continue;
    for (int j = i + 1; j < b->nel; j++) {
      Elem *f = &b->el[j];
      if (f->circle || fabsf(e->z0 - f->z0) > 1e-4f || fabsf(e->z1 - f->z1) > 1e-4f) continue;
      int merged = 0;
      if (fabsf(e->cx - f->cx) < 1e-4f && fabsf(e->hx - f->hx) < 1e-4f && fabsf(fabsf(e->cy - f->cy) - (e->hy + f->hy)) < 1e-4f) {
        float y0 = fminf(e->cy - e->hy, f->cy - f->hy), y1 = fmaxf(e->cy + e->hy, f->cy + f->hy);
        e->cy = (y0 + y1) * 0.5f;
        e->hy = (y1 - y0) * 0.5f;
        merged = 1;
      } else if (fabsf(e->cy - f->cy) < 1e-4f && fabsf(e->hy - f->hy) < 1e-4f && fabsf(fabsf(e->cx - f->cx) - (e->hx + f->hx)) < 1e-4f) {
        float x0 = fminf(e->cx - e->hx, f->cx - f->hx), x1 = fmaxf(e->cx + e->hx, f->cx + f->hx);
        e->cx = (x0 + x1) * 0.5f;
        e->hx = (x1 - x0) * 0.5f;
        merged = 1;
      }
      if (merged) {
        *f = b->el[b->nel - 1];
        b->nel--;
        j = i;
      }
    }
  }
  arena_release(mark);
  arena_alloc(sizeof(Elem) * (b->nel ? b->nel : 1)); /* keep only the merged elements */
  nelems += b->nel;
  b->bound = 0;
  for (int i = 0; i < b->nel; i++) {
    Elem *e = &b->el[i];
    float cz = (e->z0 + e->z1) * 0.5f, hz = (e->z1 - e->z0) * 0.5f;
    float r = sqrtf(e->cx * e->cx + e->cy * e->cy + cz * cz) + (e->circle ? e->hx : sqrtf(e->hx * e->hx + e->hy * e->hy + hz * hz));
    if (r > b->bound) b->bound = r;
  }
  return true;
}

static void body_zrange(Body *b) {
  b->zmin = 1e9f;
  b->zmax = -1e9f;
  for (int i = 0; i < b->nel; i++) {
    const Elem *e = &b->el[i];
    float cz = (e->z0 + e->z1) * 0.5f, hz = (e->z1 - e->z0) * 0.5f;
    float zc = b->z + b->m20 * e->cx + b->m21 * e->cy + b->m22 * cz;
    float ez = e->circle ? hz : fabsf(b->m20) * e->hx + fabsf(b->m21) * e->hy + fabsf(b->m22) * hz;
    if (zc - ez < b->zmin) b->zmin = zc - ez;
    if (zc + ez > b->zmax) b->zmax = zc + ez;
  }
}

static void body_mass(Body *b) {
  Obj *o = &objs[b->obj];
  const Shape *s = o->shape;
  b->mass = o->mass * k_mass_scale;
  if (b->mass <= 0) b->mass = 0.01f;
  /* inertia about z from voxel distribution, scaled to the mass */
  float I = 0, m = 0;
  for (int i = 0; i < s->np; i++) {
    const Block *bk = blocks[s->blk[i]];
    int comp = PK_C(s->key[i]);
    float cx = PK_X(s->key[i]) - s->com.x, cy = PK_Y(s->key[i]) - s->com.y, cz = PK_Z(s->key[i]) - s->com.z;
    for (int z = 0; z < 8; z++)
      for (int y = 0; y < 8; y++)
        for (int x = 0; x < 8; x++) {
          if (!blk_solid(bk, x, y, z) || blk_comp(bk, x, y, z) != comp) continue;
          /* about the world z axis: the body may be turned about x or y */
          float lx = cx + (x + 0.5f) / 8, ly = cy + (y + 0.5f) / 8, lz = cz + (z + 0.5f) / 8;
          float dx = b->m00 * lx + b->m01 * ly + b->m02 * lz, dy = b->m10 * lx + b->m11 * ly + b->m12 * lz;
          I += dx * dx + dy * dy + 1.0f / 384.0f;
          m += 1;
        }
  }
  if (m <= 0) {
    I = 1;
    m = 1;
  }
  b->inertia = I / m * b->mass;
  if (s->sphere) b->inertia = 0.4f * b->mass * s->radius * s->radius;
  if (b->inertia < 1e-4f) b->inertia = 1e-4f;
  b->invm = 1.0f / b->mass;
  b->invi = 1.0f / b->inertia;
}

void phys_make_dynamic(int o) {
  if (o < 0 || o >= nobj) return;
  Obj *ob = &objs[o];
  if ((ob->flags & OF_DYNAMIC) || (ob->flags & OF_DEAD)) return;
  if (nbodies >= body_cap) {
    OOM("bodies");
    return;
  }
  Body *b = &bodies[nbodies];
  memset(b, 0, sizeof *b);
  b->obj = o;
  sync_body(b);
  b->lockx = (ob->lockp & 1) ? 1.0f : 0.0f;
  b->locky = (ob->lockp & 2) ? 1.0f : 0.0f;
  b->lockr = (ob->lockr & 4) ? 1.0f : 0.0f;
  body_mass(b);
  if (!build_elems(b)) return;
  body_zrange(b);
  ob->flags |= OF_DYNAMIC;
  ob->body = (int16_t)nbodies++;
  /* joints made while this object was static now pull on its body */
  for (int ji = 0; ji < njoints; ji++) {
    Joint *j = &joints[ji];
    if (j->a >= 0 || j->sobj != o) continue;
    vec3 w = obj_world(ob, j->la), ax = qrot(ob->rot, j->axx);
    float a0 = atan2f(ax.y, ax.x);
    j->a = ob->body;
    j->sobj = -1;
    j->la = qrot(qconj(b->rot), vsub(w, v3(b->x, b->y, b->z)));
    j->axx = qrot(qconj(b->rot), ax);
    j->ref += a0 - b->a;
  }
}

static Body *body(int o) {
  if (o < 0 || o >= nobj || !(objs[o].flags & OF_DYNAMIC)) return 0;
  int bi = objs[o].body;
  if (bi < 0 || bi >= nbodies || bodies[bi].obj != o) return 0;
  return &bodies[bi];
}

#ifdef HOST
#include <stdio.h>
#include <stdlib.h>
static float k_fmul = 1.0f;
static void host_tunables(void) {
  if (getenv("ND_KMOTOR")) k_motor_scale = atof(getenv("ND_KMOTOR"));
  if (getenv("ND_KLMOTOR")) k_lin_motor_scale = atof(getenv("ND_KLMOTOR"));
  if (getenv("ND_KMASS")) k_mass_scale = atof(getenv("ND_KMASS"));
  if (getenv("ND_KFRIC")) k_default_friction = atof(getenv("ND_KFRIC"));
  if (getenv("ND_FMUL")) k_fmul = atof(getenv("ND_FMUL"));
}
#endif

void phys_reset(void) {
#ifdef HOST
  host_tunables();
#endif
  phys_gravity = v3(0, -9.8f, 0);
  nbodies = body_cap = 0;
  nelems = 0;
  njoints = joint_cap = 0;
  ncon = 0;
  nevents = 0;
}

void phys_start(void) {
  /* pools sized from the capacities measured for this level */
  int nphys = 0;
  for (int i = 0; i < nobj; i++)
    if (objs[i].flags & OF_PHYSICS) nphys++;
  body_cap = level.body_cap > nphys ? level.body_cap : nphys;
  joint_cap = level.joint_cap;
  bodies = arena_alloc(sizeof(Body) * (body_cap ? body_cap : 1));
  joints = arena_alloc(sizeof(Joint) * (joint_cap ? joint_cap : 1));
  if (!bodies || !joints) body_cap = joint_cap = 0;
  for (int i = 0; i < nobj; i++) {
    Obj *o = &objs[i];
    if ((o->flags & OF_PHYSICS) && !(o->flags & (OF_TEMPLATE | OF_DEAD))) phys_make_dynamic(i);
  }
}

void phys_get_velocity(int o, vec3 *vel, vec3 *spin) {
  Body *b = body(o);
  if (!b) {
    *vel = v3(0, 0, 0);
    *spin = v3(0, 0, 0);
    return;
  }
  *vel = v3(b->vx, b->vy, 0);
  *spin = v3(b->spin3.x, b->spin3.y, b->w * RAD2DEG);
}

/* like Fancade, physics blocks turn a plain object into a physics object */
static Body *wake(int o) {
  if (o >= 0 && o < nobj && !(objs[o].flags & (OF_DYNAMIC | OF_TEMPLATE | OF_DEAD))) {
#ifdef HOST
    if (getenv("ND_WAKE")) fprintf(stderr, "wake obj%d np=%d\n", o, objs[o].shape ? objs[o].shape->np : -1);
#endif
    phys_make_dynamic(o);
  }
  return body(o);
}

void phys_set_velocity(int o, const vec3 *vel, const vec3 *spin) {
  Body *b = wake(o);
  if (!b) return;
  if (vel) {
    b->vx = vel->x;
    b->vy = vel->y;
  }
  if (spin) {
    b->w = spin->z * DEG2RAD;
    b->spin3.x = spin->x;
    b->spin3.y = spin->y;
  }
}

void phys_add_force(int o, const vec3 *f, const vec3 *at, const vec3 *t) {
  Body *b = wake(o);
  if (!b) return;
  if (f) {
    b->fx += f->x;
    b->fy += f->y;
    if (at) b->tq += cross2(at->x - b->x, at->y - b->y, f->x, f->y);
  }
  if (t) b->tq += t->z * DEG2RAD;
}

void phys_set_locked(int o, const vec3 *p, const vec3 *r) {
  Obj *ob = &objs[o];
  if (p) ob->lockp = (p->x != 0 ? 1 : 0) | (p->y != 0 ? 2 : 0) | (p->z != 0 ? 4 : 0);
  if (r) ob->lockr = (r->x != 0 ? 1 : 0) | (r->y != 0 ? 2 : 0) | (r->z != 0 ? 4 : 0);
  Body *b = wake(o);
  if (!b) return;
  b->lockx = (ob->lockp & 1) ? 1.0f : 0.0f;
  b->locky = (ob->lockp & 2) ? 1.0f : 0.0f;
  b->lockr = (ob->lockr & 4) ? 1.0f : 0.0f;
  if (!b->lockx) b->vx = 0;
  if (!b->locky) b->vy = 0;
  if (!b->lockr) b->w = 0;
}

void phys_set_mass(int o, float m) {
  objs[o].mass = m;
  Body *b = wake(o);
  if (b) body_mass(b);
}
void phys_set_friction(int o, float f) { objs[o].friction = f; }
void phys_set_bounce(int o, float v) { objs[o].bounce = v; }
void phys_set_gravity(vec3 g) { phys_gravity = g; }

void phys_moved(int o) {
  Body *b = body(o);
  if (!b) return;
  float m02 = b->m02, m12 = b->m12, m20 = b->m20, m21 = b->m21, z0 = b->z;
  sync_body(b);
  /* the joints hold depth: bodies jointed to a body moved in z go with it */
  float dz = b->z - z0;
  if (fabsf(dz) > 1e-4f) {
    for (int i = 0; i < nbodies; i++) bodies[i].zmark = 0;
    b->zmark = 1;
    for (bool more = true; more;) {
      more = false;
      for (int ji = 0; ji < njoints; ji++) {
        Joint *j = &joints[ji];
        if (j->a < 0 || j->b < 0 || bodies[j->a].zmark == bodies[j->b].zmark) continue;
        Body *m = bodies[j->a].zmark ? &bodies[j->b] : &bodies[j->a];
        if (m->obj < 0) continue;
        m->zmark = 1;
        m->z += dz;
        sync_obj(m);
        body_zrange(m);
        more = true;
      }
    }
    body_zrange(b);
  }
  /* turned about x or y: the inertia about the plane normal changes */
  if (fabsf(m02 - b->m02) + fabsf(m12 - b->m12) + fabsf(m20 - b->m20) + fabsf(m21 - b->m21) > 1e-3f) {
    body_mass(b);
    body_zrange(b);
  }
}

/* hiding an object takes it out of the physics world, which drops its constraints */
void phys_hidden(int o) {
  Body *b = body(o);
  int bi = b ? (int)(b - bodies) : -2;
  for (int j = 0; j < njoints; j++)
    if (joints[j].a == bi || joints[j].b == bi || (joints[j].a < 0 && joints[j].sobj == o)) joints[j].b = -2;
}

void phys_destroyed(int o) {
  Body *b = body(o);
  if (!b) return;
  int bi = (int)(b - bodies);
  /* detach joints */
  for (int j = 0; j < njoints; j++)
    if (joints[j].a == bi || joints[j].b == bi) joints[j].b = -2;
  b->obj = -1;
  objs[o].body = -1;
}

/* ----------------------------------------------------------------- joints */
int phys_add_constraint(int base, int part, vec3 pivot) {
  if (njoints >= joint_cap) {
    OOM("joints");
    return -1;
  }
  /* like Fancade, a constraint needs a base object */
  if (base < 0 || base >= nobj || part < 0 || part >= nobj || base == part) return -1;
  phys_make_dynamic(part);
  Body *pb = body(part);
  if (!pb) return -1;
  Body *bb = body(base);
  Joint *j = &joints[njoints];
  memset(j, 0, sizeof *j);
  j->b = (int)(pb - bodies);
  j->a = bb ? (int)(bb - bodies) : -1;
  j->sobj = bb ? -1 : base;
  if (bb) {
    j->la = qrot(qconj(bb->rot), vsub(pivot, v3(bb->x, bb->y, bb->z)));
    j->axx = qrot(qconj(bb->rot), v3(1, 0, 0));
    j->ref = pb->a - bb->a;
  } else {
    /* static base: anchor kept in its rest frame so it follows the object when scripts move it */
    Obj *so = &objs[base];
    j->la = obj_local(so, pivot);
    j->axx = qrot(qconj(so->rot), v3(1, 0, 0));
    vec3 ax = qrot(so->rot, j->axx);
    j->ref = pb->a - atan2f(ax.y, ax.x);
  }
  j->lb = qrot(qconj(pb->rot), vsub(pivot, v3(pb->x, pb->y, pb->z)));
  return njoints++;
}

void phys_con_limits(int c, bool ang, vec3 lo, vec3 hi) {
  if (c < 0 || c >= njoints) return;
  Joint *j = &joints[c];
  if (ang) {
    j->lo[2] = lo.z * DEG2RAD;
    j->hi[2] = hi.z * DEG2RAD;
  } else {
    j->lo[0] = lo.x;
    j->hi[0] = hi.x;
    j->lo[1] = lo.y;
    j->hi[1] = hi.y;
  }
}

void phys_con_spring(int c, bool ang, vec3 k, vec3 d) {
  if (c < 0 || c >= njoints) return;
  Joint *j = &joints[c];
  if (ang) {
    j->k[2] = k.z;
    j->c[2] = d.z;
    j->spring[2] = k.z > 0 || d.z > 0;
  } else {
    j->k[0] = k.x;
    j->c[0] = d.x;
    j->spring[0] = k.x > 0 || d.x > 0;
    j->k[1] = k.y;
    j->c[1] = d.y;
    j->spring[1] = k.y > 0 || d.y > 0;
  }
}

void phys_con_motor(int c, bool ang, vec3 v, vec3 f) {
  if (c < 0 || c >= njoints) return;
  Joint *j = &joints[c];
  if (ang) {
    j->mv[2] = v.z * DEG2RAD;
    j->mf[2] = fabsf(f.z) * k_motor_scale;
  } else {
    j->mv[0] = v.x;
    j->mf[0] = fabsf(f.x) * k_lin_motor_scale;
    j->mv[1] = v.y;
    j->mf[1] = fabsf(f.y) * k_lin_motor_scale;
  }
}

/* --------------------------------------------------------------- contacts */
static int cur_a, cur_b, cur_sobj;
static float cur_mu, cur_rest;

static void add_contact(float px, float py, float nx, float ny, float depth) {
  if (ncon >= con_cap) {
    OOM("contacts");
    return;
  }
  Contact *c = &con[ncon++];
  memset(c, 0, sizeof *c);
  c->a = cur_a;
  c->b = cur_b;
  c->sobj = cur_sobj;
  c->px = px;
  c->py = py;
  c->nx = nx;
  c->ny = ny;
  c->depth = depth;
  c->mu = cur_mu;
  c->rest = cur_rest;
}

/* world-space 2D element */
typedef struct {
  uint8_t circle;
  float cx, cy, ux, uy, hx, hy, z0, z1;
} WElem;

static void elem_world(const Body *b, const Elem *e, WElem *w) {
  float cz = (e->z0 + e->z1) * 0.5f, hz = (e->z1 - e->z0) * 0.5f;
  w->circle = e->circle;
  w->cx = b->x + b->m00 * e->cx + b->m01 * e->cy + b->m02 * cz;
  w->cy = b->y + b->m10 * e->cx + b->m11 * e->cy + b->m12 * cz;
  float zc = b->z + b->m20 * e->cx + b->m21 * e->cy + b->m22 * cz;
  if (e->circle) {
    w->ux = 1;
    w->uy = 0;
    w->hx = w->hy = e->hx;
    w->z0 = zc - hz;
    w->z1 = zc + hz;
    return;
  }
  /* the box turned by any 3D rotation, as a rectangle in the plane: oriented along its x axis (or its
     y axis turned a quarter when x points along z), extents of all three axes projected */
  float ux = b->m00, uy = b->m10;
  float l = sqrtf(ux * ux + uy * uy);
  if (l < 0.5f) {
    ux = b->m11;
    uy = -b->m01;
    l = sqrtf(ux * ux + uy * uy);
  }
  if (l < 1e-4f) {
    ux = 1;
    uy = 0;
  } else {
    ux /= l;
    uy /= l;
  }
  w->ux = ux;
  w->uy = uy;
  w->hx = fabsf(b->m00 * ux + b->m10 * uy) * e->hx + fabsf(b->m01 * ux + b->m11 * uy) * e->hy + fabsf(b->m02 * ux + b->m12 * uy) * hz;
  w->hy = fabsf(b->m10 * ux - b->m00 * uy) * e->hx + fabsf(b->m11 * ux - b->m01 * uy) * e->hy + fabsf(b->m12 * ux - b->m02 * uy) * hz;
  float ez = fabsf(b->m20) * e->hx + fabsf(b->m21) * e->hy + fabsf(b->m22) * hz;
  w->z0 = zc - ez;
  w->z1 = zc + ez;
}

/* contacts between circle A and circle B, normal points from B to A */
static void col_cc(const WElem *a, const WElem *b) {
  float dx = a->cx - b->cx, dy = a->cy - b->cy, r = a->hx + b->hx;
  float d2 = dx * dx + dy * dy;
  if (d2 >= r * r) return;
  float d = sqrtf(d2);
  float nx = d > 1e-6f ? dx / d : 0, ny = d > 1e-6f ? dy / d : 1;
  add_contact(b->cx + nx * b->hx, b->cy + ny * b->hx, nx, ny, r - d);
}

/* circle A vs box B, normal from B to A */
static void col_cb(const WElem *a, const WElem *b, bool flip) {
  float dx = a->cx - b->cx, dy = a->cy - b->cy;
  float vx = b->uy * -1, vy = b->ux; /* box y axis */
  float lx = dx * b->ux + dy * b->uy, ly = dx * vx + dy * vy;
  float qx = clampf(lx, -b->hx, b->hx), qy = clampf(ly, -b->hy, b->hy);
  float nx, ny, depth;
  if (lx == qx && ly == qy) {
    /* centre inside: push out along smallest penetration */
    float px = b->hx - fabsf(lx), py = b->hy - fabsf(ly);
    if (px < py) {
      nx = lx > 0 ? 1 : -1;
      ny = 0;
      depth = px + a->hx;
      qx = lx > 0 ? b->hx : -b->hx;
    } else {
      nx = 0;
      ny = ly > 0 ? 1 : -1;
      depth = py + a->hx;
      qy = ly > 0 ? b->hy : -b->hy;
    }
  } else {
    float ex = lx - qx, ey = ly - qy;
    float d2 = ex * ex + ey * ey;
    if (d2 >= a->hx * a->hx) return;
    float d = sqrtf(d2);
    nx = ex / d;
    ny = ey / d;
    depth = a->hx - d;
  }
  float wx = nx * b->ux + ny * vx, wy = nx * b->uy + ny * vy;
  float px = b->cx + qx * b->ux + qy * vx, py = b->cy + qx * b->uy + qy * vy;
  if (flip) add_contact(px - wx * depth, py - wy * depth, -wx, -wy, depth);
  else add_contact(px, py, wx, wy, depth);
}

/* box A vs box B (SAT + clipping, Box2D-lite style). normal from B to A */
static void col_bb(const WElem *A, const WElem *B) {
  float ax[2][2] = {{A->ux, A->uy}, {-A->uy, A->ux}};
  float bx[2][2] = {{B->ux, B->uy}, {-B->uy, B->ux}};
  float ha[2] = {A->hx, A->hy}, hb[2] = {B->hx, B->hy};
  float dx = A->cx - B->cx, dy = A->cy - B->cy;
  float best = 1e30f;
  int bi = -1;
  float bnx = 0, bny = 0;
  for (int k = 0; k < 4; k++) {
    float nx = k < 2 ? ax[k][0] : bx[k - 2][0], ny = k < 2 ? ax[k][1] : bx[k - 2][1];
    float ra = ha[0] * fabsf(ax[0][0] * nx + ax[0][1] * ny) + ha[1] * fabsf(ax[1][0] * nx + ax[1][1] * ny);
    float rb = hb[0] * fabsf(bx[0][0] * nx + bx[0][1] * ny) + hb[1] * fabsf(bx[1][0] * nx + bx[1][1] * ny);
    float d = dx * nx + dy * ny;
    float sep = fabsf(d) - (ra + rb);
    if (sep > 0) return;
    /* prefer B's axes slightly to stabilise resting on static boxes */
    float score = -sep * (k >= 2 ? 0.98f : 1.0f);
    if (score < best) {
      best = score;
      bi = k;
      bnx = d < 0 ? -nx : nx;
      bny = d < 0 ? -ny : ny;
    }
  }
  /* reference box: the one owning the axis. incident: the other */
  const WElem *ref = bi >= 2 ? B : A, *inc = bi >= 2 ? A : B;
  float nx = bi >= 2 ? bnx : -bnx, ny = bi >= 2 ? bny : -bny; /* normal pointing from ref to inc */
  /* incident edge: the edge of inc most anti-parallel to n */
  float ix[2][2] = {{inc->ux, inc->uy}, {-inc->uy, inc->ux}};
  float hi[2] = {inc->hx, inc->hy};
  int ie = 0;
  float mind = 1e30f;
  for (int k = 0; k < 4; k++) {
    float ex = (k & 1 ? -1 : 1) * ix[k >> 1][0], ey = (k & 1 ? -1 : 1) * ix[k >> 1][1];
    float d = ex * nx + ey * ny;
    if (d < mind) {
      mind = d;
      ie = k;
    }
  }
  float fnx = (ie & 1 ? -1 : 1) * ix[ie >> 1][0], fny = (ie & 1 ? -1 : 1) * ix[ie >> 1][1];
  float tnx = -fny, tny = fnx; /* along the face */
  float fh = hi[ie >> 1], th = hi[1 - (ie >> 1)];
  float c0x = inc->cx + fnx * fh + tnx * th, c0y = inc->cy + fny * fh + tny * th;
  float c1x = inc->cx + fnx * fh - tnx * th, c1y = inc->cy + fny * fh - tny * th;
  /* reference face */
  float rx[2][2] = {{ref->ux, ref->uy}, {-ref->uy, ref->ux}};
  float rh[2] = {ref->hx, ref->hy};
  int ra = fabsf(rx[0][0] * nx + rx[0][1] * ny) > fabsf(rx[1][0] * nx + rx[1][1] * ny) ? 0 : 1;
  float sideX = rx[1 - ra][0], sideY = rx[1 - ra][1], sideH = rh[1 - ra];
  float faceD = (ref->cx * nx + ref->cy * ny) + rh[ra];
  float cs = ref->cx * sideX + ref->cy * sideY;
  /* clip against side planes */
  float px[2] = {c0x, c1x}, py[2] = {c0y, c1y};
  for (int side = 0; side < 2; side++) {
    float sx = side ? -sideX : sideX, sy = side ? -sideY : sideY;
    float off = (side ? -cs : cs) + sideH;
    float d0 = px[0] * sx + py[0] * sy - off, d1 = px[1] * sx + py[1] * sy - off;
    if (d0 > 0 && d1 > 0) return;
    if (d0 > 0 || d1 > 0) {
      float t = d0 / (d0 - d1);
      float qx = px[0] + (px[1] - px[0]) * t, qy = py[0] + (py[1] - py[0]) * t;
      if (d0 > 0) {
        px[0] = qx;
        py[0] = qy;
      } else {
        px[1] = qx;
        py[1] = qy;
      }
    }
  }
  for (int k = 0; k < 2; k++) {
    float sep = px[k] * nx + py[k] * ny - faceD;
    if (sep <= 0.001f) {
      /* contact point on inc; normal convention: from B to A */
      if (ref == B) add_contact(px[k], py[k], nx, ny, -sep);
      else add_contact(px[k] - nx * sep, py[k] - ny * sep, -nx, -ny, -sep);
    }
  }
}

static void collide_elems(const WElem *a, const WElem *b) {
  if (a->z1 <= b->z0 + 1e-4f || b->z1 <= a->z0 + 1e-4f) return;
  if (a->circle && b->circle) col_cc(a, b);
  else if (a->circle) col_cb(a, b, false);
  else if (b->circle) col_cb(b, a, true);
  else col_bb(a, b);
}

/* a box with half axes a, b, e (any 3D orientation) as a rectangle in the plane: its outline along
   the axis that projects longest, and its z range */
static void box_welem(vec3 c, vec3 a, vec3 b, vec3 e, WElem *w) {
  float la = a.x * a.x + a.y * a.y, lb = b.x * b.x + b.y * b.y, le = e.x * e.x + e.y * e.y;
  float ux = a.x, uy = a.y, l = la;
  if (lb > l) ux = b.x, uy = b.y, l = lb;
  if (le > l) ux = e.x, uy = e.y, l = le;
  l = sqrtf(l);
  if (l < 1e-6f) ux = 1, uy = 0;
  else ux /= l, uy /= l;
  w->circle = 0;
  w->cx = c.x;
  w->cy = c.y;
  w->ux = ux;
  w->uy = uy;
  w->hx = fabsf(a.x * ux + a.y * uy) + fabsf(b.x * ux + b.y * uy) + fabsf(e.x * ux + e.y * uy);
  w->hy = fabsf(a.y * ux - a.x * uy) + fabsf(b.y * ux - b.x * uy) + fabsf(e.y * ux - e.x * uy);
  float hz = fabsf(a.z) + fabsf(b.z) + fabsf(e.z);
  w->z0 = c.z - hz;
  w->z1 = c.z + hz;
}

/* dynamic element vs a static object turned about x or y: its parts' outlines in world space */
static void collide_static_3d(const WElem *we, int so) {
  Obj *s = &objs[so];
  const Shape *sh = s->shape;
  vec3 ax = qrot(s->rot, v3(1, 0, 0)), ay = qrot(s->rot, v3(0, 1, 0)), az = qrot(s->rot, v3(0, 0, 1));
  float ext = we->circle ? we->hx : sqrtf(we->hx * we->hx + we->hy * we->hy);
  for (int i = 0; i < sh->np; i++) {
    const Block *bk = blocks[sh->blk[i]];
    int coll = bk->flags & 3;
    if (!coll) continue;
    uint32_t k = sh->key[i];
    int comp = PK_C(k);
    WElem se;
    if (coll == 2) {
      vec3 c;
      float r;
      part_sphere(bk, comp, k, &c, &r);
      c = obj_world(s, c);
      if (fabsf(c.x - we->cx) > r + ext || fabsf(c.y - we->cy) > r + ext) continue;
      se.circle = 1;
      se.cx = c.x;
      se.cy = c.y;
      se.hx = se.hy = r;
      se.ux = 1;
      se.uy = 0;
      se.z0 = c.z - r;
      se.z1 = c.z + r;
    } else {
      const uint8_t *bb = bk->bb + comp * 6;
      if (bb[0] > bb[3]) continue;
      float hx = (bb[3] + 1 - bb[0]) / 16.0f, hy = (bb[4] + 1 - bb[1]) / 16.0f, hz = (bb[5] + 1 - bb[2]) / 16.0f;
      vec3 lc = v3(PK_X(k) + (bb[0] + bb[3] + 1) / 16.0f, PK_Y(k) + (bb[1] + bb[4] + 1) / 16.0f, PK_Z(k) + (bb[2] + bb[5] + 1) / 16.0f);
      vec3 c = obj_world(s, lc);
      float r = hx + hy + hz;
      if (fabsf(c.x - we->cx) > r + ext || fabsf(c.y - we->cy) > r + ext) continue;
      box_welem(c, vscale(ax, hx), vscale(ay, hy), vscale(az, hz), &se);
    }
    int before = ncon;
    collide_elems(we, &se);
    /* faces against another part of the object are internal */
    float zm = (fmaxf(we->z0, se.z0) + fminf(we->z1, se.z1)) * 0.5f;
    for (int c = before; c < ncon; c++) {
      vec3 q = obj_local(s, v3(con[c].px + con[c].nx * 0.02f, con[c].py + con[c].ny * 0.02f, zm));
      if (!shape_solid_at(sh, q)) continue;
      con[c] = con[ncon - 1];
      ncon--;
      c--;
    }
  }
}

/* dynamic element vs static object parts */
static void collide_static(Body *bd, const WElem *we, int so) {
  Obj *s = &objs[so];
  const Shape *sh = s->shape;
  /* transform element into the static object's rest frame */
  quat inv = qconj(s->rot);
  vec3 cl = obj_local(s, v3(we->cx, we->cy, (we->z0 + we->z1) * 0.5f));
  vec3 ul = qrot(inv, v3(we->ux, we->uy, 0));
  float hz = (we->z1 - we->z0) * 0.5f;
  /* in-plane check: static rotation must keep the xy plane */
  vec3 zl = qrot(inv, v3(0, 0, 1));
  if (fabsf(zl.z) < 0.99f) {
    collide_static_3d(we, so);
    return;
  }
  WElem le = *we;
  le.cx = cl.x;
  le.cy = cl.y;
  float l = sqrtf(ul.x * ul.x + ul.y * ul.y);
  le.ux = ul.x / l;
  le.uy = ul.y / l;
  le.z0 = cl.z - hz;
  le.z1 = cl.z + hz;
  float ext = we->circle ? we->hx : fabsf(le.ux) * we->hx + fabsf(le.uy) * we->hy;
  float exty = we->circle ? we->hx : fabsf(le.uy) * we->hx + fabsf(le.ux) * we->hy;
  int x0 = (int)floorf(cl.x - ext - 0.05f), x1 = (int)floorf(cl.x + ext + 0.05f);
  int y0 = (int)floorf(cl.y - exty - 0.05f), y1 = (int)floorf(cl.y + exty + 0.05f);
  int z0 = (int)floorf(le.z0), z1 = (int)floorf(le.z1 - 1e-4f);
  if (x1 < 0 || y1 < 0 || z1 < 0) return;
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (z0 < 0) z0 = 0;
  /* contacts are generated in local space; transform back */
  int start = ncon;
  for (int x = x0; x <= x1; x++)
    for (int y = y0; y <= y1; y++)
      for (int z = z0; z <= z1; z++) {
        int pi = shape_find(sh, x, y, z);
        if (pi < 0) continue;
        for (int j = pi; j < sh->np && (sh->key[j] & ~7u) == (sh->key[pi] & ~7u); j++) {
          const Block *bk = blocks[sh->blk[j]];
          int coll = bk->flags & 3;
          if (!coll) continue;
          int comp = PK_C(sh->key[j]);
          WElem se;
          if (coll == 2) {
            vec3 c;
            float r;
            part_sphere(bk, comp, sh->key[j], &c, &r);
            se.circle = 1;
            se.cx = c.x;
            se.cy = c.y;
            se.hx = r;
            se.ux = 1;
            se.uy = 0;
            se.z0 = c.z - r;
            se.z1 = c.z + r;
            collide_elems(&le, &se);
            continue;
          }
          /* Fancade box colliders are the bounds of the block's voxels */
          const uint8_t *bbx = bk->bb + comp * 6;
          if (bbx[0] > bbx[3]) continue;
          float mnx = x + bbx[0] / 8.0f, mny = y + bbx[1] / 8.0f, mnz = z + bbx[2] / 8.0f;
          float mxx = x + (bbx[3] + 1) / 8.0f, mxy = y + (bbx[4] + 1) / 8.0f, mxz = z + (bbx[5] + 1) / 8.0f;
          se.circle = 0;
          se.cx = (mnx + mxx) * 0.5f;
          se.cy = (mny + mxy) * 0.5f;
          se.ux = 1;
          se.uy = 0;
          se.hx = (mxx - mnx) * 0.5f;
          se.hy = (mxy - mny) * 0.5f;
          se.z0 = mnz;
          se.z1 = mxz;
          int before = ncon;
          collide_elems(&le, &se);
          /* faces shared with a neighbouring collider are internal: drop contacts pushing through them, and
             at a corner where one of the two faces is internal, push along the other face only */
          float qz = (fmaxf(le.z0, mnz) + fminf(le.z1, mxz)) * 0.5f;
          for (int c = before; c < ncon; c++) {
            Contact *k = &con[c];
            float nx = k->nx, ny = k->ny, px = k->px, py = k->py;
            bool drop = false;
            if (fabsf(nx) < 0.02f || fabsf(ny) < 0.02f) {
              drop = shape_solid_at(sh, v3(px + nx * 0.02f, py + ny * 0.02f, qz));
            } else {
              float sx = nx > 0 ? 1.0f : -1.0f, sy = ny > 0 ? 1.0f : -1.0f;
              bool inx = shape_solid_at(sh, v3(px + sx * 0.02f, py, qz)), iny = shape_solid_at(sh, v3(px, py + sy * 0.02f, qz));
              if (inx && iny) drop = true;
              else if ((inx || iny) && le.circle) {
                float r = le.hx, ccx = px + nx * (r - k->depth), ccy = py + ny * (r - k->depth);
                float dep = iny ? r - (ccx - px) * sx : r - (ccy - py) * sy;
                if (dep <= 0) drop = true;
                else if (iny) k->nx = sx, k->ny = 0, k->py = ccy, k->depth = dep;
                else k->nx = 0, k->ny = sy, k->px = ccx, k->depth = dep;
              }
            }
            if (drop) {
              con[c] = con[ncon - 1];
              ncon--;
              c--;
            }
          }
        }
      }
  /* transform new contacts back to world */
  for (int c = start; c < ncon; c++) {
    vec3 p = obj_world(s, v3(con[c].px, con[c].py, cl.z));
    vec3 n = qrot(s->rot, v3(con[c].nx, con[c].ny, 0));
    con[c].px = p.x;
    con[c].py = p.y;
    float l = sqrtf(n.x * n.x + n.y * n.y);
    con[c].nx = n.x / l;
    con[c].ny = n.y / l;
  }
  (void)bd;
}

static bool joined(int a, int b) {
  for (int j = 0; j < njoints; j++)
    if ((joints[j].a == a && joints[j].b == b) || (joints[j].a == b && joints[j].b == a)) return true;
  return false;
}

static float mix_friction(float a, float b) {
  float f = a * b * 1.05f;
#ifdef HOST
  f *= k_fmul;
#endif
  return f > 10 ? 10 : f;
}

/* is the world point inside a box collider of a static object other than skip? */
static bool static_solid_at(vec3 w, int skip) {
  for (int s = 0; s < nobj; s++) {
    const Obj *os = &objs[s];
    if (s == skip || (os->flags & (OF_DYNAMIC | OF_DEAD | OF_TEMPLATE | OF_COLLIDE | OF_VISIBLE)) != (OF_COLLIDE | OF_VISIBLE)) continue;
    const Shape *sh = os->shape;
    vec3 l = obj_local(os, w);
    if (l.x < sh->bmin.x || l.y < sh->bmin.y || l.z < sh->bmin.z || l.x > sh->bmax.x || l.y > sh->bmax.y || l.z > sh->bmax.z) continue;
    int cx = (int)floorf(l.x), cy = (int)floorf(l.y), cz = (int)floorf(l.z);
    int pj = shape_find(sh, cx, cy, cz);
    for (int j = pj; pj >= 0 && j < sh->np && (sh->key[j] & ~7u) == (sh->key[pj] & ~7u); j++) {
      const Block *nb = blocks[sh->blk[j]];
      if ((nb->flags & 3) != 1) continue;
      const uint8_t *bb = nb->bb + PK_C(sh->key[j]) * 6;
      float vx = (l.x - cx) * 8, vy = (l.y - cy) * 8, vz = (l.z - cz) * 8;
      if (vx >= bb[0] && vx <= bb[3] + 1 && vy >= bb[1] && vy <= bb[4] + 1 && vz >= bb[2] && vz <= bb[5] + 1) return true;
    }
  }
  return false;
}

static void gen_contacts(void) {
  ncon = 0;
  for (int i = 0; i < nbodies; i++) {
    Body *A = &bodies[i];
    if (A->obj < 0 || (objs[A->obj].flags & (OF_COLLIDE | OF_VISIBLE)) != (OF_COLLIDE | OF_VISIBLE)) continue;
    Obj *oa = &objs[A->obj];
    int first = ncon;
    WElem wa[MAX_ELEMS];
    int na = A->nel < MAX_ELEMS ? A->nel : MAX_ELEMS;
    for (int e = 0; e < na; e++) elem_world(A, &A->el[e], &wa[e]);
    /* static objects */
    for (int s = 0; s < nobj; s++) {
      Obj *os = &objs[s];
      if ((os->flags & (OF_DYNAMIC | OF_DEAD | OF_TEMPLATE | OF_COLLIDE | OF_VISIBLE)) != (OF_COLLIDE | OF_VISIBLE)) continue;
      /* AABB check in world */
      const Shape *sh = os->shape;
      vec3 c = obj_world(os, vscale(vadd(sh->bmin, sh->bmax), 0.5f));
      vec3 h = vscale(vsub(sh->bmax, sh->bmin), 0.5f);
      float r = sqrtf(h.x * h.x + h.y * h.y);
      if (fabsf(c.x - A->x) > r + A->bound || fabsf(c.y - A->y) > r + A->bound) continue;
      float zc = c.z, hz = fabsf(qrot(os->rot, v3(0, 0, 1)).z) * h.z + 1;
      if (A->zmax < zc - hz || A->zmin > zc + hz) continue;
      cur_a = i;
      cur_b = -1;
      cur_sobj = s;
      cur_mu = mix_friction(oa->friction, os->friction);
      cur_rest = oa->bounce * os->bounce;
      for (int e = 0; e < na; e++) collide_static(A, &wa[e], s);
    }
    /* faces buried in another static object (overlapping track pieces) are internal too */
    for (int c = first; c < ncon; c++) {
      float zc = (A->zmin + A->zmax) * 0.5f;
      if (!static_solid_at(v3(con[c].px + con[c].nx * 0.02f, con[c].py + con[c].ny * 0.02f, zc), con[c].sobj)) continue;
      con[c] = con[ncon - 1];
      ncon--;
      c--;
    }
    /* Fancade's floor: an endless plane at y = 0 */
    if (A->y - A->bound < 0) {
      cur_a = i;
      cur_b = -1;
      cur_sobj = -1;
      cur_mu = mix_friction(oa->friction, 0.5f);
      cur_rest = 0;
      for (int e = 0; e < na; e++) {
        const WElem *w = &wa[e];
        if (w->circle) {
          if (w->cy < w->hx) add_contact(w->cx, 0, 0, 1, w->hx - w->cy);
          continue;
        }
        for (int c = 0; c < 4; c++) {
          float sx = c & 1 ? w->hx : -w->hx, sy = c & 2 ? w->hy : -w->hy;
          float px = w->cx + w->ux * sx - w->uy * sy, py = w->cy + w->uy * sx + w->ux * sy;
          if (py < 0) add_contact(px, 0, 0, 1, -py);
        }
      }
    }
    /* other bodies */
    for (int j = i + 1; j < nbodies; j++) {
      Body *B = &bodies[j];
      if (B->obj < 0 || (objs[B->obj].flags & (OF_COLLIDE | OF_VISIBLE)) != (OF_COLLIDE | OF_VISIBLE)) continue;
      float dx = B->x - A->x, dy = B->y - A->y, rr = A->bound + B->bound;
      if (dx * dx + dy * dy > rr * rr) continue;
      if (A->zmax <= B->zmin || B->zmax <= A->zmin) continue;
      if (joined(i, j)) continue;
      Obj *ob = &objs[B->obj];
      cur_a = i;
      cur_b = j;
      cur_sobj = -1;
      cur_mu = mix_friction(oa->friction, ob->friction);
      cur_rest = oa->bounce * ob->bounce;
      WElem wb[MAX_ELEMS];
      int nb = B->nel < MAX_ELEMS ? B->nel : MAX_ELEMS;
      for (int e = 0; e < nb; e++) elem_world(B, &B->el[e], &wb[e]);
      for (int e = 0; e < na; e++)
        for (int f = 0; f < nb; f++) collide_elems(&wa[e], &wb[f]);
    }
  }
}

/* ----------------------------------------------------------------- solver */
static inline void vel_at(const Body *b, float rx, float ry, float *vx, float *vy) {
  *vx = b->vx - b->w * ry;
  *vy = b->vy + b->w * rx;
}

static inline void apply(Body *b, float rx, float ry, float jx, float jy) {
  b->vx += jx * b->invm * b->lockx;
  b->vy += jy * b->invm * b->locky;
  b->w += cross2(rx, ry, jx, jy) * b->invi * b->lockr;
}

static float eff_mass(const Body *a, float rax, float ray, const Body *b, float rbx, float rby, float nx, float ny) {
  float k = 0;
  if (a) {
    float rn = cross2(rax, ray, nx, ny);
    k += a->invm * (nx * nx * a->lockx + ny * ny * a->locky) + a->invi * a->lockr * rn * rn;
  }
  if (b) {
    float rn = cross2(rbx, rby, nx, ny);
    k += b->invm * (nx * nx * b->lockx + ny * ny * b->locky) + b->invi * b->lockr * rn * rn;
  }
  return k > 1e-9f ? 1.0f / k : 0;
}

static void prep_contacts(void) {
  for (int i = 0; i < ncon; i++) {
    Contact *c = &con[i];
    Body *A = &bodies[c->a];
    Body *B = c->b >= 0 ? &bodies[c->b] : 0;
    c->rax = c->px - A->x;
    c->ray = c->py - A->y;
    if (B) {
      c->rbx = c->px - B->x;
      c->rby = c->py - B->y;
    }
    c->mn = eff_mass(A, c->rax, c->ray, B, c->rbx, c->rby, c->nx, c->ny);
    c->mt = eff_mass(A, c->rax, c->ray, B, c->rbx, c->rby, -c->ny, c->nx);
    float vax, vay, vbx = 0, vby = 0;
    vel_at(A, c->rax, c->ray, &vax, &vay);
    if (B) vel_at(B, c->rbx, c->rby, &vbx, &vby);
    float vn = (vax - vbx) * c->nx + (vay - vby) * c->ny;
    c->bias = 0;
    c->pbias = 0.2f / DT * fmaxf(0, c->depth - 0.005f);
    if (c->rest > 0 && vn < -1.0f) c->bias += -c->rest * vn;
  }
}

static void solve_contacts(void) {
  for (int i = 0; i < ncon; i++) {
    Contact *c = &con[i];
    Body *A = &bodies[c->a];
    Body *B = c->b >= 0 ? &bodies[c->b] : 0;
    float vax, vay, vbx = 0, vby = 0;
    vel_at(A, c->rax, c->ray, &vax, &vay);
    if (B) vel_at(B, c->rbx, c->rby, &vbx, &vby);
    float dvx = vax - vbx, dvy = vay - vby;
    /* normal */
    float vn = dvx * c->nx + dvy * c->ny;
    float dj = c->mn * (-vn + c->bias);
    float jn0 = c->jn;
    c->jn = fmaxf(jn0 + dj, 0);
    dj = c->jn - jn0;
    apply(A, c->rax, c->ray, c->nx * dj, c->ny * dj);
    if (B) apply(B, c->rbx, c->rby, -c->nx * dj, -c->ny * dj);
    /* friction */
    vel_at(A, c->rax, c->ray, &vax, &vay);
    if (B) vel_at(B, c->rbx, c->rby, &vbx, &vby);
    float tx = -c->ny, ty = c->nx;
    float vt = (vax - vbx) * tx + (vay - vby) * ty;
    float dt = c->mt * -vt;
    float lim = c->mu * c->jn;
    float jt0 = c->jt;
    c->jt = clampf(jt0 + dt, -lim, lim);
    dt = c->jt - jt0;
    apply(A, c->rax, c->ray, tx * dt, ty * dt);
    if (B) apply(B, c->rbx, c->rby, -tx * dt, -ty * dt);
  }
}

static inline void apply_p(Body *b, float rx, float ry, float jx, float jy) {
  b->pvx += jx * b->invm * b->lockx;
  b->pvy += jy * b->invm * b->locky;
  b->pw += cross2(rx, ry, jx, jy) * b->invi * b->lockr;
}

/* split impulse: push penetrating contacts apart without adding kinetic energy */
static void solve_split(void) {
  for (int i = 0; i < ncon; i++) {
    Contact *c = &con[i];
    if (c->pbias <= 0) continue;
    Body *A = &bodies[c->a];
    Body *B = c->b >= 0 ? &bodies[c->b] : 0;
    float vx = A->pvx - A->pw * c->ray, vy = A->pvy + A->pw * c->rax;
    if (B) {
      vx -= B->pvx - B->pw * c->rby;
      vy -= B->pvy + B->pw * c->rbx;
    }
    float vn = vx * c->nx + vy * c->ny;
    float dj = c->mn * (c->pbias - vn);
    float j0 = c->jp;
    c->jp = fmaxf(j0 + dj, 0);
    dj = c->jp - j0;
    apply_p(A, c->rax, c->ray, c->nx * dj, c->ny * dj);
    if (B) apply_p(B, c->rbx, c->rby, -c->nx * dj, -c->ny * dj);
  }
}

/* joints: solved per degree of freedom in the base frame */
static inline void rot2(const Body *b, vec3 v, float *x, float *y) {
  *x = b->m00 * v.x + b->m01 * v.y + b->m02 * v.z;
  *y = b->m10 * v.x + b->m11 * v.y + b->m12 * v.z;
}

static void joint_frame(Joint *j, Body **pa, Body **pb, float *rax, float *ray, float *rbx, float *rby, float *axx,
                        float *axy, float *ex, float *ey, float *ang) {
  Body *A = j->a >= 0 ? &bodies[j->a] : 0;
  Body *B = &bodies[j->b];
  *pa = A;
  *pb = B;
  float wax, way, cx, cy;
  if (A) {
    rot2(A, j->la, rax, ray);
    wax = A->x + *rax;
    way = A->y + *ray;
    rot2(A, j->axx, &cx, &cy);
    float l = sqrtf(cx * cx + cy * cy);
    if (l > 1e-4f) {
      cx /= l;
      cy /= l;
    } else {
      cx = 1;
      cy = 0;
    }
  } else {
    *rax = *ray = 0;
    const Obj *so = &objs[j->sobj];
    vec3 w = obj_world(so, j->la), ax = qrot(so->rot, j->axx);
    wax = w.x;
    way = w.y;
    float l = sqrtf(ax.x * ax.x + ax.y * ax.y);
    cx = l > 1e-4f ? ax.x / l : 1;
    cy = l > 1e-4f ? ax.y / l : 0;
  }
  rot2(B, j->lb, rbx, rby);
  float wbx = B->x + *rbx, wby = B->y + *rby;
  *axx = cx;
  *axy = cy;
  float dx = wbx - wax, dy = wby - way;
  *ex = dx * cx + dy * cy;
  *ey = -dx * cy + dy * cx;
  *ang = B->a - (A ? A->a : atan2f(cy, cx)) - j->ref;
}

static void solve_joints(bool first) {
  for (int ji = 0; ji < njoints; ji++) {
    Joint *j = &joints[ji];
    if (j->b < 0 || bodies[j->b].obj < 0 || (j->a >= 0 && bodies[j->a].obj < 0)) continue;
    Body *A, *B;
    float rax, ray, rbx, rby, cx, cy, ex, ey, ang;
    joint_frame(j, &A, &B, &rax, &ray, &rbx, &rby, &cx, &cy, &ex, &ey, &ang);
    float err[3] = {ex, ey, ang};
    for (int k = 0; k < 3; k++) {
      float nx = 0, ny = 0;
      float m;
      if (k < 2) {
        nx = k == 0 ? cx : -cy;
        ny = k == 0 ? cy : cx;
        m = eff_mass(A, rax, ray, B, rbx, rby, nx, ny);
      } else {
        float ik = (A ? A->invi * A->lockr : 0) + B->invi * B->lockr;
        m = ik > 1e-9f ? 1.0f / ik : 0;
      }
      if (m <= 0) continue;
      /* relative velocity along dof (B relative to A) */
      float rv;
      if (k < 2) {
        float vax = 0, vay = 0, vbx, vby;
        if (A) vel_at(A, rax, ray, &vax, &vay);
        vel_at(B, rbx, rby, &vbx, &vby);
        rv = (vbx - vax) * nx + (vby - vay) * ny;
      } else {
        rv = B->w - (A ? A->w : 0);
      }
      float lo = j->lo[k], hi = j->hi[k];
      bool free_ = lo > hi;
      float imp = 0;
      /* motor */
      if (j->mf[k] > 0) {
        float maxi = j->mf[k] * DT;
        float d = m * (j->mv[k] - rv);
        float old = j->accm[k];
        j->accm[k] = clampf(old + d, -maxi, maxi);
        imp += j->accm[k] - old;
        rv += (j->accm[k] - old) / m;
      }
      /* spring: Bullet's 6DofSpring2 row, velocity target rv0 + f with the impulse clamped to f */
      if (j->spring[k] && (free_ || lo < hi)) {
        if (first) {
          /* Bullet limits stiff springs to what the time step can sample, and over-damping */
          float mr;
          if (k < 2) mr = 1.0f / ((A ? A->invm : 0) + B->invm);
          else mr = 1.0f / ((A ? A->invi * A->lockr : 0) + B->invi * B->lockr + 1e-9f);
          float ks = j->k[k], kd = j->c[k];
          if (0.25f < sqrtf(ks / mr) * DT) ks = mr / (DT * DT * 16.0f);
          if (kd * DT > mr) kd = mr / DT;
          float fd = -kd * rv * DT, f = -ks * err[k] * DT + fd;
          j->spt[k] = rv + f;
          j->splo[k] = fminf(fminf(f, fd), 0);
          j->sphi[k] = fmaxf(fmaxf(f, fd), 0);
        }
        float d = m * (j->spt[k] - rv);
        float old = j->acc[k];
        j->acc[k] = clampf(old + d, j->splo[k], j->sphi[k]);
        d = j->acc[k] - old;
        imp += d;
        rv += d / m;
      }
      /* limits / lock */
      if (!free_) {
        float target;
        if (lo == hi) {
          target = -0.2f / DT * (err[k] - lo);
          float d = m * (target - rv);
          j->accl[k] += d;
          imp += d;
        } else if (err[k] <= lo) {
          target = -0.2f / DT * (err[k] - lo);
          float d = m * (fmaxf(target, 0) - rv);
          float old = j->accl[k];
          j->accl[k] = fmaxf(old + d, 0);
          imp += j->accl[k] - old;
        } else if (err[k] >= hi) {
          target = -0.2f / DT * (err[k] - hi);
          float d = m * (fminf(target, 0) - rv);
          float old = j->accl[k];
          j->accl[k] = fminf(old + d, 0);
          imp += j->accl[k] - old;
        }
      }
      if (imp == 0) continue;
      if (k < 2) {
        if (A) apply(A, rax, ray, -nx * imp, -ny * imp);
        apply(B, rbx, rby, nx * imp, ny * imp);
      } else {
        if (A) A->w -= imp * A->invi * A->lockr;
        B->w += imp * B->invi * B->lockr;
      }
    }
  }
  (void)first;
}

/* ------------------------------------------------------------------- step */
void phys_step(void) {
  /* contacts live in the free arena space: nothing is allocated during a step */
  uint32_t avail;
  con = arena_top(&avail);
  con_cap = (int)(avail / sizeof(Contact));
  if (con_cap > MAX_CONTACTS) con_cap = MAX_CONTACTS;
  /* external forces */
  for (int i = 0; i < nbodies; i++) {
    Body *b = &bodies[i];
    if (b->obj < 0 || !(objs[b->obj].flags & OF_VISIBLE)) continue;
    b->vx += (phys_gravity.x + b->fx * b->invm) * DT * b->lockx;
    b->vy += (phys_gravity.y + b->fy * b->invm) * DT * b->locky;
    b->w += b->tq * b->invi * DT * b->lockr;
    b->fx = b->fy = b->tq = 0;
    if (k_linear_damping > 0) {
      float f = 1 - k_linear_damping * DT;
      b->vx *= f;
      b->vy *= f;
    }
    if (k_angular_damping > 0) b->w *= 1 - k_angular_damping * DT;
  }
  gen_contacts();
  prep_contacts();
  for (int ji = 0; ji < njoints; ji++) {
    Joint *j = &joints[ji];
    for (int k = 0; k < 3; k++) j->acc[k] = j->accm[k] = j->accl[k] = 0;
  }
  for (int it = 0; it < ITER; it++) {
    solve_joints(it == 0);
    solve_contacts();
  }
  for (int i = 0; i < nbodies; i++) bodies[i].pvx = bodies[i].pvy = bodies[i].pw = 0;
  for (int it = 0; it < ITER; it++) solve_split();
  /* events */
  nevents = 0;
  for (int i = 0; i < ncon; i++) {
    Contact *c = &con[i];
    int oa = bodies[c->a].obj, ob = c->b >= 0 ? bodies[c->b].obj : c->sobj;
    int k;
    for (k = 0; k < nevents; k++)
      if (events[k].a == oa && events[k].b == ob) break;
    if (k == nevents) {
      if (nevents >= MAX_EVENTS) continue;
      events[k].a = oa;
      events[k].b = ob;
      events[k].imp = 0;
      nevents++;
    }
    if (c->jn >= events[k].imp) {
      events[k].imp = c->jn;
      events[k].nx = c->nx;
      events[k].ny = c->ny;
    }
  }
  /* integrate */
  for (int i = 0; i < nbodies; i++) {
    Body *b = &bodies[i];
    if (b->obj < 0 || !(objs[b->obj].flags & OF_VISIBLE)) continue;
    b->x += (b->vx + b->pvx) * DT;
    b->y += (b->vy + b->pvy) * DT;
    b->a += (b->w + b->pw) * DT;
    vec3 s = v3(b->spin3.x * DEG2RAD * DT, b->spin3.y * DEG2RAD * DT, (b->w + b->pw) * DT);
    float l = vlen(s);
    if (l > 1e-9f) b->rot = qnorm(qmul(qaxis(vscale(s, 1 / l), l), b->rot));
    body_matrix(b);
    body_zrange(b);
    sync_obj(b);
  }
}

bool phys_collision(int o, int *other, float *impulse, vec3 *normal) {
  float best = -1;
  for (int k = 0; k < nevents; k++) {
    if (events[k].a == o || events[k].b == o) {
      if (events[k].imp > best) {
        best = events[k].imp;
        *other = events[k].a == o ? events[k].b : events[k].a;
        float s = events[k].a == o ? 1.0f : -1.0f;
        *normal = v3(events[k].nx * s, events[k].ny * s, 0);
        *impulse = events[k].imp;
      }
    }
  }
  return best >= 0;
}

#ifdef HOST
#include <stdio.h>
#include <stdlib.h>
int st_bodies, st_joints, st_con, st_elems, st_obj, st_elem_body;
static void stats_dump(void) {
  extern uint32_t arena_peak;
  extern int st_obj_setup;
  extern uint32_t st_arena_setup;
  printf("STATS setup_arena=%u setup_obj=%d ", st_arena_setup, st_obj_setup);
  printf("STATS arena=%u obj=%d bodies=%d joints=%d con=%d elems=%d maxel=%d\n", arena_peak, st_obj, st_bodies,
         st_joints, st_con, st_elems, st_elem_body);
}
void phys_debug(int frame) {
  static int reg;
  if (!reg && getenv("ND_STATS")) {
    reg = 1;
    atexit(stats_dump);
  }
  if (nbodies > st_bodies) st_bodies = nbodies;
  if (njoints > st_joints) st_joints = njoints;
  if (ncon > st_con) st_con = ncon;
  if (nelems > st_elems) st_elems = nelems;
  if (nobj > st_obj) st_obj = nobj;
  for (int i = 0; i < nbodies; i++)
    if (bodies[i].nel > st_elem_body) st_elem_body = bodies[i].nel;
  static int on = -1, tr = -1;
  if (tr < 0) tr = getenv("ND_TRACK") != 0;
  if (tr && nbodies && frame % (getenv("ND_T5") ? 5 : 10) == 0) {
    static float x0, y0;
    Body *b = &bodies[0];
    if (getenv("ND_TOBJ") && body(atoi(getenv("ND_TOBJ")))) b = body(atoi(getenv("ND_TOBJ")));
    if (frame == 0) {
      x0 = b->x;
      y0 = b->y;
    }
    printf("%d:%.2f,%.2f,%.0f ", frame, b->x - x0, b->y - y0, b->a * 57.3f);
    if (frame % 100 == 0) printf("\n");
    fflush(stdout);
  }
  if (getenv("ND_JDBG") && frame >= atoi(getenv("ND_JDBG")) && frame < atoi(getenv("ND_JDBG")) + 70) {
    printf("f%d", frame);
    for (int ji = 0; ji < njoints; ji++) {
      Joint *j = &joints[ji];
      Body *A, *B;
      float rax, ray, rbx, rby, cx, cy, ex, ey, ang;
      joint_frame(j, &A, &B, &rax, &ray, &rbx, &rby, &cx, &cy, &ex, &ey, &ang);
      printf(" j%d:%.3f", ji, ey);
    }
    printf(" ncon=%d\n", ncon);
  }
  if (getenv("ND_CON") && frame >= atoi(getenv("ND_CON")) && frame < atoi(getenv("ND_CON")) + 80) {
    printf("f%d:", frame);
    for (int i = 0; i < ncon; i++) {
      Contact *c = &con[i];
      printf(" b%d-s%d(%.2f,%.2f p=%.2f,%.2f jn=%.2f)", c->a, c->sobj, c->nx, c->ny, c->px, c->py, c->jn);
    }
    printf("\n");
  }
  if (on < 0) on = getenv("ND_DEBUG") != 0;
  if (!on) return;
  printf("frame %d bodies %d joints %d contacts %d\n", frame, nbodies, njoints, ncon);
  for (int i = 0; i < nbodies; i++) {
    Body *b = &bodies[i];
    if (b->obj < 0) continue;
    printf("  b%d obj%d m=%.2f I=%.3f pos=(%.3f,%.3f,%.3f) a=%.3f v=(%.3f,%.3f) w=%.3f nel=%d z=[%.2f,%.2f] lock=%g%g%g\n", i, b->obj,
           b->mass, b->inertia, b->x, b->y, b->z, b->a, b->vx, b->vy, b->w, b->nel, b->zmin, b->zmax, b->lockx, b->locky, b->lockr);
  }
  for (int j = 0; j < njoints; j++) {
    Joint *jt = &joints[j];
    printf("  j%d a=%d b=%d lo=(%.2f,%.2f,%.2f) hi=(%.2f,%.2f,%.2f) k=(%.1f,%.1f,%.1f) mv=%.2f mf=%.2f\n", j, jt->a, jt->b, jt->lo[0],
           jt->lo[1], jt->lo[2], jt->hi[0], jt->hi[1], jt->hi[2], jt->k[0], jt->k[1], jt->k[2], jt->mv[2], jt->mf[2]);
  }
}
#endif
