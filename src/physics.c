/* 2D rigid body physics (motion in the xy plane, rotation about z) for Fancade objects.
 * Collision shapes keep their z extents so only overlapping depth ranges interact. */
#include <string.h>
#ifdef HOST
#include <stdlib.h>
#endif
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
float k_motor_scale = 60.0f; /* Fancade's Bullet takes the motor force as the impulse per 1/60 s step */
float k_lin_motor_scale = 60.0f;
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
  float vz, pvz, lockz;      /* depth: only bodies free of joints and locks move in z */
  vec3 spin3;     /* extra free rotation (deg/s) about x,y for tumbling */
  uint8_t nel;
  uint8_t zmark;  /* scratch: already shifted in z */
  int16_t wgrp;   /* scratch: bodies welded into one rigid assembly share it */
  Elem *el;
  float bound;    /* bounding radius around com */
  float zmin, zmax;
} Body;

typedef struct {
  int16_t a, b;           /* body indices; b may be -1 for static */
  int16_t sobj;           /* static object id when b == -1 */
  uint8_t acirc;          /* a touches with a circle: its lever reaches its own surface, p - n * depth */
  uint8_t z3;             /* a body that can move in z takes part: normal and friction in 3D (2: a ball
                             touches, which rolls across instead of sliding) */
  float px, py;           /* world contact point (on b's surface) */
  float nx, ny;           /* normal from a to b (pointing out of b into a) */
  float nz, jz;           /* z3 only: the normal's depth part, friction impulse along z */
  float depth;
  float mu, rest;
  float jn, jt, jp;
  float mn, mt, bias, pbias;
  float rax, ray, rbx, rby;
} Contact;

/* last step's contact impulses, to start the solver from (Bullet's warm starting) */
typedef struct {
  int16_t a, o;   /* body, other body (>= 0) or -2 - static object, -1 for the floor */
  int16_t lx, ly; /* point in a's frame, 1/256 units */
  float jn, jt;
} Warm;

/* springs and motors, allocated for the joints that get one */
typedef struct {
  float k[3], c[3];   /* spring stiffness and damping */
  float mv[3], mf[3]; /* motor target velocity and max force */
} JointExt;

typedef struct {
  int16_t a, b;       /* body indices (a = base, -1 = static base object sobj), b = part, < 0 detached */
  int16_t sobj;       /* static base object (moves with Set Position), -1 = none */
  uint8_t spring;     /* bit k: dof k has a spring */
  JointExt *ext;
  vec3 la, lb;        /* anchors in rest frames (relative to com) */
  vec3 axx;           /* base frame x axis in the base rest frame */
  float ref;          /* reference relative angle */
  float lo[3], hi[3]; /* x, y, angle limits (lo > hi = free) */
} Joint;

/* solver state of one step, kept in the free arena with the contacts */
typedef struct {
  float accl[3]; /* accumulated limit / lock impulses */
} JointTmp;
typedef struct {
  float acc[3], accm[3];          /* accumulated spring and motor impulses */
  float spt[3], splo[3], sphi[3]; /* spring row target velocity and impulse range */
} JointExtTmp;

/* per level pools (arena) */
static Body *bodies;
static int nbodies, body_cap;
static int nelems;
static Contact *con; /* scratch space, valid during a step */
static int ncon, con_cap;
static Warm *warm;
static int nwarm, warm_cap;
static Joint *joints;
static int njoints, joint_cap;
static JointTmp *jtmp;
static JointExtTmp *etmp;
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
  /* a ray starting inside a box, or on its surface, does not hit it */
  if (t1 < 0 || t0 <= 1e-5f) return false;
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
    if (shape_solid_at(s, t > e ? vadd(o, vscale(d, t - e)) : o)) return false;
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
  vec3 seg = vsub(to, from);
  float sl = vdot(seg, seg), isl = sl > 1e-12f ? 1.0f / sl : 0;
  for (int i = 0; i < nobj; i++) {
    Obj *ob = &objs[i];
    if (!(ob->flags & OF_COLLIDE) || (ob->flags & OF_DEAD) || i == ignore) continue;
    /* quick reject: the segment misses the sphere around the object's bounds */
    const Shape *sh = ob->shape;
    vec3 lo = vsub(sh->bmin, sh->origin), hi = vsub(sh->bmax, sh->origin);
    vec3 ext = v3(fmaxf(-lo.x, hi.x), fmaxf(-lo.y, hi.y), fmaxf(-lo.z, hi.z));
    vec3 rel = vsub(ob->pos, from);
    float u = clampf(vdot(rel, seg) * isl, 0, 1);
    vec3 dd = vsub(rel, vscale(seg, u));
    if (vdot(dd, dd) > vdot(ext, ext) * 1.0001f + 1e-4f) continue;
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
  /* Bullet's compound shape: the inertia of a solid box the size of the colliders' bounds */
  float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
  for (int i = 0; i < s->np; i++) {
    const Block *bk = blocks[s->blk[i]];
    int coll = bk->flags & 3;
    if (!coll) continue;
    uint32_t k = s->key[i];
    float a[3], c[3];
    if (coll == 2) {
      vec3 cc;
      float r;
      part_sphere(bk, PK_C(k), k, &cc, &r);
      a[0] = cc.x - r, a[1] = cc.y - r, a[2] = cc.z - r, c[0] = cc.x + r, c[1] = cc.y + r, c[2] = cc.z + r;
    } else {
      const uint8_t *bb = bk->bb + PK_C(k) * 6;
      if (bb[0] > bb[3]) continue;
      int p[3] = {PK_X(k), PK_Y(k), PK_Z(k)};
      for (int d = 0; d < 3; d++) a[d] = p[d] + bb[d] / 8.0f, c[d] = p[d] + (bb[d + 3] + 1) / 8.0f;
    }
    for (int d = 0; d < 3; d++) {
      if (a[d] < lo[d]) lo[d] = a[d];
      if (c[d] > hi[d]) hi[d] = c[d];
    }
  }
  if (lo[0] > hi[0]) {
    /* no colliders: the bounds of its voxels */
    lo[0] = s->bmin.x, lo[1] = s->bmin.y, lo[2] = s->bmin.z;
    hi[0] = s->bmax.x, hi[1] = s->bmax.y, hi[2] = s->bmax.z;
  }
  float lx = hi[0] - lo[0], ly = hi[1] - lo[1], lz = hi[2] - lo[2], k = b->mass / 12;
  float ix = k * (ly * ly + lz * lz), iy = k * (lx * lx + lz * lz), iz = k * (lx * lx + ly * ly);
  /* about the world z axis (the body may be turned about x or y) */
  b->inertia = b->m20 * b->m20 * ix + b->m21 * b->m21 * iy + b->m22 * b->m22 * iz;
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
  warm = 0;
  nwarm = warm_cap = 0;
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
#ifdef HOST
  if (getenv("ND_PDBG")) fprintf(stderr, "setvel obj%d v=%s(%.2f %.2f %.2f) w=%s(%.2f %.2f %.2f)\n", o, vel ? "" : "-", vel ? vel->x : 0, vel ? vel->y : 0, vel ? vel->z : 0, spin ? "" : "-", spin ? spin->x : 0, spin ? spin->y : 0, spin ? spin->z : 0);
#endif
  Body *b = wake(o);
  if (!b) return;
  if (vel) {
    b->vx = vel->x;
    b->vy = vel->y;
    b->vz = vel->z;
  }
  if (spin) {
    b->w = spin->z * DEG2RAD;
    b->spin3.x = spin->x;
    b->spin3.y = spin->y;
  }
}

void phys_add_force(int o, const vec3 *f, const vec3 *at, const vec3 *t) {
  Body *b = wake(o);
#ifdef HOST
  if (getenv("ND_FDBG")) fprintf(stderr, "force obj%d b=%d f=%s(%.2f,%.2f,%.2f) at=%s(%.2f,%.2f,%.2f) t=%s\n", o, b ? (int)(b - bodies) : -1, f ? "" : "-", f ? f->x : 0, f ? f->y : 0, f ? f->z : 0, at ? "" : "-", at ? at->x : 0, at ? at->y : 0, at ? at->z : 0, t ? "y" : "-");
#endif
  if (!b) return;
  if (f) {
    b->fx += f->x;
    b->fy += f->y;
    if (at) b->tq += cross2(at->x - b->x, at->y - b->y, f->x, f->y);
  }
  if (t) b->tq += t->z * DEG2RAD;
}

void phys_set_locked(int o, const vec3 *p, const vec3 *r) {
#ifdef HOST
  if (getenv("ND_PDBG")) fprintf(stderr, "setlocked obj%d p=%s(%.1f %.1f %.1f) r=%s(%.1f %.1f %.1f)\n", o, p ? "" : "-", p ? p->x : 0, p ? p->y : 0, p ? p->z : 0, r ? "" : "-", r ? r->x : 0, r ? r->y : 0, r ? r->z : 0);
#endif
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
#ifdef HOST
  if (getenv("ND_PDBG")) fprintf(stderr, "setmass obj%d %.3f\n", o, m);
#endif
#ifdef HOST
  {
    static float sm = -1;
    if (sm < 0) sm = getenv("ND_SETMASS") ? atof(getenv("ND_SETMASS")) : 1;
    m *= sm;
  }
#endif
  objs[o].mass = m;
  Body *b = wake(o);
  if (b) body_mass(b);
}
void phys_set_friction(int o, float f) {
#ifdef HOST
  if (getenv("ND_PDBG")) fprintf(stderr, "setfriction obj%d %.3f\n", o, f);
#endif
  objs[o].mat = mat_find(f, obj_bounce(&objs[o]));
}
void phys_set_bounce(int o, float v) {
#ifdef HOST
  if (getenv("ND_PDBG")) fprintf(stderr, "setbounce obj%d %.3f\n", o, v);
#endif
  objs[o].mat = mat_find(obj_friction(&objs[o]), v);
}
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
  j->b = (int16_t)(pb - bodies);
  j->a = (int16_t)(bb ? (int)(bb - bodies) : -1);
  j->sobj = (int16_t)(bb ? -1 : base);
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
#ifdef HOST
  if (getenv("ND_CDBG")) fprintf(stderr, "limits c%d %s lo=(%.3f %.3f %.3f) hi=(%.3f %.3f %.3f)\n", c, ang ? "ang" : "lin", lo.x, lo.y, lo.z, hi.x, hi.y, hi.z);
#endif
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

static JointExt *joint_ext(int c) {
  if (c < 0 || c >= njoints) return 0;
  Joint *j = &joints[c];
  if (!j->ext) {
    j->ext = arena_alloc(sizeof(JointExt));
    if (j->ext) memset(j->ext, 0, sizeof(JointExt));
  }
  return j->ext;
}

void phys_con_spring(int c, bool ang, vec3 k, vec3 d) {
  JointExt *x = joint_ext(c);
  if (!x) return;
#ifdef HOST
  {
    static float ks = -1, kd = -1;
    if (ks < 0) ks = getenv("ND_KSPR") ? atof(getenv("ND_KSPR")) : 1, kd = getenv("ND_KDMP") ? atof(getenv("ND_KDMP")) : 1;
    k = vscale(k, ks);
    d = vscale(d, kd);
  }
#endif
  Joint *j = &joints[c];
  if (ang) {
    x->k[2] = k.z;
    x->c[2] = d.z;
    j->spring = (uint8_t)((j->spring & 3) | (k.z > 0 || d.z > 0) << 2);
  } else {
    x->k[0] = k.x;
    x->c[0] = d.x;
    x->k[1] = k.y;
    x->c[1] = d.y;
    j->spring = (uint8_t)((j->spring & 4) | (k.x > 0 || d.x > 0) | (k.y > 0 || d.y > 0) << 1);
  }
}

void phys_con_motor(int c, bool ang, vec3 v, vec3 f) {
  JointExt *x = joint_ext(c);
  if (!x) return;
  if (ang) {
    x->mv[2] = v.z * DEG2RAD;
    x->mf[2] = fabsf(f.z) * k_motor_scale;
  } else {
    x->mv[0] = v.x;
    x->mf[0] = fabsf(f.x) * k_lin_motor_scale;
    x->mv[1] = v.y;
    x->mf[1] = fabsf(f.y) * k_lin_motor_scale;
  }
}

/* --------------------------------------------------------------- contacts */
static int cur_a, cur_b, cur_sobj;
static uint8_t cur_acirc, cur_bcirc;
static float cur_mu, cur_rest;
static float cur_zc; /* a body free in depth: its com's z in the static object's frame (else NAN) */

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
  c->acirc = cur_acirc | (cur_bcirc ? 2 : 0);
  c->px = px;
  c->py = py;
  c->nx = nx;
  c->ny = ny;
  c->depth = depth;
  c->mu = cur_mu;
  c->rest = cur_rest;
  c->z3 = bodies[cur_a].lockz != 0 || (cur_b >= 0 && bodies[cur_b].lockz != 0);
  if (c->z3 && (cur_acirc || cur_bcirc)) c->z3 = 2;
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
  cur_acirc = a->circle;
  cur_bcirc = b->circle;
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
            /* a body free in depth rests only on parts under its com (past an edge it would tip off) */
            if (fabsf(cur_zc - c.z) > r) continue;
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
          if (cur_zc < mnz || cur_zc > mxz) continue;
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
          /* a sphere touches at its centre's depth; a box anywhere in the shared depth */
          float qz = le.circle ? fminf(fmaxf((le.z0 + le.z1) * 0.5f, mnz), mxz) : (fmaxf(le.z0, mnz) + fminf(le.z1, mxz)) * 0.5f;
          for (int c = before; c < ncon; c++) {
            Contact *k = &con[c];
            float nx = k->nx, ny = k->ny, px = k->px, py = k->py;
            bool drop = false;
            if (fabsf(nx) < 0.02f || fabsf(ny) < 0.02f) {
              /* the point is on the body: sample the neighbour just past the collider's face, level with
                 the collider (never on its edges) */
              if (fabsf(nx) < 0.02f) drop = shape_solid_at(sh, v3(fminf(fmaxf(px, mnx + 0.02f), mxx - 0.02f), (ny > 0 ? mxy : mny) + ny * 0.02f, qz));
              else drop = shape_solid_at(sh, v3((nx > 0 ? mxx : mnx) + nx * 0.02f, fminf(fmaxf(py, mny + 0.02f), mxy - 0.02f), qz));
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
              } else if (inx || iny) {
                /* a box on a tiled surface: push along the surface's face, from the box's deepest corner */
                float fx = iny ? sx : 0, fy = iny ? 0 : sy;
                float face = fx > 0 ? mxx : fx < 0 ? -mnx : fy > 0 ? mxy : -mny;
                float ex = fabsf(le.ux * fx + le.uy * fy) * le.hx + fabsf(-le.uy * fx + le.ux * fy) * le.hy;
                float dep = face - ((le.cx * fx + le.cy * fy) - ex);
                if (dep <= 0) drop = true;
                else {
                  /* the corner of the box furthest against the face */
                  float ax = (le.ux * fx + le.uy * fy) > 0 ? -1.0f : 1.0f, ay = (-le.uy * fx + le.ux * fy) > 0 ? -1.0f : 1.0f;
                  float qx = le.cx + ax * le.hx * le.ux - ay * le.hy * le.uy, qy = le.cy + ax * le.hx * le.uy + ay * le.hy * le.ux;
                  k->nx = fx, k->ny = fy, k->depth = dep;
                  if (fx != 0) k->px = fx > 0 ? mxx : mnx, k->py = fminf(fmaxf(qy, mny), mxy);
                  else k->py = fy > 0 ? mxy : mny, k->px = fminf(fmaxf(qx, mnx), mxx);
                }
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
  /* parts welded into one assembly behave as one body: they never push each other apart */
  if (bodies[a].wgrp == bodies[b].wgrp) return true;
  for (int j = 0; j < njoints; j++)
    if ((joints[j].a == a && joints[j].b == b) || (joints[j].a == b && joints[j].b == a)) return true;
  return false;
}

static int wroot(int i) {
  while (bodies[i].wgrp != i) i = bodies[i].wgrp = bodies[bodies[i].wgrp].wgrp;
  return i;
}

static void weld_groups(void) {
  for (int i = 0; i < nbodies; i++) bodies[i].wgrp = (int16_t)i;
  for (int k = 0; k < njoints; k++) {
    const Joint *j = &joints[k];
    if (j->a < 0 || j->b < 0 || j->spring || j->lo[0] != j->hi[0] || j->lo[1] != j->hi[1] || j->lo[2] != j->hi[2]) continue;
    int ra = wroot(j->a), rb = wroot(j->b);
    if (ra != rb) bodies[ra].wgrp = (int16_t)rb;
  }
  for (int i = 0; i < nbodies; i++) bodies[i].wgrp = (int16_t)wroot(i);
}

/* like Bullet, a body does not collide with the static object its constraint is attached to */
static bool joined_static(int bi, int s) {
  for (int j = 0; j < njoints; j++)
    if (joints[j].a < 0 && joints[j].sobj == s && joints[j].b == bi) return true;
  return false;
}

static float mix_friction(float a, float b) {
  float f = a * b; /* Bullet's combined friction */
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

/* bodies turned out of the plane are outlined generously in 2D: check their parts really meet in 3D */
static bool tilted(const Body *b) { return fabsf(b->m20) > 1e-3f || fabsf(b->m21) > 1e-3f; }

static void elem_frame(const Body *b, const Elem *e, float c[3], float u[3][3], float h[3]) {
  float cz = (e->z0 + e->z1) * 0.5f;
  c[0] = b->x + b->m00 * e->cx + b->m01 * e->cy + b->m02 * cz;
  c[1] = b->y + b->m10 * e->cx + b->m11 * e->cy + b->m12 * cz;
  c[2] = b->z + b->m20 * e->cx + b->m21 * e->cy + b->m22 * cz;
  float m[3][3] = {{b->m00, b->m10, b->m20}, {b->m01, b->m11, b->m21}, {b->m02, b->m12, b->m22}};
  memcpy(u, m, sizeof m);
  h[0] = e->hx, h[1] = e->circle ? e->hx : e->hy, h[2] = (e->z1 - e->z0) * 0.5f;
}

/* do the parts really meet in 3D? n (when given) gets the axis of least overlap, pointing from B's part into
   A's, and that overlap */
static bool elems_meet3d(const Body *A, const Elem *ea, const Body *B, const Elem *eb, float n[4]) {
  const float M = 0.05f;
  float ca[3], ua[3][3], ha[3], cb[3], ub[3][3], hb[3];
  elem_frame(A, ea, ca, ua, ha);
  elem_frame(B, eb, cb, ub, hb);
  float t[3] = {cb[0] - ca[0], cb[1] - ca[1], cb[2] - ca[2]};
  if (ea->circle && eb->circle) {
    float d = sqrtf(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]), o = ha[0] + hb[0] - d;
    if (d < 1e-6f) {
      n[0] = 0, n[1] = 1, n[2] = 0, n[3] = o;
      return true;
    }
    n[0] = -t[0] / d, n[1] = -t[1] / d, n[2] = -t[2] / d, n[3] = o;
    return o > -M;
  }
  if (ea->circle || eb->circle) {
    /* sphere against a box: the box's closest point */
    const float(*u)[3] = ea->circle ? ub : ua;
    const float *h = ea->circle ? hb : ha, *cs = ea->circle ? ca : cb, *cx = ea->circle ? cb : ca;
    float r = ea->circle ? ha[0] : hb[0], q[3] = {cx[0], cx[1], cx[2]}, d2 = 0, best = 1e9f;
    int bi = 0;
    for (int i = 0; i < 3; i++) {
      float p = (cs[0] - cx[0]) * u[i][0] + (cs[1] - cx[1]) * u[i][1] + (cs[2] - cx[2]) * u[i][2];
      float cl = fminf(fmaxf(p, -h[i]), h[i]), e = fabsf(p) - h[i];
      if (e > 0) d2 += e * e;
      if (-e < best) best = -e, bi = i;
      for (int k = 0; k < 3; k++) q[k] += cl * u[i][k];
    }
    if (d2 >= (r + M) * (r + M)) return false;
    /* from the box to the sphere, then turned to point into A */
    float v[3] = {cs[0] - q[0], cs[1] - q[1], cs[2] - q[2]}, d = sqrtf(d2), sg = ea->circle ? 1.0f : -1.0f, o = r - d;
    if (d < 1e-6f) {
      float p = (cs[0] - cx[0]) * u[bi][0] + (cs[1] - cx[1]) * u[bi][1] + (cs[2] - cx[2]) * u[bi][2];
      for (int k = 0; k < 3; k++) v[k] = u[bi][k] * (p < 0 ? -1.0f : 1.0f);
      d = 1, o = r + best;
    }
    n[0] = sg * v[0] / d, n[1] = sg * v[1] / d, n[2] = sg * v[2] / d, n[3] = o;
    return true;
  }
  /* box against box: separating axes */
  float R[3][3], AR[3][3], ta[3];
  for (int i = 0; i < 3; i++) {
    ta[i] = t[0] * ua[i][0] + t[1] * ua[i][1] + t[2] * ua[i][2];
    for (int j = 0; j < 3; j++) {
      R[i][j] = ua[i][0] * ub[j][0] + ua[i][1] * ub[j][1] + ua[i][2] * ub[j][2];
      AR[i][j] = fabsf(R[i][j]) + 1e-5f;
    }
  }
  float best = 1e9f, bd = 0, bl[3] = {0, 1, 0};
  for (int i = 0; i < 3; i++) {
    float o = ha[i] + hb[0] * AR[i][0] + hb[1] * AR[i][1] + hb[2] * AR[i][2] - fabsf(ta[i]);
    if (o < -M) return false;
    if (o < best) best = o, bd = ta[i], memcpy(bl, ua[i], sizeof bl);
  }
  for (int j = 0; j < 3; j++) {
    float d = ta[0] * R[0][j] + ta[1] * R[1][j] + ta[2] * R[2][j];
    float o = ha[0] * AR[0][j] + ha[1] * AR[1][j] + ha[2] * AR[2][j] + hb[j] - fabsf(d);
    if (o < -M) return false;
    if (o < best) best = o, bd = d, memcpy(bl, ub[j], sizeof bl);
  }
  for (int i = 0; i < 3; i++) {
    int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
    for (int j = 0; j < 3; j++) {
      int j1 = (j + 1) % 3, j2 = (j + 2) % 3;
      float ra = ha[i1] * AR[i2][j] + ha[i2] * AR[i1][j], rb = hb[j1] * AR[i][j2] + hb[j2] * AR[i][j1];
      float d = ta[i2] * R[i1][j] - ta[i1] * R[i2][j], o = ra + rb - fabsf(d);
      if (o < -M) return false;
      float l = sqrtf(fmaxf(1 - R[i][j] * R[i][j], 0));
      /* edge against edge only when clearly the least: faces are steadier */
      if (l > 1e-3f && o / l * 1.05f + 0.01f < best) {
        best = o / l, bd = d;
        bl[0] = (ua[i][1] * ub[j][2] - ua[i][2] * ub[j][1]) / l;
        bl[1] = (ua[i][2] * ub[j][0] - ua[i][0] * ub[j][2]) / l;
        bl[2] = (ua[i][0] * ub[j][1] - ua[i][1] * ub[j][0]) / l;
      }
    }
  }
  float sg = bd > 0 ? -1.0f : 1.0f;
  n[0] = sg * bl[0], n[1] = sg * bl[1], n[2] = sg * bl[2], n[3] = best;
  return true;
}

static void gen_contacts(void) {
  ncon = 0;
  weld_groups();
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
      if (joined_static(i, s)) continue;
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
      cur_mu = mix_friction(obj_friction(oa), obj_friction(os));
      cur_rest = obj_bounce(oa) * obj_bounce(os);
      cur_zc = A->lockz ? obj_local(os, v3(A->x, A->y, A->z)).z : NAN;
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
      cur_mu = mix_friction(obj_friction(oa), 0.5f);
      cur_rest = 0;
      for (int e = 0; e < na; e++) {
        const WElem *w = &wa[e];
        cur_acirc = w->circle;
        cur_bcirc = 0;
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
      cur_mu = mix_friction(obj_friction(oa), obj_friction(ob));
      cur_rest = obj_bounce(oa) * obj_bounce(ob);
      WElem wb[MAX_ELEMS];
      int nb = B->nel < MAX_ELEMS ? B->nel : MAX_ELEMS;
      for (int e = 0; e < nb; e++) elem_world(B, &B->el[e], &wb[e]);
      bool zf = A->lockz || B->lockz, t3 = zf || tilted(A) || tilted(B);
      for (int e = 0; e < na; e++)
        for (int f = 0; f < nb; f++) {
          float n3[4];
          if (!t3) collide_elems(&wa[e], &wb[f]);
          else if (elems_meet3d(A, &A->el[e], B, &B->el[f], n3)) {
            int before = ncon;
            collide_elems(&wa[e], &wb[f]);
            /* a part free to move in depth is pushed along the parts' real 3D normal */
            if (zf && fabsf(n3[2]) > 0.1f)
              for (int c = before; c < ncon; c++) con[c].nx = n3[0], con[c].ny = n3[1], con[c].nz = n3[2], con[c].depth = n3[3];
          }
        }
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

/* contacts with a body free in depth: the normal and a second friction direction reach into z (bodies
   still only turn about z) */
static inline void apply3(Body *b, float rx, float ry, float jx, float jy, float jz) {
  apply(b, rx, ry, jx, jy);
  b->vz += jz * b->invm * b->lockz;
}

static float eff_mass3(const Body *a, float rax, float ray, const Body *b, float rbx, float rby, float nx, float ny, float nz) {
  float k = 0;
  if (a) {
    float rn = cross2(rax, ray, nx, ny);
    k += a->invm * (nx * nx * a->lockx + ny * ny * a->locky + nz * nz * a->lockz) + a->invi * a->lockr * rn * rn;
  }
  if (b) {
    float rn = cross2(rbx, rby, nx, ny);
    k += b->invm * (nx * nx * b->lockx + ny * ny * b->locky + nz * nz * b->lockz) + b->invi * b->lockr * rn * rn;
  }
  return k > 1e-9f ? 1.0f / k : 0;
}

/* the in-plane tangent and the one across it */
static void tangents3(const Contact *c, float t1[3], float t2[3]) {
  float l = sqrtf(c->nx * c->nx + c->ny * c->ny);
  t1[0] = l > 1e-4f ? -c->ny / l : 1.0f, t1[1] = l > 1e-4f ? c->nx / l : 0.0f, t1[2] = 0;
  t2[0] = -c->nz * t1[1], t2[1] = c->nz * t1[0], t2[2] = c->nx * t1[1] - c->ny * t1[0];
}

static void solve_contact3(Contact *c) {
  Body *A = &bodies[c->a];
  Body *B = c->b >= 0 ? &bodies[c->b] : 0;
  float vax, vay, vbx = 0, vby = 0;
  vel_at(A, c->rax, c->ray, &vax, &vay);
  if (B) vel_at(B, c->rbx, c->rby, &vbx, &vby);
  float d[3] = {vax - vbx, vay - vby, A->vz - (B ? B->vz : 0)};
  float vn = d[0] * c->nx + d[1] * c->ny + d[2] * c->nz;
  float dj = c->mn * (-vn + c->bias), j0 = c->jn;
  c->jn = fmaxf(j0 + dj, 0);
  dj = c->jn - j0;
  apply3(A, c->rax, c->ray, c->nx * dj, c->ny * dj, c->nz * dj);
  if (B) apply3(B, c->rbx, c->rby, -c->nx * dj, -c->ny * dj, -c->nz * dj);
  float t[2][3], lim = c->mu * c->jn;
  tangents3(c, t[0], t[1]);
  for (int k = 0; k < (c->z3 == 2 ? 1 : 2); k++) {
    const float *u = t[k];
    vel_at(A, c->rax, c->ray, &vax, &vay);
    if (B) vel_at(B, c->rbx, c->rby, &vbx, &vby);
    float vt = (vax - vbx) * u[0] + (vay - vby) * u[1] + (A->vz - (B ? B->vz : 0)) * u[2];
    float m = eff_mass3(A, c->rax, c->ray, B, c->rbx, c->rby, u[0], u[1], u[2]);
    float *acc = k ? &c->jz : &c->jt, a0 = *acc;
    *acc = clampf(a0 + m * -vt, -lim, lim);
    float dt = *acc - a0;
    apply3(A, c->rax, c->ray, u[0] * dt, u[1] * dt, u[2] * dt);
    if (B) apply3(B, c->rbx, c->rby, -u[0] * dt, -u[1] * dt, -u[2] * dt);
  }
}

static inline int warm_other(const Contact *c) { return c->b >= 0 ? c->b : c->sobj >= 0 ? -2 - c->sobj : -1; }

static inline void warm_point(const Contact *c, int *lx, int *ly) {
  const Body *A = &bodies[c->a];
  float dx = c->px - A->x, dy = c->py - A->y;
  float x = (A->m00 * dx + A->m10 * dy) * 256, y = (A->m01 * dx + A->m11 * dy) * 256;
  *lx = x > 32767 ? 32767 : x < -32767 ? -32767 : (int)x;
  *ly = y > 32767 ? 32767 : y < -32767 ? -32767 : (int)y;
}

/* start each contact from the impulses of the same contact last step (both lists run in body order) */
static void warm_start(void) {
  int w = 0;
  for (int i = 0; i < ncon; i++) {
    Contact *c = &con[i];
    while (w < nwarm && warm[w].a < c->a) w++;
    /* wheels keep the cold start their handling was matched with */
    if (c->acirc) continue;
    int o = warm_other(c), lx, ly;
    warm_point(c, &lx, &ly);
    for (int k = w; k < nwarm && warm[k].a == c->a; k++) {
      Warm *m = &warm[k];
      int dx = m->lx - lx, dy = m->ly - ly;
      /* the same point: within Bullet's contact breaking threshold, 0.02 */
      if (m->o != o || dx * dx + dy * dy > 5 * 5) continue;
      c->jn = m->jn * 0.85f;
      c->jt = m->jt * 0.85f;
      m->o = -32768; /* used */
      break;
    }
  }
}

static void warm_store(void) {
  nwarm = 0;
  for (int i = 0; i < ncon && nwarm < warm_cap; i++) {
    const Contact *c = &con[i];
    Warm *m = &warm[nwarm++];
    int lx, ly;
    warm_point(c, &lx, &ly);
    m->a = c->a, m->o = (int16_t)warm_other(c), m->lx = (int16_t)lx, m->ly = (int16_t)ly;
    m->jn = c->jn, m->jt = c->jt;
  }
}

static void prep_contacts(void) {
  for (int i = 0; i < ncon; i++) {
    Contact *c = &con[i];
    Body *A = &bodies[c->a];
    Body *B = c->b >= 0 ? &bodies[c->b] : 0;
    /* like Bullet, a wheel rolls on its full radius however deep it sits */
    float sa = (c->acirc & 1) ? c->depth : 0;
    c->rax = c->px - c->nx * sa - A->x;
    c->ray = c->py - c->ny * sa - A->y;
    if (B) {
      c->rbx = c->px - B->x;
      c->rby = c->py - B->y;
    }
    c->mn = c->z3 ? eff_mass3(A, c->rax, c->ray, B, c->rbx, c->rby, c->nx, c->ny, c->nz)
                  : eff_mass(A, c->rax, c->ray, B, c->rbx, c->rby, c->nx, c->ny);
    c->mt = eff_mass(A, c->rax, c->ray, B, c->rbx, c->rby, -c->ny, c->nx);
    float vax, vay, vbx = 0, vby = 0;
    vel_at(A, c->rax, c->ray, &vax, &vay);
    if (B) vel_at(B, c->rbx, c->rby, &vbx, &vby);
    float vn = (vax - vbx) * c->nx + (vay - vby) * c->ny + (c->z3 ? (A->vz - (B ? B->vz : 0)) * c->nz : 0);
    /* parts only near each other in 3D may close the gap first */
    c->bias = c->z3 && c->depth < 0 ? c->depth / DT : 0;
    c->pbias = 0.2f / DT * fmaxf(0, c->depth - 0.005f);
    if (!c->acirc) {
      /* between boxes, like Bullet: shallow penetration is pushed out in the velocity solve (erp 0.2),
         deep with split impulses (erp2 0.8), and parts just apart may close their gap */
      if (c->depth < 0) c->bias = c->depth / DT, c->pbias = 0;
      else if (c->depth < 0.04f) c->bias = 0.2f * c->depth / DT, c->pbias = 0;
      else c->bias = 0, c->pbias = 0.8f * c->depth / DT;
    }
    if (c->rest > 0 && vn < -1.0f) c->bias += -c->rest * vn;
    if (c->jn > 0 || c->jt != 0) {
      /* the warm start: last step's impulses, applied up front */
      float jx = c->nx * c->jn - c->ny * c->jt, jy = c->ny * c->jn + c->nx * c->jt;
      if (c->z3) {
        float t[2][3];
        tangents3(c, t[0], t[1]);
        jx = c->nx * c->jn + t[0][0] * c->jt, jy = c->ny * c->jn + t[0][1] * c->jt;
        apply3(A, c->rax, c->ray, jx, jy, c->nz * c->jn);
        if (B) apply3(B, c->rbx, c->rby, -jx, -jy, -c->nz * c->jn);
      } else {
        apply(A, c->rax, c->ray, jx, jy);
        if (B) apply(B, c->rbx, c->rby, -jx, -jy);
      }
    }
  }
}

static void solve_contacts(void) {
  for (int i = 0; i < ncon; i++) {
    Contact *c = &con[i];
    if (c->z3) {
      solve_contact3(c);
      continue;
    }
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
    if (c->z3) vn += (A->pvz - (B ? B->pvz : 0)) * c->nz;
    float dj = c->mn * (c->pbias - vn);
    float j0 = c->jp;
    c->jp = fmaxf(j0 + dj, 0);
    dj = c->jp - j0;
    apply_p(A, c->rax, c->ray, c->nx * dj, c->ny * dj);
    if (B) apply_p(B, c->rbx, c->rby, -c->nx * dj, -c->ny * dj);
    if (c->z3) {
      A->pvz += c->nz * dj * A->invm * A->lockz;
      if (B) B->pvz -= c->nz * dj * B->invm * B->lockz;
    }
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
  JointExtTmp *et = etmp;
  for (int ji = 0; ji < njoints; ji++) {
    Joint *j = &joints[ji];
    JointExt *x = j->ext;
    JointExtTmp *t = x ? et++ : 0;
    JointTmp *jt = &jtmp[ji];
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
      if (x && x->mf[k] > 0) {
        float maxi = x->mf[k] * DT;
        float d = m * (x->mv[k] - rv);
        float old = t->accm[k];
        t->accm[k] = clampf(old + d, -maxi, maxi);
        imp += t->accm[k] - old;
        rv += (t->accm[k] - old) / m;
      }
      /* spring: Bullet's 6DofSpring2 row, velocity target rv0 + f with the impulse clamped to f */
      if (x && (j->spring >> k & 1) && (free_ || lo < hi)) {
        if (first) {
          /* Bullet limits stiff springs to what the time step can sample, and over-damping, by the lighter body */
          float mr = 1.0f / B->invm;
          if (A && A->mass < mr) mr = A->mass;
          float ks = x->k[k], kd = x->c[k];
          if (0.25f < sqrtf(ks / mr) * DT) ks = mr / (DT * DT * 16.0f);
          if (kd * DT > mr) kd = mr / DT;
          /* Fancade's Bullet takes the spring's velocity from the bodies' centres, not the anchors */
          float rs = k < 2 ? (B->vx - (A ? A->vx : 0)) * nx + (B->vy - (A ? A->vy : 0)) * ny : rv;
          float fd = -kd * rs * DT, f = -ks * err[k] * DT + fd;
          t->spt[k] = rs + f;
          t->splo[k] = fminf(fminf(f, fd), 0);
          t->sphi[k] = fmaxf(fmaxf(f, fd), 0);
        }
        float d = m * (t->spt[k] - rv);
        float old = t->acc[k];
        t->acc[k] = clampf(old + d, t->splo[k], t->sphi[k]);
        d = t->acc[k] - old;
        imp += d;
        rv += d / m;
      }
      /* limits / lock */
      if (!free_) {
        float target;
        if (lo == hi) {
          target = -0.2f / DT * (err[k] - lo);
          float d = m * (target - rv);
          jt->accl[k] += d;
          imp += d;
        } else if (err[k] <= lo) {
          target = -0.2f / DT * (err[k] - lo);
          float d = m * (fmaxf(target, 0) - rv);
          float old = jt->accl[k];
          jt->accl[k] = fmaxf(old + d, 0);
          imp += jt->accl[k] - old;
        } else if (err[k] >= hi) {
          target = -0.2f / DT * (err[k] - hi);
          float d = m * (fminf(target, 0) - rv);
          float old = jt->accl[k];
          jt->accl[k] = fminf(old + d, 0);
          imp += jt->accl[k] - old;
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
  /* joint solver state and contacts live in the free arena space: nothing is allocated during a step
     (but for the warm start cache, once: an eighth of the space left after the level is set up) */
  uint32_t avail;
  uint8_t *top = arena_top(&avail);
  if (!warm && nbodies) {
    int cap = (int)(avail / 8 / sizeof(Warm));
    if (cap > MAX_CONTACTS) cap = MAX_CONTACTS;
    warm = cap > 0 ? arena_alloc(sizeof(Warm) * cap) : 0;
    warm_cap = warm ? cap : 0;
    nwarm = 0;
    top = arena_top(&avail);
  }
  int next = 0;
  for (int ji = 0; ji < njoints; ji++)
    if (joints[ji].ext) next++;
  uint32_t jbytes = (uint32_t)(njoints * sizeof(JointTmp) + next * sizeof(JointExtTmp) + 7) & ~7u;
  bool jok = jbytes <= avail;
  if (!jok) {
    OOM("joint state");
    jbytes = 0;
  }
  memset(top, 0, jbytes);
  jtmp = (JointTmp *)top;
  etmp = (JointExtTmp *)(top + njoints * sizeof(JointTmp));
  con = (Contact *)(top + jbytes);
  avail -= jbytes;
  con_cap = (int)(avail / sizeof(Contact));
  if (con_cap > MAX_CONTACTS) con_cap = MAX_CONTACTS;
  /* like Fancade's Bullet bodies, one left free of joints and locks also moves in depth */
  for (int i = 0; i < nbodies; i++) bodies[i].lockz = bodies[i].obj >= 0 && (objs[bodies[i].obj].lockp & 4) ? 1.0f : 0.0f;
  for (int j = 0; j < njoints; j++) {
    if (joints[j].a >= 0) bodies[joints[j].a].lockz = 0;
    if (joints[j].b >= 0) bodies[joints[j].b].lockz = 0;
  }
  /* external forces */
  for (int i = 0; i < nbodies; i++) {
    Body *b = &bodies[i];
    if (!b->lockz) b->vz = 0;
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
  warm_start();
  prep_contacts();
  for (int it = 0; it < ITER; it++) {
    if (jok) solve_joints(it == 0);
    solve_contacts();
  }
  warm_store();
  for (int i = 0; i < nbodies; i++) bodies[i].pvx = bodies[i].pvy = bodies[i].pw = bodies[i].pvz = 0;
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
    b->z += (b->vz + b->pvz) * DT;
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
long st_slack = 1L << 30;
uint32_t st_slack_pos;
int st_slack_con, st_slack_obj;
static void stats_dump(void) {
  extern uint32_t arena_peak;
  extern int st_obj_setup;
  extern uint32_t st_arena_setup;
  printf("STATS setup_arena=%u setup_obj=%d ", st_arena_setup, st_obj_setup);
  printf("SLACK %ld pos %u con %d obj %d ", st_slack, st_slack_pos, st_slack_con, st_slack_obj);
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
  {
    extern uint32_t arena_free(void);
    long slack = (long)arena_free() - (long)(ncon * sizeof(Contact));
    if (slack < st_slack) {
      st_slack = slack;
      extern uint32_t arena_used(void);
      st_slack_pos = arena_used();
      st_slack_con = ncon;
      st_slack_obj = nobj;
    }
  }
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
  if (getenv("ND_GEO") && frame == 0) {
    float g0, g1, g2, g3;
    sscanf(getenv("ND_GEO"), "%f,%f,%f,%f", &g0, &g1, &g2, &g3);
    for (int so = 0; so < nobj; so++) {
      Obj *os = &objs[so];
      if (os->flags & (OF_DYNAMIC | OF_DEAD | OF_TEMPLATE)) continue;
      const Shape *sh = os->shape;
      if (!sh) continue;
      for (int i = 0; i < sh->np; i++) {
        const Block *bk = blocks[sh->blk[i]];
        if (!(bk->flags & 3)) continue;
        uint32_t k = sh->key[i];
        const uint8_t *bb = bk->bb + PK_C(k) * 6;
        vec3 mn = obj_world(os, v3(PK_X(k) + bb[0] / 8.0f, PK_Y(k) + bb[1] / 8.0f, PK_Z(k) + bb[2] / 8.0f));
        vec3 mx = obj_world(os, v3(PK_X(k) + (bb[3] + 1) / 8.0f, PK_Y(k) + (bb[4] + 1) / 8.0f, PK_Z(k) + (bb[5] + 1) / 8.0f));
        if (fmaxf(mn.x, mx.x) < g0 || fminf(mn.x, mx.x) > g2 || fmaxf(mn.y, mx.y) < g1 || fminf(mn.y, mx.y) > g3) continue;
        printf("geo s%d coll%d x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]\n", so, bk->flags & 3, mn.x, mx.x, mn.y, mx.y, mn.z, mx.z);
      }
    }
  }
  if (getenv("ND_CON") && frame >= atoi(getenv("ND_CON")) && frame < atoi(getenv("ND_CON")) + 80) {
    printf("f%d:", frame);
    for (int i = 0; i < ncon; i++) {
      Contact *c = &con[i];
      printf(" b%d-%c%d(%.2f,%.2f p=%.2f,%.2f jn=%.2f jt=%.2f mu=%.2f)", c->a, c->b >= 0 ? (char)98 : (char)115, c->b >= 0 ? c->b : c->sobj, c->nx, c->ny, c->px, c->py, c->jn, c->jt, c->mu);
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
    if (frame == 0)
      for (int k = 0; k < b->nel; k++)
        printf("    el%d %s c=(%.3f,%.3f) h=(%.3f,%.3f) z=[%.3f,%.3f] fr=%.2f\n", k, b->el[k].circle ? "circ" : "box", b->el[k].cx, b->el[k].cy,
               b->el[k].hx, b->el[k].hy, b->el[k].z0, b->el[k].z1, obj_friction(&objs[b->obj]));
  }
  for (int j = 0; j < njoints; j++) {
    Joint *jt = &joints[j];
    JointExt z = {{0}}, *x = jt->ext ? jt->ext : &z;
    printf("  j%d a=%d b=%d lo=(%.2f,%.2f,%.2f) hi=(%.2f,%.2f,%.2f) k=(%.1f,%.1f,%.1f) c=%.1f mv=%.2f mf=%.2f\n", j, jt->a, jt->b, jt->lo[0],
           jt->lo[1], jt->lo[2], jt->hi[0], jt->hi[1], jt->hi[2], x->k[0], x->k[1], x->k[2], x->c[1], x->mv[2], x->mf[2]);
    if (frame == 0) printf("    la=(%.3f,%.3f,%.3f) lb=(%.3f,%.3f,%.3f)\n", jt->la.x, jt->la.y, jt->la.z, jt->lb.x, jt->lb.y, jt->lb.z);
  }
}
#endif
