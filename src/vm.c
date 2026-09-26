/* Fancade script interpreter. */
#include <string.h>
#ifdef HOST
#include <stdio.h>
#include <stdlib.h>
#endif
#include "vm.h"
#include "world.h"
#include "physics.h"
#include "ops.h"
#include "platform.h"
#include "ui.h"

typedef struct {
  uint8_t type;
  uint8_t cap2; /* capacity: 0, or 1 << (cap2 - 1) elements */
  uint16_t len;
  uint8_t *data;
} VarStore;
#define VS_CAP(vs) ((vs)->cap2 ? 1 << ((vs)->cap2 - 1) : 0)

typedef struct {
  VarStore *vs;
  int32_t idx;
} Ptr;

typedef struct {
  uint8_t t;
  uint8_t isptr;
  union {
    float f;
    vec3 v;
    quat q;
    int32_t i;
    Ptr p;
  } u;
} Val;

struct Env {
  Prog *prog;
  uint16_t parent, pnode;
  uint16_t nsub; /* environments in this subtree, itself included */
  int16_t bx, by, bz;
  int16_t anchor;
  uint16_t tbase;
  int16_t *selfobj;
  VarStore *locals;
  Val *slots;
};

static Env *envs;
static int nenvs;
static VarStore *globals;
int vm_frame_count;
uint32_t vm_buttons;
static int button_counter;
static uint32_t rng_state = 0x12345678u;

#define MAXLATE 64
static struct { uint16_t env, node; } lateq[MAXLATE];
static int nlate;

static const uint8_t esize[6] = {4, 12, 16, 1, 2, 2};

static uint16_t rd16(const uint8_t *p) { return p[0] | p[1] << 8; }
static float rdf(const uint8_t *p) {
  uint32_t v = p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
  float f;
  memcpy(&f, &v, 4);
  return f;
}

static float rnd01(void) {
  rng_state ^= rng_state << 13;
  rng_state ^= rng_state >> 17;
  rng_state ^= rng_state << 5;
  return (rng_state >> 8) * (1.0f / 16777216.0f);
}

/* ---------------------------------------------------------------- program */
static inline const uint8_t *node_ptr(const Prog *p, int i) { return p->data + p->off[i]; }
static inline uint16_t node_in(const uint8_t *nd, int k) { return rd16(nd + 1 + 2 * k); }
/* exec list e (0..nexec-1), e == -1 for After. returns pointer to count byte */
static const uint8_t *node_exec(const uint8_t *nd, int e) {
  int op = nd[0];
  const uint8_t *q = nd + 1 + 2 * op_shape[op][0];
  int ne = op_shape[op][2];
  if (e < 0) e = ne;
  for (int i = 0; i < e; i++) q += 1 + 2 * q[0];
  return q;
}
static const uint8_t *node_data(const uint8_t *nd) {
  const uint8_t *q = node_exec(nd, -1);
  return q + 1 + 2 * q[0];
}

/* -------------------------------------------------------------- variables */
static VarStore *var_of(Env *e, const uint8_t *data) {
  uint16_t v = rd16(data);
  if (v & 0x8000) return &globals[v & 0x7fff];
  return &e->locals[v];
}

static void val_default(Val *v, int t) {
  memset(v, 0, sizeof *v);
  v->t = t;
  if (t == T_ROT) v->u.q = qident();
  if (t == T_OBJ || t == T_CON) v->u.i = -1;
}

static void var_read(VarStore *vs, int idx, Val *out) {
  val_default(out, vs->type);
  if (idx < 0 || idx >= vs->len) return;
  const uint8_t *d = vs->data + idx * esize[vs->type];
  switch (vs->type) {
    case T_NUM: memcpy(&out->u.f, d, 4); break;
    case T_VEC: memcpy(&out->u.v, d, 12); break;
    case T_ROT: memcpy(&out->u.q, d, 16); break;
    case T_TRU: out->u.i = d[0]; break;
    default: {
      int16_t s;
      memcpy(&s, d, 2);
      out->u.i = s;
    }
  }
}

/* list storage given up when a list grows elsewhere, reused by later growth (the arena never frees) */
#define NFREE 32
static struct {
  uint8_t *p;
  uint32_t n;
} freeblk[NFREE];
static int nfree;

static void give_back(uint8_t *p, uint32_t n) {
  n = (n + 7) & ~7u;
  if (!p || !n || arena_release_top(p, n)) return;
  int k = nfree;
  if (nfree == NFREE) { /* full: replace the smallest if this one is bigger */
    k = 0;
    for (int i = 1; i < NFREE; i++)
      if (freeblk[i].n < freeblk[k].n) k = i;
    if (freeblk[k].n >= n) return;
  } else {
    nfree++;
  }
  freeblk[k].p = p;
  freeblk[k].n = n;
}

static uint8_t *list_alloc(uint32_t n) {
  n = (n + 7) & ~7u;
  int best = -1;
  for (int i = 0; i < nfree; i++)
    if (freeblk[i].n >= n && (best < 0 || freeblk[i].n < freeblk[best].n)) best = i;
  if (best < 0) return arena_alloc(n);
  uint8_t *p = freeblk[best].p;
  uint32_t rest = freeblk[best].n - n;
  if (rest >= 16) {
    freeblk[best].p = p + n;
    freeblk[best].n = rest;
  } else {
    freeblk[best] = freeblk[--nfree];
  }
  return p;
}

static void var_write(VarStore *vs, int idx, const Val *v) {
  if (idx < 0 || idx >= 4096) return;
  int es = esize[vs->type];
  int cap = VS_CAP(vs);
  if (idx >= cap) {
    int c2 = vs->cap2 ? vs->cap2 : 1;
    while ((1 << (c2 - 1)) <= idx) c2++;
    int nc = 1 << (c2 - 1);
    if (vs->data && arena_extend(vs->data, cap * es, nc * es)) {
      vs->cap2 = (uint8_t)c2;
    } else {
      uint8_t *nd = list_alloc(nc * es);
      if (!nd) return;
      if (vs->len) memcpy(nd, vs->data, vs->len * es);
      give_back(vs->data, cap * es);
      vs->data = nd;
      vs->cap2 = (uint8_t)c2;
    }
  }
  while (vs->len <= idx) {
    Val d;
    val_default(&d, vs->type);
    uint8_t *p = vs->data + vs->len * es;
    switch (vs->type) {
      case T_NUM: memcpy(p, &d.u.f, 4); break;
      case T_VEC: memcpy(p, &d.u.v, 12); break;
      case T_ROT: memcpy(p, &d.u.q, 16); break;
      case T_TRU: p[0] = 0; break;
      default: {
        int16_t s = -1;
        memcpy(p, &s, 2);
      }
    }
    vs->len++;
  }
  uint8_t *p = vs->data + idx * es;
  switch (vs->type) {
    case T_NUM: memcpy(p, &v->u.f, 4); break;
    case T_VEC: memcpy(p, &v->u.v, 12); break;
    case T_ROT: memcpy(p, &v->u.q, 16); break;
    case T_TRU: p[0] = v->u.i ? 1 : 0; break;
    default: {
      int16_t s = (int16_t)v->u.i;
      memcpy(p, &s, 2);
    }
  }
}

/* ------------------------------------------------------------- evaluation */
static void eval(Env *e, uint16_t ref, Val *out);
static void run_chain(Env *e, uint16_t node);
static void run_list(Env *e, const uint8_t *lst);

static void deref(Val *v) {
  if (v->isptr) {
    Ptr p = v->u.p;
    var_read(p.vs, p.idx, v);
  }
}

static float in_num(Env *e, const uint8_t *nd, int k) {
  Val v;
  eval(e, node_in(nd, k), &v);
  deref(&v);
  return v.t == T_NUM ? v.u.f : 0;
}
static vec3 in_vec(Env *e, const uint8_t *nd, int k) {
  Val v;
  eval(e, node_in(nd, k), &v);
  deref(&v);
  return v.t == T_VEC ? v.u.v : v3(0, 0, 0);
}
static quat in_rot(Env *e, const uint8_t *nd, int k) {
  Val v;
  eval(e, node_in(nd, k), &v);
  deref(&v);
  return v.t == T_ROT ? v.u.q : qident();
}
static int in_tru(Env *e, const uint8_t *nd, int k) {
  Val v;
  eval(e, node_in(nd, k), &v);
  deref(&v);
  return v.t == T_TRU ? v.u.i != 0 : 0;
}
static int in_obj(Env *e, const uint8_t *nd, int k) {
  Val v;
  eval(e, node_in(nd, k), &v);
  deref(&v);
  if (v.t != T_OBJ && v.t != T_CON) return -1;
  return v.u.i;
}
static bool connected(const uint8_t *nd, int k) { return node_in(nd, k) != NONE16; }

static Env *child_env(Env *e, int node) {
  int ei = (int)(e - envs);
  for (int i = ei + 1; i < nenvs; i++)
    if (envs[i].parent == ei && envs[i].pnode == node) return &envs[i];
  return 0;
}

/* block position of this environment (for Get Position without object) */
static void env_block_pose(Env *e, vec3 *pos, quat *rot) {
  /* a script block's position is the centre of its model (a low plate for most); inside an object it is
     that block's centre */
  vec3 rest = v3(e->bx + 7 / 16.0f, e->by + e->prog->yc / 16.0f, e->bz + 7 / 16.0f);
  if (e->anchor >= 0) {
    /* the block's offset in its object is not turned with the object (scripts turn it themselves) */
    Obj *o = &objs[e->anchor];
    *pos = vadd(o->pos, vsub(shape_block_center(o->shape, e->bx, e->by, e->bz), o->shape->origin));
    *rot = o->rot;
  } else if (e->parent != NONE16 && !envs[e->parent].prog->is_level) {
    /* not part of an object in its custom block: the enclosing block's position */
    env_block_pose(&envs[e->parent], pos, rot);
  } else {
    *pos = rest;
    *rot = qident();
  }
}

static bool obj_valid(int o) { return o >= 0 && o < nobj && !(objs[o].flags & OF_DEAD); }

static quat quat_lerp(quat a, quat b, float t) {
  float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  float s = d < 0 ? -1.0f : 1.0f;
  quat r = {a.x + (s * b.x - a.x) * t, a.y + (s * b.y - a.y) * t, a.z + (s * b.z - a.z) * t,
            a.w + (s * b.w - a.w) * t};
  return qnorm(r);
}

static quat look_rotation(vec3 fwd, vec3 up) {
  if (fwd.x == 0 && fwd.y == 0 && fwd.z == 0) return qident();
  fwd = vnorm(fwd);
  up = vnorm(up);
  vec3 right = vcross(up, fwd);
  if (right.x == 0 && right.y == 0 && right.z == 0) right = v3(1, 0, 0);
  else right = vnorm(right);
  up = vcross(fwd, right);
  /* matrix rows: right, up, fwd (System.Numerics row-vector convention) */
  float m00 = right.x, m01 = right.y, m02 = right.z;
  float m10 = up.x, m11 = up.y, m12 = up.z;
  float m20 = fwd.x, m21 = fwd.y, m22 = fwd.z;
  float tr = m00 + m11 + m22;
  quat q;
  if (tr > 0) {
    float s = sqrtf(tr + 1.0f);
    q.w = s * 0.5f;
    s = 0.5f / s;
    q.x = (m12 - m21) * s;
    q.y = (m20 - m02) * s;
    q.z = (m01 - m10) * s;
  } else if (m00 >= m11 && m00 >= m22) {
    float s = sqrtf(1.0f + m00 - m11 - m22);
    float inv = 0.5f / s;
    q.x = 0.5f * s;
    q.y = (m01 + m10) * inv;
    q.z = (m02 + m20) * inv;
    q.w = (m12 - m21) * inv;
  } else if (m11 > m22) {
    float s = sqrtf(1.0f + m11 - m00 - m22);
    float inv = 0.5f / s;
    q.x = (m10 + m01) * inv;
    q.y = 0.5f * s;
    q.z = (m21 + m12) * inv;
    q.w = (m20 - m02) * inv;
  } else {
    float s = sqrtf(1.0f + m22 - m00 - m11);
    float inv = 0.5f / s;
    q.x = (m20 + m02) * inv;
    q.y = (m21 + m12) * inv;
    q.z = 0.5f * s;
    q.w = (m01 - m10) * inv;
  }
  return qnorm(q);
}

extern vec3 cam_world_to_screen(vec3 p);
extern void cam_screen_to_world(float sx, float sy, vec3 *near, vec3 *far);

#define RND_MEMO 8
static struct {
  const Env *e;
  uint16_t node;
  float v;
} rnd_memo[RND_MEMO];
static int nrnd;

static void eval_node(Env *e, int ni, int out, Val *r) {
  const Prog *p = e->prog;
  const uint8_t *nd = node_ptr(p, ni);
  int op = nd[0];
  if (op == OP_CUSTOM) {
    Env *c = child_env(e, ni);
    if (!c || out >= c->prog->noutputs) {
      val_default(r, T_NUM);
      return;
    }
    eval(c, rd16(c->prog->outrefs + 2 * out), r);
    return;
  }
  if (p->slot[ni] != NONE16) {
    *r = e->slots[p->slot[ni] + out];
    return;
  }
  memset(r, 0, sizeof *r);
  switch (op) {
    case OP_NUMBER: r->t = T_NUM; r->u.f = rdf(node_data(nd)); return;
    case OP_VECTOR: {
      const uint8_t *d = node_data(nd);
      r->t = T_VEC;
      r->u.v = v3(rdf(d), rdf(d + 4), rdf(d + 8));
      return;
    }
    case OP_ROTATION: {
      const uint8_t *d = node_data(nd);
      r->t = T_ROT;
      r->u.q = qeuler(rdf(d), rdf(d + 4), rdf(d + 8));
      return;
    }
    case OP_TRUE: r->t = T_TRU; r->u.i = 1; return;
    case OP_FALSE: r->t = T_TRU; r->u.i = 0; return;
    case OP_GET_VAR_NUM: case OP_GET_VAR_VEC: case OP_GET_VAR_ROT: case OP_GET_VAR_TRU: case OP_GET_VAR_OBJ:
    case OP_GET_VAR_CON: {
      VarStore *vs = var_of(e, node_data(nd));
      r->t = vs->type;
      r->isptr = 1;
      r->u.p.vs = vs;
      r->u.p.idx = 0;
      return;
    }
    case OP_LIST_NUM: case OP_LIST_OBJ: case OP_LIST_VEC: case OP_LIST_ROT: case OP_LIST_TRU: case OP_LIST_CON: {
      Val v;
      eval(e, node_in(nd, 0), &v);
      float idx = in_num(e, nd, 1);
      if (!v.isptr) {
        val_default(r, T_NUM);
        return;
      }
      *r = v;
      r->u.p.idx += (int32_t)idx;
      return;
    }
    case OP_NEGATE: r->t = T_NUM; r->u.f = -in_num(e, nd, 0); return;
    case OP_ADD_NUMBERS: r->t = T_NUM; r->u.f = in_num(e, nd, 0) + in_num(e, nd, 1); return;
    case OP_SUBTRACT_NUMBERS: r->t = T_NUM; r->u.f = in_num(e, nd, 0) - in_num(e, nd, 1); return;
    case OP_MULTIPLY: r->t = T_NUM; r->u.f = in_num(e, nd, 0) * in_num(e, nd, 1); return;
    case OP_DIVIDE: {
      float a = in_num(e, nd, 0), b = in_num(e, nd, 1);
      r->t = T_NUM;
      r->u.f = b != 0 ? a / b : 0;
      return;
    }
    case OP_ADD_VECTORS: r->t = T_VEC; r->u.v = vadd(in_vec(e, nd, 0), in_vec(e, nd, 1)); return;
    case OP_SUBTRACT_VECTORS: r->t = T_VEC; r->u.v = vsub(in_vec(e, nd, 0), in_vec(e, nd, 1)); return;
    case OP_SCALE: r->t = T_VEC; r->u.v = vscale(in_vec(e, nd, 0), in_num(e, nd, 1)); return;
    case OP_ROTATE: r->t = T_VEC; r->u.v = qrot(in_rot(e, nd, 1), in_vec(e, nd, 0)); return;
    case OP_COMBINE: r->t = T_ROT; r->u.q = qnorm(qmul(in_rot(e, nd, 0), in_rot(e, nd, 1))); return;
    case OP_LESS_THAN: r->t = T_TRU; r->u.i = in_num(e, nd, 0) < in_num(e, nd, 1); return;
    case OP_GREATER_THAN: r->t = T_TRU; r->u.i = in_num(e, nd, 0) > in_num(e, nd, 1); return;
    case OP_EQUALS_NUMBERS: r->t = T_TRU; r->u.i = fabsf(in_num(e, nd, 0) - in_num(e, nd, 1)) < 0.001f; return;
    case OP_EQUALS_VECTORS: {
      vec3 d = vsub(in_vec(e, nd, 0), in_vec(e, nd, 1));
      r->t = T_TRU;
      r->u.i = vdot(d, d) < 1.0000001e-6f;
      return;
    }
    case OP_EQUALS_OBJECTS: {
      int a = in_obj(e, nd, 0), b = in_obj(e, nd, 1);
      if (a >= 0 && !obj_valid(a)) a = -1;
      if (b >= 0 && !obj_valid(b)) b = -1;
      r->t = T_TRU;
      r->u.i = a == b;
      return;
    }
    case OP_EQUALS_TRUTHS: r->t = T_TRU; r->u.i = in_tru(e, nd, 0) == in_tru(e, nd, 1); return;
    case OP_NOT: r->t = T_TRU; r->u.i = !in_tru(e, nd, 0); return;
    case OP_AND: r->t = T_TRU; r->u.i = in_tru(e, nd, 0) && in_tru(e, nd, 1); return;
    case OP_OR: r->t = T_TRU; r->u.i = in_tru(e, nd, 0) || in_tru(e, nd, 1); return;
    case OP_MAKE_VECTOR: r->t = T_VEC; r->u.v = v3(in_num(e, nd, 0), in_num(e, nd, 1), in_num(e, nd, 2)); return;
    case OP_BREAK_VECTOR: {
      vec3 v = in_vec(e, nd, 0);
      r->t = T_NUM;
      r->u.f = out == 0 ? v.x : out == 1 ? v.y : v.z;
      return;
    }
    case OP_MAKE_ROTATION: r->t = T_ROT; r->u.q = qeuler(in_num(e, nd, 0), in_num(e, nd, 1), in_num(e, nd, 2)); return;
    case OP_BREAK_ROTATION: {
      quat q = in_rot(e, nd, 0);
      float v;
      if (out == 0) {
        float s = 2.0f * (q.w * q.x - q.y * q.z);
        v = s >= 1 ? 90.0f : s <= -1 ? -90.0f : asinf(s) * RAD2DEG;
      } else if (out == 1) {
        v = atan2f(2.0f * (q.x * q.z + q.w * q.y), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)) * RAD2DEG;
      } else {
        v = atan2f(2.0f * (q.x * q.y + q.w * q.z), 1.0f - 2.0f * (q.x * q.x + q.z * q.z)) * RAD2DEG;
      }
      r->t = T_NUM;
      r->u.f = v;
      return;
    }
    case OP_RANDOM: {
      /* a value block is worked out once per statement: every use shares the same number */
      for (int i = 0; i < nrnd; i++)
        if (rnd_memo[i].e == e && rnd_memo[i].node == ni) {
          r->t = T_NUM;
          r->u.f = rnd_memo[i].v;
          return;
        }
      float lo = in_num(e, nd, 0), hi = connected(nd, 1) ? in_num(e, nd, 1) : 1.0f;
      r->t = T_NUM;
      r->u.f = lo + (hi - lo) * rnd01();
      if (nrnd < RND_MEMO) {
        rnd_memo[nrnd].e = e;
        rnd_memo[nrnd].node = (uint16_t)ni;
        rnd_memo[nrnd++].v = r->u.f;
      }
      return;
    }
    case OP_MODULO: {
      float a = in_num(e, nd, 0), b = in_num(e, nd, 1);
      float m = b != 0 ? fmodf(a, b) : 0;
      r->t = T_NUM;
      r->u.f = m >= 0 ? m : b + m;
      return;
    }
    case OP_MIN: { float a = in_num(e, nd, 0), b = in_num(e, nd, 1); r->t = T_NUM; r->u.f = a < b ? a : b; return; }
    case OP_MAX: { float a = in_num(e, nd, 0), b = in_num(e, nd, 1); r->t = T_NUM; r->u.f = a > b ? a : b; return; }
    case OP_ROUND: r->t = T_NUM; r->u.f = rintf(in_num(e, nd, 0)); return;
    case OP_FLOOR: r->t = T_NUM; r->u.f = floorf(in_num(e, nd, 0)); return;
    case OP_CEILING: r->t = T_NUM; r->u.f = ceilf(in_num(e, nd, 0)); return;
    case OP_ABSOLUTE: r->t = T_NUM; r->u.f = fabsf(in_num(e, nd, 0)); return;
    case OP_SIN: r->t = T_NUM; r->u.f = sinf(in_num(e, nd, 0) * DEG2RAD); return;
    case OP_COS: r->t = T_NUM; r->u.f = cosf(in_num(e, nd, 0) * DEG2RAD); return;
    case OP_POWER: r->t = T_NUM; r->u.f = powf(in_num(e, nd, 0), in_num(e, nd, 1)); return;
    case OP_LOGARITHM: {
      float a = in_num(e, nd, 0), b = in_num(e, nd, 1);
      r->t = T_NUM;
      r->u.f = logf(a) / logf(b);
      return;
    }
    case OP_DISTANCE: r->t = T_NUM; r->u.f = vlen(vsub(in_vec(e, nd, 0), in_vec(e, nd, 1))); return;
    case OP_DOT_PRODUCT: r->t = T_NUM; r->u.f = vdot(in_vec(e, nd, 0), in_vec(e, nd, 1)); return;
    case OP_CROSS_PRODUCT: r->t = T_VEC; r->u.v = vcross(in_vec(e, nd, 0), in_vec(e, nd, 1)); return;
    case OP_NORMALIZE: r->t = T_VEC; r->u.v = vnorm(in_vec(e, nd, 0)); return;
    case OP_INVERSE: r->t = T_ROT; r->u.q = qconj(in_rot(e, nd, 0)); return;
    case OP_LERP: r->t = T_ROT; r->u.q = quat_lerp(in_rot(e, nd, 0), in_rot(e, nd, 1), in_num(e, nd, 2)); return;
    case OP_AXIS_ANGLE: r->t = T_ROT; r->u.q = qaxis(vnorm(in_vec(e, nd, 0)), in_num(e, nd, 1) * DEG2RAD); return;
    case OP_LOOK_ROTATION: {
      vec3 up = connected(nd, 1) ? in_vec(e, nd, 1) : v3(0, 1, 0);
      r->t = T_ROT;
      r->u.q = look_rotation(in_vec(e, nd, 0), up);
      return;
    }
    case OP_LINE_VS_PLANE: {
      vec3 a = in_vec(e, nd, 0), b = in_vec(e, nd, 1), pp = in_vec(e, nd, 2), pn = in_vec(e, nd, 3);
      float den = vdot(vsub(b, a), pn);
      float t = den != 0 ? vdot(vsub(pp, a), pn) / den : 0;
      r->t = T_VEC;
      r->u.v = vadd(a, vscale(vsub(b, a), t));
      return;
    }
    case OP_SCREEN_SIZE: r->t = T_NUM; r->u.f = out == 0 ? SCREEN_W : SCREEN_H; return;
    case OP_ACCELEROMETER: r->t = T_VEC; r->u.v = v3(0, -9.8f, 0); return;
    case OP_CURRENT_FRAME: r->t = T_NUM; r->u.f = (float)vm_frame_count; return;
    case OP_WORLD_TO_SCREEN: {
      vec3 s = cam_world_to_screen(in_vec(e, nd, 0));
      r->t = T_NUM;
      r->u.f = out == 0 ? s.x : s.y;
      return;
    }
    case OP_SCREEN_TO_WORLD: {
      vec3 n, f;
      cam_screen_to_world(in_num(e, nd, 0), in_num(e, nd, 1), &n, &f);
      r->t = T_VEC;
      r->u.v = out == 0 ? n : f;
      return;
    }
    case OP_RAYCAST: {
      vec3 from = in_vec(e, nd, 0), to = in_vec(e, nd, 1), hit;
      int o = -1;
      bool h = phys_raycast(from, to, &hit, &o);
#ifdef HOST
      if (getenv("ND_RDBG") && (vm_frame_count == atoi(getenv("ND_RDBG")) || atoi(getenv("ND_RDBG")) < 0))
      {
        fprintf(stderr, "ray (%.2f %.2f %.2f)->(%.2f %.2f %.2f) hit=%d obj=%d at (%.2f %.2f %.2f) f%d\n", from.x, from.y, from.z, to.x, to.y,
                to.z, h, o, hit.x, hit.y, hit.z, vm_frame_count);
        if (getenv("ND_RDBG2") && fabsf(from.x - atof(getenv("ND_RDBG2"))) < 0.01f)
          for (int q = 0; q < nobj; q++) {
            vec3 l = obj_local(&objs[q], from), l2 = obj_local(&objs[q], to);
            const Shape *sh = objs[q].shape;
            float ax = l.x < l2.x ? l.x : l2.x, bx = l.x < l2.x ? l2.x : l.x;
            float ay = l.y < l2.y ? l.y : l2.y, by = l.y < l2.y ? l2.y : l.y;
            float az = l.z < l2.z ? l.z : l2.z, bz = l.z < l2.z ? l2.z : l.z;
            if (bx < sh->bmin.x || ax > sh->bmax.x || by < sh->bmin.y || ay > sh->bmax.y || bz < sh->bmin.z || az > sh->bmax.z) continue;
            fprintf(stderr, "   f%d cand obj %d flags %x local (%.3f %.3f %.3f)->(%.3f %.3f %.3f) bounds (%.2f %.2f %.2f)-(%.2f %.2f %.2f) np=%d\n", vm_frame_count, q, objs[q].flags, l.x, l.y, l.z, l2.x, l2.y, l2.z,
                    sh->bmin.x, sh->bmin.y, sh->bmin.z, sh->bmax.x, sh->bmax.y, sh->bmax.z, sh->np);
          }
        if (h) {
          vec3 l = obj_local(&objs[o], from), l2 = obj_local(&objs[o], to);
          fprintf(stderr, "   local (%.3f %.3f %.3f)->(%.3f %.3f %.3f) np=%d env anchor=%d\n", l.x, l.y, l.z, l2.x, l2.y, l2.z, objs[o].shape->np, e->anchor);
        }
      }
#endif
      if (out == 0) {
        r->t = T_TRU;
        r->u.i = h;
      } else if (out == 1) {
        r->t = T_VEC;
        r->u.v = h ? hit : to;
      } else {
        r->t = T_OBJ;
        r->u.i = h ? o : -1;
      }
      return;
    }
    case OP_GET_POSITION: {
      vec3 pos;
      quat rot;
      if (!connected(nd, 0)) {
        env_block_pose(e, &pos, &rot);
      } else {
        int o = in_obj(e, nd, 0);
        if (obj_valid(o)) {
          pos = objs[o].pos;
          rot = objs[o].rot;
        } else {
          pos = v3(0, 0, 0);
          rot = qident();
        }
      }
      if (out == 0) {
        r->t = T_VEC;
        r->u.v = pos;
      } else {
        r->t = T_ROT;
        r->u.q = rot;
      }
      return;
    }
    case OP_GET_VELOCITY: {
      int o = in_obj(e, nd, 0);
      vec3 vel = v3(0, 0, 0), spin = v3(0, 0, 0);
      if (obj_valid(o)) phys_get_velocity(o, &vel, &spin);
      r->t = T_VEC;
      r->u.v = out == 0 ? vel : spin;
      return;
    }
    case OP_GET_SIZE: {
      vec3 mn, mx;
      if (!connected(nd, 0)) {
        mn = v3(0, 0, 0);
        mx = v3(level.sx, level.sy, level.sz);
      } else {
        int o = in_obj(e, nd, 0);
        if (!obj_valid(o)) {
          mn = mx = v3(0, 0, 0);
        } else {
          /* local bounds relative to the object's position, not rotated */
          const Shape *s = objs[o].shape;
          mn = vsub(s->bmin, s->origin);
          mx = vsub(s->bmax, s->origin);
        }
      }
      r->t = T_VEC;
      r->u.v = out == 0 ? mn : mx;
      return;
    }
    default: val_default(r, T_NUM); return;
  }
}

static void eval(Env *e, uint16_t ref, Val *out) {
  if (ref == NONE16) {
    memset(out, 0, sizeof *out);
    out->t = 0xff;
    return;
  }
  int kind = ref >> 13;
  switch (kind) {
    case 0: eval_node(e, ref & 2047, (ref >> 11) & 3, out); return;
    case 1: {
      if (e->parent == NONE16) {
        out->t = 0xff;
        return;
      }
      Env *pe = &envs[e->parent];
      const uint8_t *nd = node_ptr(pe->prog, e->pnode);
      int k = ref & 8191;
      if (k >= nd[9]) {
        out->t = 0xff;
        return;
      }
      eval(pe, rd16(nd + 10 + 2 * k), out);
      return;
    }
    case 2: {
      memset(out, 0, sizeof *out);
      out->t = T_OBJ;
      int k = ref & 8191;
      out->u.i = k < e->prog->nself ? e->selfobj[k] : -1;
      return;
    }
    default: {
      memset(out, 0, sizeof *out);
      out->t = T_OBJ;
      out->u.i = e->tbase + (ref & 8191);
      return;
    }
  }
}

/* ------------------------------------------------------------- statements */
static void set_slot(Env *e, int ni, int k, const Val *v) {
  uint16_t s = e->prog->slot[ni];
  if (s != NONE16) e->slots[s + k] = *v;
}

static void exec_stmt(Env *e, int ni) {
  nrnd = 0;
  const Prog *p = e->prog;
  const uint8_t *nd = node_ptr(p, ni);
  int op = nd[0];
  switch (op) {
    case OP_SET_VAR_NUM: case OP_SET_VAR_VEC: case OP_SET_VAR_ROT: case OP_SET_VAR_TRU: case OP_SET_VAR_OBJ:
    case OP_SET_VAR_CON: {
      VarStore *vs = var_of(e, node_data(nd));
      Val v;
      eval(e, node_in(nd, 0), &v);
      deref(&v);
      /* a value wired from an input the outer script left unconnected changes nothing (scripts use
         "set a default, then set the input" for optional inputs) */
      if (v.t == 0xff && node_in(nd, 0) != NONE16) return;
      if (v.t != vs->type && !(vs->type == T_CON && v.t == T_OBJ)) val_default(&v, vs->type);
      var_write(vs, 0, &v);
      return;
    }
    case OP_SET_PTR_NUM: case OP_SET_PTR_VEC: case OP_SET_PTR_ROT: case OP_SET_PTR_TRU: case OP_SET_PTR_OBJ:
    case OP_SET_PTR_CON: {
      Val ptr, v;
      eval(e, node_in(nd, 0), &ptr);
      if (!ptr.isptr) return;
      eval(e, node_in(nd, 1), &v);
      deref(&v);
      if (v.t == 0xff && node_in(nd, 1) != NONE16) return;
      if (v.t != ptr.u.p.vs->type && !(ptr.u.p.vs->type == T_CON && v.t == T_OBJ)) val_default(&v, ptr.u.p.vs->type);
      var_write(ptr.u.p.vs, ptr.u.p.idx, &v);
      return;
    }
    case OP_INCREASE_NUMBER: case OP_DECREASE_NUMBER: {
      Val ptr, v;
      eval(e, node_in(nd, 0), &ptr);
      if (!ptr.isptr) return;
      var_read(ptr.u.p.vs, ptr.u.p.idx, &v);
      if (v.t != T_NUM) return;
      v.u.f += op == OP_INCREASE_NUMBER ? 1 : -1;
      var_write(ptr.u.p.vs, ptr.u.p.idx, &v);
      return;
    }
    case OP_IF: {
      int c = in_tru(e, nd, 0);
      run_list(e, node_exec(nd, c ? 0 : 1));
      return;
    }
    case OP_PLAY_SENSOR:
      if (vm_frame_count == 0) run_list(e, node_exec(nd, 0));
      return;
    case OP_LATE_UPDATE: {
      const uint8_t *l = node_exec(nd, 0);
      for (int i = 0; i < l[0] && nlate < MAXLATE; i++) {
        lateq[nlate].env = (uint16_t)(e - envs);
        lateq[nlate].node = rd16(l + 1 + 2 * i);
        nlate++;
      }
      return;
    }
    case OP_BUTTON: {
      int b = button_counter++;
      if (vm_buttons & (1u << b)) run_list(e, node_exec(nd, 0));
      return;
    }
    case OP_BOX_ART_SENSOR: case OP_TOUCH_SENSOR: case OP_SWIPE_SENSOR: case OP_JOYSTICK: return;
    case OP_COLLISION: {
      int o = in_obj(e, nd, 0);
      int other;
      float imp;
      vec3 n;
      if (obj_valid(o) && phys_collision(o, &other, &imp, &n)) {
        Val v;
        memset(&v, 0, sizeof v);
        v.t = T_OBJ;
        v.u.i = other;
        set_slot(e, ni, 0, &v);
        v.t = T_NUM;
        v.u.f = imp;
        set_slot(e, ni, 1, &v);
        v.t = T_VEC;
        v.u.v = n;
        set_slot(e, ni, 2, &v);
        run_list(e, node_exec(nd, 0));
      }
      return;
    }
    case OP_LOOP: {
      int start = (int)in_num(e, nd, 0);
      int stop = (int)ceilf(in_num(e, nd, 1));
      int step = stop > start ? 1 : stop < start ? -1 : 0;
      int value = start - step;
      Val v;
      memset(&v, 0, sizeof v);
      v.t = T_NUM;
      v.u.f = (float)value;
      set_slot(e, ni, 0, &v);
      if (!step) return;
      const uint8_t *body = node_exec(nd, 0);
      for (int guard = 0; guard < 100000; guard++) {
        stop = (int)ceilf(in_num(e, nd, 1));
        int next = value + step;
        if (step > 0 ? next >= stop : next <= stop) break;
        value = next;
        v.u.f = (float)value;
        set_slot(e, ni, 0, &v);
        run_list(e, body);
      }
      return;
    }
    case OP_WIN:
#ifdef HOST
      if (getenv("ND_WDBG")) fprintf(stderr, "win at frame %d env %d node %d\n", vm_frame_count, (int)(e - envs), ni);
#endif
      game_win(node_data(nd)[0]);
      return;
    case OP_LOSE: game_lose(node_data(nd)[0]); return;
    case OP_SET_SCORE:
      if (connected(nd, 0)) hud_score = in_num(e, nd, 0);
#ifdef HOST
      if (getenv("ND_SDBG")) fprintf(stderr, "score f%d %.2f\n", vm_frame_count, hud_score);
#endif
      if (connected(nd, 1)) hud_coins = in_num(e, nd, 1);
      return;
    case OP_SET_CAMERA: {
      vec3 pos;
      quat rot;
      float range;
      if (connected(nd, 0)) pos = in_vec(e, nd, 0);
      if (connected(nd, 1)) rot = in_rot(e, nd, 1);
      if (connected(nd, 2)) range = in_num(e, nd, 2);
      cam_set(connected(nd, 0) ? &pos : 0, connected(nd, 1) ? &rot : 0, connected(nd, 2) ? &range : 0,
              node_data(nd)[0]);
      return;
    }
    case OP_SET_LIGHT: {
      if (connected(nd, 1)) {
        quat q = in_rot(e, nd, 1);
        light_set(&q);
      }
      return;
    }
    case OP_SET_POSITION: {
      int o = connected(nd, 0) ? in_obj(e, nd, 0) : e->anchor;
      if (!obj_valid(o)) return;
      if (connected(nd, 1)) objs[o].pos = in_vec(e, nd, 1);
      if (connected(nd, 2)) objs[o].rot = qnorm(in_rot(e, nd, 2));
#ifdef HOST
      if (getenv("ND_PDBG")) fprintf(stderr, "f%d setpos obj%d (%.3f %.3f %.3f) rot(%.3f %.3f %.3f %.3f)%s\n", vm_frame_count, o, objs[o].pos.x, objs[o].pos.y, objs[o].pos.z, objs[o].rot.x, objs[o].rot.y, objs[o].rot.z, objs[o].rot.w, connected(nd, 2) ? " +rot" : "");
#endif
      objs[o].flags |= OF_MOVED;
      phys_moved(o);
      return;
    }
    case OP_SET_VELOCITY: {
      int o = connected(nd, 0) ? in_obj(e, nd, 0) : e->anchor;
      if (!obj_valid(o)) return;
      vec3 vel, spin;
      if (connected(nd, 1)) vel = in_vec(e, nd, 1);
      if (connected(nd, 2)) spin = in_vec(e, nd, 2);
      phys_set_velocity(o, connected(nd, 1) ? &vel : 0, connected(nd, 2) ? &spin : 0);
      return;
    }
    case OP_ADD_FORCE: {
      int o = connected(nd, 0) ? in_obj(e, nd, 0) : e->anchor;
      if (!obj_valid(o)) return;
      vec3 f, at, t;
      if (connected(nd, 1)) f = in_vec(e, nd, 1);
      if (connected(nd, 2)) at = in_vec(e, nd, 2);
      if (connected(nd, 3)) t = in_vec(e, nd, 3);
      phys_add_force(o, connected(nd, 1) ? &f : 0, connected(nd, 2) ? &at : 0, connected(nd, 3) ? &t : 0);
      return;
    }
    case OP_SET_LOCKED: {
      int o = connected(nd, 0) ? in_obj(e, nd, 0) : e->anchor;
      if (!obj_valid(o)) return;
      vec3 a, b;
      if (connected(nd, 1)) a = in_vec(e, nd, 1);
      if (connected(nd, 2)) b = in_vec(e, nd, 2);
      phys_set_locked(o, connected(nd, 1) ? &a : 0, connected(nd, 2) ? &b : 0);
      return;
    }
    case OP_SET_VISIBLE: {
      int o = connected(nd, 0) ? in_obj(e, nd, 0) : e->anchor;
      if (!obj_valid(o)) return;
      if (in_tru(e, nd, 1)) objs[o].flags |= OF_VISIBLE;
      else if (objs[o].flags & OF_VISIBLE) {
        objs[o].flags &= ~OF_VISIBLE;
        phys_hidden(o);
      }
      return;
    }
    case OP_CREATE_OBJECT: {
      int o = in_obj(e, nd, 0);
      Val v;
      memset(&v, 0, sizeof v);
      v.t = T_OBJ;
      v.u.i = obj_valid(o) ? obj_clone(o) : -1;
      if (v.u.i >= 0 && (objs[v.u.i].flags & OF_PHYSICS)) phys_make_dynamic(v.u.i);
      set_slot(e, ni, 0, &v);
      return;
    }
    case OP_DESTROY_OBJECT: {
      int o = in_obj(e, nd, 0);
      if (obj_valid(o)) {
        phys_destroyed(o);
        obj_destroy(o);
      }
      return;
    }
    case OP_SET_GRAVITY: phys_set_gravity(in_vec(e, nd, 0)); return;
    case OP_SET_MASS: {
      int o = connected(nd, 0) ? in_obj(e, nd, 0) : e->anchor;
      if (obj_valid(o)) phys_set_mass(o, in_num(e, nd, 1));
      return;
    }
    case OP_SET_FRICTION: {
      int o = connected(nd, 0) ? in_obj(e, nd, 0) : e->anchor;
      if (obj_valid(o)) phys_set_friction(o, in_num(e, nd, 1));
      return;
    }
    case OP_SET_BOUNCINESS: {
      int o = connected(nd, 0) ? in_obj(e, nd, 0) : e->anchor;
      if (obj_valid(o)) phys_set_bounce(o, in_num(e, nd, 1));
      return;
    }
    case OP_ADD_CONSTRAINT: {
      int base = in_obj(e, nd, 0), part = in_obj(e, nd, 1);
      vec3 pivot = connected(nd, 2) ? in_vec(e, nd, 2) : (obj_valid(part) ? objs[part].pos : v3(0, 0, 0));
      Val v;
      memset(&v, 0, sizeof v);
      v.t = T_CON;
      v.u.i = obj_valid(part) ? phys_add_constraint(obj_valid(base) ? base : -1, part, pivot) : -1;
#ifdef HOST
      if (getenv("ND_CDBG"))
        fprintf(stderr, "constraint base=%d part=%d pivot=(%.2f %.2f %.2f) partpos=(%.2f %.2f %.2f)\n", base, part, pivot.x, pivot.y,
                pivot.z, obj_valid(part) ? objs[part].pos.x : 0, obj_valid(part) ? objs[part].pos.y : 0,
                obj_valid(part) ? objs[part].pos.z : 0);
#endif
      set_slot(e, ni, 0, &v);
      return;
    }
    case OP_LINEAR_LIMITS: case OP_ANGULAR_LIMITS: {
      int c = in_obj(e, nd, 0);
      if (c >= 0) phys_con_limits(c, op == OP_ANGULAR_LIMITS, in_vec(e, nd, 1), in_vec(e, nd, 2));
      return;
    }
    case OP_LINEAR_SPRING: case OP_ANGULAR_SPRING: {
      int c = in_obj(e, nd, 0);
      if (c >= 0) phys_con_spring(c, op == OP_ANGULAR_SPRING, in_vec(e, nd, 1), in_vec(e, nd, 2));
      return;
    }
    case OP_LINEAR_MOTOR: case OP_ANGULAR_MOTOR: {
      int c = in_obj(e, nd, 0);
      if (c >= 0) phys_con_motor(c, op == OP_ANGULAR_MOTOR, in_vec(e, nd, 1), in_vec(e, nd, 2));
      return;
    }
    case OP_PLAY_SOUND: {
      static int channel;
      Val v;
      memset(&v, 0, sizeof v);
      v.t = T_NUM;
      v.u.f = (float)(channel++ & 7);
      set_slot(e, ni, 0, &v);
      return;
    }
    case OP_RANDOM_SEED: rng_state = (uint32_t)in_num(e, nd, 0) * 2654435761u + 1; return;
    default: return; /* sound, inspect */
  }
}

static void run_list(Env *e, const uint8_t *lst) {
  int n = lst[0];
  for (int i = 0; i < n; i++) run_chain(e, rd16(lst + 1 + 2 * i));
}

static void run_chain(Env *e, uint16_t node) {
  while (node != NONE16 && node < e->prog->nnodes) {
    exec_stmt(e, node);
    const uint8_t *after = node_exec(node_ptr(e->prog, node), -1);
    if (after[0] == 0) return;
    /* several wires from one output run one after the other, in block order */
    for (int i = 0; i + 1 < after[0]; i++) run_chain(e, rd16(after + 1 + 2 * i));
    node = rd16(after + 1 + 2 * (after[0] - 1));
  }
}

/* ------------------------------------------------------------------ setup */
extern int world_add_template(Shape *s);

static int count_envs(Prog *p) {
  int n = 1;
  for (int i = 0; i < p->nnodes; i++)
    if (prog_child(p, i)) n += count_envs(prog_child(p, i));
  return n;
}

static bool make_env(Prog *p, int parent, int pnode, int bx, int by, int bz) {
  int idx = nenvs++;
  Env *e = &envs[idx];
  memset(e, 0, sizeof *e);
  e->prog = p;
  e->parent = parent < 0 ? NONE16 : parent;
  e->pnode = pnode;
  e->bx = bx;
  e->by = by;
  e->bz = bz;
  e->anchor = -1;
  if (parent >= 0) {
    Env *pe = &envs[parent];
    Prog *pp = pe->prog;
    e->tbase = nobj;
    for (int t = 0; t < p->ntmpl; t++)
      if (world_add_template(p->tmpl[t]) < 0) return false;
    /* anchor and self objects, resolved by the pack tool as indices in the parent grid */
    const uint8_t *nd = node_ptr(pp, pnode);
    const uint8_t *ob = nd + 10 + 2 * nd[9];
    uint16_t a = rd16(ob);
    e->anchor = a == NONE16 ? -1 : pe->tbase + a;
    e->selfobj = arena_alloc(2 * (p->nself ? p->nself : 1));
    if (!e->selfobj) return false;
    for (int k = 0; k < p->nself; k++) {
      uint16_t o = k < ob[2] ? rd16(ob + 3 + 2 * k) : NONE16;
      e->selfobj[k] = o == NONE16 ? -1 : pe->tbase + o;
    }
  }
  e->locals = arena_alloc(sizeof(VarStore) * (p->nlocals ? p->nlocals : 1));
  if (!e->locals) return false;
  for (int k = 0; k < p->nlocals; k++) {
    e->locals[k].type = p->ltypes[k];
    e->locals[k].len = e->locals[k].cap2 = 0;
    e->locals[k].data = 0;
  }
  if (p->nslots) {
    e->slots = arena_alloc(sizeof(Val) * p->nslots);
    if (!e->slots) return false;
    for (int k = 0; k < p->nslots; k++) val_default(&e->slots[k], T_NUM);
  }
  for (int i = 0; i < p->nnodes; i++) {
    Prog *ch = prog_child(p, i);
    if (!ch) continue;
    const uint8_t *nd = node_ptr(p, i);
    if (!make_env(ch, idx, i, rd16(nd + 3), rd16(nd + 5), rd16(nd + 7))) return false;
  }
  envs[idx].nsub = (uint16_t)(nenvs - idx);
  return true;
}

bool vm_setup_envs(void) {
  nfree = 0;
  int n = count_envs(level.prog);
  envs = arena_alloc(sizeof(Env) * n);
  globals = arena_alloc(sizeof(VarStore) * (nglobals ? nglobals : 1));
  if (!envs || !globals) return false;
  for (int i = 0; i < nglobals; i++) {
    globals[i].type = global_types[i];
    globals[i].len = globals[i].cap2 = 0;
    globals[i].data = 0;
  }
  nenvs = 0;
  vm_frame_count = 0;
  nlate = 0;
  rng_state = 0x12345678u ^ plat_random();
  return make_env(level.prog, -1, NONE16, 0, 0, 0);
}

/* Fancade runs the script chains of a program and the custom script blocks placed in it
 * interleaved, in block order (nodes are sorted that way); child environments follow their
 * parent in node order, each followed by its own subtree */
static void run_env(int ei) {
  Env *e = &envs[ei];
  const Prog *p = e->prog;
  int child = ei + 1, k = 0;
  for (int i = 0; i < p->nnodes; i++) {
    const uint8_t *nd = node_ptr(p, i);
    if (nd[0] == OP_CUSTOM) {
      if (child < nenvs && envs[child].parent == ei && envs[child].pnode == i) {
        int n = envs[child].nsub;
        run_env(child);
        child += n;
      }
      continue;
    }
    for (int j = k; j < p->nentries; j++)
      if (rd16(p->entries + 2 * j) == i) {
        run_chain(e, (uint16_t)i);
        if (j == k) k++;
        break;
      }
  }
}

void vm_frame(void) {
  button_counter = 0;
  nlate = 0;
  if (nenvs) run_env(0);
}

void vm_late(void) {
  for (int i = 0; i < nlate; i++) run_chain(&envs[lateq[i].env], lateq[i].node);
  nlate = 0;
  vm_frame_count++;
}

/* --------------------------------------------------------- engine access */
extern const char *vm_global_names[];
int vm_global_count(void);
