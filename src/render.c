/* Strip-based z-buffered renderer for Fancade voxel objects (orthographic camera). */
#include <string.h>
#include <math.h>
#ifdef HOST
#include <stdio.h>
#include <stdlib.h>
#endif
#include "render.h"
#include "world.h"
#include "platform.h"
#include "vm.h"
#include "physics.h"

const uint8_t palette_rgb[34][3] = {
    {0, 0, 0}, {29, 29, 40}, {63, 63, 80}, {101, 103, 121}, {144, 147, 164}, {191, 194, 205}, {255, 255, 255},
    {148, 83, 94}, {192, 109, 128}, {225, 153, 152}, {253, 181, 168}, {255, 219, 197}, {255, 243, 242},
    {192, 54, 80}, {255, 74, 104}, {255, 154, 154}, {201, 80, 64}, {255, 112, 83}, {255, 168, 129},
    {232, 168, 0}, {255, 213, 0}, {255, 255, 122}, {0, 136, 76}, {66, 195, 79}, {181, 255, 110},
    {0, 108, 218}, {0, 142, 255}, {0, 193, 255}, {104, 83, 162}, {140, 111, 217}, {176, 138, 255},
    {235, 112, 171}, {255, 149, 206}, {255, 185, 232}};

Camera cam;
uint16_t sky565;
static vec3 light_to; /* direction towards the light */
static const float AMB[3] = {0.764f, 0.773f, 0.839f};
static const float LEFF[3] = {0.444f, 0.423f, 0.304f};

static uint16_t cbuf[SCREEN_W * STRIP_H] __attribute__((aligned(4)));
static uint16_t zbuf[SCREEN_W * STRIP_H] __attribute__((aligned(4)));
typedef uint32_t __attribute__((may_alias)) u32a;
static int strip_y0, strip_y1; /* current strip rows [y0, y1) */
static int clip_x0, clip_x1;
/* per object, per frame: screen rows covered and part cache offset */
typedef struct {
  int16_t ymin, ymax;
  uint16_t pcache;
} RObj;
static RObj *robj;

uint16_t rgb565(int r, int g, int b) {
  if (r > 255) r = 255;
  if (g > 255) g = 255;
  if (b > 255) b = 255;
  if (r < 0) r = 0;
  if (g < 0) g = 0;
  if (b < 0) b = 0;
  return (uint16_t)((r >> 3) << 11 | (g >> 2) << 5 | (b >> 3));
}

void shade_color(int pal, vec3 n, int *r, int *g, int *b) {
  float k = vdot(n, light_to);
  if (k < 0) k = 0;
  const uint8_t *c = palette_rgb[pal];
  *r = (int)(c[0] * (AMB[0] + k * LEFF[0]) + 0.5f);
  *g = (int)(c[1] * (AMB[1] + k * LEFF[1]) + 0.5f);
  *b = (int)(c[2] * (AMB[2] + k * LEFF[2]) + 0.5f);
}

static uint16_t axis_col[34 * 6];
static uint8_t axis_ok[34 * 6 / 8 + 1];

void light_set(const quat *rot) {
  vec3 d = qrot(*rot, v3(0, 0, 1));
  light_to = vscale(d, -1);
  memset(axis_ok, 0, sizeof axis_ok);
}

/* ------------------------------------------------------------------ camera */
void cam_default(void) {
  cam.focus = v3(0, 0, 0);
  cam.rot = qeuler(22.5f, 22.5f, 0);
  cam.range = 50;
  cam.perspective = 0;
  cam.shake = v3(0, 0, 0);
  cam.cx = SCREEN_W * 0.5f;
  cam.cy = SCREEN_H * 0.5f;
  cam.zoom_mul = 1;
  cam.roll = 0;
  quat l = qeuler(45, -45, 0);
  light_set(&l);
  cam_update();
}

void cam_update(void) {
  cam.right = qrot(cam.rot, v3(1, 0, 0));
  cam.up = qrot(cam.rot, v3(0, 1, 0));
  cam.fwd = qrot(cam.rot, v3(0, 0, 1));
  float zoom = cam.range * 0.1f;
  if (zoom < 0.1f) zoom = 0.1f;
  cam.scale = (SCREEN_H * 0.5f) / zoom * cam.zoom_mul;
  if (cam.roll != 0) {
    float c = cosf(cam.roll), sn = sinf(cam.roll);
    vec3 r = cam.right, u = cam.up;
    cam.right = vadd(vscale(r, c), vscale(u, sn));
    cam.up = vsub(vscale(u, c), vscale(r, sn));
  }
}

void cam_view(float cx, float cy, float zoom_mul, float roll) {
  cam.cx = cx;
  cam.cy = cy;
  cam.zoom_mul = zoom_mul;
  cam.roll = roll;
  cam_update();
}

void cam_set(const vec3 *pos, const quat *rot, const float *range, int perspective) {
  if (pos) cam.focus = *pos;
  if (rot) cam.rot = qnorm(*rot);
  if (range) cam.range = *range;
  cam.perspective = perspective;
  cam_update();
}

vec3 cam_world_to_screen(vec3 p) {
  vec3 d = vsub(p, cam.focus);
  return v3(cam.cx + vdot(d, cam.right) * cam.scale, cam.cy - vdot(d, cam.up) * cam.scale,
            vdot(d, cam.fwd));
}

void cam_screen_to_world(float sx, float sy, vec3 *near, vec3 *far) {
  float x = (sx - cam.cx) / cam.scale, y = (cam.cy - sy) / cam.scale;
  vec3 b = vadd(cam.focus, vadd(vscale(cam.right, x), vscale(cam.up, y)));
  *near = vsub(b, vscale(cam.fwd, 50));
  *far = vadd(b, vscale(cam.fwd, 150));
}

/* --------------------------------------------------------------- raster */
#define ZOFF 48.0f
#define ZSCALE 512.0f

static inline int iceil(float v) {
  int i = (int)v;
  return i + (v > (float)i);
}

/* Rasterise the parallelogram P0 + s*U + t*V (s, t in [0,1)) with a planar depth.
 * ds/dx, ds/dy, dt/dx, dt/dy map screen offsets from P0 to (s, t); ix* are 1/(ds/dx), 1/(dt/dx). */
typedef struct {
  float dsdx, dsdy, dtdx, dtdy, isx, itx;
} PInv;

/* Fancade's floor, an endless plane at y = 0, hides whatever is below it: while an object reaching below it is
 * drawn, world y at a pixel is fl_k + fl_x * x + fl_y * y + fl_z * depth */
static bool fl_on;
static float fl_k, fl_x, fl_y, fl_z;

static void raster(float X0, float Y0, float ux, float uy, float vx, float vy, const PInv *pi, float z0, float zx,
                   float zy, uint16_t color) {
  float ymin = Y0, ymax = Y0;
  float ya = Y0 + uy, yb = Y0 + vy, yc2 = Y0 + uy + vy;
  if (ya < ymin) ymin = ya;
  if (ya > ymax) ymax = ya;
  if (yb < ymin) ymin = yb;
  if (yb > ymax) ymax = yb;
  if (yc2 < ymin) ymin = yc2;
  if (yc2 > ymax) ymax = yc2;
  int y0 = iceil(ymin - 0.5f), y1 = iceil(ymax - 0.5f);
  if (y0 < strip_y0) y0 = strip_y0;
  if (y1 > strip_y1) y1 = strip_y1;
  if (y0 >= y1) return;
  /* depth in fixed point: Z(x,y) = (z0 + zx*(x - X0) + zy*(y - Y0) + ZOFF) * ZSCALE */
  float zbase = (z0 + ZOFF - zx * X0 - zy * Y0) * ZSCALE;
  int32_t dz = (int32_t)(zx * ZSCALE * 256.0f);
  float dy = y0 + 0.5f - Y0;
  float sr = pi->dsdy * dy, tr = pi->dtdy * dy; /* s, t at (X0, row centre) */
  for (int y = y0; y < y1; y++, sr += pi->dsdy, tr += pi->dtdy) {
    float xl, xr;
    if (pi->isx != 0) {
      float a = -sr * pi->isx, b = (1 - sr) * pi->isx;
      if (a < b) xl = a, xr = b;
      else xl = b, xr = a;
    } else {
      if (sr < 0 || sr >= 1) continue;
      xl = -1e9f, xr = 1e9f;
    }
    if (pi->itx != 0) {
      float a = -tr * pi->itx, b = (1 - tr) * pi->itx;
      if (a > b) {
        float t = a;
        a = b;
        b = t;
      }
      if (a > xl) xl = a;
      if (b < xr) xr = b;
    } else if (tr < 0 || tr >= 1) {
      continue;
    }
    if (xl >= xr) continue;
    int x0 = iceil(X0 + xl - 0.5f), x1 = iceil(X0 + xr - 0.5f);
    if (x0 < clip_x0) x0 = clip_x0;
    if (x1 > clip_x1) x1 = clip_x1;
    float yc = y + 0.5f;
    if (fl_on) {
      /* world y along the row: a + b * (pixel centre x) */
      float a = fl_k + fl_y * yc + fl_z * (z0 - zx * X0 + zy * (yc - Y0)), b = fl_x + fl_z * zx, e = -0.01f - a;
      if (b > 1e-7f) {
        int xa = iceil(e / b - 0.5f);
        if (xa > x0) x0 = xa;
      } else if (b < -1e-7f) {
        int xb = iceil(e / b - 0.5f);
        if (xb < x1) x1 = xb;
      } else if (e > 0) {
        continue;
      }
    }
    if (x0 >= x1) continue;
    float zs = zbase + (zx * (x0 + 0.5f) + zy * yc) * ZSCALE;
    if (zs < 0) zs = 0;
    int32_t z = (int32_t)(zs * 256.0f);
    int row = (y - strip_y0) * SCREEN_W;
    uint16_t *c = cbuf + row + x0, *ce = cbuf + row + x1;
    uint16_t *zp = zbuf + row + x0;
    int32_t zend = z + dz * (x1 - x0 - 1);
    if (z >= 0 && zend >= 0 && z < (65535 << 8) && zend < (65535 << 8)) {
      for (; c < ce; c++, zp++, z += dz) {
        uint16_t zz = (uint16_t)(z >> 8);
        if (zz <= *zp) {
          *zp = zz;
          *c = color;
        }
      }
    } else {
      for (; c < ce; c++, zp++, z += dz) {
        int32_t zz = z >> 8;
        if (zz < 0) zz = 0;
        if (zz > 65535) zz = 65535;
        if ((uint16_t)zz <= *zp) {
          *zp = (uint16_t)zz;
          *c = color;
        }
      }
    }
  }
}

/* --------------------------------------------------------- object drawing */
typedef struct {
  float ox, oy, oz;      /* screen x, y, depth of rest origin (0,0,0) */
  float bx[3], by[3], bz[3]; /* screen delta per rest unit along x,y,z */
  uint8_t vis;           /* visible local face directions */
  uint16_t col[34 * 6];  /* shaded colours cache: filled lazily */
  uint8_t colok[34 * 6 / 8 + 1];
  vec3 n[6];
  float zx[6], zy[6];
  float fa[6], fb[6], fc[6], fd[6], fia[6], fic[6]; /* per face: (s,t) per screen unit, for 1 voxel quads */
  uint8_t ident;                                  /* unrotated: shared colour cache */
} ObjXf;

static ObjXf xf;

static void setup_xf(const Obj *o) {
  const Shape *s = o->shape;
  vec3 ax[3] = {qrot(o->rot, v3(1, 0, 0)), qrot(o->rot, v3(0, 1, 0)), qrot(o->rot, v3(0, 0, 1))};
  for (int k = 0; k < 3; k++) {
    xf.bx[k] = vdot(ax[k], cam.right) * cam.scale;
    xf.by[k] = -vdot(ax[k], cam.up) * cam.scale;
    xf.bz[k] = vdot(ax[k], cam.fwd);
  }
  /* origin of rest space: world = pos + R*(rest - com)  => rest 0 maps to pos - R*com */
  vec3 w0 = vsub(o->pos, qrot(o->rot, s->origin));
  vec3 d = vsub(w0, vadd(cam.focus, cam.shake));
  xf.ox = cam.cx + vdot(d, cam.right) * cam.scale;
  xf.oy = cam.cy - vdot(d, cam.up) * cam.scale;
  xf.oz = vdot(d, cam.fwd);
  {
    vec3 lo = vsub(s->bmin, s->origin), hi = vsub(s->bmax, s->origin);
    vec3 m = v3(fmaxf(-lo.x, hi.x), fmaxf(-lo.y, hi.y), fmaxf(-lo.z, hi.z));
    fl_on = o->pos.y * o->pos.y < vdot(m, m) || o->pos.y < 0;
    vec3 f = vadd(cam.focus, cam.shake);
    fl_x = cam.right.y / cam.scale;
    fl_y = -cam.up.y / cam.scale;
    fl_z = cam.fwd.y;
    fl_k = f.y - fl_x * cam.cx - fl_y * cam.cy;
  }
  xf.vis = 0;
  for (int f = 0; f < 6; f++) {
    vec3 n = vscale(ax[f >> 1], (f & 1) ? -1.0f : 1.0f);
    xf.n[f] = n;
    float nf = vdot(n, cam.fwd);
    if (nf < -1e-4f) {
      int axis = f >> 1, ua = axis == 0 ? 1 : 0, va = axis == 2 ? 1 : 2;
      float eux = xf.bx[ua] * 0.125f, euy = xf.by[ua] * 0.125f, evx = xf.bx[va] * 0.125f, evy = xf.by[va] * 0.125f;
      float det = eux * evy - euy * evx;
      if (fabsf(det) < 1e-6f) continue;
      float id = 1.0f / det;
      xf.fa[f] = evy * id;
      xf.fb[f] = -evx * id;
      xf.fc[f] = -euy * id;
      xf.fd[f] = eux * id;
      xf.fia[f] = fabsf(xf.fa[f]) > 1e-6f ? 1.0f / xf.fa[f] : 0;
      xf.fic[f] = fabsf(xf.fc[f]) > 1e-6f ? 1.0f / xf.fc[f] : 0;
      xf.vis |= 1 << f;
      float nr = vdot(n, cam.right), nu = vdot(n, cam.up);
      xf.zx[f] = -nr / (cam.scale * nf);
      xf.zy[f] = nu / (cam.scale * nf);
    }
  }
  xf.ident = fabsf(o->rot.x) + fabsf(o->rot.y) + fabsf(o->rot.z) < 1e-5f;
  if (!xf.ident) memset(xf.colok, 0, sizeof xf.colok);
}

static uint16_t face_color(int pal, int f) {
  int i = pal * 6 + f;
  if (xf.ident) {
    if (!(axis_ok[i >> 3] & (1 << (i & 7)))) {
      int r, g, b;
      shade_color(pal, xf.n[f], &r, &g, &b);
      axis_col[i] = rgb565(r, g, b);
      axis_ok[i >> 3] |= 1 << (i & 7);
    }
    return axis_col[i];
  }
  if (!(xf.colok[i >> 3] & (1 << (i & 7)))) {
    int r, g, b;
    shade_color(pal, xf.n[f], &r, &g, &b);
    xf.col[i] = rgb565(r, g, b);
    xf.colok[i >> 3] |= 1 << (i & 7);
  }
  return xf.col[i];
}

/* project rest-space point given in voxel units */
static inline void proj(float vx, float vy, float vz, float *sx, float *sy, float *sz) {
  vx *= 0.125f;
  vy *= 0.125f;
  vz *= 0.125f;
  *sx = xf.ox + xf.bx[0] * vx + xf.bx[1] * vy + xf.bx[2] * vz;
  *sy = xf.oy + xf.by[0] * vx + xf.by[1] * vy + xf.by[2] * vz;
  *sz = xf.oz + xf.bz[0] * vx + xf.bz[1] * vy + xf.bz[2] * vz;
}

static const float inv_n[9] = {0, 1, 0.5f, 1.0f / 3, 0.25f, 0.2f, 1.0f / 6, 1.0f / 7, 0.125f};

static void draw_part(const Shape *s, int pi) {
  uint32_t k = s->key[pi];
  const Block *b = blocks[s->blk[pi]];
  int cx = PK_X(k) * 8, cy = PK_Y(k) * 8, cz = PK_Z(k) * 8, comp = PK_C(k);
  /* quick strip reject using the cell centre (rows first) */
  float r = 0.9f * cam.scale;
  float vx = (cx + 4) * 0.125f, vy = (cy + 4) * 0.125f, vz = (cz + 4) * 0.125f;
  float sy = xf.oy + xf.by[0] * vx + xf.by[1] * vy + xf.by[2] * vz;
  if (sy + r < strip_y0 || sy - r > strip_y1) return;
  float sx = xf.ox + xf.bx[0] * vx + xf.bx[1] * vy + xf.bx[2] * vz;
  if (sx + r < clip_x0 || sx - r > clip_x1) return;
  uint8_t occ = s->occ ? s->occ[pi] : 0;
  for (int f = 0; f < 6; f++) {
    if (!(xf.vis & (1 << f)) || !b->nq[f]) continue;
    const uint8_t *q = b->fq[f];
    int axis = f >> 1, ua = axis == 0 ? 1 : 0, va = axis == 2 ? 1 : 2;
    int pos = !(f & 1);
    float eux = xf.bx[ua] * 0.125f, euy = xf.by[ua] * 0.125f, evx = xf.bx[va] * 0.125f, evy = xf.by[va] * 0.125f;
    for (int i = 0; i < b->nq[f]; i++, q += 3) {
      uint32_t w = q[0] | q[1] << 8 | (uint32_t)q[2] << 16;
      int layer = w & 7, u = (w >> 3) & 7, v = (w >> 6) & 7, du = ((w >> 9) & 7) + 1, dv = ((w >> 12) & 7) + 1;
      int col = (w >> 15) & 63, qc = (w >> 21) & 7;
      if (qc != comp) continue;
      if ((occ & (1 << f)) && layer == (pos ? 7 : 0)) continue;
      float p[3];
      p[axis] = (float)(layer + pos);
      p[ua] = (float)u;
      p[va] = (float)v;
      float X, Y, Z;
      proj(cx + p[0], cy + p[1], cz + p[2], &X, &Y, &Z);
      PInv pv;
      float iu = inv_n[du], iv = inv_n[dv];
      pv.dsdx = xf.fa[f] * iu;
      pv.dsdy = xf.fb[f] * iu;
      pv.dtdx = xf.fc[f] * iv;
      pv.dtdy = xf.fd[f] * iv;
      pv.isx = xf.fia[f] * du;
      pv.itx = xf.fic[f] * dv;
      raster(X, Y, eux * du, euy * du, evx * dv, evy * dv, &pv, Z, xf.zx[f], xf.zy[f], face_color(col, f));
    }
  }
}

/* per frame cache of the strips each part may touch (one byte per part: first << 4 | last), kept in
 * the free arena space which is untouched while rendering */
static uint8_t *pc_base;
static uint32_t pc_avail, pc_used;


static void draw_object(Obj *ob, RObj *o) {
  setup_xf(ob);
  const Shape *s = ob->shape;
  if (o->pcache == 0xFFFF && pc_used + s->np <= pc_avail && pc_used + s->np < 0xFFFF) {
    uint8_t *pc = pc_base + pc_used;
    float r = 0.9f * cam.scale;
    for (int i = 0; i < s->np; i++) {
      uint32_t k = s->key[i];
      float vy = PK_Y(k) + 0.5f, vx = PK_X(k) + 0.5f, vz = PK_Z(k) + 0.5f;
      float sy = xf.oy + xf.by[0] * vx + xf.by[1] * vy + xf.by[2] * vz;
      int a = (int)floorf((sy - r) * (1.0f / STRIP_H)), b = (int)floorf((sy + r) * (1.0f / STRIP_H));
      if (b < 0 || a > SCREEN_H / STRIP_H - 1) {
        pc[i] = 0xF0;
        continue;
      }
      if (a < 0) a = 0;
      if (b > 15) b = 15;
      pc[i] = (uint8_t)(a << 4 | b);
    }
    o->pcache = (uint16_t)pc_used;
    pc_used += s->np;
  }
  if (o->pcache != 0xFFFF) {
    const uint8_t *pc = pc_base + o->pcache;
    int st = strip_y0 / STRIP_H;
    for (int i = 0; i < s->np; i++)
      if (st >= (pc[i] >> 4) && st <= (pc[i] & 15)) draw_part(s, i);
  } else {
    for (int i = 0; i < s->np; i++) draw_part(s, i);
  }
  fl_on = false;
}

/* ------------------------------------------------------------------ frame */
static bool obj_screen_bounds(const Obj *o, int *y0, int *y1, int *x0, int *x1);

bool render_init_level(void) {
  robj = arena_alloc(sizeof(RObj) * (obj_cap ? obj_cap : 1));
  if (!robj) return false;
  int r = palette_rgb[level.bg][0], g = palette_rgb[level.bg][1], b = palette_rgb[level.bg][2];
  r = r + (255 - r) * 2 / 5;
  g = g + (255 - g) * 2 / 5;
  b = b + (255 - b) * 2 / 5;
  /* the background of the levels built on Fancade's floor, as seen in the original */
  static const uint8_t floor_bg[][4] = {
      {2, 63, 63, 77}, {4, 132, 135, 153}, {5, 172, 175, 191}, {7, 89, 58, 73}, {8, 150, 87, 102}, {22, 18, 114, 76}};
  for (unsigned i = 0; i < sizeof floor_bg / sizeof floor_bg[0]; i++)
    if (floor_bg[i][0] == level.bg) r = floor_bg[i][1], g = floor_bg[i][2], b = floor_bg[i][3];
  sky565 = rgb565(r, g, b);
  return true;
}

static bool obj_screen_bounds(const Obj *o, int *y0, int *y1, int *x0, int *x1) {
  const Shape *s = o->shape;
  float mnx = 1e9f, mxx = -1e9f, mny = 1e9f, mxy = -1e9f;
  for (int c = 0; c < 8; c++) {
    vec3 pt = v3(c & 1 ? s->bmax.x : s->bmin.x, c & 2 ? s->bmax.y : s->bmin.y, c & 4 ? s->bmax.z : s->bmin.z);
    vec3 w = obj_world(o, pt);
    vec3 d = vsub(w, vadd(cam.focus, cam.shake));
    float sx = cam.cx + vdot(d, cam.right) * cam.scale;
    float sy = cam.cy - vdot(d, cam.up) * cam.scale;
    if (sx < mnx) mnx = sx;
    if (sx > mxx) mxx = sx;
    if (sy < mny) mny = sy;
    if (sy > mxy) mxy = sy;
  }
  *x0 = (int)mnx - 1;
  *x1 = (int)mxx + 1;
  *y0 = (int)mny - 1;
  *y1 = (int)mxy + 1;
  return !(mxx < 0 || mnx > SCREEN_W || mxy < 0 || mny > SCREEN_H);
}


/* ------------------------------------------------------------------ shadows */
/* Moving objects (and the props following them, like the tyres) cast Fancade style shadows on the
 * ground below: the box faces lit by the sun are projected along the light onto the plane found by a
 * raycast, and ground pixels at that depth are darkened to their ambient only colour. */
#define MAX_CASTERS 32
#define SHADOW_TOL 40 /* depth units */
static struct {
  int16_t obj, floor;
  float ground;
} casters[MAX_CASTERS];
static int ncasters;
static uint8_t smask[SCREEN_W * STRIP_H / 8];
typedef struct {
  float X, Y, ux, uy, vx, vy, z;
  int16_t y0, y1;
  uint8_t floor; /* on Fancade's floor: also darkens the background */
} SQuad;
static SQuad *squads; /* projected shadow faces for this frame (scratch) */
static int nsquads, squad_cap;
#ifdef HOST
#include <stdio.h>
#include <stdlib.h>
static int st_sq_max, st_sq_drop, st_sq_cap = 1 << 30;
static void sq_dump(void) { printf("SQUADS max %d dropped %d mincap %d\n", st_sq_max, st_sq_drop, st_sq_cap); }
#endif
static float sh_zx, sh_zy; /* screen depth gradient of a horizontal plane */
static bool sh_any;

static void find_casters(void) {
  ncasters = 0;
  vec3 d = vscale(light_to, -1);
  if (d.y > -0.05f) return;
  vec3 n = v3(0, 1, 0);
  float nf = vdot(n, cam.fwd);
  if (fabsf(nf) < 1e-4f) return;
  sh_zx = -vdot(n, cam.right) / (cam.scale * nf);
  sh_zy = vdot(n, cam.up) / (cam.scale * nf);
  /* dynamic objects first, then small moved props close to them */
  int ndyn = 0;
  for (int pass = 0; pass < 2; pass++, ndyn = ncasters)
    for (int i = 0; i < nobj && ncasters < MAX_CASTERS; i++) {
      const Obj *o = &objs[i];
      if (!(o->flags & OF_VISIBLE) || (o->flags & (OF_DEAD | OF_TEMPLATE)) || robj[i].ymin > robj[i].ymax) continue;
      if (pass == 0 && !(o->flags & OF_DYNAMIC)) continue;
      if (pass == 1) {
        if ((o->flags & OF_DYNAMIC) || !(o->flags & OF_MOVED) || o->shape->np > 48) continue;
        bool near = false;
        for (int c = 0; c < ndyn && !near; c++) {
          const Obj *q = &objs[casters[c].obj];
          float dx = q->pos.x - o->pos.x, dy = q->pos.y - o->pos.y;
          near = dx * dx + dy * dy < 9;
        }
        if (!near) continue;
      }
      const Shape *s = o->shape;
      vec3 from = obj_world(o, vscale(vadd(s->bmin, s->bmax), 0.5f)), hit;
      int ho;
      bool fl = false;
      if (!phys_raycast_ex(from, v3(from.x, from.y - 12, from.z), &hit, &ho, i)) {
        if (from.y > 12 || from.y < 0) continue;
        hit.y = 0; /* Fancade's floor */
        fl = true;
      }
      casters[ncasters].obj = (int16_t)i;
      casters[ncasters].floor = fl;
      casters[ncasters].ground = hit.y;
      ncasters++;
    }
}

/* mark pixels of the parallelogram whose stored depth matches the plane */
static void raster_mark(float X0, float Y0, float ux, float uy, float vx, float vy, float z0, bool floor) {
  float det = ux * vy - uy * vx;
  if (fabsf(det) < 1e-3f) return;
  float id = 1.0f / det;
  PInv pv;
  pv.dsdx = vy * id;
  pv.dsdy = -vx * id;
  pv.dtdx = -uy * id;
  pv.dtdy = ux * id;
  pv.isx = fabsf(pv.dsdx) > 1e-6f ? 1.0f / pv.dsdx : 0;
  pv.itx = fabsf(pv.dtdx) > 1e-6f ? 1.0f / pv.dtdx : 0;
  const PInv *pi = &pv;
  float ymin = Y0, ymax = Y0;
  float ya = Y0 + uy, yb = Y0 + vy, yc2 = Y0 + uy + vy;
  if (ya < ymin) ymin = ya;
  if (ya > ymax) ymax = ya;
  if (yb < ymin) ymin = yb;
  if (yb > ymax) ymax = yb;
  if (yc2 < ymin) ymin = yc2;
  if (yc2 > ymax) ymax = yc2;
  int y0 = iceil(ymin - 0.5f), y1 = iceil(ymax - 0.5f);
  if (y0 < strip_y0) y0 = strip_y0;
  if (y1 > strip_y1) y1 = strip_y1;
  if (y0 >= y1) return;
  float zbase = (z0 + ZOFF - sh_zx * X0 - sh_zy * Y0) * ZSCALE;
  float dy = y0 + 0.5f - Y0;
  float sr = pi->dsdy * dy, tr = pi->dtdy * dy;
  for (int y = y0; y < y1; y++, sr += pi->dsdy, tr += pi->dtdy) {
    float xl, xr;
    if (pi->isx != 0) {
      float a = -sr * pi->isx, b = (1 - sr) * pi->isx;
      if (a < b) xl = a, xr = b;
      else xl = b, xr = a;
    } else {
      if (sr < 0 || sr >= 1) continue;
      xl = -1e9f, xr = 1e9f;
    }
    if (pi->itx != 0) {
      float a = -tr * pi->itx, b = (1 - tr) * pi->itx;
      if (a > b) {
        float t = a;
        a = b;
        b = t;
      }
      if (a > xl) xl = a;
      if (b < xr) xr = b;
    } else if (tr < 0 || tr >= 1) {
      continue;
    }
    if (xl >= xr) continue;
    int x0 = iceil(X0 + xl - 0.5f), x1 = iceil(X0 + xr - 0.5f);
    if (x0 < clip_x0) x0 = clip_x0;
    if (x1 > clip_x1) x1 = clip_x1;
    if (x0 >= x1) continue;
    float zs = zbase + (sh_zx * (x0 + 0.5f) + sh_zy * (y + 0.5f)) * ZSCALE;
    float dz = sh_zx * ZSCALE;
    int row = (y - strip_y0) * SCREEN_W;
    const uint16_t *zp = zbuf + row;
    for (int x = x0; x < x1; x++, zs += dz) {
      int d = (int)zs - zp[x];
      if ((d <= SHADOW_TOL && d >= -SHADOW_TOL) || (floor && zp[x] == 0xFFFF)) smask[(row + x) >> 3] |= (uint8_t)(1 << ((row + x) & 7));
    }
    sh_any = true;
  }
}

/* project a lit face (corner p, edges u, v in world space) onto the plane y = g */
static void shadow_face(vec3 p, vec3 u, vec3 v, float g, vec3 d, bool floor) {
  float k = (g - p.y) / d.y;
#ifdef HOST
  if (k >= 0 && nsquads >= squad_cap) st_sq_drop++;
#endif
  if (k < 0 || nsquads >= squad_cap) return; /* face below the ground plane */
  vec3 p0 = vadd(p, vscale(d, k));
  vec3 u0 = vsub(u, vscale(d, u.y / d.y)), v0 = vsub(v, vscale(d, v.y / d.y));
  vec3 q = vsub(p0, vadd(cam.focus, cam.shake));
  SQuad *sq = &squads[nsquads];
  sq->X = cam.cx + vdot(q, cam.right) * cam.scale;
  sq->Y = cam.cy - vdot(q, cam.up) * cam.scale;
  sq->z = vdot(q, cam.fwd) - 0.01f;
  sq->ux = vdot(u0, cam.right) * cam.scale;
  sq->uy = -vdot(u0, cam.up) * cam.scale;
  sq->vx = vdot(v0, cam.right) * cam.scale;
  sq->vy = -vdot(v0, cam.up) * cam.scale;
  float ymin = sq->Y + fminf(0, sq->uy) + fminf(0, sq->vy), ymax = sq->Y + fmaxf(0, sq->uy) + fmaxf(0, sq->vy);
  if (ymax < 0 || ymin > SCREEN_H) return;
  sq->y0 = (int16_t)(ymin - 1);
  sq->y1 = (int16_t)(ymax + 1);
  sq->floor = floor;
  nsquads++;
}

/* project the shadows of all casters once per frame */
static void build_shadows(void) {
#ifdef HOST
  if (nsquads > st_sq_max) st_sq_max = nsquads;
#endif
  nsquads = 0;
  if (!ncasters) return;
  vec3 d = vscale(light_to, -1);
  for (int c = 0; c < ncasters; c++) {
    const Obj *o = &objs[casters[c].obj];
    const Shape *s = o->shape;
    float g = casters[c].ground;
    vec3 ax[3] = {qrot(o->rot, v3(1, 0, 0)), qrot(o->rot, v3(0, 1, 0)), qrot(o->rot, v3(0, 0, 1))};
    bool lit[6];
    for (int f = 0; f < 6; f++) lit[f] = vdot(ax[f >> 1], light_to) * ((f & 1) ? -1.0f : 1.0f) > 1e-3f;
    for (int i = 0; i < s->np; i++) {
      uint32_t k = s->key[i];
      const Block *b = blocks[s->blk[i]];
      int comp = PK_C(k);
      uint8_t occ = s->occ ? s->occ[i] : 0;
      /* use the collision boxes, or the component bounds when the block has none */
      int nb = b->nbox;
      for (int j = 0; j < (nb ? nb : 1); j++) {
        float lo[3], hi[3];
        if (nb) {
          const uint8_t *bx = b->boxes + 3 * j;
          uint32_t w = bx[0] | bx[1] << 8 | (uint32_t)bx[2] << 16;
          if ((int)(w & 7) != comp) continue;
          lo[0] = ((w >> 3) & 7), lo[1] = ((w >> 6) & 7), lo[2] = ((w >> 9) & 7);
          hi[0] = ((w >> 12) & 7) + 1, hi[1] = ((w >> 15) & 7) + 1, hi[2] = ((w >> 18) & 7) + 1;
        } else {
          const uint8_t *bb = b->bb + comp * 6;
          if (bb[0] > bb[3]) continue;
          lo[0] = bb[0], lo[1] = bb[1], lo[2] = bb[2];
          hi[0] = bb[3] + 1, hi[1] = bb[4] + 1, hi[2] = bb[5] + 1;
        }
        vec3 rest = v3(PK_X(k) + lo[0] / 8, PK_Y(k) + lo[1] / 8, PK_Z(k) + lo[2] / 8);
        vec3 p0 = obj_world(o, rest);
        vec3 e[3] = {vscale(ax[0], (hi[0] - lo[0]) / 8), vscale(ax[1], (hi[1] - lo[1]) / 8),
                     vscale(ax[2], (hi[2] - lo[2]) / 8)};
        for (int f = 0; f < 6; f++) {
          if (!lit[f]) continue;
          int a = f >> 1, ua = a == 0 ? 1 : 0, va = a == 2 ? 1 : 2;
          /* a face against a full face of the same object adds nothing to its shadow */
          if ((occ >> f & 1) && ((f & 1) ? lo[a] == 0 : hi[a] == 8)) continue;
          vec3 p = (f & 1) ? p0 : vadd(p0, e[a]);
          shadow_face(p, e[ua], e[va], g, d, casters[c].floor);
        }
      }
    }
  }
}

static void draw_shadows(void) {
  if (!nsquads) return;
  memset(smask, 0, sizeof smask);
  sh_any = false;
  for (int i = 0; i < nsquads; i++) {
    const SQuad *q = &squads[i];
    if (q->y1 < strip_y0 || q->y0 >= strip_y1) continue;
    raster_mark(q->X, q->Y, q->ux, q->uy, q->vx, q->vy, q->z, q->floor);
  }
  if (!sh_any) return;
  int n = (strip_y1 - strip_y0) * SCREEN_W;
  for (int i = 0; i < n; i += 8) {
    uint8_t m = smask[i >> 3];
    if (!m) continue;
    for (int k = 0; k < 8; k++)
      if (m & (1 << k)) {
        uint16_t c = cbuf[i + k];
        int r = (c >> 11) * 23 >> 5, gg = ((c >> 5) & 63) * 23 >> 5, bl = (c & 31) * 26 >> 5;
        cbuf[i + k] = (uint16_t)(r << 11 | gg << 5 | bl);
      }
  }
}

void render_prepare(int rx0, int rx1) {
  uint32_t avail;
  uint8_t *scr = arena_top(&avail);
  squads = (SQuad *)scr;
  squad_cap = (int)((avail / 3) / sizeof(SQuad)); /* at most a third of the free space */
  if (squad_cap > 512) squad_cap = 512;
#ifdef HOST
  {
    static int reg;
    if (!reg++ && getenv("ND_STATS")) atexit(sq_dump);
    if (squad_cap < st_sq_cap) st_sq_cap = squad_cap;
  }
#endif
  uint32_t used = (uint32_t)squad_cap * sizeof(SQuad);
  pc_base = scr + used;
  pc_avail = avail - used;
  pc_used = 0;
  for (int i = 0; i < nobj; i++) {
    Obj *o = &objs[i];
    RObj *r = &robj[i];
    r->pcache = 0xFFFF;
    r->ymin = 1;
    r->ymax = 0;
    if (!(o->flags & OF_VISIBLE) || (o->flags & OF_DEAD) || !o->shape->np) continue;
    int y0, y1, x0, x1;
    if (!obj_screen_bounds(o, &y0, &y1, &x0, &x1)) continue;
    if (x1 < rx0 || x0 > rx1) continue;
    r->ymin = (int16_t)(y0 < -32000 ? -32000 : y0);
    r->ymax = (int16_t)(y1 > 32000 ? 32000 : y1);
  }
  find_casters();
  build_shadows();
}

uint16_t *render_buffer(void) { return cbuf; }
void *render_scratch(void) { return zbuf; } /* depth buffer, free between frames */

uint16_t *render_strip(int sy, int n, int x0, int x1) {
  strip_y0 = sy;
  strip_y1 = sy + n;
  clip_x0 = x0;
  clip_x1 = x1;
  if (x0 == 0 && x1 == SCREEN_W) {
    /* whole rows: fill two pixels per word */
    uint32_t sky2 = sky565 | (uint32_t)sky565 << 16;
    u32a *c = (u32a *)cbuf, *z = (u32a *)zbuf;
    for (int i = 0; i < n * SCREEN_W / 2; i++) c[i] = sky2, z[i] = 0xFFFFFFFFu;
  } else {
    for (int r = 0; r < n; r++) {
      uint16_t *c = cbuf + r * SCREEN_W, *z = zbuf + r * SCREEN_W;
      for (int x = x0; x < x1; x++) {
        c[x] = sky565;
        z[x] = 0xFFFF;
      }
    }
  }
  for (int i = 0; i < nobj; i++) {
    RObj *r = &robj[i];
    if (r->ymin > r->ymax || r->ymax < strip_y0 || r->ymin >= strip_y1) continue;
    draw_object(&objs[i], r);
  }
  draw_shadows();
  return cbuf;
}

void render_frame(void (*post)(uint16_t *px, int y, int n)) {
  render_prepare(0, SCREEN_W);
  for (int sy = 0; sy < SCREEN_H; sy += STRIP_H) {
    int n = SCREEN_H - sy < STRIP_H ? SCREEN_H - sy : STRIP_H;
    render_strip(sy, n, 0, SCREEN_W);
    if (post) post(cbuf, sy, n);
    plat_push(0, sy, SCREEN_W, n, cbuf);
  }
}
