/* Strip-based z-buffered renderer for Fancade voxel objects (orthographic camera). */
#include <string.h>
#include <math.h>
#include "render.h"
#include "world.h"
#include "platform.h"
#include "vm.h"

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

static uint16_t cbuf[SCREEN_W * STRIP_H];
static uint16_t zbuf[SCREEN_W * STRIP_H];
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
    if (x0 >= x1) continue;
    float yc = y + 0.5f;
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


void render_prepare(int rx0, int rx1) {
  pc_base = arena_top(&pc_avail);
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
}

uint16_t *render_buffer(void) { return cbuf; }
void *render_scratch(void) { return zbuf; } /* depth buffer, free between frames */

uint16_t *render_strip(int sy, int n, int x0, int x1) {
  strip_y0 = sy;
  strip_y1 = sy + n;
  clip_x0 = x0;
  clip_x1 = x1;
  for (int r = 0; r < n; r++) {
    uint16_t *c = cbuf + r * SCREEN_W, *z = zbuf + r * SCREEN_W;
    for (int x = x0; x < x1; x++) {
      c[x] = sky565;
      z[x] = 0xFFFF;
    }
  }
  for (int i = 0; i < nobj; i++) {
    RObj *r = &robj[i];
    if (r->ymin > r->ymax || r->ymax < strip_y0 || r->ymin >= strip_y1) continue;
    draw_object(&objs[i], r);
  }
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
