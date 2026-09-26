#ifndef FMATH_H
#define FMATH_H

#include <math.h>
#include <stdint.h>

#define PI_F 3.14159265f
#define DEG2RAD (PI_F / 180.0f)
#define RAD2DEG (180.0f / PI_F)

typedef struct { float x, y, z; } vec3;
typedef struct { float x, y, z, w; } quat;

static inline vec3 v3(float x, float y, float z) { vec3 r = {x, y, z}; return r; }
static inline vec3 vadd(vec3 a, vec3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline vec3 vsub(vec3 a, vec3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline vec3 vscale(vec3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline float vdot(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline vec3 vcross(vec3 a, vec3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static inline float vlen(vec3 a) { return sqrtf(vdot(a, a)); }
static inline vec3 vnorm(vec3 a) {
  float l = vlen(a);
  return l > 1e-12f ? vscale(a, 1.0f / l) : v3(0, 0, 0);
}
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static inline quat qident(void) { quat q = {0, 0, 0, 1}; return q; }
static inline quat qmul(quat a, quat b) {
  quat r;
  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return r;
}
static inline quat qconj(quat a) { quat r = {-a.x, -a.y, -a.z, a.w}; return r; }
static inline quat qnorm(quat a) {
  float l = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w);
  if (l < 1e-12f) return qident();
  l = 1.0f / l;
  quat r = {a.x * l, a.y * l, a.z * l, a.w * l};
  return r;
}
static inline vec3 qrot(quat q, vec3 v) {
  vec3 u = v3(q.x, q.y, q.z);
  vec3 t = vscale(vcross(u, v), 2.0f);
  return vadd(vadd(v, vscale(t, q.w)), vcross(u, t));
}
static inline quat qaxis(vec3 axis, float rad) {
  float s = sinf(rad * 0.5f), c = cosf(rad * 0.5f);
  quat r = {axis.x * s, axis.y * s, axis.z * s, c};
  return r;
}
/* System.Numerics CreateFromYawPitchRoll (yaw about Y, pitch about X, roll about Z), degrees */
static inline quat qeuler(float xdeg, float ydeg, float zdeg) {
  float hr = zdeg * 0.5f * DEG2RAD, hp = xdeg * 0.5f * DEG2RAD, hy = ydeg * 0.5f * DEG2RAD;
  float sr = sinf(hr), cr = cosf(hr), sp = sinf(hp), cp = cosf(hp), sy = sinf(hy), cy = cosf(hy);
  quat q;
  q.x = cy * sp * cr + sy * cp * sr;
  q.y = sy * cp * cr - cy * sp * sr;
  q.z = cy * cp * sr - sy * sp * cr;
  q.w = cy * cp * cr + sy * sp * sr;
  return q;
}
static inline quat qrotz(float rad) { return qaxis(v3(0, 0, 1), rad); }

#endif
