#ifndef PHYSICS_H
#define PHYSICS_H
#include <stdbool.h>
#include "fmath.h"

#define PHYS_HZ 60

void phys_reset(void);
void phys_step(void);
void phys_start(void); /* after scripts' first frame: create bodies for physics objects */

bool phys_raycast(vec3 from, vec3 to, vec3 *hit, int *obj);
bool phys_raycast_ex(vec3 from, vec3 to, vec3 *hit, int *obj, int ignore);
void phys_get_velocity(int o, vec3 *vel, vec3 *spin);
void phys_set_velocity(int o, const vec3 *vel, const vec3 *spin);
void phys_add_force(int o, const vec3 *force, const vec3 *at, const vec3 *torque);
void phys_set_locked(int o, const vec3 *pos, const vec3 *rot);
void phys_set_mass(int o, float m);
void phys_set_friction(int o, float f);
void phys_set_bounce(int o, float b);
void phys_set_gravity(vec3 g);
void phys_moved(int o); /* object teleported by script */
void phys_make_dynamic(int o);
void phys_destroyed(int o);
void phys_hidden(int o);

int phys_add_constraint(int base, int part, vec3 pivot);
void phys_con_limits(int c, bool angular, vec3 lower, vec3 upper);
void phys_con_spring(int c, bool angular, vec3 stiffness, vec3 damping);
void phys_con_motor(int c, bool angular, vec3 speed, vec3 force);

bool phys_collision(int o, int *other, float *impulse, vec3 *normal);

extern vec3 phys_gravity;

#endif
