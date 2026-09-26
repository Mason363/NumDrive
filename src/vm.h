#ifndef VM_H
#define VM_H
#include <stdbool.h>
#include "fmath.h"

enum { T_NUM, T_VEC, T_ROT, T_TRU, T_OBJ, T_CON };

bool vm_setup_envs(void);
void vm_frame(void);        /* run all scripts for one frame */
void vm_late(void);         /* run queued Late Update branches (after physics) */
extern int vm_frame_count;
extern uint32_t vm_buttons;  /* bit0 = left (brake), bit1 = right (gas) */

/* engine hooks implemented elsewhere */
void game_win(int delay);
void game_lose(int delay);
void cam_set(const vec3 *pos, const quat *rot, const float *range, int perspective);
void light_set(const quat *rot);

/* global variable access by name index (for the engine) */
int vm_global(const char *name, int type);
float vm_global_num(int idx);
int vm_global_tru(int idx);

#endif
