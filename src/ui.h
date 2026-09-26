#ifndef UI_H
#define UI_H
#include <stdint.h>
#include <stdbool.h>

enum { CARD_PAUSE, CARD_WIN, CARD_LOSE };

/* in-game overlay */
void hud_reset(void);
extern float hud_score, hud_coins; /* Set Score values, hidden when negative */
void hud_tick(uint8_t buttons);
void hud_draw(uint16_t *px, int y, int n);
void viewfinder_draw(uint16_t *px, int y, int n, int t);

/* menus: capture the blurred background from the current view */
void ui_capture_background(void);
/* card screen; anim counts frames since the card appeared; sel is the focused button */
void card_setup(int kind, int level);
void card_draw(int anim, int sel, bool full);
int card_buttons(void); /* number of buttons (1 or 2) */
/* level select */
void levels_draw(int sel, int scroll);

#endif
