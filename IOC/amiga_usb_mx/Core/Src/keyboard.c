#include "keyboard.h"
#include "keycode.h" // For KC_ defines

/*
 * Amiga 1200 Keyboard Matrix Scancode Layout
 *
 * This ASCII art represents the logical 6x22 keyboard matrix,
 * with each cell corresponding to a position in the `scancode_lut`.
 * Empty spaces ("  ") indicate an unmapped key (KC_NO / 0x00).
 * Modifier keys are shown with their short forms (e.g., LCtl for KC_LCTRL).
 * F-keys go up to F10 as per the original layout.
 * Number pad keys are prefixed with KP.
 *
 *       C0    C1    C2    C3    C4    C5    C6    C7    C8    C9    C10   C11   C12   C13   C14   C15   C16   C17   C18   C19   C20   C21
 * R0 :  Esc         F1    F2    F3    F4    F5    F6    F7    F8    F9    F10                                                            
 * R1 :  Grv   1     2     3     4     5     6     7     8     9     0     -     =     \     Bksp  Ins         Del   NumLk Home  PgUp  End
 * R2 :  Tab   Q     W     E     R     T     Y     U     I     O     P     [     ]           Enter                   KP7   KP8   KP9   KP-
 * R3 :  LCtl  CapsL A     S     D     F     G     H     J     K     L     ;     '     Int1                    Up    KP4   KP5   KP6   KP+
 * R4 :  LShft Int2  Z     X     C     V     B     N     M     ,     .     /                 RShft Left  Down  Right KP1   KP2   KP3   KPEnt
 * R5 :        LAlt  LGUI  Int3                          Space                   Int4  RGUI  RAlt                          KP0         KP.   
 */

const uint8_t scancode_lut[MATRIX_ROWS][MATRIX_COLS] = {
//  C0           C1           C2           C3           C4           C5           C6           C7           C8           C9          C10          C11          C12          C13          C14          C15          C16          C17          C18          C19          C20          C21
// R0
  {KC_ESCAPE,    KC_NO,       KC_F1,       KC_F2,       KC_F3,       KC_F4,       KC_F5,       KC_F6,       KC_F7,       KC_F8,       KC_F9,       KC_F10,      KC_NO,       KC_NO,       KC_NO,       KC_NO,       KC_NO,       KC_NO,       KC_NO,       KC_NO,       KC_NO,       KC_NO },
// R1
  {KC_GRAVE,     KC_1,        KC_2,        KC_3,        KC_4,        KC_5,        KC_6,        KC_7,        KC_8,        KC_9,        KC_0,        KC_MINUS,    KC_EQUAL,    KC_BSLASH,   KC_BSPACE,   KC_INSERT,   KC_NO,       KC_DELETE,   KC_NUMLOCK,  KC_HOME,     KC_PGUP,     KC_END },
// R2
  {KC_TAB,       KC_Q,        KC_W,        KC_E,        KC_R,        KC_T,        KC_Y,        KC_U,        KC_I,        KC_O,        KC_P,        KC_LBRACKET, KC_RBRACKET, KC_NO,       KC_ENTER,    KC_NO,       KC_NO,       KC_NO,       KC_KP_7,     KC_KP_8,     KC_KP_9,     KC_KP_MINUS },
// R3
  {KC_LCTRL,     KC_CAPSLOCK, KC_A,        KC_S,        KC_D,        KC_F,        KC_G,        KC_H,        KC_J,        KC_K,        KC_L,        KC_SCOLON,   KC_QUOTE,    KC_INT1,     KC_NO,       KC_NO,       KC_UP,       KC_NO,       KC_KP_4,     KC_KP_5,     KC_KP_6,     KC_KP_PLUS },
// R4
  {KC_LSHIFT,    KC_INT2,     KC_Z,        KC_X,        KC_C,        KC_V,        KC_B,        KC_N,        KC_M,        KC_COMMA,    KC_DOT,      KC_SLASH,    KC_NO,       KC_NO,       KC_RSHIFT,   KC_LEFT,     KC_DOWN,     KC_RIGHT,    KC_KP_1,     KC_KP_2,     KC_KP_3,     KC_KP_ENTER },
// R5
  {KC_NO,        KC_LALT,     KC_LGUI,     KC_INT3,     KC_NO,       KC_NO,       KC_NO,       KC_NO,       KC_SPACE,    KC_NO,       KC_NO,       KC_NO,       KC_INT4,     KC_RGUI,     KC_RALT,     KC_NO,       KC_NO,       KC_NO,       KC_KP_0,     KC_NO,       KC_KP_DOT,   KC_NO },
};
