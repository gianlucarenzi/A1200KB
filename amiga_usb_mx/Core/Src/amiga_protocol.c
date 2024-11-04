/**
 * Amiga housekeeping as MPU.
 * Written by Gianluca Renzi R.G.
 * License for this code: LGPLv3
 * Please refer to this license at GNU Web Site
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * 
 **/
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "syscall.h"
#include "keyboard.h"
#include "keycode.h"
#include "config.h"
#include "amiga_config.h"
#include "debug.h"
#include "amiga_protocol.h"
#include "debug.h"
#include "main.h" // For GPIO pins definition
#include "hook.h"

static int debuglevel = DBG_INFO;

#define KEY_NONE                               0x00
#define KEY_ERRORROLLOVER                      0x01
#define KEY_POSTFAIL                           0x02
#define KEY_ERRORUNDEFINED                     0x03
#define KEY_A                                  0x04
#define KEY_B                                  0x05
#define KEY_C                                  0x06
#define KEY_D                                  0x07
#define KEY_E                                  0x08
#define KEY_F                                  0x09
#define KEY_G                                  0x0A
#define KEY_H                                  0x0B
#define KEY_I                                  0x0C
#define KEY_J                                  0x0D
#define KEY_K                                  0x0E
#define KEY_L                                  0x0F
#define KEY_M                                  0x10
#define KEY_N                                  0x11
#define KEY_O                                  0x12
#define KEY_P                                  0x13
#define KEY_Q                                  0x14
#define KEY_R                                  0x15
#define KEY_S                                  0x16
#define KEY_T                                  0x17
#define KEY_U                                  0x18
#define KEY_V                                  0x19
#define KEY_W                                  0x1A
#define KEY_X                                  0x1B
#define KEY_Y                                  0x1C
#define KEY_Z                                  0x1D
#define KEY_1_EXCLAMATION_MARK                 0x1E
#define KEY_2_AT                               0x1F
#define KEY_3_NUMBER_SIGN                      0x20
#define KEY_4_DOLLAR                           0x21
#define KEY_5_PERCENT                          0x22
#define KEY_6_CARET                            0x23
#define KEY_7_AMPERSAND                        0x24
#define KEY_8_ASTERISK                         0x25
#define KEY_9_OPARENTHESIS                     0x26
#define KEY_0_CPARENTHESIS                     0x27
#define KEY_ENTER                              0x28
#define KEY_ESCAPE                             0x29
#define KEY_BACKSPACE                          0x2A
#define KEY_TAB                                0x2B
#define KEY_SPACEBAR                           0x2C
#define KEY_MINUS_UNDERSCORE                   0x2D
#define KEY_EQUAL_PLUS                         0x2E
#define KEY_OBRACKET_AND_OBRACE                0x2F
#define KEY_CBRACKET_AND_CBRACE                0x30
#define KEY_BACKSLASH_VERTICAL_BAR             0x31
#define KEY_NONUS_NUMBER_SIGN_TILDE            0x32
#define KEY_SEMICOLON_COLON                    0x33
#define KEY_SINGLE_AND_DOUBLE_QUOTE            0x34
#define KEY_GRAVE_ACCENT_AND_TILDE             0x35
#define KEY_COMMA_AND_LESS                     0x36
#define KEY_DOT_GREATER                        0x37
#define KEY_SLASH_QUESTION                     0x38
#define KEY_CAPS_LOCK                          0x39
#define KEY_F1                                 0x3A
#define KEY_F2                                 0x3B
#define KEY_F3                                 0x3C
#define KEY_F4                                 0x3D
#define KEY_F5                                 0x3E
#define KEY_F6                                 0x3F
#define KEY_F7                                 0x40
#define KEY_F8                                 0x41
#define KEY_F9                                 0x42
#define KEY_F10                                0x43
#define KEY_F11                                0x44
#define KEY_F12                                0x45
#define KEY_PRINTSCREEN                        0x46
#define KEY_SCROLL_LOCK                        0x47
#define KEY_PAUSE                              0x48
#define KEY_INSERT                             0x49
#define KEY_HOME                               0x4A
#define KEY_PAGEUP                             0x4B
#define KEY_DELETE                             0x4C
#define KEY_END1                               0x4D
#define KEY_PAGEDOWN                           0x4E
#define KEY_RIGHTARROW                         0x4F
#define KEY_LEFTARROW                          0x50
#define KEY_DOWNARROW                          0x51
#define KEY_UPARROW                            0x52
#define KEY_KEYPAD_NUM_LOCK_AND_CLEAR          0x53
#define KEY_KEYPAD_SLASH                       0x54
#define KEY_KEYPAD_ASTERIKS                    0x55
#define KEY_KEYPAD_MINUS                       0x56
#define KEY_KEYPAD_PLUS                        0x57
#define KEY_KEYPAD_ENTER                       0x58
#define KEY_KEYPAD_1_END                       0x59
#define KEY_KEYPAD_2_DOWN_ARROW                0x5A
#define KEY_KEYPAD_3_PAGEDN                    0x5B
#define KEY_KEYPAD_4_LEFT_ARROW                0x5C
#define KEY_KEYPAD_5                           0x5D
#define KEY_KEYPAD_6_RIGHT_ARROW               0x5E
#define KEY_KEYPAD_7_HOME                      0x5F
#define KEY_KEYPAD_8_UP_ARROW                  0x60
#define KEY_KEYPAD_9_PAGEUP                    0x61
#define KEY_KEYPAD_0_INSERT                    0x62
#define KEY_KEYPAD_DECIMAL_SEPARATOR_DELETE    0x63
#define KEY_NONUS_BACK_SLASH_VERTICAL_BAR      0x64
#define KEY_APPLICATION                        0x65
#define KEY_POWER                              0x66
#define KEY_KEYPAD_EQUAL                       0x67
#define KEY_F13                                0x68
#define KEY_F14                                0x69
#define KEY_F15                                0x6A
#define KEY_F16                                0x6B
#define KEY_F17                                0x6C
#define KEY_F18                                0x6D
#define KEY_F19                                0x6E
#define KEY_F20                                0x6F
#define KEY_F21                                0x70
#define KEY_F22                                0x71
#define KEY_F23                                0x72
#define KEY_F24                                0x73
#define KEY_EXECUTE                            0x74
#define KEY_HELP                               0x75
#define KEY_MENU                               0x76
#define KEY_SELECT                             0x77
#define KEY_STOP                               0x78
#define KEY_AGAIN                              0x79
#define KEY_UNDO                               0x7A
#define KEY_CUT                                0x7B
#define KEY_COPY                               0x7C
#define KEY_PASTE                              0x7D
#define KEY_FIND                               0x7E
#define KEY_MUTE                               0x7F
#define KEY_VOLUME_UP                          0x80
#define KEY_VOLUME_DOWN                        0x81
#define KEY_LOCKING_CAPS_LOCK                  0x82
#define KEY_LOCKING_NUM_LOCK                   0x83
#define KEY_LOCKING_SCROLL_LOCK                0x84
#define KEY_KEYPAD_COMMA                       0x85
#define KEY_KEYPAD_EQUAL_SIGN                  0x86
#define KEY_INTERNATIONAL1                     0x87
#define KEY_INTERNATIONAL2                     0x88
#define KEY_INTERNATIONAL3                     0x89
#define KEY_INTERNATIONAL4                     0x8A
#define KEY_INTERNATIONAL5                     0x8B
#define KEY_INTERNATIONAL6                     0x8C
#define KEY_INTERNATIONAL7                     0x8D
#define KEY_INTERNATIONAL8                     0x8E
#define KEY_INTERNATIONAL9                     0x8F
#define KEY_LANG1                              0x90
#define KEY_LANG2                              0x91
#define KEY_LANG3                              0x92
#define KEY_LANG4                              0x93
#define KEY_LANG5                              0x94
#define KEY_LANG6                              0x95
#define KEY_LANG7                              0x96
#define KEY_LANG8                              0x97
#define KEY_LANG9                              0x98
#define KEY_ALTERNATE_ERASE                    0x99
#define KEY_SYSREQ                             0x9A
#define KEY_CANCEL                             0x9B
#define KEY_CLEAR                              0x9C
#define KEY_PRIOR                              0x9D
#define KEY_RETURN                             0x9E
#define KEY_SEPARATOR                          0x9F
#define KEY_OUT                                0xA0
#define KEY_OPER                               0xA1
#define KEY_CLEAR_AGAIN                        0xA2
#define KEY_CRSEL                              0xA3
#define KEY_EXSEL                              0xA4
#define KEY_KEYPAD_00                          0xB0
#define KEY_KEYPAD_000                         0xB1
#define KEY_THOUSANDS_SEPARATOR                0xB2
#define KEY_DECIMAL_SEPARATOR                  0xB3
#define KEY_CURRENCY_UNIT                      0xB4
#define KEY_CURRENCY_SUB_UNIT                  0xB5
#define KEY_KEYPAD_OPARENTHESIS                0xB6
#define KEY_KEYPAD_CPARENTHESIS                0xB7
#define KEY_KEYPAD_OBRACE                      0xB8
#define KEY_KEYPAD_CBRACE                      0xB9
#define KEY_KEYPAD_TAB                         0xBA
#define KEY_KEYPAD_BACKSPACE                   0xBB
#define KEY_KEYPAD_A                           0xBC
#define KEY_KEYPAD_B                           0xBD
#define KEY_KEYPAD_C                           0xBE
#define KEY_KEYPAD_D                           0xBF
#define KEY_KEYPAD_E                           0xC0
#define KEY_KEYPAD_F                           0xC1
#define KEY_KEYPAD_XOR                         0xC2
#define KEY_KEYPAD_CARET                       0xC3
#define KEY_KEYPAD_PERCENT                     0xC4
#define KEY_KEYPAD_LESS                        0xC5
#define KEY_KEYPAD_GREATER                     0xC6
#define KEY_KEYPAD_AMPERSAND                   0xC7
#define KEY_KEYPAD_LOGICAL_AND                 0xC8
#define KEY_KEYPAD_VERTICAL_BAR                0xC9
#define KEY_KEYPAD_LOGIACL_OR                  0xCA
#define KEY_KEYPAD_COLON                       0xCB
#define KEY_KEYPAD_NUMBER_SIGN                 0xCC
#define KEY_KEYPAD_SPACE                       0xCD
#define KEY_KEYPAD_AT                          0xCE
#define KEY_KEYPAD_EXCLAMATION_MARK            0xCF
#define KEY_KEYPAD_MEMORY_STORE                0xD0
#define KEY_KEYPAD_MEMORY_RECALL               0xD1
#define KEY_KEYPAD_MEMORY_CLEAR                0xD2
#define KEY_KEYPAD_MEMORY_ADD                  0xD3
#define KEY_KEYPAD_MEMORY_SUBTRACT             0xD4
#define KEY_KEYPAD_MEMORY_MULTIPLY             0xD5
#define KEY_KEYPAD_MEMORY_DIVIDE               0xD6
#define KEY_KEYPAD_PLUSMINUS                   0xD7
#define KEY_KEYPAD_CLEAR                       0xD8
#define KEY_KEYPAD_CLEAR_ENTRY                 0xD9
#define KEY_KEYPAD_BINARY                      0xDA
#define KEY_KEYPAD_OCTAL                       0xDB
#define KEY_KEYPAD_DECIMAL                     0xDC
#define KEY_KEYPAD_HEXADECIMAL                 0xDD
#define KEY_LEFTCONTROL                        0xE0
#define KEY_LEFTSHIFT                          0xE1
#define KEY_LEFTALT                            0xE2
#define KEY_LEFT_GUI                           0xE3
#define KEY_RIGHTCONTROL                       0xE4
#define KEY_RIGHTSHIFT                         0xE5
#define KEY_RIGHTALT                           0xE6
#define KEY_RIGHT_GUI                          0xE7

/* 
 * Custom defined codes, just to be sure the table is scan from the
 * beginning to the end, without a duplicate
 */
#define KEY_LMOUSE_BUTTON                      0xE8
#define KEY_RMOUSE_BUTTON                      0xE9
#define KEY_MMOUSE_BUTTON                      0xEA


#define KEYCODE_TAB_SIZE      121
static uint8_t leds = 0;

static const uint8_t scancodeamiga[KEYCODE_TAB_SIZE][2] =
{
 	// SCANCODE USB to AMIGA
 	// --------------------------------------------------------------
 	// Need the real scancode by testing all keys on a real keyboard.
 	// --------------------------------------------------------------
 	// Keycodes from: https://wiki.amigaos.net/wiki/Keymap_Library
 	// --------------------------------------------------------------
	{KEY_GRAVE_ACCENT_AND_TILDE, 0x00 }, // `~
	{KEY_1_EXCLAMATION_MARK,     0x01 }, // 1!
	{KEY_2_AT,                   0x02 }, // 2@
	{KEY_3_NUMBER_SIGN,          0x03 }, // 3#
	{KEY_4_DOLLAR,               0x04 }, // 4$
	{KEY_5_PERCENT,              0x05 }, // 5%
	{KEY_6_CARET,                0x06 }, // 6^
	{KEY_7_AMPERSAND,            0x07 }, // 7&
	{KEY_8_ASTERISK,             0x08 }, // 8*
	{KEY_9_OPARENTHESIS,         0x09 }, // 9(
	{KEY_0_CPARENTHESIS,         0x0A }, // 0)
	{KEY_MINUS_UNDERSCORE,       0x0B }, // -_
	{KEY_EQUAL_PLUS,             0x0C }, // =+
	{KEY_BACKSLASH_VERTICAL_BAR, 0x0D }, // \|
	{KEY_INTERNATIONAL3,         0x0E }, // Intl 3 Yen
	{KEY_KEYPAD_0_INSERT,        0x0F }, // NUM 0
	{KEY_Q,                      0x10 }, // Q
	{KEY_W,                      0x11 }, // W
	{KEY_E,                      0x12 }, // E
	{KEY_R,                      0x13 }, // R
	{KEY_T,                      0x14 }, // T
	{KEY_Y,                      0x15 }, // Y
	{KEY_U,                      0x16 }, // U
	{KEY_I,                      0x17 }, // I
	{KEY_O,                      0x18 }, // O
	{KEY_P,                      0x19 }, // P
	{KEY_OBRACKET_AND_OBRACE,    0x1A }, // [{
	{KEY_CBRACKET_AND_CBRACE,    0x1B }, // }]
	{KEY_INTERNATIONAL5,         0x1C }, // undefined Intl 5
	{KEY_KEYPAD_1_END,           0x1D }, // NUM 1
	{KEY_KEYPAD_2_DOWN_ARROW,    0x1E }, // NUM 2
	{KEY_KEYPAD_3_PAGEDN,        0x1F }, // NUM 3
	{KEY_A,                      0x20 }, // A
	{KEY_S,                      0x21 }, // S
	{KEY_D,                      0x22 }, // D
	{KEY_F,                      0x23 }, // F
	{KEY_G,                      0x24 }, // G
	{KEY_H,                      0x25 }, // H
	{KEY_J,                      0x26 }, // J
	{KEY_K,                      0x27 }, // K
	{KEY_L,                      0x28 }, // L
	{KEY_SEMICOLON_COLON,        0x29 }, // ;:
	{KEY_SINGLE_AND_DOUBLE_QUOTE,0x2A }, // '"
	{KEY_INTERNATIONAL1,         0x2B }, // undefined Intl 1
	{KEY_INTERNATIONAL4,         0x2C }, // undefined Intl 4
	{KEY_KEYPAD_4_LEFT_ARROW,    0x2D }, // NUM 4
	{KEY_KEYPAD_5,               0x2E }, // NUM 5
	{KEY_KEYPAD_6_RIGHT_ARROW,   0x2F }, // NUM 6
	{KEY_INTERNATIONAL2,         0x30 }, // <SHIFT> international? Intl 2
	{KEY_Z,                      0x31 }, // Z
	{KEY_X,                      0x32 }, // X
	{KEY_C,                      0x33 }, // C
	{KEY_V,                      0x34 }, // V
	{KEY_B,                      0x35 }, // B
	{KEY_N,                      0x36 }, // N
	{KEY_M,                      0x37 }, // M
	{KEY_COMMA_AND_LESS,         0x38 }, // ,<
	{KEY_DOT_GREATER,            0x39 }, // .>
	{KEY_SLASH_QUESTION,         0x3A }, // /?
	{KEY_INTERNATIONAL6,         0x3B }, // undefined Intl 6
	{KEY_KEYPAD_DECIMAL_SEPARATOR_DELETE, 0x3C }, // KEYPAD '.'
	{KEY_KEYPAD_7_HOME,          0x3D }, // NUM 7
	{KEY_KEYPAD_8_UP_ARROW,      0x3E }, // NUM 8
	{KEY_KEYPAD_9_PAGEUP,        0x3F }, // NUM 9
	{KEY_SPACEBAR,               0x40 }, // SPACE
	{KEY_BACKSPACE,              0x41 }, // BACKSPACE
	{KEY_TAB,                    0x42 }, // TAB
	{KEY_ENTER,                  0x43 }, // ENTER
	{KEY_RETURN,                 0x43 }, // RETURN
	{KEY_KEYPAD_ENTER,           0x44 }, // NUM Enter
	{KEY_ESCAPE,                 0x45 }, // ESC
	{KEY_DELETE,                 0x46 }, // DEL
	{KEY_INSERT,                 0x47 }, // INS (usually undefined)
	{KEY_PAGEUP,                 0x48 }, // PAGEUP (usually undefined)
	{KEY_PAGEDOWN,               0x49 }, // PAGEDOWN (usually undefined)
	{KEY_KEYPAD_MINUS,           0x4A }, // NUM -
	{KEY_F11,                    0x4B }, // F11
	{KEY_UPARROW,                0x4C }, // CURSOR U
	{KEY_DOWNARROW,              0x4D }, // CURSOR D
	{KEY_RIGHTARROW,             0x4E }, // CURSOR R
	{KEY_LEFTARROW,              0x4F }, // CURSOR L
	{KEY_F1,                     0x50 }, // F1
	{KEY_F2,                     0x51 }, // F2
	{KEY_F3,                     0x52 }, // F3
	{KEY_F4,                     0x53 }, // F4
	{KEY_F5,                     0x54 }, // F5
	{KEY_F6,                     0x55 }, // F6
	{KEY_F7,                     0x56 }, // F7
	{KEY_F8,                     0x57 }, // F8
	{KEY_F9,                     0x58 }, // F9
	{KEY_F10,                    0x59 }, // F10
	{KEY_KEYPAD_OPARENTHESIS,    0x5A }, // NUM (
	{KEY_KEYPAD_CPARENTHESIS,    0x5B }, // NUM )
	{KEY_KEYPAD_SLASH,           0x5C }, // /
	{KEY_KEYPAD_ASTERIKS,        0x5D }, // NUM *
	{KEY_KEYPAD_PLUS,            0x5E }, // NUM +
	{KEY_SCROLL_LOCK,            0x5F }, // HELP (ScrollLock)
	{KEY_APPLICATION,            0x5F }, // APP - HELP
	{KEY_LEFTSHIFT,              0x60 }, // LSHIFT
	{KEY_RIGHTSHIFT,             0x61 }, // RSHIFT
	{KEY_CAPS_LOCK,              0x62 }, // CAPS
	{KEY_LEFTCONTROL,            0x63 }, // LCTRL
	{KEY_RIGHTCONTROL,           0x63 }, // RCTRL
	{KEY_LEFTALT,                0x64 }, // LALT
	{KEY_RIGHTALT,               0x65 }, // RALT
	{KEY_LEFT_GUI,               0x66 }, // LWIN
	{KEY_RIGHT_GUI,              0x67 }, // RWIN
	{KEY_LMOUSE_BUTTON,          0x68 }, // LMOUSE BUTTON
	{KEY_RMOUSE_BUTTON,          0x69 }, // RMOUSE BUTTON
	{KEY_MMOUSE_BUTTON,          0x6A }, // MMOUSE BUTTON
	{KEY_MENU,                   0x6B }, // MENU, GUI, COMPOSE (mappable to RAmiga in Firmware)
	{KEY_KEYPAD_COMMA,           0x6C }, // Brazil NP . (named "Keypad ," in USB specs)
	{KEY_PRINTSCREEN,            0x6D }, // PrintScreen/SysReq (mappable to Help in Firmware)
	{KEY_SYSREQ,                 0x6E }, // BREAK
	{KEY_F12,                    0x6F }, // F12
	{KEY_HOME,                   0x70 }, // HOME
	{KEY_END1,                   0x71 }, // END
	{KEY_STOP,                   0x72 }, // STOP/CD32 Blue Stop, CDTV Stop, Port 0 Blue
	{KEY_PAUSE,                  0x73 }, // Play/Pause CD32 Grey Play/Pause CDTV Play/Pause Port 0 Blue
	{KEY_VOLUME_UP,              0x74 }, // Prev Track CDTV << REV
	{KEY_VOLUME_DOWN,            0x75 }, // Next Track CDTV >> FF
};

static const uint8_t asciiscancode[KEYCODE_TAB_SIZE][2] =
{
	{KEY_GRAVE_ACCENT_AND_TILDE, '~' }, // ~
	{KEY_1_EXCLAMATION_MARK,     '!' }, // 1!
	{KEY_2_AT,                   '@' }, // 2@
	{KEY_3_NUMBER_SIGN,          '#' }, // 3#
	{KEY_4_DOLLAR,               '$' }, // 4$
	{KEY_5_PERCENT,              '%' }, // 5%
	{KEY_6_CARET,                '^' }, // 6^
	{KEY_7_AMPERSAND,            '&' }, // 7&
	{KEY_8_ASTERISK,             '*' }, // 8*
	{KEY_9_OPARENTHESIS,         '(' }, // 9(
	{KEY_0_CPARENTHESIS,         ')' }, // 0)
	{KEY_MINUS_UNDERSCORE,       '-' }, // -_
	{KEY_EQUAL_PLUS,             '+' }, // +=
	{KEY_BACKSLASH_VERTICAL_BAR, '|' }, // |
	{KEY_NONE,                   '-' }, // @
	{KEY_KEYPAD_0_INSERT,        '0' }, // NUM 0
	{KEY_Q,                      'Q' }, // Q
	{KEY_W,                      'W' }, // W
	{KEY_E,                      'E' }, // E
	{KEY_R,                      'R' }, // R
	{KEY_T,                      'T' }, // T
	{KEY_Y,                      'Y' }, // Y
	{KEY_U,                      'U' }, // U
	{KEY_I,                      'I' }, // I
	{KEY_O,                      'O' }, // O
	{KEY_P,                      'P' }, // P
	{KEY_OBRACKET_AND_OBRACE,    '{' }, // [{
	{KEY_CBRACKET_AND_CBRACE,    '}' }, // }]
	{KEY_SPACEBAR,               ' ' }, // SPACE
	{KEY_KEYPAD_1_END,           '1' }, // NUM 1
	{KEY_KEYPAD_2_DOWN_ARROW,    '2' }, // NUM 2
	{KEY_KEYPAD_3_PAGEDN,        '3' }, // NUM 3
	{KEY_A,                      'A' }, // A
	{KEY_S,                      'S' }, // S
	{KEY_D,                      'D' }, // D
	{KEY_F,                      'F' }, // F
	{KEY_G,                      'G' }, // G
	{KEY_H,                      'H' }, // H
	{KEY_J,                      'J' }, // J
	{KEY_K,                      'K' }, // K
	{KEY_L,                      'L' }, // L
	{KEY_SEMICOLON_COLON,        ':' }, // :;
	{KEY_SINGLE_AND_DOUBLE_QUOTE,'"' }, // "'
	{KEY_ENTER,                  '\n'}, // <RET> international?
	{KEY_KEYPAD_4_LEFT_ARROW,    '4' }, // NUM 4
	{KEY_KEYPAD_5,               '5' }, // NUM 5
	{KEY_KEYPAD_6_RIGHT_ARROW,   '6' }, // NUM 6
	{KEY_INTERNATIONAL2,         '-' }, // <SHIFT> international?
	{KEY_Z,                      'Z' }, // Z
	{KEY_X,                      'X' }, // X
	{KEY_C,                      'C' }, // C
	{KEY_V,                      'V' }, // V
	{KEY_B,                      'B' }, // B
	{KEY_N,                      'N' }, // N
	{KEY_M,                      'M' }, // M
	{KEY_COMMA_AND_LESS,         '<' }, // <,
	{KEY_KEYPAD_COMMA,           ',' }, // NUM ,
	{KEY_DOT_GREATER,            '>' }, // >.
	{KEY_SLASH_QUESTION,         '?' }, // ?/
	{KEY_KEYPAD_7_HOME,          '7' }, // NUM 7
	{KEY_KEYPAD_8_UP_ARROW,      '8' }, // NUM 8
	{KEY_KEYPAD_9_PAGEUP,        '9' }, // NUM 9
	{KEY_SPACEBAR,               ' ' }, // SPACE
	{KEY_BACKSPACE,              '\r'}, // BACKSPACE
	{KEY_TAB,                    '\t'}, // TAB
	{KEY_KEYPAD_ENTER,           '\n'}, // ENTER
	{KEY_RETURN,                 '\n'}, // RETURN
	{KEY_ESCAPE,                 '-' }, // ESC
	{KEY_DELETE,                 '\r'}, // DEL
	{KEY_KEYPAD_MINUS,           '-' }, // NUM -
	{KEY_KEYPAD_SLASH,           '/' }, // /
	{KEY_KEYPAD_ASTERIKS,        '*' }, // NUM *
	{KEY_KEYPAD_PLUS,            '+' }, // NUM +
	{KEY_APPLICATION,            'h' }, // APP - HELP
	{KEY_NONE,                   '(' }, // (
	{KEY_NONE,                   ')' }, // )
	{KEY_NONE,                   ' ' }, // SPARE
	{KEY_NONE,                   ' ' }, // SPARE
	{KEY_NONE,                   ' ' }, // SPARE
	{KEY_NONE,                   ' ' }, // SPARE
	{KEY_NONE,                   ' ' }, // SPARE
	{KEY_NONE,                   ' ' }, // SPARE
	{KEY_UPARROW,                ' ' }, // CURSOR U
	{KEY_DOWNARROW,              ' ' }, // CURSOR D
	{KEY_RIGHTARROW,             ' ' }, // CURSOR R
	{KEY_LEFTARROW,              ' ' }, // CURSOR L
	{KEY_F1,                     ' ' }, // F1
	{KEY_F2,                     ' ' }, // F2
	{KEY_F3,                     ' ' }, // F3
	{KEY_F4,                     ' ' }, // F4
	{KEY_F5,                     ' ' }, // F5
	{KEY_F6,                     ' ' }, // F6
	{KEY_F7,                     ' ' }, // F7
	{KEY_F8,                     ' ' }, // F8
	{KEY_F9,                     ' ' }, // F9
	{KEY_F10,                    ' ' }, // F10
	{KEY_LEFTSHIFT,              ' ' }, // LSHIFT
	{KEY_RIGHTSHIFT,             ' ' }, // RSHIFT
	{KEY_LOCKING_CAPS_LOCK,      ' ' }, // CAPS
	{KEY_LEFTCONTROL,            ' ' }, // LCTRL
	{KEY_LEFTALT,                ' ' }, // LALT
	{KEY_RIGHTALT,               ' ' }, // RALT
	{KEY_LEFT_GUI,               ' ' }, // LWIN
	{KEY_RIGHT_GUI,              ' ' }, // RWIN
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
	{KEY_NONE,                   ' ' }, // STUB
};


/**
	The keyboard transmits 8-bit data words serially to the main unit. Before
	the transmission starts, both KCLK and KDAT are high.  The keyboard starts
	the transmission by putting out the first data bit (on KDAT), followed by
	a pulse on KCLK (low then high); then it puts out the second data bit and
	pulses KCLK until all eight data bits have been sent.  After the end of
	the last KCLK pulse, the keyboard pulls KDAT high again.

	When the computer has received the eighth bit, it must pulse KDAT low for
	at least 1 (one) microsecond, as a handshake signal to the keyboard.  The
	handshake detection on the keyboard end will typically use a hardware
	latch.  The keyboard must be able to detect pulses greater than or equal
	to 1 microsecond.  Software MUST pulse the line low for 85 microseconds to
	ensure compatibility with all keyboard models.

	All codes transmitted to the computer are rotated one bit before
	transmission.  The transmitted order is therefore 6-5-4-3-2-1-0-7. The
	reason for this is to transmit the  up/down flag  last, in order to cause
	a key-up code to be transmitted in case the keyboard is forced to restore
	 lost sync  (explained in more detail below).

	The KDAT line is active low; that is, a high level (+5V) is interpreted as
	0, and a low level (0V) is interpreted as 1.

				 _____   ___   ___   ___   ___   ___   ___   ___   _________
			KCLK      \_/   \_/   \_/   \_/   \_/   \_/   \_/   \_/
				 ___________________________________________________________
			KDAT    \_____X_____X_____X_____X_____X_____X_____X_____/
					  (6)   (5)   (4)   (3)   (2)   (1)   (0)   (7)

					 First                                     Last
					 sent                                      sent

	The keyboard processor sets the KDAT line about 20 microseconds before it
	pulls KCLK low.  KCLK stays low for about 20 microseconds, then goes high
	again.  The processor waits another 20 microseconds before changing KDAT.

	Therefore, the bit rate during transmission is about 60 microseconds per
	bit, or 17 kbits/sec.
	*
	*
	*
	*
	*
	* Is this LookUpTable needed for translating from ascii to internal keycode?


			Row 5   Row 4   Row 3   Row 2   Row 1   Row 0
	Column    (Bit 7) (Bit 6) (Bit 5) (Bit 4) (Bit 3) (Bit 2)
		   +-------+-------+-------+-------+-------+-------+
	 15    |(spare)|(spare)|(spare)|(spare)|(spare)|(spare)|
	(PD.7) |       |       |       |       |       |       |
		   | (0E)  | (1C)  | (2C)  | (47)  | (48)  | (49)  |
		   +-------+-------+-------+-------+-------+-------+
	 14    |   *   |<SHIFT>| CAPS  |  TAB  |   ~   |  ESC  |
	(PD.6) |note 1 |note 2 | LOCK  |       |   `   |       |
		   | (5D)  | (30)  | (62)  | (42)  | (00)  | (45)  |
		   +-------+-------+-------+-------+-------+-------+
	 13    |   +   |   Z   |   A   |   Q   |   !   |   (   |
	(PD.5) |note 1 |       |       |       |   1   |note 1 |
		   | (5E)  | (31)  | (20)  | (10)  | (01)  | (5A)  |
		   +-------+-------+-------+-------+-------+-------+
	 12    |  9    |   X   |   S   |   W   |   @   |  F1   |
	(PD.4) |note 3 |       |       |       |   2   |       |
		   | (3F)  | (32)  | (21)  | (11)  | (02)  | (50)  |
		   +-------+-------+-------+-------+-------+-------+
	 11    |  6    |   C   |   D   |   E   |   #   |  F2   |
	(PD.3) |note 3 |       |       |       |   3   |       |
		   | (2F)  | (33)  | (22)  | (12)  | (03)  | (51)  |
		   +-------+-------+-------+-------+-------+-------+
	 10    |  3    |   V   |   F   |   R   |   $   |  F3   |
	(PD.2) |note 3 |       |       |       |   4   |       |
		   | (1F)  | (34)  | (23)  | (13)  | (04)  | (52)  |
		   +-------+-------+-------+-------+-------+-------+
	  9    |  .    |   B   |   G   |   T   |   %   |  F4   |
	(PD.1) |note 3 |       |       |       |   5   |       |
		   | (3C)  | (35)  | (24)  | (14)  | (05)  | (53)  |
		   +-------+-------+-------+-------+-------+-------+
	  8    |  8    |   N   |   H   |   Y   |   ^   |  F5   |
	(PD.0) |note 3 |       |       |       |   6   |       |
		   | (3E)  | (36)  | (25)  | (15)  | (06)  | (54)  |
		   +-------+-------+-------+-------+-------+-------+
	  7    |  5    |   M   |   J   |   U   |   &   |   )   |
	(PC.7) |note 3 |       |       |       |   7   |note 1 |
		   | (2E)  | (37)  | (26)  | (16)  | (07)  | (5B)  |
		   +-------+-------+-------+-------+-------+-------+
	  6    |  2    |   <   |   K   |   I   |   *   |  F6   |
	(PC.6) |note 3 |   ,   |       |       |   8   |       |
		   | (1E)  | (38)  | (27)  | (17)  | (08)  | (55)  |
		   +-------+-------+-------+-------+-------+-------+
	  5    | ENTER |   >   |   L   |   O   |   (   |   /   |
	(PC.5) |note 3 |   .   |       |       |   9   |note 1 |
		   | (43)  | (39)  | (28)  | (18)  | (09)  | (5C)  |
		   +-------+-------+-------+-------+-------+-------+
	  4    |  7    |   ?   |   :   |   P   |   )   |  F7   |
	(PC.4) |note 3 |   /   |   ;   |       |   0   |       |
		   | (3D)  | (3A)  | (29)  | (19)  | (0A)  | (56)  |
		   +-------+-------+-------+-------+-------+-------+
	  3    |  4    |(spare)|   "   |   {   |   _   |  F8   |
	(PC.3) |note 3 |       |   '   |   [   |   -   |       |
		   | (2D)  | (3B)  | (2A)  | (1A)  | (0B)  | (57)  |
		   +-------+-------+-------+-------+-------+-------+
	  2    |  1    | SPACE | <RET> |   }   |   +   |  F9   |
	(PC.2) |note 3 |  BAR  |note 2 |   ]   |   =   |       |
		   | (1D)  | (40)  | (2B)  | (1B)  | (0C)  | (58)  |
		   +-------+-------+-------+-------+-------+-------+
	  1    |  0    | BACK  |  DEL  |RETURN |   |   |  F10  |
	(PC.1) |note 3 | SPACE |       |       |   \   |       |
		   | (0F)  | (41)  | (46)  | (44)  | (0D)  | (59)  |
		   +-------+-------+-------+-------+-------+-------+
	  0    |  -    | CURS  | CURS  | CURS  | CURS  | HELP  |
	(PC.0) |note 3 | DOWN  | RIGHT | LEFT  |  UP   |       |
		   | (4A)  | (4D)  | (4E)  | (4F)  | (4C)  | (5F)  |
		   +-------+-------+-------+-------+-------+-------+

	note 1: A500, A2000 and A3000 keyboards only (numeric pad )
	note 2: International keyboards only (these keys are cutouts of the
		   larger key on the US ASCII version.)  The key that generates
		   $30 is cut out of the left Shift key.  Key $2B is cut out of
		   return.  These keys are labeled with country-specific markings.
	note 3: Numeric pad.


	The following table shows which keys are independently readable.  These
	keys never generate ghosts or phantoms.


		(Bit 6) (Bit 5) (Bit 4) (Bit 3) (Bit 2) (Bit 1) (Bit 0)
	   +-------+-------+-------+-------+-------+-------+-------+
	   | LEFT  | LEFT  | LEFT  | CTRL  | RIGHT | RIGHT | RIGHT |
	   | AMIGA | ALT   | SHIFT |       | AMIGA |  ALT  | SHIFT |
	   | (66)  | (64)  | (60)  | (63)  | (67)  | (65)  | (61)  |
	   +-------+-------+-------+-------+-------+-------+-------+

	About Hard Reset.
	-----------------
	Hard Reset happens after  Reset Warning . Valid for all keyboards
	except the Amiga 500.

	The keyboard Hard Resets the Amiga by pulling KCLK low and starting a 500
	millisecond timer.   When one or more of the keys is released and 500
	milliseconds have passed, the keyboard will release KCLK. 500 milliseconds
	is the minimum time KCLK must be held low.  The maximum KCLK time depends
	on how long the user holds the three  reset keys  down.  Circuitry on the
	Amiga motherboard detects the 500 millisecond KCLK pulse.

	After releasing KCLK, the keyboard jumps to its start-up code (internal
	RESET).  This will initialize the keyboard in the same way as cold
	power-on .

 **/

typedef enum {
	DAT_OUTPUT = 0,
	DAT_INPUT,
} kbd_dir;

static unsigned char prev_keycode = 0xff;
static unsigned char capslk = 0;
static unsigned char numlk = 0;
static unsigned char scrolllk = 0;

/* Local functions */
static uint8_t scancode_to_amiga(uint8_t lkey);
static uint8_t ascii_to_scancode(uint8_t ascii);
static void amikb_direction(kbd_dir dir);
static uint8_t amiga_keycode_send(uint8_t keycode, int press);
static void amiga_do_reset(void);
static bool amiga_protocol_is_reset(void);

static uint8_t scancode_to_amiga(uint8_t lkey)
{
	uint8_t i = 0, keyvalue = lkey;
	DBG_N("Enter with 0x%02x lkey ScanCode\r\n", lkey);

	for (i = 0; i < KEYCODE_TAB_SIZE; i++)
	{
		DBG_N("SCANCODE: 0x%02x - AMIGA: 0x%02x\r\n",
			scancodeamiga[i][0], scancodeamiga[i][1]);
		if (lkey == scancodeamiga[i][0])
		{
			keyvalue = scancodeamiga[i][1];
			break;
		}
	}

	DBG_V("Exit with: 0x%02x Amiga ScanCode\r\n", keyvalue);
	return keyvalue;
}

static uint8_t ascii_to_scancode(uint8_t ascii)
{
	uint8_t i = 0, keyvalue = ascii;
	DBG_N("Enter with '%c' 0x%02x ASCII\r\n", ascii, ascii);

	for (i = 0; i < KEYCODE_TAB_SIZE; i++)
	{
		if (ascii == asciiscancode[i][1])
		{
			keyvalue = asciiscancode[i][0];
			break;
		}
	}

	DBG_N("Exit with: 0x%02x ScanCode\r\n", keyvalue);
	return keyvalue;
}
// **************************
static void amikb_direction(kbd_dir dir)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	DBG_N("Enter with: %d\r\n", dir);
	/*Configure GPIO KB_DAT, GPIO KB_CLK as inputs and KB_RST as output */
	GPIO_InitStruct.Pin = KB_CLK_Pin | KB_DAT_Pin;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	switch (dir)
	{
		case DAT_INPUT:
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			break;

		default:
		case DAT_OUTPUT:
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			break;
	}
	HAL_GPIO_Init(KB_AMIGA_GPIO_Port, &GPIO_InitStruct);

	/* Now configure KBD_RST_Pin as output */
	GPIO_InitStruct.Pin = KB_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(KB_AMIGA_GPIO_Port, &GPIO_InitStruct);

	DBG_N("Exit\r\n");
}

void amikb_startup(void)
{
	uint8_t AMIGA_INITPOWER = 0xFD; //11111101
	uint8_t AMIGA_TERMPOWER = 0xFE; //11111110

	DBG_N("Enter\r\n");

	// De-assert nRESET for Amiga...
	amiga_protocol_reset();

	amikb_direction(DAT_OUTPUT); // Default

	mdelay(1000);              // wait for sync
	amiga_keycode_send((uint8_t) AMIGA_INITPOWER, 0); // send "initiate power-up"
	udelay(200);
	amiga_keycode_send((uint8_t) AMIGA_TERMPOWER, 0); // send "terminate power-up"

	DBG_N("Exit\r\n");
}

static int keyboard_is_present = 0;
void amikb_ready(int isready)
{
	keyboard_is_present = isready;
}

void amikb_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	DBG_N("Enter\r\n");

	/* GPIO Port A Clock Enable for Amiga Protocol Pins */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port,
		KB_CLK_Pin |
		KB_DAT_Pin  |
		KB_RST_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : KBD_CLOCK_Pin KBD_DATA_Pin KBD_RESET_Pin */
	GPIO_InitStruct.Pin = KB_CLK_Pin | KB_DAT_Pin | KB_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(KB_AMIGA_GPIO_Port, &GPIO_InitStruct);

	// Lasciamo l'Amiga in RESET finche' non parte la macchina a stati
	HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_RST_Pin, GPIO_PIN_RESET); // Clear KBD_RESET pin

	DBG_N("Exit\r\n");
}


static uint8_t amiga_keycode_send(uint8_t keycode, int press)
{
	int i;
	uint8_t rval = 0; /* No LEDS */

	DBG_N("Amiga Keycode 0x%02x - %s\r\n", keycode, press ? "PRESSED" : "RELEASED");
	if (keycode == 0x62 || keycode == 0x68 || keycode == 0x1c) // Caps Lock, Num Lock or Scroll Lock Pressed or Released
	{
		// caps lock doesn't get a key release event when the key is released
		// but rather when the caps lock is toggled off again
		// But what about num lock?

		switch (keycode)
		{
			case 0x62: // CAPS LOCK LED
				if (!capslk)
				{
					if (press)
					{
						DBG_V("### SEND TURN-ON CAPS LOCK LED. ALL UPPERCASE FROM NOW ###\r\n");
						rval = (1 << 1); //D1
						prev_keycode = 0;
						break;
					}
					else
					{
						DBG_N("### IGNORING RELEASE FOR CAPS LOCK ###\n\r");
						// Toggle for next time press
						capslk = 1;
						prev_keycode = 0;
						return 0;
					}
				}
				else
				{
					if (press)
					{
						DBG_V("### IGNORING PRESS FOR CAPS LOCK. IT WAS ALREADY PRESSED ###\r\n");
						prev_keycode = 0;
						return 0;
					}
					else
					{
						DBG_N("### SEND TURN-OFF CAPS LOCK LED. ALL LOWERCASE FROM NOW ###\r\n");
						capslk = 0;
						rval = (0 << 1); //D1
						prev_keycode = 0;
						break;
					}
				}
				break;
			case 0x68: // NUM LOCK LED
				if (!numlk)
				{
					if (press)
					{
						DBG_V("### SEND TURN-ON NUM LOCK LED. NUMERIC KEYPAD LOCKED FROM NOW ###\r\n");
						rval = (1 << 0); //D0
						prev_keycode = 0;
						break;
					}
					else
					{
						DBG_N("### IGNORING RELEASE FOR NUM LOCK ###\n\r");
						// Toggle for next time press
						numlk = 1;
						prev_keycode = 0;
						return 0;
					}
				}
				else
				{
					if (press)
					{
						DBG_V("### IGNORING PRESS FOR NUM LOCK. IT WAS ALREADY PRESSED ###\r\n");
						prev_keycode = 0;
						return 0;
					}
					else
					{
						DBG_N("### SEND TURN-OFF NUM LOCK LED. NUMERIC KEYPAD UNLOCKED FROM NOW ###\r\n");
						numlk = 0;
						rval = (0 << 0);
						prev_keycode = 0;
						break;
					}
				}
				break;
			case 0x1c: // SCROLL LOCK LED
				if (!scrolllk)
				{
					if (press)
					{
						DBG_V("### SEND TURN-ON SCROLL LOCK LED. SCROLL IS LOCKED FROM NOW ###\r\n");
						rval = (1 << 2);
						prev_keycode = 0;
						break;
					}
					else
					{
						DBG_N("### IGNORING RELEASE FOR SCROLL LOCK ###\n\r");
						// Toggle for next time press
						scrolllk = 1;
						prev_keycode = 0;
						return 0;
					}
				}
				else
				{
					if (press)
					{
						DBG_V("### IGNORING PRESS FOR SCROLL LOCK. IT WAS ALREADY PRESSED ###\r\n");
						prev_keycode = 0;
						return 0;
					}
					else
					{
						DBG_N("### SEND TURN-OFF SCROLL LOCK LED. SCROLL IS UNLOCKED FROM NOW ###\r\n");
						scrolllk = 0;
						rval = (0 << 2);
						prev_keycode = 0;
						break;
					}
				}
				break;
			default:
				{
					DBG_V("NUMLOCK %d - CAPSLOCK %d - SCROLLLOCK %d - PRESSED: %d\r\n",
						numlk, capslk, scrolllk, press);
					return 0;
				}
				break;
		}
	}

	// keycode bit transfer order: 6 5 4 3 2 1 0 7 (7 is pressed flag)
	keycode = (keycode << 1) | (~press & 1);
	if (keycode == prev_keycode)
	{
		DBG_N("NO SENDING THE SAME KEYCODE TWO TIMES IN A ROW\r\n");
		return 0;
	}

	prev_keycode = keycode;

	// Set direction DAT & CLOCK as output to ensure the correct
	// movement of the edges (CLK and DAT)
	amikb_direction(DAT_OUTPUT);

	// make sure we don't pulse the lines while grabbing control
	// by first reinstating the pullups before changing direction
	HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_DAT_Pin, GPIO_PIN_SET); // Normally KB_DAT pin is HIGH

	// pulse the data line and wait for about 20us
	HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_DAT_Pin, GPIO_PIN_RESET); // KB_DAT pin is LOW
	udelay(20);
	HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_DAT_Pin, GPIO_PIN_SET); // KBD_DATA pin is HIGH
	udelay(100);

	for (i = 0; i < 8; i++)
	{
		// data line is inverted
		if (keycode & 0x80)
		{
			// a logic 1 is low in hardware
			HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_DAT_Pin, GPIO_PIN_RESET);
		}
		else
		{
			// a logic 0 is high in hardware
			HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_DAT_Pin, GPIO_PIN_SET);
		}
		keycode <<= 1;
		udelay(20);
		/* pulse the clock */
		HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_CLK_Pin, GPIO_PIN_RESET); // Clear KB_CLOCK pin
		udelay(20);
		HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_CLK_Pin, GPIO_PIN_SET); // Set KB_CLK pin
		udelay(20);
	}

	HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_DAT_Pin, GPIO_PIN_SET); // Set KB_DATA pin

	DBG_N("DATA AND CLOCK AS INPUTS. WAITING FOR SYNC FROM CPU\r\n");
	amikb_direction( DAT_INPUT );

#ifdef REAL_AMIKEYBOARD_SYNC
	{
		int hshakepulse_us = 143;
		int sync;
		for (;;)
		{
			sync = HAL_GPIO_ReadPin(GPIOC, KBD_DATA_Pin);
			if (sync == 0 || hshakepulse_us == 0)
				break;
			hshakepulse_us--;
			udelay(1);
		}
		if (hshakepulse_us == 0)
		{
			DBG_E("CPU KEYBOARD SYNC not received within 143usec!\r\n");
		}
	}
#else
	// This can slow down the speed of typing... ????
	udelay(143);	// handshake wait 143msec
#endif
	// The following instructions should be useless as the port has been configured as input few
	// lines above... :-/
	HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_DAT_Pin,  GPIO_PIN_SET); // Set KB_DATA pin
	HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_CLK_Pin, GPIO_PIN_SET); // Set KB_CLK pin
	DBG_N("Exit: %d\r\n", rval);
	return rval;
}

static void amiga_do_reset(void)
{
	/* LIT all LEDS */
	LED_CAPS_LOCK_ON();
	LED_SCROLL_LOCK_ON();
	LED_NUM_LOCK_ON();

	/* invoke an Amiga reset */
	amiga_protocol_reset(); // This will lockup system for 600msec

	LED_CAPS_LOCK_OFF();
	LED_SCROLL_LOCK_OFF();
	LED_NUM_LOCK_OFF();

	leds = 0;
	/* Re-establish the communication protocol to the Amiga System */
	amiga_protocol_init();
}

/* **** GLOBAL FUNCTIONS ***** */

/*
 * This is done after the pins are alrerady configured. It must be done
 * in main() and after an Amiga Reset
 */
void amiga_protocol_init(void)
{
	uint8_t AMIGA_INITPOWER = 0xFD; //11111101
	uint8_t AMIGA_TERMPOWER = 0xFE; //11111110

	DBG_N("Enter\r\n");

	// De-assert nRESET for Amiga...
	amiga_protocol_reset();

	amikb_direction(DAT_OUTPUT); // Default

	mdelay(1000);              // wait for sync
	amiga_keycode_send((uint8_t) AMIGA_INITPOWER, 0); // send "initiate power-up"
	udelay(200);
	amiga_keycode_send((uint8_t) AMIGA_TERMPOWER, 0); // send "terminate power-up"

	DBG_N("Exit\r\n");
}

void amiga_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	DBG_N("Enter\r\n");

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin( KB_AMIGA_GPIO_Port,
		KB_CLK_Pin |
		KB_DAT_Pin, GPIO_PIN_SET);

	/* Keep Amiga in RESET */
	HAL_GPIO_WritePin( KB_AMIGA_GPIO_Port,
		KB_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : KB_CLK_Pin KB_DAT_Pin KB_RST_Pin */
	GPIO_InitStruct.Pin = KB_CLK_Pin | KB_DAT_Pin | KB_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(KB_AMIGA_GPIO_Port, &GPIO_InitStruct);

	DBG_N("Exit\r\n");
}

void amiga_protocol_reset(void)
{
	amikb_direction(DAT_OUTPUT);
	DBG_N("Enter\r\n");
	HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_RST_Pin, GPIO_PIN_RESET); // Clear KB_RST pin
	HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_CLK_Pin, GPIO_PIN_RESET); // Clear KB_CLK pin
	mdelay(600);
	HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_RST_Pin, GPIO_PIN_SET);   // Set KB_RESET pin
	HAL_GPIO_WritePin(KB_AMIGA_GPIO_Port, KB_CLK_Pin, GPIO_PIN_SET);   // Set KB_CLK pin
	prev_keycode = 0xff;
	/*Reset all status led */
	capslk = 0;
	numlk = 0;
	scrolllk = 0;
	DBG_N("Exit\r\n");
}

void amiga_protocol_send(keyevent_t event)
{
	/*
	 * CTRL-LGUI-RGUI == RESET AMIGA for 500msec
	 * 
	 * CTRL-[R/L]ALT-DEL   == RESET x86
	 */
	static int ctrl_pressed = 0;
	static int lgui_pressed = 0;
	static int rgui_pressed = 0;
	static int lalt_pressed = 0;
	static int ralt_pressed = 0;
	static int del_pressed  = 0;

	uint8_t leds;

	uint8_t scancode = keymaps[0][event.key.row][event.key.col];

	// Scan each report to find a group of keys pressed/released
	uint8_t amiga_scancode = scancode_to_amiga(scancode);

	/* Send to Amiga the correct scancode and returns the LED status */
	leds = amiga_keycode_send(amiga_scancode, event.pressed );

	// Now we can check if there is a special combination for invoking a
	// reset
	switch( scancode )
	{
		case KC_LCTRL:
			if (event.pressed) ctrl_pressed = 1;
			else ctrl_pressed = 0;
			break;
		case KC_LGUI:
			if (event.pressed) lgui_pressed = 1;
			else lgui_pressed = 0;
			break;
		case KC_RGUI:
			if (event.pressed) rgui_pressed = 1;
			else rgui_pressed = 0;
			break;
		case KC_LALT:
			if (event.pressed) lalt_pressed = 1;
			else lalt_pressed = 0;
			break;
		case KC_RALT:
			if (event.pressed) ralt_pressed = 1;
			else ralt_pressed = 0;
			break;
		case KC_DEL:
			if (event.pressed) del_pressed = 1;
			else del_pressed = 0;
		default:
			break;
	}

	/* 
	 * We can reset the Amiga, using the classic 3 finger salute for
	 * Amiga Computers CTRL-Amiga-Amiga or the more common CTRL-ALT-DEL
	 * LALT or RALT does not matter. Even for CTRL it does not matter
	 * if RCTRL or LCTRL
	 */
	if (ctrl_pressed && lgui_pressed && rgui_pressed)
	{
		amiga_do_reset();
	}
	else
	if (ctrl_pressed && del_pressed)
	{
		if (lalt_pressed || ralt_pressed)
		{
			amiga_do_reset();
		}
	}

	// Now the leds can be modified
	hook_keyboard_leds_change(leds);
}


// ****************************
static bool amiga_protocol_is_reset(void)
{
	/* If connected to a real Amiga Computers, the RST Line (reset) can be
	 * driven without our intervention. As long as the reset line stays
	 * low for at least more than 400msec, a polling is enough to achieve
	 * that. Even the CLK line can be used as reset indicator, so we need
	 * to check that too.
	 */
	bool is_low_clk, is_low_rst;
	bool is_reset = false;
	DBG_N("Enter\r\n");
	amikb_direction( DAT_INPUT );
	is_low_clk = HAL_GPIO_ReadPin(KB_AMIGA_GPIO_Port, KB_CLK_Pin) == GPIO_PIN_RESET ? true : false;
	is_low_rst = HAL_GPIO_ReadPin(KB_AMIGA_GPIO_Port, KB_RST_Pin) == GPIO_PIN_RESET ? true : false;
	DBG_N("KBD_CLOCK is %s\r\n", is_low_clk == false ? "LOW" : "HIGH");
	DBG_N("KBD_RESET is %s\r\n", is_low_rst == false ? "LOW" : "HIGH");
	if ((is_low_clk == true) || (is_low_rst == true)) is_reset = true;
	DBG_N("Exit with %s\r\n", is_reset ? "RESET ASSERTED" : "NOT IN RESET");
	return is_reset;
}

/* This function is declared __weak somewhere. Now it is real. Called from keyboard_task()
 */
void hook_keyboard_loop(void)
{
	int reset_active_ms = AMIGA_RESET_TIMEOUT_MS;
	/* Check for any RST assert from Amiga hardware */
	if (amiga_protocol_is_reset())
	{
		/* need to check if there is a running nRESET!
		 * It is a reset if it is more than 300ms!
		 */
		DBG_N("RESET count: %d\r\n", reset_active_ms);
		reset_active_ms--;
		if (reset_active_ms < 1)
		{
			DBG_E("AMIGA PROTOCOL RESET ASSERT. Restart Communication with the motherboard\n\r");
			amiga_protocol_init();
			reset_active_ms = AMIGA_RESET_TIMEOUT_MS;
		}
	}
	else
	{
		DBG_N("NOT IN RESET\r\n");
		/* If there is no nRESET asserted, reload the timer count */
		reset_active_ms = AMIGA_RESET_TIMEOUT_MS;
	}
}
