#include <stdint.h>
#include "main.h"
#include "keyboard.h"
#include "keycode.h"
#include "config.h"
#include "amiga_config.h"
#include "debug.h"
#include "host.h"
#include "host_driver.h"
#include "usbd_hid.h"
#include "usb_device.h"
#include "amiga_protocol.h"

/* Here we can use USB and a couple of led:
 * the main led (usable for reporting errors
 * the caps lock led, the num lock led and others
 */

uint8_t usb_keyboard_leds(void);
void usb_send_keyboard(report_keyboard_t *report);

host_driver_t usbdriver = {
	usb_keyboard_leds,
	usb_send_keyboard,
	NULL, // void (*send_mouse)(report_mouse_t *);
	NULL, // void (*send_system)(uint16_t);
	NULL, // (*send_consumer)(uint16_t);
};

static int debuglevel = DBG_INFO;


#define KC_SPARE_1 KC_NO
#define KC_SPARE_2 KC_NO
#define KC_SPARE_3 KC_NO

	/*
	 * This keyboard layout needs a specific layout. It must be done
	 * in Linux with X11, Windows or MacOS specific.
	 * 
	 * 
	 * Layer 0: Default Layer
	 * 
	 * ,---.     ,------------------------.   ,-------------------------.
	 * |Esc|     | F1 | F2 | F3 | F4 | F5 |   | F6 | F7 | F8 | F9 | F10 |
	 * `---.     `------------------------.   `-------------------------.
	 *
	 * ,--------------------------------------------------------------.    ,-----. ,-----.      ,---------------.
	 * |  `  |  1|  2|  3|  4|  5|  6|  7|  8|  9|  0|  -|  =|  \| BS |    | DEL | | HELP|      | ( | ) | / | * |
	 * |--------------------------------------------------------------|    `-----. `-----.      `---------------| 
	 * |Tab    |  Q|  W|  E|  R|  T|  Y|  U|  I|  O|  P| [ | ] |  RET |                         | 7 | 8 | 9 | - |
	 * |----------------------------------------------------------+   |         ,----.          `---------------| 
	 * |Ctrl| CAPS|  A|  S|  D|  F|  G|  H|  J|  K|  L|  ;|  '| BL|   |         | UP |          | 4 | 5 | 6 | + |  
	 * |--------------------------------------------------------------|    ,----+----+----.     `---------------|
	 * | Shift | BL|  Z|  X|  C|  V|  B|  N|  M|  ,|  .|  /|  Shift   |    |LFT |DOWN| RGT|     | 1 | 2 | 3 | E |
	 * `--------------------------------------------------------------'    `--------------'     `------------ N |
	 *     | LAlt| LGUI|   |       Space      |   | RGUI| RAlt |                                |   0   | . | T |
	 *     `---------------------------------------------------'                                `---------------'
	 *
	 * BL = spare blank keys (not used in Standard International Mode)
	 * 
	 */

const uint8_t keymaps[][KEYBOARD_ROWS][KEYBOARD_COLUMNS] = {
	[0] = {
		{ KC_ESC,     KC_KP_LPAREN,  KC_F1,  KC_F2,  KC_F3,  KC_F4,    KC_F5,  KC_KP_RPAREN,  KC_F6,    KC_PSLS,  KC_F7,    KC_F8,       KC_F9,      KC_F10,   KC_HELP,  KC_NO },
		{ KC_GRV,     KC_1,          KC_2,   KC_3,   KC_4,   KC_5,     KC_6,   KC_7,          KC_8,     KC_9,     KC_0,     KC_MINS,     KC_EQL,     KC_BSLS,  KC_UP,    KC_NO },
		{ KC_TAB,     KC_Q,          KC_W,   KC_E,   KC_R,   KC_T,     KC_Y,   KC_U,          KC_I,     KC_O,     KC_P,     KC_LBRC,     KC_RBRC,    KC_ENT,   KC_LEFT,  KC_NO },
		{ KC_CAPS,    KC_A,          KC_S,   KC_D,   KC_F,   KC_G,     KC_H,   KC_J,          KC_K,     KC_L,     KC_SCLN,  KC_QUOT,     KC_SPARE_1, KC_DEL,   KC_RIGHT, KC_NO },
		{ KC_SPARE_2, KC_Z,          KC_X,   KC_C,   KC_V,   KC_B,     KC_N,   KC_M,          KC_COMM,  KC_DOT,   KC_SLSH,  KC_SPARE_3,  KC_SPC,     KC_BSPC,  KC_DOWN,  KC_NO },
		{ KC_PAST,    KC_PPLS,       KC_P9,  KC_P6,  KC_P3,  KC_PCMM,  KC_P8,  KC_P5,         KC_P2,    KC_PENT,  KC_P7,    KC_P4,       KC_P1,      KC_P0,    KC_PMNS,  KC_NO },
		{ KC_NO,      KC_NO,         KC_NO,  KC_NO,  KC_NO,  KC_NO,    KC_NO,  KC_NO,         KC_NO,    KC_NO,    KC_NO,    KC_NO,       KC_NO,      KC_NO,    KC_NO,    KC_RSFT },
		{ KC_NO,      KC_NO,         KC_NO,  KC_NO,  KC_NO,  KC_NO,    KC_NO,  KC_NO,         KC_NO,    KC_NO,    KC_NO,    KC_NO,       KC_NO,      KC_NO,    KC_NO,    KC_RALT },
		{ KC_NO,      KC_NO,         KC_NO,  KC_NO,  KC_NO,  KC_NO,    KC_NO,  KC_NO,         KC_NO,    KC_NO,    KC_NO,    KC_NO,       KC_NO,      KC_NO,    KC_NO,    KC_RGUI },
		{ KC_NO,      KC_NO,         KC_NO,  KC_NO,  KC_NO,  KC_NO,    KC_NO,  KC_NO,         KC_NO,    KC_NO,    KC_NO,    KC_NO,       KC_NO,      KC_NO,    KC_NO,    KC_LCTRL },
		{ KC_NO,      KC_NO,         KC_NO,  KC_NO,  KC_NO,  KC_NO,    KC_NO,  KC_NO,         KC_NO,    KC_NO,    KC_NO,    KC_NO,       KC_NO,      KC_NO,    KC_NO,    KC_LSFT },
		{ KC_NO,      KC_NO,         KC_NO,  KC_NO,  KC_NO,  KC_NO,    KC_NO,  KC_NO,         KC_NO,    KC_NO,    KC_NO,    KC_NO,       KC_NO,      KC_NO,    KC_NO,    KC_LALT },
		{ KC_NO,      KC_NO,         KC_NO,  KC_NO,  KC_NO,  KC_NO,    KC_NO,  KC_NO,         KC_NO,    KC_NO,    KC_NO,    KC_NO,       KC_NO,      KC_NO,    KC_NO,    KC_LGUI },
	},
};

gpioPort_t lut_row[ KEYBOARD_ROWS ] = {
	{ .port = ROW0_GPIO_Port,  .pin = ROW0_Pin },
	{ .port = ROW1_GPIO_Port,  .pin = ROW1_Pin },
	{ .port = ROW2_GPIO_Port,  .pin = ROW2_Pin },
	{ .port = ROW3_GPIO_Port,  .pin = ROW3_Pin },
	{ .port = ROW4_GPIO_Port,  .pin = ROW4_Pin },
	{ .port = ROW5_GPIO_Port,  .pin = ROW5_Pin },
	{ .port = ROW6_GPIO_Port,  .pin = ROW6_Pin },
	{ .port = ROW7_GPIO_Port,  .pin = ROW7_Pin },
	{ .port = ROW8_GPIO_Port,  .pin = ROW8_Pin },
	{ .port = ROW9_GPIO_Port,  .pin = ROW9_Pin },
	{ .port = ROW10_GPIO_Port, .pin = ROW10_Pin },
	{ .port = ROW11_GPIO_Port, .pin = ROW11_Pin },
	{ .port = ROW12_GPIO_Port, .pin = ROW12_Pin },
};

gpioPort_t lut_col[ KEYBOARD_COLUMNS ] = {
	{ .port = COL0_GPIO_Port,  .pin = COL0_Pin },
	{ .port = COL1_GPIO_Port,  .pin = COL1_Pin },
	{ .port = COL2_GPIO_Port,  .pin = COL2_Pin },
	{ .port = COL3_GPIO_Port,  .pin = COL3_Pin },
	{ .port = COL4_GPIO_Port,  .pin = COL4_Pin },
	{ .port = COL5_GPIO_Port,  .pin = COL5_Pin },
	{ .port = COL6_GPIO_Port,  .pin = COL6_Pin },
	{ .port = COL7_GPIO_Port,  .pin = COL7_Pin },
	{ .port = COL8_GPIO_Port,  .pin = COL8_Pin },
	{ .port = COL9_GPIO_Port,  .pin = COL9_Pin },
	{ .port = COL10_GPIO_Port, .pin = COL10_Pin },
	{ .port = COL11_GPIO_Port, .pin = COL11_Pin },
	{ .port = COL12_GPIO_Port, .pin = COL12_Pin },
	{ .port = COL13_GPIO_Port, .pin = COL13_Pin },
	{ .port = COL14_GPIO_Port, .pin = COL14_Pin },
	{ .port = COL15_GPIO_Port, .pin = COL15_Pin },
};

static uint8_t leds = 0;

/* MASK of leds:
 * D7 D6 D5 D4 D3 D2 D1 D0
 * -----------------------
 * 
 * D0: NUM LOCK
 * D1: CAPS LOCK
 * D2: SCROLL LOCK
 * D3..D7 = not used
 */
uint8_t usb_keyboard_leds(void)
{
	return leds;
}

uint8_t keymap_key_to_keycode(uint8_t layer, keypos_t key)
{
	return keymaps[(layer)][key.col][key.row];
}

action_t keymap_fn_to_action(uint8_t keycode)
{
    return (action_t) fn_actions[FN_INDEX(keycode)];
}

/* From Middlewares Initialization Stuff... */
extern USBD_HandleTypeDef hUsbDeviceFS;

void usb_send_keyboard(report_keyboard_t *report)
{
	int i;
	unsigned char * ptr = (unsigned char *) report;
	if (debuglevel >= DBG_VERBOSE)
	{
		printf(ANSI_GREEN "AMIGA USB REPORT: ");
		for (i = 0; i < sizeof(report_keyboard_t); i++)
		{
			printf("[" ANSI_RED "0x%02x" ANSI_RESET "] ", *(ptr+i));
		}
		printf(ANSI_RESET "\r\n");
	}
	USBD_HID_SendReport(&hUsbDeviceFS, (unsigned char *) report, 8 ); // buffer size
}

void USBD_HID_GetReport(uint8_t * report, int len)
{
	// see from http://www.microchip.com/forums/m433757.aspx
	// report[0] is the report id
	// report[1] is the led bit field
	// D0: NUM lock
	// D1: CAPS lock
	// D2: SCROLL lock
	const char * LED[3] = {"NUM lock", "CAPS lock", "SCROLL lock", };

	int i;
	unsigned char * ptr = (unsigned char *) report;
	if (debuglevel >= DBG_VERBOSE)
	{
		printf(ANSI_GREEN "USB GET REPORT (LED): ");
		for (i = 0; i < len; i++)
		{
			printf(ANSI_GREEN "[" ANSI_RED "0x%02x" ANSI_RESET "] ", *(ptr+i));
		}
		// Scan each led and PRINTOUT Which LED Has to be Light or Not
		uint8_t led = *(ptr);
		for (i = 0; i < 5; i++)
		{
			uint8_t idx;
			idx = led & (1 << i);
			if (idx)
			{
				printf(ANSI_YELLOW "%s " ANSI_RESET, LED[idx-1]);
			}
		}
		printf(ANSI_RESET "\r\n");
	}
	leds = *(report);
}

void hook_matrix_change(keyevent_t event)
{
	/*
	 * https://github.com/tmk/tmk_keyboard/blob/6271878a021fcf578b71e2b7e97cd43786efa7dd/tmk_core/common/action.c#L45
	 */
	DBG_N("MATRIX CHANGED EVENT KEY: ROW: %d - COL: %d -- STATUS: %s \r\n",
			event.key.row, event.key.col, event.pressed ? "PRESSED" : "RELEASED");
	DBG_N("AMIGA KEYMAP[%d, %d] = value %d (hex) 0x%02x\r\n", event.key.row, event.key.col,
			keymaps[0][event.key.row][event.key.col],
			keymaps[0][event.key.row][event.key.col]);

	/* Now we can send the keyevent to the Amiga Protocol layer */
	amiga_protocol_send(event);
}

void hook_keyboard_leds_change(uint8_t led_status)
{
	DBG_N("Called: %d\r\n", led_status);

	/* AMIGA KEYBOARD HAS 3 LEDS:
	 * Power Supply LED (Hardwired to 5V but not present in a standard layout)
	 * D0: NUM lock
	 * D1: CAPS lock
	 * D2: SCROLL lock
	 */

	if (led_status & (1 << 0))
		LED_NUM_LOCK_ON();
	else
		LED_NUM_LOCK_OFF();

	if (led_status & (1 << 1))
		LED_CAPS_LOCK_ON();
	else
		LED_CAPS_LOCK_OFF();

	if (led_status & (1 << 2))
		LED_SCROLL_LOCK_ON();
	else
		LED_SCROLL_LOCK_OFF();

}
