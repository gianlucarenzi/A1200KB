#ifndef __AMIGA_PROTOCOL_INCLUDED__
#define __AMIGA_PROTOCOL_INCLUDED__

#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include "keyboard.h"
#include "keycode.h"
#include "report.h"
#include "config.h"
#include "amiga_config.h"
#include "debug.h"

/* If the nRESET line is tied low for more than this timeout,
 * let's re-initiate the protocol to be sure, Amiga is running fine
 */
#define AMIGA_RESET_TIMEOUT_MS    300

extern void amiga_gpio_init(void);
extern void amiga_protocol_init(void);
extern void amiga_protocol_send(keyevent_t event);
extern void amiga_protocol_reset(void);

#endif
