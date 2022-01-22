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

extern void amiga_gpio_init(void);
extern void amiga_protocol_init(void);
extern void amiga_protocol_send(keyevent_t event);
extern void amiga_protocol_reset(void);
extern bool amiga_protocol_is_reset(void);

#endif
