// SPDX-License-Identifier: MIT

#include "debug_output.h"
#include <stdint.h>
#include "ch32v003fun.h"

static int debug_module_likely_attached = 1;
static uint32_t calls_since_recheck = 0;
#define DEBUG_MODULE_RECHECK_INTERVAL 200

void debug_output_write(const char* buf, int size) {
    if (!debug_module_likely_attached) {
        if (++calls_since_recheck < DEBUG_MODULE_RECHECK_INTERVAL) {
            return;
        }
        calls_since_recheck = 0;  // worth another real attempt, in case a debugger just attached
    }

    char buffer[4] = {0};
    int place = 0;
    uint32_t lastdmd;
    uint32_t timeout = FUNCONF_DEBUGPRINTF_TIMEOUT;  // Give up after ~40ms

    while (place < size) {
        int tosend = size - place;
        if (tosend > 7) {
            tosend = 7;
        }

        while ((lastdmd = (*DMDATA0)) & 0x80) {
            if (timeout-- == 0) {
                debug_module_likely_attached = 0;
                return;
            }
        }
        debug_module_likely_attached = 1;
        (void)lastdmd;

        timeout = FUNCONF_DEBUGPRINTF_TIMEOUT;

        int t = 3;
        while (t < tosend) {
            buffer[t - 3] = buf[t + place];
            t++;
        }
        *DMDATA1 = *(uint32_t*)&(buffer[0]);
        t = 0;
        while (t < tosend && t < 3) {
            buffer[t + 1] = buf[t + place];
            t++;
        }
        buffer[0] = 0x80 | (tosend + 4);
        *DMDATA0 = *(uint32_t*)&(buffer[0]);

        place += tosend;
    }
}
