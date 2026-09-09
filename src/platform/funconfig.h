#pragma once

#define FUNCONF_USE_DEBUGPRINTF 1
// printf()/putchar() write to the SWD debug interface register (DMDATA0/1). Nothing
// reads that register in normal operation (no debugger attached), so without a
// timeout _write()/putchar() (src/platform/ch32v003fun.c) busy-wait for
// FUNCONF_DEBUGPRINTF_TIMEOUT loop iterations per call trying to hand off to a
// debugger that isn't there - with interrupts disabled during a hard fault/assert,
// that stalls the whole system. 0 means "give up immediately instead of waiting".
#define FUNCONF_DEBUGPRINTF_TIMEOUT 0
#define FUNCONF_DEBUG 1
