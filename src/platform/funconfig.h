#pragma once

// Left at 0 (unlike most ch32fun projects) because this project provides its
// own putchar()/_write() (src/application/main.c), which both buffer into a
// software ring buffer the host can read back over I2C (I2C_REG_DEBUG_0..7)
// and forward to the SWD debug interface via debug_output_write()
// (src/application/debug_output.c) - setting this to 1 would make
// src/platform/ch32fun.c provide its own conflicting putchar()/_write()
// definitions (duplicate symbol at link time).
#define FUNCONF_USE_DEBUGPRINTF 0
// printf()/putchar() write to the SWD debug interface register (DMDATA0/1) via
// debug_output_write() (src/application/debug_output.c) - a debugger (e.g.
// `minichlink -T`) has to actively poll DMDATA0 over USB to drain each 7-byte
// chunk before the next one can be written, which takes on the order of a
// few ms of round-trip latency. With this at 0, debug_output_write()'s very
// first busy-check fails instantly (before the debugger has any realistic
// chance to have drained anything yet), marks the debug module as "not
// attached", and skips actually sending for the next
// DEBUG_MODULE_RECHECK_INTERVAL (200) calls - in practice this made the SWD
// output permanently empty even with a debugger attached and polling,
// because most log lines are longer than one 7-byte chunk and so always
// hit a busy DMDATA0 on their second chunk. Confirmed on real hardware:
// with this at 0, `minichlink -T` showed nothing at all, while the
// software ring buffer -> I2C_REG_DEBUG_0..7 path (unaffected by this
// timeout, see putchar() in src/application/main.c) worked fine. A few
// hundred thousand iterations is enough headroom for that USB round trip
// while still bounding the worst-case stall (e.g. from fault_blink() with
// interrupts disabled and no debugger attached) to a handful of ms.
#define FUNCONF_DEBUGPRINTF_TIMEOUT 1000000
#define FUNCONF_DEBUG 1
