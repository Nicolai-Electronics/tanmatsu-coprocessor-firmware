#pragma once

/*
 * The vendored FreeRTOS WCH RISC-V port (port.c/portmacro.h) is written
 * against upstream ch32fun's "ch32fun.h" header. This project's platform
 * layer is an older fork of the same project under the name
 * "ch32v003fun.h" (see src/platform/ch32v003fun.h), functionally
 * equivalent but with a couple of naming differences. This shim bridges
 * those differences so the port sources can be used unmodified.
 */
#include "ch32v003fun.h"

/* This fork's IRQn enum has a typo (capital K) upstream doesn't have. */
#ifndef SysTick_IRQn
#define SysTick_IRQn SysTicK_IRQn
#endif

/* Fast/high-speed code section placement attribute, not present in this
   fork. Defining it away just keeps the affected functions in normal
   flash, same as every other ISR in this codebase. */
#ifndef __HIGH_CODE
#define __HIGH_CODE
#endif

#ifndef __INTERRUPT
#define __INTERRUPT __attribute__((interrupt))
#endif
