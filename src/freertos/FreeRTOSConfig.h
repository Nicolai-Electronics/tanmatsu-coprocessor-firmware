#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions for the Tanmatsu coprocessor firmware.
 *
 * Based on ch32fun_freertos/FreeRTOS/FreeRTOSConfig.h (the WCH QingKe V4C
 * RISC-V port), adjusted for the CH32V203 coprocessor firmware:
 *  - configTICK_RATE_HZ raised to 1000 to preserve the 1ms keyboard scan.
 *  - configMAX_PRIORITIES trimmed to the priorities actually used.
 *  - configUSE_TIMERS disabled: no software timers are used, every
 *    periodic job is a task with its own vTaskDelay() loop.
 *
 * See https://www.freertos.org/Using-FreeRTOS-on-RISC-V.html
 *----------------------------------------------------------*/

/* Don't have MTIME/MTIMECMP (CLINT) on this core, the port uses SysTick instead. */
#define configMTIME_BASE_ADDRESS                  ( 0 )
#define configMTIMECMP_BASE_ADDRESS               ( 0 )

#define configUSE_PREEMPTION                      1
#define configUSE_TIME_SLICING                    0
#define configUSE_IDLE_HOOK                       0
#define configUSE_TICK_HOOK                       0
#define configCPU_CLOCK_HZ                        FUNCONF_SYSTEM_CORE_CLOCK
#define configTICK_RATE_HZ                        ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                      ( 6 )
#define configMINIMAL_STACK_SIZE                  ( ( unsigned short ) 128 )
#define configISR_STACK_SIZE_WORDS                ( 128 )
/* heap_4 arena: every task's stack (xTaskCreate's usStackDepth, in words) and TCB, plus
   the idle task and any semaphores, are pvPortMalloc()'d out of this one fixed-size
   pool. src/application/main.c currently creates 6 tasks whose stacks alone total
   2560 words = 10240 bytes; add the idle task's stack, 7 TCBs, and the PMIC I2C mutex
   (src/hal/i2c_master.c) and the real requirement is close to 12KB. This must stay
   comfortably above that sum - xTaskCreate()/xSemaphoreCreateMutex() fail silently
   (return NULL/pdFAIL) rather than fault if the heap runs out, so undersizing this
   silently drops whichever tasks are created last once the pool is exhausted instead
   of producing any visible error. main() now checks xTaskCreate()'s return value and
   calls fault_blink() if this ever happens again. */
#define configTOTAL_HEAP_SIZE                     ( ( size_t ) ( 14 * 1024 ) )
#define configSUPPORT_DYNAMIC_ALLOCATION          1
#define configMAX_TASK_NAME_LEN                   ( 16 )
#define configUSE_TRACE_FACILITY                  0
#define configUSE_16_BIT_TICKS                    0
#define configIDLE_SHOULD_YIELD                   0
#define configUSE_MUTEXES                         1
#define configQUEUE_REGISTRY_SIZE                 8
#define configCHECK_FOR_STACK_OVERFLOW            2
#define configUSE_RECURSIVE_MUTEXES               1
#define configUSE_MALLOC_FAILED_HOOK              0
#define configUSE_APPLICATION_TASK_TAG            0
#define configUSE_COUNTING_SEMAPHORES             1
#define configGENERATE_RUN_TIME_STATS             0
#define configUSE_PORT_OPTIMISED_TASK_SELECTION   0

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                     0
#define configMAX_CO_ROUTINE_PRIORITIES           ( 2 )

/* Software timer definitions. Unused by this firmware: every periodic job is
   its own task with a vTaskDelay() loop, so the timer daemon task is not
   needed. */
#define configUSE_TIMERS                          0

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet                  1
#define INCLUDE_uxTaskPriorityGet                 1
#define INCLUDE_vTaskDelete                       1
#define INCLUDE_vTaskCleanUpResources              1
#define INCLUDE_vTaskSuspend                      1
#define INCLUDE_vTaskDelayUntil                   1
#define INCLUDE_vTaskDelay                        1
#define INCLUDE_eTaskGetState                     1
#define INCLUDE_xTaskAbortDelay                   1
#define INCLUDE_xTaskGetHandle                    1
#define INCLUDE_xSemaphoreGetMutexHolder          1

/* Defined in src/application/main.c: blinks the addressable power LED forever to
   give a visible fault indication, for use where neither a debugger nor the host
   I2C bus can be relied on to report what went wrong (e.g. here, a FreeRTOS
   assertion failure). Never returns. */
extern void fault_blink(unsigned int code);

/* Normal assert() semantics without relying on the provision of an assert.h
header file. Code 5 identifies this as a FreeRTOS configASSERT() failure, as
opposed to a CPU trap (src/application/main.c) or a task stack overflow. */
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); printf("err at line %d of file \"%s\". \r\n ",__LINE__,__FILE__); fault_blink(5); }

/* Map to the platform printf function. */
#define configPRINT_STRING( pcString )  printf( pcString )

#endif /* FREERTOS_CONFIG_H */
