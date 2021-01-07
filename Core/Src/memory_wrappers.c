/*
 * Wrappers for malloc and free
 * Copyright © 2020, Mario Senecic
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * https://github.com/802-15/FreeRTOS-robustness-improvement
 *
 */

#include "memory_wrappers.h"


#if ( USE_MALLOC == 0 )

void * __real_malloc(size_t size);
void * __wrap_malloc(size_t size)
{
  /* Prevent user's application from ever calling malloc while using FreeRTOS
   * with newlib - if we don't provide the application heap.
   *
   * To wrap malloc "-Wl,--wrap=malloc" needs to be added to the linker flags
   * variable 'LDFLAGS' in the Makefile
   */
  (void) size;
  for(;;);
}

/* Use FreeRTOS's memory management scheme. */

void * user_malloc(size_t size)
{
    void * ptr = NULL;
    ptr = pvPortMalloc(size);
    return ptr;
}

void user_free(void * ptr)
{
    if(ptr) {
        vPortFree(ptr);
    }
}

#endif

#if ( USE_MALLOC == 1 )

/* Use newlib's memory management scheme. Wrap the newlib malloc/free
 * in a critical section to provide thread safety. Provide a minimal
 * sbrk implementation and allocate enough heap in the linker script.
 */

void * user_malloc(size_t size)
{
    void * ptr = NULL;

    taskENTER_CRITICAL();

    ptr = malloc(size);

    taskEXIT_CRITICAL();

    return ptr;
}

void user_free(void * ptr)
{
    taskENTER_CRITICAL();

    if(ptr) {
        free(ptr);
    }

    taskEXIT_CRITICAL();
}

/* Minimal sbrk implementation: increase program data space
 * https://sourceware.org/newlib/libc.html#index-sbrk
 * Defined as weak here:
 * https://github.com/eblot/newlib/blob/master/libgloss/arm/linux-syscalls1.c
 */

void * _sbrk(int incr)
{
    /* _end symbol is defined by the linker (see linker script) */
    extern char _end;

    taskENTER_CRITICAL();

    static char *heap_end;
    static char *heap_start;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_end;
        heap_start = &_end;
    }

    prev_heap_end = heap_end;

    /* FreeRTOS friendly heap and stack collision check */
    if ( (prev_heap_end + incr) - heap_start > HEAP_MAX_SIZE) {
        for(;;);
    }

    heap_end += incr;

    taskEXIT_CRITICAL();

    return (void *) prev_heap_end;
}

/* These memory pool locks are provided as in structed in the newlib manual.
 * sbrk already contains the appropriate modifications, these locks
 * are kept just in case since other standard library functions must also
 * provide memory pool protection and they get may get called if the standard
 * library functions are not used carefully.
 */

void __malloc_lock(struct _reent *r)
{
    (void) r;
    taskENTER_CRITICAL();
}

void __malloc_unlock(struct _reent *r)
{
    (void) r;
    taskEXIT_CRITICAL();
}

#endif