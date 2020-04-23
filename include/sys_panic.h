/*
 * Interface to panic through when impossible or otherwise unrecoverable
 * situations occur.
 *
 * Expect debug builds to simply halt execution when panics happen.
 * Expect release builds to reboot, possibly recording the event somewhere in
 * a way that does not depend on the system being in a reasonable state.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef SYS_PANIC_H_
#define SYS_PANIC_H_

// Make use of the __MODUUL__ variable when already defined for LLL.
#ifndef __MODUUL__
    //#warning no __MODUUL__ defined, using __FILE__
    #define __MODUUL__ __FILE__
#endif

#ifndef TOSSIM
    #define STATIC_CONST static const
#else
    #define STATIC_CONST
#endif

extern const char * g_panic_file;
extern int          g_panic_line;
extern const char * g_panic_str;

//-----------------------------------------------------------
// What to do when panic state has been recorded.
// !!! Currently only supports ARM-CortexM !!!
#ifndef NDEBUG

    // Halt the device - disable interrupts and loop forever
    #define sys_panic_action()                                  \
    ({                                                          \
        __asm__ volatile("cpsid i" : : : "memory"); while(1);     \
    })

#else
    #include "device.h" // Need NVIC_SystemReset
    // Reboot the device
    #define sys_panic_action()                                  \
    ({                                                          \
        NVIC_SystemReset();                                     \
    })

#endif//NDEBUG
//-----------------------------------------------------------

/**
 * Panic, recording the source(file), line and an info string.
 */
#define sys_panic(str)                                      \
({                                                          \
    STATIC_CONST char moduulROM[] PROGMEM = __MODUUL__;     \
    STATIC_CONST char strROM[] PROGMEM = str;               \
        g_panic_file = moduulROM;                           \
        g_panic_line = __LINE__;                            \
        g_panic_str = strROM;                               \
        sys_panic_action();                                 \
})

#endif//SYS_PANIC_H_
