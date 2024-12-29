// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// --------------- //
// cmd_strobe_trig //
// --------------- //

#define cmd_strobe_trig_wrap_target 0
#define cmd_strobe_trig_wrap 2

static const uint16_t cmd_strobe_trig_program_instructions[] = {
            //     .wrap_target
    0x201c, //  0: wait   0 gpio, 28                 
    0xc021, //  1: irq    wait 1                     
    0x209c, //  2: wait   1 gpio, 28                 
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program cmd_strobe_trig_program = {
    .instructions = cmd_strobe_trig_program_instructions,
    .length = 3,
    .origin = -1,
};

static inline pio_sm_config cmd_strobe_trig_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + cmd_strobe_trig_wrap_target, offset + cmd_strobe_trig_wrap);
    return c;
}

#include "hardware/clocks.h"
#include "hardware/gpio.h"
static inline void cmd_strobe_trig_program_init(PIO pio, uint sm, uint offset) 
{    
    pio_sm_config c = cmd_strobe_trig_program_get_default_config(offset);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

#endif

