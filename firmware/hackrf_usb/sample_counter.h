/*
 * Minimal SCTimer counter interface for PPS experimentation.
 */

#ifndef HACKRF_USB_PPS_COUNTER_H
#define HACKRF_USB_PPS_COUNTER_H

#include <stdbool.h>
#include <stdint.h>

uint32_t sample_counter_read(void);

#endif /* HACKRF_USB_PPS_COUNTER_H */
