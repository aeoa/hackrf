/*
 * Minimal SCTimer counter interface for PPS experimentation.
 */

#ifndef HACKRF_USB_PPS_COUNTER_H
#define HACKRF_USB_PPS_COUNTER_H

#include <stdbool.h>
#include <stdint.h>

void pps_counter_init(void);
uint32_t pps_counter_read(void);

#endif /* HACKRF_USB_PPS_COUNTER_H */
