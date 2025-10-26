/*
 * Simple PPS capture helper for HackRF using the SCTimer.
 */

#ifndef HACKRF_USB_PPS_CAPTURE_H
#define HACKRF_USB_PPS_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	uint32_t ticks; /* Raw SCT tick count captured on the edge. */
} pps_capture_event_t;

void pps_capture_init(void);
void pps_capture_shutdown(void);
void pps_capture_reset(void);
bool pps_capture_dequeue(pps_capture_event_t* event);
uint32_t pps_capture_counter(void);

extern volatile uint32_t debug_counter_irq_handler;

#endif /* HACKRF_USB_PPS_CAPTURE_H */
