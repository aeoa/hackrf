/*
 * Minimal SCTimer counter interface for PPS experimentation.
 */

#ifndef HACKRF_USB_PPS_COUNTER_H
#define HACKRF_USB_PPS_COUNTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
	SAMPLE_COUNTER_NO_EDGE = 0,
	SAMPLE_COUNTER_EDGE_FALLING = 1,
	SAMPLE_COUNTER_EDGE_RISING = 2,
} sample_counter_edge_t;

typedef struct sample_counter_event {
	uint32_t timestamp;
	sample_counter_edge_t edge;
} sample_counter_event_t;

uint32_t sample_counter_read(void);

void sample_counter_capture_enable(void);
void sample_counter_capture_disable(void);
size_t sample_counter_capture_drain(
	sample_counter_event_t* dest,
	size_t max_events);

#endif /* HACKRF_USB_PPS_COUNTER_H */
