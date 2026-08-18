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
	uint8_t source;
	uint8_t edge;
	uint16_t reserved;
} sample_counter_event_t;

typedef char sample_counter_event_must_be_8_bytes[
	(sizeof(sample_counter_event_t) == 8U) ? 1 : -1];

uint32_t sample_counter_read(void);

void sample_counter_capture_enable(void);
void sample_counter_capture_disable(void);
size_t sample_counter_capture_drain(
	sample_counter_event_t* dest,
	size_t max_events);
uint32_t sample_counter_capture_dropped(void);
uint32_t sample_counter_capture_high_water(void);
uint32_t sample_counter_capture_capacity(void);

#endif /* HACKRF_USB_PPS_COUNTER_H */
