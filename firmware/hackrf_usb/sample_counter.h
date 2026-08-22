#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
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

void sample_counter_capture_enable(void);
void sample_counter_capture_disable(void);

/*
 * Pack each event into one word: timestamp[29:0], source, rising-edge flag.
 * The receiver reconstructs the two omitted timestamp bits relative to the
 * IQ block index carried in the same metadata group.
 */
size_t sample_counter_capture_drain_packed(uint32_t* dest, size_t max_events);
uint32_t sample_counter_capture_dropped(void);
uint32_t sample_counter_capture_high_water(void);
uint32_t sample_counter_capture_capacity(void);
uint32_t sample_counter_capture_filtered(uint8_t source);
