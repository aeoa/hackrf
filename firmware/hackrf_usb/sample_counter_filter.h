#pragma once

#include "sample_counter.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * The SCT runs at twice the IQ sample rate. Legitimate time-pulse edges are
 * many thousands of ticks apart even at the 587 Hz high-rate setting. Keep a
 * deliberately small fixed window so a transition and its electrical bounce
 * cannot both reach the metadata stream without shifting the timestamp of the
 * first valid transition.
 */
#define SAMPLE_COUNTER_DEBOUNCE_TICKS 200U

typedef struct sample_counter_filter_state {
	uint32_t last_timestamp;
	sample_counter_edge_t last_edge;
	bool initialized;
} sample_counter_filter_state_t;

static inline void sample_counter_filter_reset(
	sample_counter_filter_state_t* state)
{
	state->last_timestamp = 0U;
	state->last_edge = SAMPLE_COUNTER_EDGE_FALLING;
	state->initialized = false;
}

static inline bool sample_counter_filter_accept(
	sample_counter_filter_state_t* state,
	sample_counter_event_t const event)
{
	if (state->initialized) {
		/* Real transitions alternate. Reject a duplicate latch first so an
		 * opposite edge at the same timestamp can still be accepted. */
		if (event.edge == state->last_edge) {
			return false;
		}

		if ((event.timestamp - state->last_timestamp) <
		    SAMPLE_COUNTER_DEBOUNCE_TICKS) {
			return false;
		}
	}

	state->last_timestamp = event.timestamp;
	state->last_edge = (sample_counter_edge_t) event.edge;
	state->initialized = true;
	return true;
}
