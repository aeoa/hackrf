/*
 * Lightweight wrapper around the Operacake SCTimer configuration so we can
 * expose a monotonically increasing counter driven by the SGPIO sample clock.
 */

#include "pps_counter.h"

#include <hackrf_core.h>

#include <libopencm3/lpc43xx/sgpio.h>
#include <libopencm3/lpc43xx/rgu.h>
#include <libopencm3/lpc43xx/gima.h>

#include "operacake_sctimer.h"
#include "sct.h"

/* Value that routes SGPIO12 into SCT input 1 (matches Operacake). */
#define SGPIO12_TO_SCT_INPUT1 (0x2U << 4)
#define SCT_INPUT_CLOCK_MASK (SCT_INPUT_AIN1_MASK | SCT_INPUT_SIN1_MASK)

static bool initialized;

static bool wait_for_sgpio_clock(void)
{
	uint32_t last = SCT_INPUT & SCT_INPUT_CLOCK_MASK;
	for (uint32_t i = 0; i < 200000U; ++i) {
		uint32_t current = SCT_INPUT & SCT_INPUT_CLOCK_MASK;
		if ((current ^ last) != 0U) {
			return true;
		}
	}
	return false;
}

void pps_counter_init(void)
{
    if (initialized) {
        return;
    }

	for (uint32_t attempt = 0; attempt < 4U; ++attempt) {
		operacake_sctimer_init();

		/* Give SGPIO clock a moment to settle. */
		delay(2);

		if (wait_for_sgpio_clock()) {
			SCT_CTRL |= SCT_CTRL_CLRCTR_L(1);
			SCT_CTRL |= SCT_CTRL_CLRCTR_H(1);
			initialized = true;
			return;
		}

		operacake_sctimer_reset_state();
		delay(1);
	}

	/* Last resort: leave the timer running even if we failed to detect the clock. */
	operacake_sctimer_init();
	SCT_CTRL |= SCT_CTRL_CLRCTR_L(1);
	SCT_CTRL |= SCT_CTRL_CLRCTR_H(1);
	initialized = true;
}

uint32_t pps_counter_read(void)
{
	return SCT_COUNT;
}
