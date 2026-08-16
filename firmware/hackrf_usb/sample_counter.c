#include "sample_counter.h"

#include <hackrf_core.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/lpc43xx/gima.h>
#include <libopencm3/lpc43xx/scu.h>

#include "sct.h"

#define SAMPLE_COUNTER_FIFO_DEPTH 16U
#define SAMPLE_COUNTER_FIFO_MASK (SAMPLE_COUNTER_FIFO_DEPTH - 1U)

#define SAMPLE_COUNTER_SCT_INPUT 2U
#define SAMPLE_COUNTER_RISE_EVENT 14U
#define SAMPLE_COUNTER_FALL_EVENT 15U
#define SAMPLE_COUNTER_RISE_CAPTURE 14U
#define SAMPLE_COUNTER_FALL_CAPTURE 15U
#define SAMPLE_COUNTER_SCT_EVENT_MASK \
	((1U << SAMPLE_COUNTER_RISE_EVENT) | (1U << SAMPLE_COUNTER_FALL_EVENT))
#define SAMPLE_COUNTER_SCT_CAPTURE_MASK \
	((1U << SAMPLE_COUNTER_RISE_CAPTURE) | (1U << SAMPLE_COUNTER_FALL_CAPTURE))

static struct sample_counter_event fifo[SAMPLE_COUNTER_FIFO_DEPTH];
static volatile uint8_t fifo_head;
static volatile uint8_t fifo_tail;
static volatile bool fifo_enabled;

static inline void fifo_reset(void)
{
	fifo_head = 0;
	fifo_tail = 0;
}

static inline bool fifo_is_full(uint8_t head, uint8_t tail)
{
	return ((head + 1U) & SAMPLE_COUNTER_FIFO_MASK) == tail;
}

static inline void fifo_push(sample_counter_event_t event)
{
	uint8_t head = fifo_head;
	uint8_t tail = fifo_tail;

	if (fifo_is_full(head, tail)) {
		// Drop newest event so the consumer remains sole writer of fifo_tail.
		return;
	}

	fifo[head] = event;
	fifo_head = (head + 1U) & SAMPLE_COUNTER_FIFO_MASK;
}

void sample_counter_capture_enable(void)
{
	if (fifo_enabled) {
		return;
	}

	cm_disable_interrupts();
	fifo_reset();
	cm_enable_interrupts();

	scu_pinmux(
		SCU_PINMUX_SGPIO15,
		SCU_GPIO_FAST | SCU_CONF_FUNCTION1); // P4_10: CTIN_2

	// Select the direct CTIN_2 pin path. Edge detection and synchronization are
	// handled by the SCT itself.
	GIMA_CTIN_2_IN = 0;

	// The SCT is also the continuously running 2x sample counter. Halt it only
	// while changing the match/capture and event configuration.
	SCT_CTRL |= SCT_CTRL_HALT_L(1);
	SCT_CONFIG |= SCT_CONFIG_INSYNC(1U << SAMPLE_COUNTER_SCT_INPUT);

	SCT_EVEN &= ~SAMPLE_COUNTER_SCT_EVENT_MASK;
	SCT_EVn_STATE(SAMPLE_COUNTER_RISE_EVENT) = 0;
	SCT_EVn_STATE(SAMPLE_COUNTER_FALL_EVENT) = 0;
	SCT_EVFLAG = SAMPLE_COUNTER_SCT_EVENT_MASK;

	SCT_REGMODE |= SAMPLE_COUNTER_SCT_CAPTURE_MASK;
	SCT_CAPCTRL14 = 1U << SAMPLE_COUNTER_RISE_EVENT;
	SCT_CAPCTRL15 = 1U << SAMPLE_COUNTER_FALL_EVENT;

	SCT_EVn_CTRL(SAMPLE_COUNTER_RISE_EVENT) =
		SCT_EVn_CTRL_OUTSEL_INPUT |
		SCT_EVn_CTRL_IOSEL(SAMPLE_COUNTER_SCT_INPUT) |
		SCT_EVn_CTRL_IOCOND_RISE |
		SCT_EVn_CTRL_COMBMODE_IO;
	SCT_EVn_CTRL(SAMPLE_COUNTER_FALL_EVENT) =
		SCT_EVn_CTRL_OUTSEL_INPUT |
		SCT_EVn_CTRL_IOSEL(SAMPLE_COUNTER_SCT_INPUT) |
		SCT_EVn_CTRL_IOCOND_FALL |
		SCT_EVn_CTRL_COMBMODE_IO;

	// State zero is the normal non-Opera-Cake state of this counter.
	SCT_EVn_STATE(SAMPLE_COUNTER_RISE_EVENT) = SCT_EVn_STATE_STATEMSK0(1);
	SCT_EVn_STATE(SAMPLE_COUNTER_FALL_EVENT) = SCT_EVn_STATE_STATEMSK0(1);
	SCT_EVEN |= SAMPLE_COUNTER_SCT_EVENT_MASK;

	fifo_enabled = true;

	nvic_clear_pending_irq(NVIC_SCT_IRQ);
	nvic_enable_irq(NVIC_SCT_IRQ);
	SCT_CTRL &= ~SCT_CTRL_HALT_L(1);
}

void sample_counter_capture_disable(void)
{
	if (!fifo_enabled) {
		return;
	}

	nvic_disable_irq(NVIC_SCT_IRQ);
	SCT_EVEN &= ~SAMPLE_COUNTER_SCT_EVENT_MASK;
	SCT_EVn_STATE(SAMPLE_COUNTER_RISE_EVENT) = 0;
	SCT_EVn_STATE(SAMPLE_COUNTER_FALL_EVENT) = 0;
	SCT_EVFLAG = SAMPLE_COUNTER_SCT_EVENT_MASK;

	cm_disable_interrupts();
	fifo_reset();
	fifo_enabled = false;
	cm_enable_interrupts();
}

size_t sample_counter_capture_drain(
	sample_counter_event_t* dest,
	size_t max_events)
{
	if ((dest == NULL) || (max_events == 0U)) {
		return 0;
	}

	size_t count = 0;
	while ((count < max_events) && (fifo_tail != fifo_head)) {
		dest[count] = fifo[fifo_tail];
		fifo_tail = (fifo_tail + 1U) & SAMPLE_COUNTER_FIFO_MASK;
		count++;
	}

	return count;
}

void sct_isr(void)
{
	uint32_t const flags = SCT_EVFLAG & SAMPLE_COUNTER_SCT_EVENT_MASK;

	if (flags & (1U << SAMPLE_COUNTER_RISE_EVENT)) {
		sample_counter_event_t const event = {
			.timestamp = SCT_CAP14,
			.edge = SAMPLE_COUNTER_EDGE_RISING,
		};
		fifo_push(event);
	}

	if (flags & (1U << SAMPLE_COUNTER_FALL_EVENT)) {
		sample_counter_event_t const event = {
			.timestamp = SCT_CAP15,
			.edge = SAMPLE_COUNTER_EDGE_FALLING,
		};
		fifo_push(event);
	}

	// Event flags are write-one-to-clear. The captured counter values remain in
	// their CAP registers, independent of when this ISR runs.
	SCT_EVFLAG = flags;
}
