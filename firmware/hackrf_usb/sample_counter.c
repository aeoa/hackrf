#include "sample_counter.h"
#include "sample_counter_filter.h"

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/lpc43xx/gima.h>
#include <libopencm3/lpc43xx/scu.h>

#include <platform_scu.h>
#include <sct.h>

#define SAMPLE_COUNTER_FIFO_DEPTH 2048U
#define SAMPLE_COUNTER_FIFO_MASK (SAMPLE_COUNTER_FIFO_DEPTH - 1U)

#define SAMPLE_COUNTER_SOURCE0 0U
#define SAMPLE_COUNTER_SOURCE1 1U

#define SAMPLE_COUNTER_SOURCE0_SCT_INPUT 2U
#define SAMPLE_COUNTER_SOURCE1_SCT_INPUT 6U

#define SAMPLE_COUNTER_SOURCE0_RISE_EVENT 12U
#define SAMPLE_COUNTER_SOURCE0_FALL_EVENT 13U
#define SAMPLE_COUNTER_SOURCE1_RISE_EVENT 14U
#define SAMPLE_COUNTER_SOURCE1_FALL_EVENT 15U

#define SAMPLE_COUNTER_SOURCE0_RISE_CAPTURE 12U
#define SAMPLE_COUNTER_SOURCE0_FALL_CAPTURE 13U
#define SAMPLE_COUNTER_SOURCE1_RISE_CAPTURE 14U
#define SAMPLE_COUNTER_SOURCE1_FALL_CAPTURE 15U

#define SAMPLE_COUNTER_SCT_EVENT_MASK \
	((1U << SAMPLE_COUNTER_SOURCE0_RISE_EVENT) | \
	 (1U << SAMPLE_COUNTER_SOURCE0_FALL_EVENT) | \
	 (1U << SAMPLE_COUNTER_SOURCE1_RISE_EVENT) | \
	 (1U << SAMPLE_COUNTER_SOURCE1_FALL_EVENT))
#define SAMPLE_COUNTER_SCT_CAPTURE_MASK \
	((1U << SAMPLE_COUNTER_SOURCE0_RISE_CAPTURE) | \
	 (1U << SAMPLE_COUNTER_SOURCE0_FALL_CAPTURE) | \
	 (1U << SAMPLE_COUNTER_SOURCE1_RISE_CAPTURE) | \
	 (1U << SAMPLE_COUNTER_SOURCE1_FALL_CAPTURE))

static struct sample_counter_event fifo[SAMPLE_COUNTER_FIFO_DEPTH];
static volatile uint16_t fifo_head;
static volatile uint16_t fifo_tail;
static volatile bool fifo_enabled;
static volatile uint32_t fifo_dropped_events;
static volatile uint32_t fifo_high_water;
static volatile uint32_t filtered_events[2];
static sample_counter_filter_state_t filter_states[2];

static inline void fifo_reset(void)
{
	fifo_head = 0;
	fifo_tail = 0;
}

static inline bool fifo_is_full(uint16_t head, uint16_t tail)
{
	return ((head + 1U) & SAMPLE_COUNTER_FIFO_MASK) == tail;
}

static inline void fifo_push(sample_counter_event_t event)
{
	uint16_t head = fifo_head;
	uint16_t tail = fifo_tail;

	if (fifo_is_full(head, tail)) {
		/* Drop newest so the consumer remains the sole writer of fifo_tail. */
		fifo_dropped_events++;
		return;
	}

	fifo[head] = event;
	uint16_t const new_head = (head + 1U) & SAMPLE_COUNTER_FIFO_MASK;
	fifo_head = new_head;
	uint32_t const occupancy = (new_head - fifo_tail) & SAMPLE_COUNTER_FIFO_MASK;
	if (occupancy > fifo_high_water) {
		fifo_high_water = occupancy;
	}
}

void sample_counter_capture_enable(void)
{
	if (fifo_enabled) {
		return;
	}

	cm_disable_interrupts();
	fifo_reset();
	fifo_dropped_events = 0;
	fifo_high_water = 0;
	filtered_events[SAMPLE_COUNTER_SOURCE0] = 0;
	filtered_events[SAMPLE_COUNTER_SOURCE1] = 0;
	sample_counter_filter_reset(&filter_states[SAMPLE_COUNTER_SOURCE0]);
	sample_counter_filter_reset(&filter_states[SAMPLE_COUNTER_SOURCE1]);
	cm_enable_interrupts();

	const platform_scu_t* const scu = platform_scu();
	/* Keep the SCU input glitch filter enabled on both pulse inputs. */
	scu_pinmux(
		scu->PINMUX_SGPIO15,
		SCU_GPIO_NOPULL | SCU_CONF_FUNCTION1); /* P4_10: CTIN_2 */
	scu_pinmux(
		scu->PINMUX_SGPIO14,
		SCU_GPIO_NOPULL | SCU_CONF_FUNCTION1); /* P4_9: CTIN_6 */

	/* Select direct CTIN paths; the SCT handles edge synchronization. */
	GIMA_CTIN_2_IN = 0;
	GIMA_CTIN_6_IN = 0;

	/* The SCT is also the continuously running 2x sample counter. */
	SCT_CTRL |= SCT_CTRL_HALT_L(1);
	SCT_CTRL &= ~SCT_CTRL_PRE_L_MASK;
	SCT_CONFIG |= SCT_CONFIG_INSYNC(
		(1U << SAMPLE_COUNTER_SOURCE0_SCT_INPUT) |
		(1U << SAMPLE_COUNTER_SOURCE1_SCT_INPUT));

	SCT_EVEN &= ~SAMPLE_COUNTER_SCT_EVENT_MASK;
	SCT_EVn_STATE(SAMPLE_COUNTER_SOURCE0_RISE_EVENT) = 0;
	SCT_EVn_STATE(SAMPLE_COUNTER_SOURCE0_FALL_EVENT) = 0;
	SCT_EVn_STATE(SAMPLE_COUNTER_SOURCE1_RISE_EVENT) = 0;
	SCT_EVn_STATE(SAMPLE_COUNTER_SOURCE1_FALL_EVENT) = 0;
	SCT_EVFLAG = SAMPLE_COUNTER_SCT_EVENT_MASK;

	SCT_REGMODE |= SAMPLE_COUNTER_SCT_CAPTURE_MASK;
	SCT_CAPCTRL12 = 1U << SAMPLE_COUNTER_SOURCE0_RISE_EVENT;
	SCT_CAPCTRL13 = 1U << SAMPLE_COUNTER_SOURCE0_FALL_EVENT;
	SCT_CAPCTRL14 = 1U << SAMPLE_COUNTER_SOURCE1_RISE_EVENT;
	SCT_CAPCTRL15 = 1U << SAMPLE_COUNTER_SOURCE1_FALL_EVENT;

	SCT_EVn_CTRL(SAMPLE_COUNTER_SOURCE0_RISE_EVENT) =
		SCT_EVn_CTRL_OUTSEL_INPUT |
		SCT_EVn_CTRL_IOSEL(SAMPLE_COUNTER_SOURCE0_SCT_INPUT) |
		SCT_EVn_CTRL_IOCOND_RISE |
		SCT_EVn_CTRL_COMBMODE_IO;
	SCT_EVn_CTRL(SAMPLE_COUNTER_SOURCE0_FALL_EVENT) =
		SCT_EVn_CTRL_OUTSEL_INPUT |
		SCT_EVn_CTRL_IOSEL(SAMPLE_COUNTER_SOURCE0_SCT_INPUT) |
		SCT_EVn_CTRL_IOCOND_FALL |
		SCT_EVn_CTRL_COMBMODE_IO;
	SCT_EVn_CTRL(SAMPLE_COUNTER_SOURCE1_RISE_EVENT) =
		SCT_EVn_CTRL_OUTSEL_INPUT |
		SCT_EVn_CTRL_IOSEL(SAMPLE_COUNTER_SOURCE1_SCT_INPUT) |
		SCT_EVn_CTRL_IOCOND_RISE |
		SCT_EVn_CTRL_COMBMODE_IO;
	SCT_EVn_CTRL(SAMPLE_COUNTER_SOURCE1_FALL_EVENT) =
		SCT_EVn_CTRL_OUTSEL_INPUT |
		SCT_EVn_CTRL_IOSEL(SAMPLE_COUNTER_SOURCE1_SCT_INPUT) |
		SCT_EVn_CTRL_IOCOND_FALL |
		SCT_EVn_CTRL_COMBMODE_IO;

	SCT_EVn_STATE(SAMPLE_COUNTER_SOURCE0_RISE_EVENT) = SCT_EVn_STATE_STATEMSK0(1);
	SCT_EVn_STATE(SAMPLE_COUNTER_SOURCE0_FALL_EVENT) = SCT_EVn_STATE_STATEMSK0(1);
	SCT_EVn_STATE(SAMPLE_COUNTER_SOURCE1_RISE_EVENT) = SCT_EVn_STATE_STATEMSK0(1);
	SCT_EVn_STATE(SAMPLE_COUNTER_SOURCE1_FALL_EVENT) = SCT_EVn_STATE_STATEMSK0(1);
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
	SCT_EVn_STATE(SAMPLE_COUNTER_SOURCE0_RISE_EVENT) = 0;
	SCT_EVn_STATE(SAMPLE_COUNTER_SOURCE0_FALL_EVENT) = 0;
	SCT_EVn_STATE(SAMPLE_COUNTER_SOURCE1_RISE_EVENT) = 0;
	SCT_EVn_STATE(SAMPLE_COUNTER_SOURCE1_FALL_EVENT) = 0;
	SCT_EVFLAG = SAMPLE_COUNTER_SCT_EVENT_MASK;

	cm_disable_interrupts();
	fifo_reset();
	fifo_enabled = false;
	cm_enable_interrupts();
}

size_t sample_counter_capture_drain_packed(uint32_t* dest, size_t max_events)
{
	if ((dest == NULL) || (max_events == 0U)) {
		return 0;
	}

	size_t count = 0;
	while ((count < max_events) && (fifo_tail != fifo_head)) {
		sample_counter_event_t const event = fifo[fifo_tail];
		dest[count] =
			(event.timestamp & 0x3fffffffU) |
			((uint32_t)(event.source & 0x1U) << 30U) |
			((uint32_t)(event.edge == SAMPLE_COUNTER_EDGE_RISING) << 31U);
		fifo_tail = (fifo_tail + 1U) & SAMPLE_COUNTER_FIFO_MASK;
		count++;
	}

	return count;
}

uint32_t sample_counter_capture_dropped(void)
{
	return fifo_dropped_events;
}

uint32_t sample_counter_capture_high_water(void)
{
	return fifo_high_water;
}

uint32_t sample_counter_capture_capacity(void)
{
	return SAMPLE_COUNTER_FIFO_DEPTH - 1U;
}

uint32_t sample_counter_capture_filtered(uint8_t source)
{
	return source < 2U ? filtered_events[source] : 0U;
}

void sct_isr(void)
{
	uint32_t const flags = SCT_EVFLAG & SAMPLE_COUNTER_SCT_EVENT_MASK;
	sample_counter_event_t events[4];
	size_t event_count = 0;

	if (flags & (1U << SAMPLE_COUNTER_SOURCE0_RISE_EVENT)) {
		events[event_count++] = (sample_counter_event_t){
			.timestamp = SCT_CAP12,
			.source = SAMPLE_COUNTER_SOURCE0,
			.edge = SAMPLE_COUNTER_EDGE_RISING,
		};
	}
	if (flags & (1U << SAMPLE_COUNTER_SOURCE0_FALL_EVENT)) {
		events[event_count++] = (sample_counter_event_t){
			.timestamp = SCT_CAP13,
			.source = SAMPLE_COUNTER_SOURCE0,
			.edge = SAMPLE_COUNTER_EDGE_FALLING,
		};
	}
	if (flags & (1U << SAMPLE_COUNTER_SOURCE1_RISE_EVENT)) {
		events[event_count++] = (sample_counter_event_t){
			.timestamp = SCT_CAP14,
			.source = SAMPLE_COUNTER_SOURCE1,
			.edge = SAMPLE_COUNTER_EDGE_RISING,
		};
	}
	if (flags & (1U << SAMPLE_COUNTER_SOURCE1_FALL_EVENT)) {
		events[event_count++] = (sample_counter_event_t){
			.timestamp = SCT_CAP15,
			.source = SAMPLE_COUNTER_SOURCE1,
			.edge = SAMPLE_COUNTER_EDGE_FALLING,
		};
	}

	/* Multiple flags can arrive in one ISR; preserve counter order. */
	for (size_t i = 1; i < event_count; ++i) {
		sample_counter_event_t const event = events[i];
		size_t j = i;
		while ((j > 0U) &&
		       ((int32_t)(event.timestamp - events[j - 1U].timestamp) < 0)) {
			events[j] = events[j - 1U];
			--j;
		}
		events[j] = event;
	}

	for (size_t i = 0; i < event_count; ++i) {
		sample_counter_event_t const event = events[i];
		if (sample_counter_filter_accept(
			    &filter_states[event.source], event)) {
			fifo_push(event);
		} else {
			filtered_events[event.source]++;
		}
	}

	SCT_EVFLAG = flags;
}
