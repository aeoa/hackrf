#include "sample_counter.h"

#include <hackrf_core.h>

#include "gpio_lpc.h"

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/lpc43xx/gima.h>
#include <libopencm3/lpc43xx/memorymap.h>
#include <libopencm3/lpc43xx/scu.h>

#include "sct.h"

#define GPIO_PIN_INTERRUPT_ISEL MMIO32(GPIO_PIN_INTERRUPT_BASE + 0x000)
#define GPIO_PIN_INTERRUPT_SIENR MMIO32(GPIO_PIN_INTERRUPT_BASE + 0x008)
#define GPIO_PIN_INTERRUPT_CIENR MMIO32(GPIO_PIN_INTERRUPT_BASE + 0x00C)
#define GPIO_PIN_INTERRUPT_IENF MMIO32(GPIO_PIN_INTERRUPT_BASE + 0x010)
#define GPIO_PIN_INTERRUPT_SIENF MMIO32(GPIO_PIN_INTERRUPT_BASE + 0x014)
#define GPIO_PIN_INTERRUPT_CIENF MMIO32(GPIO_PIN_INTERRUPT_BASE + 0x018)
#define GPIO_PIN_INTERRUPT_RISE MMIO32(GPIO_PIN_INTERRUPT_BASE + 0x01C)
#define GPIO_PIN_INTERRUPT_FALL MMIO32(GPIO_PIN_INTERRUPT_BASE + 0x020)
#define GPIO_PIN_INTERRUPT_IST MMIO32(GPIO_PIN_INTERRUPT_BASE + 0x024)

#define SAMPLE_COUNTER_FIFO_DEPTH 16U
#define SAMPLE_COUNTER_FIFO_MASK (SAMPLE_COUNTER_FIFO_DEPTH - 1U)

#define SAMPLE_COUNTER_PININT_INDEX 0U
#define SAMPLE_COUNTER_PININT_MASK (1U << SAMPLE_COUNTER_PININT_INDEX)

#define SAMPLE_COUNTER_GPIO_PORT 5U
#define SAMPLE_COUNTER_GPIO_PIN 14U

#define SAMPLE_COUNTER_SCT_INPUT_INDEX 0U
#define SAMPLE_COUNTER_SCT_EVENT_RISE 14U
#define SAMPLE_COUNTER_SCT_EVENT_FALL 15U
/* SGPIO15 (P4_10) appears as selection value 5 on CTIN input multiplexer. */
#define SAMPLE_COUNTER_GIMA_SELECT_VALUE 0x5U
#define SAMPLE_COUNTER_GIMA_ROUTE \
	(SAMPLE_COUNTER_GIMA_SELECT_VALUE << 4)
#define SAMPLE_COUNTER_GIMA_CTIN_REGISTER GIMA_CTIN_0_IN

static struct sample_counter_event fifo[SAMPLE_COUNTER_FIFO_DEPTH];
static volatile uint8_t fifo_head;
static volatile uint8_t fifo_tail;
static volatile bool fifo_enabled;

static bool sct_capture_configured;
static uint32_t saved_gima_ctin;
static uint32_t saved_ev_state_rise;
static uint32_t saved_ev_ctrl_rise;
static uint32_t saved_ev_state_fall;
static uint32_t saved_ev_ctrl_fall;
static uint32_t saved_capctrl_rise;
static uint32_t saved_capctrl_fall;
static uint32_t last_capture_rise;
static uint32_t last_capture_fall;

static const struct gpio_t slow_time_gpio = GPIO(
	SAMPLE_COUNTER_GPIO_PORT,
	SAMPLE_COUNTER_GPIO_PIN);

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

static void sample_counter_sct_configure(void)
{
	if (sct_capture_configured) {
		return;
	}

	saved_gima_ctin = SAMPLE_COUNTER_GIMA_CTIN_REGISTER;
	saved_ev_state_rise = SCT_EVn_STATE(SAMPLE_COUNTER_SCT_EVENT_RISE);
	saved_ev_ctrl_rise = SCT_EVn_CTRL(SAMPLE_COUNTER_SCT_EVENT_RISE);
	saved_ev_state_fall = SCT_EVn_STATE(SAMPLE_COUNTER_SCT_EVENT_FALL);
	saved_ev_ctrl_fall = SCT_EVn_CTRL(SAMPLE_COUNTER_SCT_EVENT_FALL);
	saved_capctrl_rise = SCT_CAPCTRL0;
	saved_capctrl_fall = SCT_CAPCTRL1;

	const uint32_t new_gima =
		(saved_gima_ctin & ~((0xFU << 4) | (1U << 0) | (1U << 1) |
				     (1U << 2) | (1U << 3))) |
		SAMPLE_COUNTER_GIMA_ROUTE |
		(1U << 1) | // enable edge detection
		(1U << 2);  // enable synchronization
	SAMPLE_COUNTER_GIMA_CTIN_REGISTER = new_gima;

	const uint32_t state_mask =
		SCT_EVn_STATE_STATEMSK0(1) | SCT_EVn_STATE_STATEMSK1(1);
	SCT_EVn_STATE(SAMPLE_COUNTER_SCT_EVENT_RISE) = state_mask;
	SCT_EVn_STATE(SAMPLE_COUNTER_SCT_EVENT_FALL) = state_mask;

	SCT_EVn_CTRL(SAMPLE_COUNTER_SCT_EVENT_RISE) =
		SCT_EVn_CTRL_MATCHSEL(0) |
		SCT_EVn_CTRL_OUTSEL_INPUT |
		SCT_EVn_CTRL_IOSEL(SAMPLE_COUNTER_SCT_INPUT_INDEX) |
		SCT_EVn_CTRL_IOCOND_RISE |
		SCT_EVn_CTRL_COMBMODE_IO |
		SCT_EVn_CTRL_DIRECTION(SCT_EVn_CTRL_DIRECTION_DIRECTION_INDEPENDEN);
	SCT_EVn_CTRL(SAMPLE_COUNTER_SCT_EVENT_FALL) =
		SCT_EVn_CTRL_MATCHSEL(0) |
		SCT_EVn_CTRL_OUTSEL_INPUT |
		SCT_EVn_CTRL_IOSEL(SAMPLE_COUNTER_SCT_INPUT_INDEX) |
		SCT_EVn_CTRL_IOCOND_FALL |
		SCT_EVn_CTRL_COMBMODE_IO |
		SCT_EVn_CTRL_DIRECTION(SCT_EVn_CTRL_DIRECTION_DIRECTION_INDEPENDEN);

	const uint32_t rise_event_mask = (1U << SAMPLE_COUNTER_SCT_EVENT_RISE);
	const uint32_t fall_event_mask = (1U << SAMPLE_COUNTER_SCT_EVENT_FALL);
	SCT_CAPCTRL0 =
		SCT_CAPCTRLn_CAPCON_L(rise_event_mask) |
		SCT_CAPCTRLn_CAPCON_H(rise_event_mask);
	SCT_CAPCTRL1 =
		SCT_CAPCTRLn_CAPCON_L(fall_event_mask) |
		SCT_CAPCTRLn_CAPCON_H(fall_event_mask);

	last_capture_rise = SCT_CAP0;
	last_capture_fall = SCT_CAP1;
	SCT_EVFLAG = (1U << SAMPLE_COUNTER_SCT_EVENT_RISE) |
		(1U << SAMPLE_COUNTER_SCT_EVENT_FALL);

	sct_capture_configured = true;
}

static void sample_counter_sct_restore(void)
{
	if (!sct_capture_configured) {
		return;
	}

	SAMPLE_COUNTER_GIMA_CTIN_REGISTER = saved_gima_ctin;
	SCT_EVn_STATE(SAMPLE_COUNTER_SCT_EVENT_RISE) = saved_ev_state_rise;
	SCT_EVn_CTRL(SAMPLE_COUNTER_SCT_EVENT_RISE) = saved_ev_ctrl_rise;
	SCT_EVn_STATE(SAMPLE_COUNTER_SCT_EVENT_FALL) = saved_ev_state_fall;
	SCT_EVn_CTRL(SAMPLE_COUNTER_SCT_EVENT_FALL) = saved_ev_ctrl_fall;
	SCT_CAPCTRL0 = saved_capctrl_rise;
	SCT_CAPCTRL1 = saved_capctrl_fall;

	sct_capture_configured = false;
}

static inline bool sample_counter_capture_timestamp(bool rising, uint32_t* timestamp_out)
{
	if (!sct_capture_configured) {
		return false;
	}

	volatile uint32_t* const cap_reg = rising ? &SCT_CAP0 : &SCT_CAP1;
	uint32_t* const last_capture =
		rising ? &last_capture_rise : &last_capture_fall;

	uint32_t captured = *cap_reg;
	if (captured != *last_capture) {
		*last_capture = captured;
		*timestamp_out = captured;
		return true;
	}

	return false;
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
		SCU_GPIO_FAST | SCU_CONF_FUNCTION4);

	gpio_input(&slow_time_gpio);

	uint32_t pintsel = SCU_PINTSEL0;
	const uint32_t pintsel_mask =
		((0x1FU << 0U) | (0x7U << 5U)); // route for PIN_INT0
	pintsel &= ~pintsel_mask;
	pintsel |=
		((SAMPLE_COUNTER_GPIO_PIN & 0x1FU) << 0U) |
		((SAMPLE_COUNTER_GPIO_PORT & 0x7U) << 5U);
	SCU_PINTSEL0 = pintsel;

	GPIO_PIN_INTERRUPT_ISEL &= ~SAMPLE_COUNTER_PININT_MASK;

	GPIO_PIN_INTERRUPT_RISE = SAMPLE_COUNTER_PININT_MASK;
	GPIO_PIN_INTERRUPT_FALL = SAMPLE_COUNTER_PININT_MASK;
	GPIO_PIN_INTERRUPT_IST = SAMPLE_COUNTER_PININT_MASK;

	GPIO_PIN_INTERRUPT_SIENR = SAMPLE_COUNTER_PININT_MASK;
	GPIO_PIN_INTERRUPT_SIENF = SAMPLE_COUNTER_PININT_MASK;

	sample_counter_sct_configure();

	nvic_clear_pending_irq(NVIC_PIN_INT0_IRQ);
	nvic_enable_irq(NVIC_PIN_INT0_IRQ);

	fifo_enabled = true;
}

void sample_counter_capture_disable(void)
{
	if (!fifo_enabled) {
		return;
	}

	nvic_disable_irq(NVIC_PIN_INT0_IRQ);

	GPIO_PIN_INTERRUPT_CIENR = SAMPLE_COUNTER_PININT_MASK;
	GPIO_PIN_INTERRUPT_CIENF = SAMPLE_COUNTER_PININT_MASK;
	GPIO_PIN_INTERRUPT_IST = SAMPLE_COUNTER_PININT_MASK;

	sample_counter_sct_restore();

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

void pin_int0_isr(void)
{
	const uint32_t fallback_timestamp = SCT_COUNT;
	const uint32_t captured_rise =
		GPIO_PIN_INTERRUPT_RISE & SAMPLE_COUNTER_PININT_MASK;
	const uint32_t captured_fall =
		GPIO_PIN_INTERRUPT_FALL & SAMPLE_COUNTER_PININT_MASK;

	if (captured_rise != 0U) {
		uint32_t timestamp = fallback_timestamp;
		bool used_capture = sample_counter_capture_timestamp(true, &timestamp);
		uint32_t input_state = SCT_INPUT;
		if (sct_capture_configured) {
			SCT_EVFLAG = (1U << SAMPLE_COUNTER_SCT_EVENT_RISE);
		}

		uint32_t source_flags = used_capture ?
			SAMPLE_COUNTER_SOURCE_CAPTURE :
			SAMPLE_COUNTER_SOURCE_FALLBACK;
		if ((input_state >> SAMPLE_COUNTER_SCT_INPUT_INDEX) & 0x1U) {
			source_flags |= SAMPLE_COUNTER_SOURCE_AIN;
		}
		if ((input_state >> (16U + SAMPLE_COUNTER_SCT_INPUT_INDEX)) & 0x1U) {
			source_flags |= SAMPLE_COUNTER_SOURCE_SIN;
		}

		sample_counter_event_t event = {
			.timestamp = timestamp,
			.edge = SAMPLE_COUNTER_EDGE_RISING,
			.source = source_flags,
		};
		fifo_push(event);
		GPIO_PIN_INTERRUPT_RISE = SAMPLE_COUNTER_PININT_MASK;
	}

	if (captured_fall != 0U) {
		uint32_t timestamp = fallback_timestamp;
		bool used_capture = sample_counter_capture_timestamp(false, &timestamp);
		uint32_t input_state = SCT_INPUT;
		if (sct_capture_configured) {
			SCT_EVFLAG = (1U << SAMPLE_COUNTER_SCT_EVENT_FALL);
		}

		uint32_t source_flags = used_capture ?
			SAMPLE_COUNTER_SOURCE_CAPTURE :
			SAMPLE_COUNTER_SOURCE_FALLBACK;
		if ((input_state >> SAMPLE_COUNTER_SCT_INPUT_INDEX) & 0x1U) {
			source_flags |= SAMPLE_COUNTER_SOURCE_AIN;
		}
		if ((input_state >> (16U + SAMPLE_COUNTER_SCT_INPUT_INDEX)) & 0x1U) {
			source_flags |= SAMPLE_COUNTER_SOURCE_SIN;
		}

		sample_counter_event_t event = {
			.timestamp = timestamp,
			.edge = SAMPLE_COUNTER_EDGE_FALLING,
			.source = source_flags,
		};
		fifo_push(event);
		GPIO_PIN_INTERRUPT_FALL = SAMPLE_COUNTER_PININT_MASK;
	}

	GPIO_PIN_INTERRUPT_IST = SAMPLE_COUNTER_PININT_MASK;
}
