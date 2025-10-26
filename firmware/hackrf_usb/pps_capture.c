/*
 * Capture PPS edges from the CPLD using the SCTimer so we can associate them
 * with IQ sample indices.
 */

#include "pps_capture.h"

#include <hackrf_core.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/lpc43xx/ccu.h>
#include <libopencm3/lpc43xx/gima.h>
#include <libopencm3/lpc43xx/rgu.h>
#include <libopencm3/lpc43xx/sgpio.h>

#include "sct.h"

#define PPS_CAPTURE_QUEUE_LEN 8U
#define PPS_CAPTURE_QUEUE_MASK (PPS_CAPTURE_QUEUE_LEN - 1U)

/* Route SGPIO clock onto SGPIO15 so the SCT can count SGPIO clock edges. */
#define PPS_SGPIO_CLOCK_SELECT 2U
#define PPS_HOST_SYNC_PIN 12U

#define GIMA_INV   (1U << 0)
#define GIMA_EDGE  (1U << 1)
#define GIMA_SYNCH (1U << 2)
#define GIMA_PULSE (1U << 3)
#define GIMA_SELECT(value) ((uint32_t)(value) << 4)

static volatile uint32_t queue_head;
static volatile uint32_t queue_tail;
static volatile uint32_t queue_ticks[PPS_CAPTURE_QUEUE_LEN];

static inline void enqueue_tick(uint32_t ticks)
{
	uint32_t next = (queue_head + 1U) & PPS_CAPTURE_QUEUE_MASK;
	if (next != queue_tail) {
		queue_ticks[queue_head] = ticks;
		queue_head = next;
	}
	/* If the queue is full we silently drop the sample. */
}

void pps_capture_reset(void)
{
	cm_disable_interrupts();
	queue_head = 0;
	queue_tail = 0;
	cm_enable_interrupts();
}

void pps_capture_init(void)
{
	pps_capture_reset();

	/* Enable the SCTimer clock. */
	CCU1_CLK_M4_SCT_CFG |= 1U; /* RUN bit */

	/* Reset the SCTimer. */
	RESET_CTRL1 = RESET_CTRL1_SCT_RST;
	delay(8);

	/* Route the SGPIO clock to SCT input 1. */
	GIMA_CTIN_1_IN = GIMA_EDGE | GIMA_SYNCH | GIMA_SELECT(PPS_SGPIO_CLOCK_SELECT);

	/* Route SGPIO12 (HOST_SYNC) to SCT input 0. */
	GIMA_CTIN_0_IN = GIMA_EDGE | GIMA_SYNCH | GIMA_SELECT(PPS_HOST_SYNC_PIN);

	/* Halt the SCT while we configure it. */
	SCT_CTRL = SCT_CTRL_HALT_L(1);

	/* 32-bit counter, clocked by SGPIO clock (input 1). */
	SCT_CONFIG = SCT_CONFIG_UNIFY_32_BIT |
		SCT_CONFIG_CLKMODE_PRESCALED_BUS_CLOCK |
		SCT_CONFIG_CKSEL_RISING_EDGES_ON_INPUT_1;

	/* Divide the SGPIO clock by two so one tick == one IQ sample. */
	SCT_CTRL &= ~SCT_CTRL_PRE_L_MASK;
	SCT_CTRL |= SCT_CTRL_PRE_L(1);

	/* Clear the counter. */
	SCT_CTRL |= SCT_CTRL_CLRCTR_L(1);

	/* Use event 0 to capture on the HOST_SYNC edge. */
	SCT_EVn_STATE(0) = SCT_EVn_STATE_STATEMSK0(1);
	SCT_EVn_CTRL(0) =
		SCT_EVn_CTRL_OUTSEL_INPUT |
		SCT_EVn_CTRL_IOSEL(0) |
		SCT_EVn_CTRL_IOCOND_RISE |
		SCT_EVn_CTRL_COMBMODE_IO;
	SCT_CAPCTRL0 = SCT_CAPCTRLn_CAPCON_L(1U);

	/* Clear any pending flags and enable the interrupt. */
	SCT_EVFLAG = SCT_EVFLAG_FLAG0(1);
	SCT_EVEN = SCT_EVEN_IEN0(1);

	/* Let the counter run. */
	SCT_CTRL &= ~SCT_CTRL_HALT_L(1);

	nvic_enable_irq(NVIC_SCT_IRQ);
}

void pps_capture_shutdown(void)
{
	nvic_disable_irq(NVIC_SCT_IRQ);
	SCT_CTRL |= SCT_CTRL_HALT_L(1);
	SCT_EVEN = 0;
	SCT_EVFLAG = SCT_EVFLAG_FLAG0(1);

	GIMA_CTIN_0_IN = 0;
	GIMA_CTIN_1_IN = 0;

	pps_capture_reset();
}

bool pps_capture_dequeue(pps_capture_event_t* event)
{
	bool have_event = false;
	cm_disable_interrupts();
	if (queue_tail != queue_head) {
		event->ticks = queue_ticks[queue_tail];
		queue_tail = (queue_tail + 1U) & PPS_CAPTURE_QUEUE_MASK;
		have_event = true;
	}
	cm_enable_interrupts();
	return have_event;
}

uint32_t pps_capture_counter(void)
{
	return SCT_COUNT; /* unified counter */
}

volatile uint32_t debug_counter_irq_handler;

void sct_isr(void)
{
	debug_counter_irq_handler += 1;

	uint32_t flags = SCT_EVFLAG;
	if (flags & SCT_EVFLAG_FLAG0_MASK) {
		enqueue_tick(SCT_CAP0);
		SCT_EVFLAG = SCT_EVFLAG_FLAG0(1);
	}
}
