#include "sample_counter.h"

#include <hackrf_core.h>

#include "gpio_lpc.h"

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
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

static struct sample_counter_event fifo[SAMPLE_COUNTER_FIFO_DEPTH];
static volatile uint8_t fifo_head;
static volatile uint8_t fifo_tail;
static volatile bool fifo_enabled;

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

uint32_t sample_counter_read(void)
{
	return SCT_COUNT;
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
	const uint32_t captured_rise =
		GPIO_PIN_INTERRUPT_RISE & SAMPLE_COUNTER_PININT_MASK;
	const uint32_t captured_fall =
		GPIO_PIN_INTERRUPT_FALL & SAMPLE_COUNTER_PININT_MASK;

	if (captured_rise != 0U) {
		sample_counter_event_t event = {
			.timestamp = sample_counter_read(),
			.edge = SAMPLE_COUNTER_EDGE_RISING,
		};
		fifo_push(event);
		GPIO_PIN_INTERRUPT_RISE = SAMPLE_COUNTER_PININT_MASK;
	}

	if (captured_fall != 0U) {
		sample_counter_event_t event = {
			.timestamp = sample_counter_read(),
			.edge = SAMPLE_COUNTER_EDGE_FALLING,
		};
		fifo_push(event);
		GPIO_PIN_INTERRUPT_FALL = SAMPLE_COUNTER_PININT_MASK;
	}

	GPIO_PIN_INTERRUPT_IST = SAMPLE_COUNTER_PININT_MASK;
}
