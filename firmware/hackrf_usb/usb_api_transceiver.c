/*
 * Copyright 2012-2022 Great Scott Gadgets <info@greatscottgadgets.com>
 * Copyright 2012 Jared Boone
 * Copyright 2013 Benjamin Vernoux
 *
 * This file is part of HackRF.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "usb_api_transceiver.h"

#include "hackrf_ui.h"
#include "operacake_sctimer.h"
#include "sample_counter.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/lpc43xx/gpdma.h>
#include "usb_bulk_buffer.h"
#include "usb_api_m0_state.h"

#include "usb_api_cpld.h" // Remove when CPLD update is handled elsewhere

#include "max2837.h"
#include "max2839.h"
#include "gpdma.h"
#include "rf_path.h"
#include "tuning.h"
#include "streaming.h"
#include "usb.h"
#include "usb_queue.h"
#include "platform_detect.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "usb_endpoint.h"
#include "usb_api_sweep.h"

#define USB_TRANSFER_SIZE 0x4000
#define RX_DMA_TRANSFER_SIZE 0x2000
#define RX_IQ_BLOCK_SIZE 0x4000
#define RX_IQ_BLOCK_SHIFT 14U
#define RX_IQ_BLOCK_MASK (RX_IQ_BLOCK_SIZE - 1U)
#define SAMPLE_HEADER_SIZE 512U
#define RX_METADATA_GROUP_BLOCKS 16U
#define RX_METADATA_GROUP_MAGIC 0x47525010U
#define SAMPLE_COUNTER_HEADER_MAX_EVENTS 96U
#define SAMPLE_COUNTER_HEADER_DROPPED_EVENT_WORD \
	(3U + SAMPLE_COUNTER_HEADER_MAX_EVENTS)
#define SAMPLE_COUNTER_HEADER_HIGH_WATER_WORD \
	(SAMPLE_COUNTER_HEADER_DROPPED_EVENT_WORD + 1U)
#define SAMPLE_COUNTER_HEADER_CAPACITY_WORD \
	(SAMPLE_COUNTER_HEADER_HIGH_WATER_WORD + 1U)
#define SAMPLE_HEADER_GROUP_MAGIC_WORD \
	(SAMPLE_COUNTER_HEADER_CAPACITY_WORD + 1U)
#define SAMPLE_HEADER_GROUP_COUNT_WORD (SAMPLE_HEADER_GROUP_MAGIC_WORD + 1U)
#define SAMPLE_HEADER_GROUP_FIRST_SAMPLE_WORD \
	(SAMPLE_HEADER_GROUP_COUNT_WORD + 1U)
#define SAMPLE_HEADER_WORDS \
	(SAMPLE_HEADER_GROUP_FIRST_SAMPLE_WORD + RX_METADATA_GROUP_BLOCKS)
#define SAMPLE_HEADER_M0_SHORTFALL_COUNT_WORD SAMPLE_HEADER_WORDS
#define SAMPLE_HEADER_M0_LONGEST_SHORTFALL_WORD \
	(SAMPLE_HEADER_M0_SHORTFALL_COUNT_WORD + 1U)
#define SAMPLE_HEADER_TOTAL_WORDS \
	(SAMPLE_HEADER_M0_LONGEST_SHORTFALL_WORD + 1U)
#define SAMPLE_HEADER_MAGIC 0xDEADBEEF
#define RX_SAMPLE_BUFFER_HALF_MASK (USB_SAMP_BUFFER_SIZE >> 1U)
#define RX_FIRST_SAMPLE_RING_BLOCKS (RX_METADATA_GROUP_BLOCKS * 2U)
#define RX_FIRST_SAMPLE_RING_MASK (RX_FIRST_SAMPLE_RING_BLOCKS - 1U)

typedef char sample_header_must_fit[
	(SAMPLE_HEADER_TOTAL_WORDS <= (SAMPLE_HEADER_SIZE / sizeof(uint32_t))) ? 1 : -1];

typedef struct {
	uint32_t freq_mhz;
	uint32_t freq_hz;
} set_freq_params_t;

set_freq_params_t set_freq_params;

struct set_freq_explicit_params {
	uint64_t if_freq_hz; /* intermediate frequency */
	uint64_t lo_freq_hz; /* front-end local oscillator frequency */
	uint8_t path;        /* image rejection filter path */
};

struct set_freq_explicit_params explicit_params;

typedef struct {
	uint32_t freq_hz;
	uint32_t divider;
} set_sample_r_params_t;

set_sample_r_params_t set_sample_r_params;

static uint32_t block_first_sample(uint32_t block_index)
{
	return (block_index == 0U) ? m0_state.block0_first_sample :
		m0_state.block1_first_sample;
}

usb_request_status_t usb_vendor_request_set_baseband_filter_bandwidth(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		const uint32_t bandwidth =
			(endpoint->setup.index << 16) | endpoint->setup.value;
		if (baseband_filter_bandwidth_set(bandwidth)) {
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		}
		return USB_REQUEST_STATUS_STALL;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_freq(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		usb_transfer_schedule_block(
			endpoint->out,
			&set_freq_params,
			sizeof(set_freq_params_t),
			NULL,
			NULL);
		return USB_REQUEST_STATUS_OK;
	} else if (stage == USB_TRANSFER_STAGE_DATA) {
		const uint64_t freq =
			set_freq_params.freq_mhz * 1000000ULL + set_freq_params.freq_hz;
		if (set_freq(freq)) {
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		}
		return USB_REQUEST_STATUS_STALL;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_sample_rate_frac(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		usb_transfer_schedule_block(
			endpoint->out,
			&set_sample_r_params,
			sizeof(set_sample_r_params_t),
			NULL,
			NULL);
		return USB_REQUEST_STATUS_OK;
	} else if (stage == USB_TRANSFER_STAGE_DATA) {
		if (sample_rate_frac_set(
			    set_sample_r_params.freq_hz * 2,
			    set_sample_r_params.divider)) {
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		}
		return USB_REQUEST_STATUS_STALL;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_amp_enable(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		switch (endpoint->setup.value) {
		case 0:
			rf_path_set_lna(&rf_path, 0);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		case 1:
			rf_path_set_lna(&rf_path, 1);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		default:
			return USB_REQUEST_STATUS_STALL;
		}
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_lna_gain(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uint8_t value;
		value = max283x_set_lna_gain(&max283x, endpoint->setup.index);
		endpoint->buffer[0] = value;
		if (value) {
			hackrf_ui()->set_bb_lna_gain(endpoint->setup.index);
		}
		usb_transfer_schedule_block(
			endpoint->in,
			&endpoint->buffer,
			1,
			NULL,
			NULL);
		usb_transfer_schedule_ack(endpoint->out);
		return USB_REQUEST_STATUS_OK;
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_vga_gain(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uint8_t value;
		value = max283x_set_vga_gain(&max283x, endpoint->setup.index);
		endpoint->buffer[0] = value;
		if (value) {
			hackrf_ui()->set_bb_vga_gain(endpoint->setup.index);
		}
		usb_transfer_schedule_block(
			endpoint->in,
			&endpoint->buffer,
			1,
			NULL,
			NULL);
		usb_transfer_schedule_ack(endpoint->out);
		return USB_REQUEST_STATUS_OK;
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_txvga_gain(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uint8_t value;
		value = max283x_set_txvga_gain(&max283x, endpoint->setup.index);
		endpoint->buffer[0] = value;
		if (value) {
			hackrf_ui()->set_bb_tx_vga_gain(endpoint->setup.index);
		}
		usb_transfer_schedule_block(
			endpoint->in,
			&endpoint->buffer,
			1,
			NULL,
			NULL);
		usb_transfer_schedule_ack(endpoint->out);
		return USB_REQUEST_STATUS_OK;
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_antenna_enable(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		switch (endpoint->setup.value) {
		case 0:
			rf_path_set_antenna(&rf_path, 0);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		case 1:
			rf_path_set_antenna(&rf_path, 1);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		default:
			return USB_REQUEST_STATUS_STALL;
		}
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_freq_explicit(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		usb_transfer_schedule_block(
			endpoint->out,
			&explicit_params,
			sizeof(struct set_freq_explicit_params),
			NULL,
			NULL);
		return USB_REQUEST_STATUS_OK;
	} else if (stage == USB_TRANSFER_STAGE_DATA) {
		if (set_freq_explicit(
			    explicit_params.if_freq_hz,
			    explicit_params.lo_freq_hz,
			    explicit_params.path)) {
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		}
		return USB_REQUEST_STATUS_STALL;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

static volatile hw_sync_mode_t _hw_sync_mode = HW_SYNC_MODE_OFF;
static volatile uint32_t _tx_underrun_limit;
static volatile uint32_t _rx_overrun_limit;

static volatile uint32_t receiver_dma_started;
static volatile uint32_t receiver_dma_pending;
static volatile uint32_t receiver_usb_started;
static volatile uint32_t receiver_usb_completed;
static uint32_t receiver_first_samples[RX_FIRST_SAMPLE_RING_BLOCKS];

void set_hw_sync_mode(const hw_sync_mode_t new_hw_sync_mode)
{
	_hw_sync_mode = new_hw_sync_mode;
}

volatile transceiver_request_t transceiver_request = {
	.mode = TRANSCEIVER_MODE_OFF,
	.seq = 0,
};

// Must be called from an atomic context (normally USB ISR)
void request_transceiver_mode(transceiver_mode_t mode)
{
	/*
	 * Do not flush the streaming endpoints from the USB setup interrupt.
	 * Publishing a new sequence makes the active mode loop leave promptly;
	 * that loop then disables streaming and flushes its endpoints from normal
	 * thread context in transceiver_shutdown(). Flushing here can wait forever
	 * on an active bulk endpoint before the request can be acknowledged.
	 */
	transceiver_request.mode = mode;
	transceiver_request.seq++;
}

void transceiver_shutdown(void)
{
	baseband_streaming_disable(&sgpio_config);
	operacake_sctimer_reset_state();

	usb_endpoint_flush(&usb_endpoint_bulk_in);
	usb_endpoint_flush(&usb_endpoint_bulk_out);

	led_off(LED2);
	led_off(LED3);
	rf_path_set_direction(&rf_path, RF_PATH_DIRECTION_OFF);
	m0_set_mode(M0_MODE_IDLE);
}

void transceiver_startup(const transceiver_mode_t mode)
{
	hackrf_ui()->set_transceiver_mode(mode);

	switch (mode) {
	case TRANSCEIVER_MODE_RX_SWEEP:
	case TRANSCEIVER_MODE_RX:
		led_off(LED3);
		led_on(LED2);
		rf_path_set_direction(&rf_path, RF_PATH_DIRECTION_RX);
		m0_set_mode(M0_MODE_RX);
		m0_state.shortfall_limit = _rx_overrun_limit;
		break;
	case TRANSCEIVER_MODE_TX:
		led_off(LED2);
		led_on(LED3);
		rf_path_set_direction(&rf_path, RF_PATH_DIRECTION_TX);
		m0_set_mode(M0_MODE_TX_START);
		m0_state.shortfall_limit = _tx_underrun_limit;
		break;
	default:
		break;
	}

	activate_best_clock_source();
	hw_sync_enable(_hw_sync_mode);
}

usb_request_status_t usb_vendor_request_set_transceiver_mode(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		switch (endpoint->setup.value) {
		case TRANSCEIVER_MODE_OFF:
		case TRANSCEIVER_MODE_RX:
		case TRANSCEIVER_MODE_TX:
		case TRANSCEIVER_MODE_RX_SWEEP:
		case TRANSCEIVER_MODE_CPLD_UPDATE:
			request_transceiver_mode(endpoint->setup.value);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		default:
			return USB_REQUEST_STATUS_STALL;
		}
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_hw_sync_mode(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		set_hw_sync_mode(endpoint->setup.value);
		usb_transfer_schedule_ack(endpoint->in);
		return USB_REQUEST_STATUS_OK;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_tx_underrun_limit(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uint32_t value = (endpoint->setup.index << 16) + endpoint->setup.value;
		_tx_underrun_limit = value;
		usb_transfer_schedule_ack(endpoint->in);
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_rx_overrun_limit(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uint32_t value = (endpoint->setup.index << 16) + endpoint->setup.value;
		_rx_overrun_limit = value;
		usb_transfer_schedule_ack(endpoint->in);
	}
	return USB_REQUEST_STATUS_OK;
}

/*
 * Keep the timing-critical M0/SGPIO writer on its original 32 KiB AHB ring,
 * then move complete 8 KiB chunks into a second 32 KiB USB ring. This is the
 * additional buffering scheme merged upstream in HackRF PR #1601.
 */
static const uint32_t RECEIVER_DMA_CHANNEL = 1U;
static const uint32_t RECEIVER_DMA_CONFIG =
	GPDMA_CCONFIG_FLOWCNTRL(0) |
	GPDMA_CCONFIG_IE(0) |
	GPDMA_CCONFIG_ITC(1) |
	GPDMA_CCONFIG_L(0) |
	GPDMA_CCONFIG_H(0);
static const uint32_t RECEIVER_DMA_CONTROL =
	GPDMA_CCONTROL_SBSIZE(7) |
	GPDMA_CCONTROL_DBSIZE(7) |
	GPDMA_CCONTROL_SWIDTH(2) |
	GPDMA_CCONTROL_DWIDTH(2) |
	GPDMA_CCONTROL_S(0) |
	GPDMA_CCONTROL_D(1) |
	GPDMA_CCONTROL_SI(1) |
	GPDMA_CCONTROL_DI(1) |
	GPDMA_CCONTROL_PROT1(0) |
	GPDMA_CCONTROL_PROT2(0) |
	GPDMA_CCONTROL_PROT3(0) |
	GPDMA_CCONTROL_I(1);

static void receiver_dma_setup(void)
{
	gpdma_controller_enable();
	GPDMA_CCONFIG(RECEIVER_DMA_CHANNEL) = RECEIVER_DMA_CONFIG;
	GPDMA_CCONTROL(RECEIVER_DMA_CHANNEL) = RECEIVER_DMA_CONTROL;
	GPDMA_CLLI(RECEIVER_DMA_CHANNEL) = 0;
	GPDMA_INTTCCLEAR = (1U << RECEIVER_DMA_CHANNEL);
	nvic_enable_irq(NVIC_DMA_IRQ);
}

static void receiver_dma_start(void* src, void* dest, size_t size)
{
	uint32_t const num_transfers = size >> 2U;
	GPDMA_CCONTROL(RECEIVER_DMA_CHANNEL) =
		RECEIVER_DMA_CONTROL | num_transfers;
	GPDMA_CSRCADDR(RECEIVER_DMA_CHANNEL) = (uint32_t) src;
	GPDMA_CDESTADDR(RECEIVER_DMA_CHANNEL) = (uint32_t) dest;
	/* Publish pending before enabling: DMA may finish immediately. */
	receiver_dma_pending = size;
	gpdma_channel_enable(RECEIVER_DMA_CHANNEL);
}

void dma_isr(void)
{
	gpdma_channel_disable(RECEIVER_DMA_CHANNEL);
	GPDMA_INTTCCLEAR = (1U << RECEIVER_DMA_CHANNEL);
	m0_state.m4_count += receiver_dma_pending;
	receiver_dma_pending = 0;
}

static void receiver_dma_start_if_possible(void)
{
	if (receiver_dma_pending != 0U) {
		return;
	}

	uint32_t const sampling_completed = m0_state.m0_count;
	uint32_t const dma_completed = m0_state.m4_count;
	uint32_t const data_available = sampling_completed - receiver_dma_started;
	uint32_t const bulk_space_available = USB_BULK_BUFFER_SIZE -
		(receiver_usb_completed - dma_completed);
	uint32_t const sample_buffer_margin = USB_SAMP_BUFFER_SIZE - data_available;

	if (data_available < RX_DMA_TRANSFER_SIZE ||
		RX_DMA_TRANSFER_SIZE > bulk_space_available) {
		return;
	}

	uint32_t const m0_buffer_half =
		sampling_completed & RX_SAMPLE_BUFFER_HALF_MASK;
	uint32_t const dma_buffer_half =
		receiver_dma_started & RX_SAMPLE_BUFFER_HALF_MASK;
	if (m0_buffer_half == dma_buffer_half &&
		sample_buffer_margin >= (USB_SAMP_BUFFER_SIZE / 2U)) {
		return;
	}

	if ((receiver_dma_started & RX_IQ_BLOCK_MASK) == 0U) {
		uint32_t const block_number =
			receiver_dma_started >> RX_IQ_BLOCK_SHIFT;
		uint32_t const sample_buffer_block = block_number & 0x1U;
		receiver_first_samples[block_number & RX_FIRST_SAMPLE_RING_MASK] =
			block_first_sample(sample_buffer_block);
	}

	uint32_t const sample_offset =
		receiver_dma_started & USB_SAMP_BUFFER_MASK;
	uint32_t const bulk_offset =
		receiver_dma_started & USB_BULK_BUFFER_MASK;
	receiver_dma_start(
		&usb_samp_buffer[sample_offset],
		&usb_bulk_buffer[bulk_offset],
		RX_DMA_TRANSFER_SIZE);
	receiver_dma_started += RX_DMA_TRANSFER_SIZE;
}

void transmitter_bulk_transfer_complete(void* user_data, unsigned int bytes_transferred)
{
	(void) user_data;
	m0_state.m4_count += bytes_transferred;
}

void receiver_header_transfer_complete(void* user_data, unsigned int bytes_transferred)
{
	(void) bytes_transferred;
	*((volatile bool*) user_data) = true;
}

static uint32_t receiver_metadata_buffers[2][SAMPLE_HEADER_SIZE / sizeof(uint32_t)];
static volatile bool receiver_metadata_buffer_available[2] = {true, true};

void receiver_bulk_transfer_complete(void* user_data, unsigned int bytes_transferred)
{
	(void) user_data;
	receiver_usb_completed += bytes_transferred;
}

static bool receiver_transfer_schedule(
	uint32_t seq,
	void* data,
	uint32_t length,
	transfer_completion_cb completion_cb,
	void* user_data)
{
	/*
	 * Do not use usb_transfer_schedule_block() in RX mode. Once the host
	 * cancels its bulk URBs during shutdown, a full device queue may never
	 * drain. A blocking enqueue would then prevent this loop from observing
	 * the control request's sequence change and make hackrf_stop_rx hang.
	 */
	while (transceiver_request.seq == seq) {
		if (usb_transfer_schedule(
				&usb_endpoint_bulk_in,
				data,
				length,
				completion_cb,
				user_data) == 0) {
			return true;
		}
	}
	return false;
}

void rx_mode(uint32_t seq)
{
	uint32_t metadata_sequence = 0;

	receiver_dma_started = 0;
	receiver_dma_pending = 0;
	receiver_usb_started = 0;
	receiver_usb_completed = 0;
	receiver_dma_setup();

	transceiver_startup(TRANSCEIVER_MODE_RX);
	/* A previous stop may have flushed metadata without its completion callback. */
	receiver_metadata_buffer_available[0] = true;
	receiver_metadata_buffer_available[1] = true;
	sample_counter_capture_enable();

	baseband_streaming_enable(&sgpio_config);

	while (transceiver_request.seq == seq) {
		receiver_dma_start_if_possible();

		uint32_t const dma_completed = m0_state.m4_count;
		if ((dma_completed - receiver_usb_started) < RX_IQ_BLOCK_SIZE) {
			continue;
		}

		uint32_t const completed_block_count =
			(receiver_usb_started >> RX_IQ_BLOCK_SHIFT) + 1U;
		bool const completes_group =
			(completed_block_count % RX_METADATA_GROUP_BLOCKS) == 0U;
		uint32_t const metadata_index = metadata_sequence & 0x1U;
		if (completes_group &&
			!receiver_metadata_buffer_available[metadata_index]) {
			continue;
		}

		uint8_t* const iq = &usb_bulk_buffer[
			receiver_usb_started & USB_BULK_BUFFER_MASK];
		if (!receiver_transfer_schedule(
				seq,
				iq,
				RX_IQ_BLOCK_SIZE,
				receiver_bulk_transfer_complete,
				NULL)) {
			goto rx_stop;
		}
		receiver_usb_started += RX_IQ_BLOCK_SIZE;

		if (completes_group) {
			uint32_t* const header =
				receiver_metadata_buffers[metadata_index];
			uint32_t const first_block_number =
				completed_block_count - RX_METADATA_GROUP_BLOCKS;
			memset(header, 0, SAMPLE_HEADER_SIZE);
			header[0] = SAMPLE_HEADER_MAGIC;
			for (uint32_t i = 0; i < RX_METADATA_GROUP_BLOCKS; ++i) {
				uint32_t const first_sample = receiver_first_samples[
					(first_block_number + i) & RX_FIRST_SAMPLE_RING_MASK];
				header[SAMPLE_HEADER_GROUP_FIRST_SAMPLE_WORD + i] =
					first_sample;
				if (i == RX_METADATA_GROUP_BLOCKS - 1U) {
					header[1] = first_sample;
				}
			}
			header[2] = sample_counter_capture_drain_packed(
				&header[3],
				SAMPLE_COUNTER_HEADER_MAX_EVENTS);
			header[SAMPLE_COUNTER_HEADER_DROPPED_EVENT_WORD] =
				sample_counter_capture_dropped();
			header[SAMPLE_COUNTER_HEADER_HIGH_WATER_WORD] =
				sample_counter_capture_high_water();
			header[SAMPLE_COUNTER_HEADER_CAPACITY_WORD] =
				sample_counter_capture_capacity();
			header[SAMPLE_HEADER_GROUP_MAGIC_WORD] = RX_METADATA_GROUP_MAGIC;
			header[SAMPLE_HEADER_GROUP_COUNT_WORD] = RX_METADATA_GROUP_BLOCKS;
			header[SAMPLE_HEADER_M0_SHORTFALL_COUNT_WORD] =
				m0_state.num_shortfalls;
			header[SAMPLE_HEADER_M0_LONGEST_SHORTFALL_WORD] =
				m0_state.longest_shortfall;
			receiver_metadata_buffer_available[metadata_index] = false;

			if (!receiver_transfer_schedule(
					seq,
					header,
					SAMPLE_HEADER_SIZE,
					receiver_header_transfer_complete,
					(void*) &receiver_metadata_buffer_available[metadata_index])) {
				goto rx_stop;
			}
			++metadata_sequence;
		}
	}

rx_stop:
	sample_counter_capture_disable();
	transceiver_shutdown();
}

void tx_mode(uint32_t seq)
{
	unsigned int usb_count = 0;
	bool started = false;

	transceiver_startup(TRANSCEIVER_MODE_TX);

	// Set up OUT transfer of buffer 0.
	usb_transfer_schedule_block(
		&usb_endpoint_bulk_out,
		&usb_samp_buffer[0x0000],
		USB_TRANSFER_SIZE,
		transmitter_bulk_transfer_complete,
		NULL);
	usb_count += USB_TRANSFER_SIZE;

	while (transceiver_request.seq == seq) {
		if (!started && (m0_state.m4_count == USB_SAMP_BUFFER_SIZE)) {
			// Buffer is now full, start streaming.
			baseband_streaming_enable(&sgpio_config);
			started = true;
		}
		if ((usb_count - m0_state.m0_count) <= USB_TRANSFER_SIZE) {
			usb_transfer_schedule_block(
				&usb_endpoint_bulk_out,
				&usb_samp_buffer[usb_count & USB_SAMP_BUFFER_MASK],
				USB_TRANSFER_SIZE,
				transmitter_bulk_transfer_complete,
				NULL);
			usb_count += USB_TRANSFER_SIZE;
		}
	}

	transceiver_shutdown();
}

void off_mode(uint32_t seq)
{
	hackrf_ui()->set_transceiver_mode(TRANSCEIVER_MODE_OFF);

	while (transceiver_request.seq == seq) {}
}
