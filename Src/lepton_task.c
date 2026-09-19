
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "usb_device.h"


#include "pt.h"
#include "lepton.h"
#include "lepton_i2c.h"
#include "tmp007_i2c.h"
#include "usbd_uvc.h"
#include "usbd_uvc_if.h"
#include "circ_buf.h"

#include "tasks.h"
#include "project_config.h"
#include "wedge_lab.h"

extern volatile uint8_t g_lepton_type_3;
extern struct uvc_streaming_control videoCommitControl;

lepton_buffer *completed_buffer;
uint32_t completed_frame_count;

uint8_t lepton_i2c_buffer[36];

#define RING_SIZE (4)
lepton_buffer lepton_buffers[RING_SIZE];

volatile struct wedge_lab g_lab = { .magic = LAB_MAGIC, .auto_cmd = LAB_AUTO_CMD };

lepton_buffer* completed_frames_buf[RING_SIZE] = { 0 };
DECLARE_CIRC_BUF_HANDLE(completed_frames_buf);

struct rgb_to_yuv_state {
  struct pt pt;
  lepton_buffer *restrict rgb;
};

#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

uint32_t get_lepton_buffer(lepton_buffer **buffer)
{
  if (buffer != NULL)
    *buffer = completed_buffer;
	return completed_frame_count;
}

lepton_buffer* dequeue_lepton_buffer()
{
  if (empty(CIRC_BUF_HANDLE(completed_frames_buf)))
    return NULL;
  else
    return shift(CIRC_BUF_HANDLE(completed_frames_buf));
}

void init_lepton_task()
{
  int i;
  for (i = 0; i < RING_SIZE; i++)
  {
    lepton_buffers[i].number = i;
    lepton_buffers[i].status = LEPTON_STATUS_OK;
    DEBUG_PRINTF("Initialized lepton buffer %d @ %p\r\n", i, &lepton_buffers[i]);
  }
}

static float k_to_c(uint16_t unitsKelvin)
{
	return ( ( (float)( unitsKelvin / 100 ) + ( (float)( unitsKelvin % 100 ) * 0.01f ) ) - 273.15f );
}

static void print_telemetry_temps(telemetry_data_l2* telemetry)
{
	//
	uint16_t fpa_temperature_k = telemetry->fpa_temp_100k[0];
	uint16_t aux_temperature_k = telemetry->housing_temp_100k[0];

	float fpa_c = k_to_c(fpa_temperature_k);
	float aux_c = k_to_c(aux_temperature_k);

	DEBUG_PRINTF("fpa %d.%d°c, aux/housing: %d.%d°c\r\n",
		(int)(fpa_c), (int)((fpa_c-(int)fpa_c)*100),
		(int)(aux_c), (int)((aux_c-(int)aux_c)*100));
}

static lepton_buffer *current_buffer = NULL;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static int current_buffer_index = 0;
	g_lab.vsync_irqs++;
	lepton_buffer *buffer = &lepton_buffers[current_buffer_index];
	current_buffer = buffer;
	current_buffer_index = ((current_buffer_index + 1) % RING_SIZE);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}

/* ---- wedge lab ----------------------------------------------------------
 * lab_action() runs one recovery action (see wedge_lab.h) as a child
 * protothread of lepton_task, always at a point where no transfer is in
 * flight and EXTI is disabled, so SCK is idle unless the action clocks it. */

static void lab_apply_format_config(void)
{
	/* the same configuration lepton_task applies at stream start */
	if (g_format_y16)
	{
		if (videoCommitControl.bFrameIndex == VS_FRAME_INDEX_TELEMETRIC)
			enable_telemetry();
		else
			disable_telemetry();
		disable_lepton_agc();
		enable_raw14();
	}
	else
	{
		disable_telemetry();
		enable_lepton_agc();
		enable_rgb888((LEP_PCOLOR_LUT_E)-1);
	}
}

#define LAB_WALK_MAX_PACKETS (2000)

static struct pt lab_pt;

static PT_THREAD( lab_action(struct pt *pt))
{
	static uint32_t cmd, hold_ms, t0;
	static int tries;
	static uint16_t hdr;
	lepton_buffer *buf = &lepton_buffers[0];

	PT_BEGIN(pt);

	cmd = g_lab.cmd;
	hold_ms = g_lab.arg ? g_lab.arg : 250;
	g_lab.state = 5;
	g_lab.last_cmd = cmd;
	g_lab.frames_ok_before = g_lab.frames_ok;

	if (cmd == LAB_CMD_MCU_RESET)
		NVIC_SystemReset();

	if (cmd == LAB_CMD_CS_ONLY || cmd == LAB_CMD_CS_WALK || cmd == LAB_CMD_HW_RESET)
		lepton_cs_release();

	if (cmd == LAB_CMD_HW_RESET)
	{
		lepton_hw_reset_assert();
		t0 = HAL_GetTick();
		PT_WAIT_UNTIL(pt, (HAL_GetTick() - t0) > 190);
		lepton_hw_pwdn_release();
		t0 = HAL_GetTick();
		PT_WAIT_UNTIL(pt, (HAL_GetTick() - t0) > 190);
		lepton_hw_reset_release();
		t0 = HAL_GetTick();
		PT_WAIT_UNTIL(pt, (HAL_GetTick() - t0) > 1500);   /* boot uses 1000 */
		lepton_reinit_after_reset();
		lab_apply_format_config();
	}
	else if (cmd == LAB_CMD_CCI_CYCLE)
	{
		lepton_low_power();
		t0 = HAL_GetTick();
		PT_WAIT_UNTIL(pt, (HAL_GetTick() - t0) > 250);
		lepton_power_on();
		t0 = HAL_GetTick();
		PT_WAIT_UNTIL(pt, (HAL_GetTick() - t0) > 250);
		lepton_restore_vsync_config();
		lab_apply_format_config();
	}
	else if (cmd == LAB_CMD_SPI_REINIT)
	{
		lepton_spi_reinit();
	}

	/* the idle window: /CS high for 1, 2, 5; /CS low for 3 */
	if (cmd == LAB_CMD_CS_ONLY || cmd == LAB_CMD_CS_WALK ||
	    cmd == LAB_CMD_IDLE_WALK || cmd == LAB_CMD_HW_RESET)
	{
		t0 = HAL_GetTick();
		PT_WAIT_UNTIL(pt, (HAL_GetTick() - t0) > hold_ms);
	}
	lepton_cs_restore();   /* no-op if /CS was never released */

	/* datasheet steps 2-4: keep clocking, discards first, until packet 0, then
	   read the rest of that segment without waiting for VSYNC */
	if (cmd == LAB_CMD_CS_WALK || cmd == LAB_CMD_IDLE_WALK ||
	    cmd == LAB_CMD_HW_RESET || cmd == LAB_CMD_SPI_REINIT)
	{
		tries = 0;
		do {
			g_lab.walk_packets++;
			lepton_transfer(buf, 1);
			t0 = HAL_GetTick();
			PT_YIELD_UNTIL(pt, buf->status != LEPTON_STATUS_TRANSFERRING || (HAL_GetTick() - t0) > 200);
			hdr = g_format_y16 ? buf->lines.y16[0].header[0] : buf->lines.rgb[0].header[0];
			g_lab.walk_last_header = hdr;
		} while (buf->status == LEPTON_STATUS_OK && (hdr & 0x8fff) != 0 &&
		         ++tries < LAB_WALK_MAX_PACKETS);

		if (buf->status == LEPTON_STATUS_OK && (hdr & 0x8fff) == 0)
		{
			g_lab.lab_walk_found++;
			lepton_transfer(buf, IMAGE_NUM_LINES + g_telemetry_num_lines - 1);
			t0 = HAL_GetTick();
			PT_YIELD_UNTIL(pt, buf->status != LEPTON_STATUS_TRANSFERRING || (HAL_GetTick() - t0) > 200);
		}
		else
		{
			g_lab.lab_walk_giveups++;
		}
	}

	g_lab.cmds_done++;
	g_lab.cmd = 0;
	PT_END(pt);
}

PT_THREAD( lepton_task(struct pt *pt))
{
	PT_BEGIN(pt);

	static uint32_t curtick = 0;
	static uint32_t last_tick = 0;
	static uint32_t last_logged_count = 0;
	static uint32_t current_frame_count = 0;
	static int transferring_timer = 0;
	static uint8_t current_segment = 0;
	static uint8_t last_end_line = 0;
	static uint8_t has_started_a_stream = 0;
	static uint8_t lab_auto_pending = 0;   /* an auto action ran, no frame since */
	curtick = last_tick = HAL_GetTick();

#ifdef THERMAL_DATA_UART
	enable_telemetry();
	enable_raw14();
#endif

	while (1)
	{
#ifndef THERMAL_DATA_UART
		if (g_uvc_stream_status == 0)
		{
			lepton_low_power();
			if (has_started_a_stream)
			{
				if (g_format_y16)
				{
					// no cleanup after y16
				}
				else
				{
					disable_rgb888();
				}
			}

			// Start slow blink (1 Hz)
			g_lab.state = 1;
			lab_auto_pending = 0;
			g_lab.bad_run = 0;
			while (g_uvc_stream_status == 0)
			{
				HAL_GPIO_TogglePin(SYSTEM_LED_GPIO_Port, SYSTEM_LED_Pin);

				transferring_timer = HAL_GetTick();
				PT_YIELD_UNTIL(pt, g_uvc_stream_status != 0 || (HAL_GetTick() - transferring_timer) > 500);
			}

			g_format_y16 = (videoCommitControl.bFormatIndex == VS_FMT_INDEX(Y16));

			if (g_format_y16)
			{
				if (videoCommitControl.bFrameIndex == VS_FRAME_INDEX_TELEMETRIC)
					enable_telemetry();
				else
					disable_telemetry();
				disable_lepton_agc();
				enable_raw14();
			}
			else
			{
				disable_telemetry();
				enable_lepton_agc();
				enable_rgb888((LEP_PCOLOR_LUT_E)-1); // -1 means attempt to continue using the current palette (PcolorLUT)
			}
			has_started_a_stream = 1;

			// flush out any old data
			while (dequeue_lepton_buffer() != NULL) {}

			// Make sure we're not about to service an old irq when the interrupts are re-enabled
			__HAL_GPIO_EXTI_CLEAR_IT(EXTI15_10_IRQn);

			lepton_power_on();
		}
#endif

		// wedge lab: run a pending action with EXTI off and nothing in flight
		if (g_lab.cmd)
		{
			HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
			current_buffer = NULL;
			while (dequeue_lepton_buffer() != NULL) {}
			PT_SPAWN(pt, &lab_pt, lab_action(&lab_pt));
			__HAL_GPIO_EXTI_CLEAR_IT(LEPTON_GPIO3_Pin);
			HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
			current_frame_count = 0;
			g_lab.bad_run = 0;
			g_lab.walk_discard_run = 0;
		}

		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

		g_lab.state = 2;
		PT_WAIT_UNTIL(pt, current_buffer != NULL || g_lab.cmd != 0);
		if (current_buffer == NULL)
			continue;   // an action was requested while waiting for VSYNC
		g_lab.state = 3;

		lepton_transfer(current_buffer, IMAGE_NUM_LINES + g_telemetry_num_lines);

		transferring_timer = HAL_GetTick();
		PT_YIELD_UNTIL(pt, current_buffer->status != LEPTON_STATUS_TRANSFERRING || ((HAL_GetTick() - transferring_timer) > 200));

		if (complete_lepton_transfer(current_buffer) != LEPTON_STATUS_OK)
		{
			DEBUG_PRINTF("Lepton transfer failed: %d\r\n", current_buffer->status);
			current_buffer = NULL;
			continue;
		}

		current_frame_count++;
		g_lab.reads++;

		if (g_format_y16)
		{
			current_segment = ((current_buffer->lines.y16[IMAGE_OFFSET_LINES + 20].header[0] & 0x7000) >> 12);
			last_end_line = (current_buffer->lines.y16[IMAGE_OFFSET_LINES + IMAGE_NUM_LINES + g_telemetry_num_lines - 1].header[0] & 0x00ff);
		}
		else
		{
			current_segment = ((current_buffer->lines.rgb[IMAGE_OFFSET_LINES + 20].header[0] & 0x7000) >> 12);
			last_end_line = (current_buffer->lines.rgb[IMAGE_OFFSET_LINES + IMAGE_NUM_LINES + g_telemetry_num_lines - 1].header[0] & 0x00ff);
		}

		current_buffer->segment = current_segment;

		if (last_end_line != (IMAGE_NUM_LINES + g_telemetry_num_lines - 1))
		{
			g_lab.frames_bad++;
			if (++g_lab.bad_run > g_lab.worst_bad_run)
				g_lab.worst_bad_run = g_lab.bad_run;
			if (g_lab.bad_run == LAB_WEDGE_BAD_RUN)
			{
				g_lab.wedges_detected++;
				if (LAB_AUTO_CMD && g_lab.cmd == 0)
				{
					g_lab.auto_fired++;
					lab_auto_pending = 1;
					g_lab.cmd = LAB_AUTO_CMD;
				}
			}

			// flush out any old data since it's no good
			while (dequeue_lepton_buffer() != NULL) {}

			if (current_frame_count > 2)
			{
				uint16_t last_header;

				DEBUG_PRINTF("Synchronization lost, status: %d, last end line %d\r\n",
					current_buffer->status, last_end_line);

				transferring_timer = HAL_GetTick();
				PT_WAIT_UNTIL(pt, (HAL_GetTick() - transferring_timer) > 185);

				// transfer packets until we've actually re-synchronized
				g_lab.resyncs++;
				g_lab.state = 4;
				g_lab.walk_discard_run = 0;
				do {
					g_lab.walk_packets++;
					lepton_transfer(current_buffer, 1);

					transferring_timer = HAL_GetTick();
					PT_YIELD_UNTIL(pt, current_buffer->status != LEPTON_STATUS_TRANSFERRING || ((HAL_GetTick() - transferring_timer) > 200));

					last_header = (g_format_y16 ?
							current_buffer->lines.y16[0].header[0] :
							current_buffer->lines.rgb[0].header[0]);

					// wedge lab bookkeeping; the loop condition is 1.3.0's plus
					// "no action pending", so an action can break a stuck walk
					g_lab.walk_last_header = last_header;
					if ((last_header & 0x0f00) == 0x0f00)
					{
						if (++g_lab.walk_discard_run == LAB_WEDGE_DISCARD_RUN)
						{
							g_lab.wedges_detected++;
							if (LAB_AUTO_CMD && g_lab.cmd == 0)
							{
								g_lab.auto_fired++;
								lab_auto_pending = 1;
								g_lab.cmd = LAB_AUTO_CMD;
							}
						}
					}

				} while (current_buffer->status == LEPTON_STATUS_OK && (last_header & 0x0f00) == 0x0f00 &&
				         g_lab.cmd == 0);

				if (g_lab.cmd == 0)
				{
					// we picked up the start of a new packet, so read the rest of it in
					lepton_transfer(current_buffer, IMAGE_NUM_LINES + g_telemetry_num_lines - 1);

					transferring_timer = HAL_GetTick();
					PT_YIELD_UNTIL(pt, current_buffer->status != LEPTON_STATUS_TRANSFERRING || ((HAL_GetTick() - transferring_timer) > 200));
				}

				// Make sure we're not about to service an old irq when the interrupts are re-enabled
				__HAL_GPIO_EXTI_CLEAR_IT(EXTI15_10_IRQn);

				current_frame_count = 0;
			}

			current_buffer = NULL;

			continue;
		}

		// this read validated
		g_lab.frames_ok++;
		g_lab.bad_run = 0;
		g_lab.walk_discard_run = 0;
		if (lab_auto_pending)
		{
			g_lab.auto_recovered++;
			lab_auto_pending = 0;
		}

		if (((curtick = HAL_GetTick()) - last_tick) > 3000)
		{
#ifdef PRINT_FPS
			DEBUG_PRINTF("fps: %lu, last end line: %d, frame #%lu, buffer %p\r\n",
				(current_frame_count - last_logged_count) / 3,
				last_end_line,
				current_frame_count, current_buffer
			);
#endif


			if (g_telemetry_num_lines > 0 && g_lepton_type_3 == 0)
			{
				if (g_format_y16)
					print_telemetry_temps(&current_buffer->lines.y16[TELEMETRY_OFFSET_LINES].data.telemetry_data);
				else
					print_telemetry_temps(&current_buffer->lines.rgb[TELEMETRY_OFFSET_LINES].data.telemetry_data);
			}

#if defined(TMP007)
			read_tmp007_regs();
#endif

			last_tick = curtick;
			last_logged_count = current_frame_count;
		}

		// Need to update completed buffer for clients?
		if (g_lepton_type_3 == 0 || (current_segment > 0 && current_segment <= 4))
		{
			static int row;

			completed_buffer = current_buffer;
			completed_frame_count = current_frame_count;

			HAL_GPIO_TogglePin(SYSTEM_LED_GPIO_Port, SYSTEM_LED_Pin);

			if (!g_format_y16)
			{
				for (row = 0; row < (IMAGE_NUM_LINES + g_telemetry_num_lines); row++)
				{
					uint16_t* lineptr = (uint16_t*)completed_buffer->lines.rgb[IMAGE_OFFSET_LINES + row].data.image_data;
					while (lineptr < (uint16_t*)&completed_buffer->lines.rgb[IMAGE_OFFSET_LINES + row].data.image_data[FRAME_LINE_LENGTH])
					{
					  uint8_t* bytes = (uint8_t*)lineptr;
					  *lineptr++ = bytes[0] << 8 | bytes[1];
					}
					PT_YIELD(pt);
				}
			}

			if (!full(CIRC_BUF_HANDLE(completed_frames_buf)))
				push(CIRC_BUF_HANDLE(completed_frames_buf), completed_buffer);
		}

		current_buffer = NULL;
	}
	PT_END(pt);
}

static inline uint8_t clamp (float x)
{
  if (x < 0)         return 0;
  else if (x > 255)  return 255;
  else               return (uint8_t)x;
}

void rgb2yuv(const rgb_t val, uint8_t *y, uint8_t *u, uint8_t *v)
{
	float r = val.r, g = val.g, b = val.b;

	float y1 = 0.299f * r + 0.587f * g + 0.114f * b;

	*y =   clamp (0.859f *      y1  +  16.0f);
	if (u)
		*u = clamp (0.496f * (b - y1) + 128.0f);
	if (v)
		*v = clamp (0.627f * (r - y1) + 128.0f);
}
