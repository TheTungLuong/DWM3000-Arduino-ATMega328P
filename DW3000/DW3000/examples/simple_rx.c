/*
 * simple_rx.c
 *
 * Created: 9/17/2021 4:45:01 PM
 *  Author: Emin Eminof
 */
#include "simple_rx.h"
#include "main.h"
#include "config_options.h"
#include "string.h"
#include "math.h"
#include "inttypes.h"

#define CIR_PRE_SAMPLES   10
#define CIR_POST_SAMPLES  50
#define ACCUMULATOR_MAX_SAMPLES 1024

static void dump_cir_window(uint16_t startIdx, uint16_t endIdx);
static void log_cir_window(const dwt_rxdiag_t *diag);

static void read_single_cir_sample(uint16_t sampleIndex, int32_t *re, int32_t *im)
{
	uint8_t accum_data[7];

	dwt_readaccdata(accum_data, sizeof(accum_data), sampleIndex);

	int32_t r = 0;
	int32_t j = 0;

	r  = accum_data[1];
	r |= ((int32_t)accum_data[2] << 8);
	r |= ((int32_t)(accum_data[3] & 0x03) << 16);

	j  = accum_data[4];
	j |= ((int32_t)accum_data[5] << 8);
	j |= ((int32_t)(accum_data[6] & 0x03) << 16);

	if (r & 0x020000) r |= 0xFFFC0000;
	if (j & 0x020000) j |= 0xFFFC0000;

	*re = r;
	*im = j;
}

static void log_cir_window(const dwt_rxdiag_t *diag)
{
	uint16_t fpi = diag->ipatovFpIndex;
	if (fpi >= ACCUMULATOR_MAX_SAMPLES)
	{
		fpi = 0;
	}

	uint16_t start_idx = (fpi > CIR_PRE_SAMPLES) ? (fpi - CIR_PRE_SAMPLES) : 0;
	uint16_t end_idx = fpi + CIR_POST_SAMPLES;
	if (end_idx >= ACCUMULATOR_MAX_SAMPLES)
	{
		end_idx = ACCUMULATOR_MAX_SAMPLES - 1;
	}

	printf("CIR_META,FP=%" PRIu16 ",START=%" PRIu16 ",END=%" PRIu16 "\r\n", fpi, start_idx, end_idx);
	dump_cir_window(start_idx, end_idx);
}

static void dump_cir_window(uint16_t startIdx, uint16_t endIdx)
{
	int32_t re = 0;
	int32_t im = 0;

	printf("FRAME_BEGIN\r\n");
	for (uint16_t idx = startIdx; idx <= endIdx; idx++)
	{
		read_single_cir_sample(idx, &re, &im);
		float magnitude = sqrtf(((float)re * (float)re) + ((float)im * (float)im));
		printf("CIR,%" PRIu16 ",%ld,%ld,%.3f\r\n", idx, (long)re, (long)im, (double)magnitude);
	}
	printf("FRAME_END\r\n");
}

int simple_rx(void)
{
	/* Buffer to store received frame. See NOTE 1 below. */
	static uint8_t rx_buffer[FRAME_LEN_MAX];
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    uint32_t status_reg;
    /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
    uint16_t frame_len;	
	dwt_rxdiag_t diag;
	port_set_dw_ic_spi_slowrate();
	dwt_softreset(); // do a soft reset with SPI due to lack of RSTn line on our board
	
	sleepms(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
	
	while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding 
	{
		UART_puts("IDLE FAILED\r\n");
		return 0;
	}

	if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
	{
		UART_puts("INIT FAILED\r\n");
		return 0;
	}
	
	// Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
	dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

	// Configure DW IC. See NOTE 5 below.
	if(dwt_configure(&config_options)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
	{
		UART_puts("CONFIG FAILED\r\n");
		return 0;
	}
	
	/* Enable CIA diagnostics so the accumulator (CIR) and first-path index can be read. */
	dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);
	port_set_dw_ic_spi_fastrate();
	
	UART_puts("CONGRATS!!! Config and Init functions complete with no errors!\r\n");
	UART_puts("Starting main loop..\r\n");
		
	// Loop forever sending frames periodically.
    while (1)
    {
        /* TESTING BREAKPOINT LOCATION #1 */

        /* Clear local RX buffer to avoid having leftovers from previous receptions  This is not necessary but is included here to aid reading
         * the RX buffer.
         * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
         * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. */
        memset(rx_buffer,0,sizeof(rx_buffer));

        /* Activate reception immediately. See NOTE 2 below. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll until a frame is properly received or an error/timeout occurs. See NOTE 3 below.
         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
         * function to access it. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR )))
        { };

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
            /* A frame has been received, copy it to our local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
            if (frame_len <= FRAME_LEN_MAX)
            {
				dwt_readrxdata(rx_buffer, frame_len-FCS_LEN, 0); /* No need to read the FCS/CRC. */
            }
			
			dwt_readdiagnostics(&diag);
			log_cir_window(&diag);

            /* Clear good RX frame event in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            UART_puts("FRAME RECEIVED\r\n");

        }
        else
        {
            /* Clear RX error events in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }
}
