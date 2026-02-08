/*
 * simple_tx.c
 *
 * Created: 9/17/2021 4:45:01 PM
 *  Author: Emin Eminof
 */ 

#include "simple_tx.h"
#include "main.h"
#include "config_options.h"

int simple_tx(void)
{
	dwt_txconfig_t txconfig_options = { 0x05, 0xfdfdfdfd, 0x0 };
		
	dwt_config_t config_options = {
		5,                  /* Channel number. */
		DWT_PLEN_128,      /* Preamble length. Used in TX only. */
		DWT_PAC8,           /* Preamble acquisition chunk size. Used in RX only. */
		9,                  /* TX preamble code. Used in TX only. */
		9,                  /* RX preamble code. Used in RX only. */
		1,                  /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
		DWT_BR_6M8,         /* Data rate. */
		DWT_PHRMODE_STD,    /* PHY header mode. */
		DWT_PHRRATE_STD,    /* PHY header rate. */
		(128 + 8 - 8),  /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
		DWT_STS_MODE_OFF,     /* Mode 1 STS enabled */
		DWT_STS_LEN_64,    /* (STS length  in blocks of 8) - 1*/
		DWT_PDOA_M0         /* PDOA mode off */
	};
	
	/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
		*     - byte 0: frame type (0xC5 for a blink).
		*     - byte 1: sequence number, incremented for each new frame.
		*     - byte 2 -> 9: device ID, see NOTE 1 below.
		*/
	
	
	
	if(dwt_configure(&config_options)){
		UART_puts("CONFIG TX FAILED\r\n");
		return 0;
	}
	
	// Configure DW IC. See NOTE 5 below.
	dwt_configuretxrf(&txconfig_options);
	
	static uint8_t tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E'};

	// Index to access to sequence number of the blink frame in the tx_msg array.
	#define BLINK_FRAME_SN_IDX 1

	#define FRAME_LENGTH    (sizeof(tx_msg)+FCS_LEN) // The real length that is going to be transmitted

	// Inter-frame delay period, in milliseconds.
	#define TX_DELAY_MS 2000

	#define OTP_ADDRESS     0x50 // Address to write - OTP
	#define OTP_DATA        0x87654321 // Data to write - OTP

	//extern dwt_config_t config_options;
	//extern dwt_txconfig_t txconfig_options;
	
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
	
	port_set_dw_ic_spi_fastrate();
	
	UART_puts("CONGRATS!!! Config and Init functions complete with no errors!\r\n");
	UART_puts("Starting main loop..\r\n");

	// Configure the TX spectrum parameters (power PG delay and PG Count) 
	dwt_configuretxrf(&txconfig_options);
		
	// Loop forever sending frames periodically.
	while (1) 
	{	
		// Write frame data to DW IC and prepare transmission. See NOTE 3 below.
		dwt_writetxdata(FRAME_LENGTH-FCS_LEN, tx_msg, 0); // Zero offset in TX buffer. 

		// In this example since the length of the transmitted frame does not change,
		// nor the other parameters of the dwt_writetxfctrl function, the
		// dwt_writetxfctrl call could be outside the main while(1) loop.
		//
		dwt_writetxfctrl(FRAME_LENGTH, 0, 0); // Zero offset in TX buffer, no ranging. //

		// Start transmission. 
		dwt_starttx(DWT_START_TX_IMMEDIATE);
		// Poll DW IC until TX frame sent event set. See NOTE 4 below.
		// STATUS register is 4 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API

		// function to access it.
		while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
		{ };

		// Clear TX frame sent event. 
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

		UART_puts("TX Frame Sent..");
		UART_puts("\r\n");

		// Execute a delay between transmissions. 
		sleepms(TX_DELAY_MS);

		// Increment the blink frame sequence number (modulo 256). 
		tx_msg[BLINK_FRAME_SN_IDX]++;
	}
}

