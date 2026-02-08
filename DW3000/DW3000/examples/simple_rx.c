/*
 * simple_rx.c
 *
 * Modified to read CIR and output dB magnitude to UART
 */ 

#include "simple_rx.h"
#include "main.h"
#include "config_options.h"
#include <stdio.h>
#include <math.h> // C?n cho hàm sqrt() và log10()

// ??nh ngh?a ?? dài vùng nh? Accumulator c?n ??c.
// V?i Preamble 64 ho?c 128, vùng quan tr?ng th??ng n?m trong kho?ng 800-1000 m?u ??u tiên.
// ??c quá nhi?u s? làm ch?m quá trình nh?n gói tin ti?p theo.
#define ACCUM_READ_LEN  900 

// Hàm h? tr? ??c và in CIR
void read_and_print_cir(void)
{
    uint8_t buffer[7]; // 6 byte d? li?u + 1 byte dummy ??u tiên theo quy ??nh c?a DW3000
    int32_t real = 0;
    int32_t imag = 0;
    float amp = 0.0;
    float db = 0.0;
    char msg[64];

    UART_puts("--- START CIR DATA ---\r\n");
    UART_puts("Index,Magnitude_dB\r\n");

    // ??c t?ng m?u m?t ?? ti?t ki?m RAM cho ATmega328p
    for(int i = 0; i < ACCUM_READ_LEN; i++)
    {
        // DW3000: M?i m?u ph?c g?m 6 byte (3 byte th?c, 3 byte ?o).
        // Hàm dwt_readaccdata yêu c?u ?? dài ??c + 1 byte dummy ? ??u.
        // offset = i * 6 (vì m?i m?u 6 byte)
        // L?u ý: dwt_readaccdata trong API DW3000 có th? ?ã x? lý dummy byte bên trong tùy implementation,
        // nh?ng theo deca_device_api.h:
        // "Because of an internal memory access delay... the first octet output is a dummy octet."
        
        // ??c 6 byte d? li?u t?i v? trí m?u th? i
        // Ta c?n ??c 7 byte vào buffer, byte ??u tiên (buffer[0]) là rác.
        dwt_readaccdata(buffer, 7, i * 6); 

        // Ghép 3 byte thành s? nguyên 18-bit cho ph?n Th?c (Real) - Little Endian
        // buffer[1] là LSB, buffer[3] là MSB c?a ph?n Real
        real = (int32_t)buffer[1] | ((int32_t)buffer[2] << 8) | ((int32_t)buffer[3] << 16);
        // X? lý d?u (Sign extension) cho s? 18-bit
        real &= 0x03FFFF; // Gi? l?i 18 bit th?p
        if (real & 0x020000) // N?u bit th? 17 (bit d?u) là 1
        {
            real |= 0xFFFC0000; // M? r?ng d?u thành s? âm 32-bit
        }

        // Ghép 3 byte thành s? nguyên 18-bit cho ph?n ?o (Imaginary) - Little Endian
        // buffer[4] là LSB, buffer[6] là MSB c?a ph?n Imag
        imag = (int32_t)buffer[4] | ((int32_t)buffer[5] << 8) | ((int32_t)buffer[6] << 16);
        // X? lý d?u
        imag &= 0x03FFFF;
        if (imag & 0x020000)
        {
            imag |= 0xFFFC0000;
        }

        // Tính biên ??: Magnitude = sqrt(Real^2 + Imag^2)
        amp = sqrt((float)(real * real + imag * imag));

        // Tính dB: dB = 20 * log10(Magnitude)
        // Tránh log(0)
        if (amp > 0) {
            db = 20.0 * log10(amp);
        } else {
            db = -100; // Giá tr? sàn n?u biên ?? b?ng 0
        }

        // In ra Serial theo ??nh d?ng CSV: Index, dB
        // S? d?ng sprintf ?? format s? float (l?u ý c?u hình linker ?? h? tr? float printf n?u c?n)
        // N?u ATmega không h? tr? in float t?t, có th? ép ki?u v? (int)db.
        sprintf(msg, "%d,%d\r\n", i, (int)db); 
        UART_puts(msg);
    }

    UART_puts("--- END CIR DATA ---\r\n");
}

int simple_rx(void)
{
	/* Default communication configuration. We use default non-STS DW mode. */
	static dwt_config_t config = {
		5,               /* Channel number. */
		DWT_PLEN_128,    /* Preamble length. Used in TX only. */
		DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
		9,               /* TX preamble code. Used in TX only. */
		9,               /* RX preamble code. Used in RX only. */
		1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
		DWT_BR_6M8,      /* Data rate. */
		DWT_PHRMODE_STD, /* PHY header mode. */
		DWT_PHRRATE_STD, /* PHY header rate. */
		(129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
		DWT_STS_MODE_OFF, /* STS disabled */
		DWT_STS_LEN_64,  /* STS length see allowed values in Enum dwt_sts_lengths_e */
		DWT_PDOA_M0      /* PDOA mode off */
	};

	/* Buffer to store received frame. See NOTE 1 below. */
	static uint8_t rx_buffer[FRAME_LEN_MAX];
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    uint32_t status_reg;
    /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
    uint16_t frame_len;	
	
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
		
	// Loop forever sending frames periodically.
    while (1)
    {
        /* Clear local RX buffer */
        memset(rx_buffer,0,sizeof(rx_buffer));

        /* Activate reception immediately. See NOTE 2 below. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll until a frame is properly received or an error/timeout occurs. */
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

            /* Clear good RX frame event in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            UART_puts("FRAME RECEIVED\r\n");
            
            // ==========================================
            // G?I HÀM ??C VÀ IN CIR T?I ?ÂY
            // ==========================================
            read_and_print_cir();
            
        }
        else
        {
            /* Clear RX error events in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }
}