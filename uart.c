
#include "uart.h"
#include "tm4c1294ncpdt.h"
#include <stdint.h>
#include <stdio.h>


//Initialize UART0, based on textbook.  Clock code modified.
void UART_Init(void) {
    // eable clock to UART0 periphral
    SYSCTL_RCGCUART_R |= 0x0001;
    
	
    //enable clock to GPIO Port A (PA0=RX, PA1=TX)
    SYSCTL_RCGCGPIO_R |= 0x0001;
    
	
    // wait till	UART0 peripheral to be ready
    while((SYSCTL_PRUART_R & SYSCTL_PRUART_R0) == 0){};

			
    // Disable UART0 before configuration (required before changing baud rate)
    UART0_CTL_R &= ~UART_CTL_UARTEN;

    // Set baud rate divisor for 115200 baud using 32 MHz system clock
    //Baud rate divisor = 32,000,000 / (16 * 115,200) = 17.361
    //int part:  IBRD =17
    //fraction part: FBRD= round(0.361 * 64)= 23
    UART0_IBRD_R = 17;
    UART0_FBRD_R = 23;

    // Configure frame: 8-bit word length, 1 stop bit, no parity,enable TX/RX FIFOs
    UART0_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);

    // Select system clock (32 MHz) as UART clock source
    // 0x0 = system clock (as opposed to PIOSC alternate clock)
    UART0_CC_R = 0x0;

    // Disable high-speed mode: clock divided by 16 (standard mode)
    // HSE=0 means baud clock = system clock / 16
    UART0_CTL_R &= ~UART_CTL_HSE;

    // Re-apply line control: 8-bit word length (0x60), enable FIFO (0x10)
    // This write finalizes frame format after baud rate is set
    UART0_LCRH_R = 0x0070;

    // Enable UART0: RXE (bit 9) + TXE (bit 8) + UARTEN (bit 0) = 0x0301
    UART0_CTL_R = 0x0301;

    // Configure PA1:PA0 as UART pins via Port Control register
    // PMC1=1, PMC0=1 selects UART alternate function for PA0 and PA1
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00) + 0x00000011;

    // Disable analog functionality on PA1:PA0 (not needed for UART)
    GPIO_PORTA_AMSEL_R &= ~0x03;

    // Enable alternate function on PA1:PA0 (routes pins to UART0 TX/RX)
    GPIO_PORTA_AFSEL_R |= 0x03;

    // Enable digital I/O on PA1:PA0
    GPIO_PORTA_DEN_R |= 0x03;
}

// Wait for new input, then return ASCII code 
	char UART_InChar(void){
		while((UART0_FR_R&0x0010) != 0);		// wait until RXFE is 0   
		return((char)(UART0_DR_R&0xFF));
	} 
	
	// Wait for buffer to be not full, then output 
	void UART_OutChar(char data){
		while((UART0_FR_R&0x0020) != 0);	// wait until TXFF is 0   
		UART0_DR_R = data;
	} 
	void UART_printf(const char* array){
		int ptr=0;
		while(array[ptr]){
			UART_OutChar(array[ptr]);
			ptr++;
		}
	}
	
	void Status_Check(char* array, int status){
			if (status != 0){
				UART_printf(array);
				sprintf(printf_buffer," failed with (%d)\r\n",status);
				UART_printf(printf_buffer);
			}else
			{
				UART_printf(array);
				UART_printf(" Successful.\r\n");
			}
	}