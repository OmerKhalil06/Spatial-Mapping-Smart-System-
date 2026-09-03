/* 
Khalio9, 400568897, Omar Khalil
 2026-04-01
 COMPENG 2DX3 - Deliverable 2
 Spatial mapping: ToF sensor + stepper motor + UART to PC + MATLAB

 Student specific values  
   J = 7  Bus speed = 32 MHz
   H = 9  Measurement LED = PN1, UART Tx LED = PN0, Additional LED = PF4


BUTTONS
   PJ0 - Start/Stop data acquisition (toggle)
   PJ1 - Trigger one full 360 degree scan (8 measurements x 45 deg steps)

LEDs
   PN1 - Flashes on each ToF distance measurement         (Measurement Status)
   PN0 - Flashes at beginning of each UART transmit block (UART Tx Status)
   PF4 - ON while motor is rotating                       (Additional Status)
   PF0 - unused
	 
	 
Pinout 

Functions 
   1. Pressing PJ0 to enable acquisition/armed mode.
   2. Press PJ1 to execute one 360-deg scan:
        - motor steps 45 deg, ToF reads distance, UART sends value (x8)
        - displacement counter increments (10 cm steps for demo)
   3. Press PJ0 again to disarm.
   4. Repeat step 2 for additional displacement positions.
 

UART output format (MATLAB reads):
   At start of each scan block: sends "SCAN,<displacement_mm>\r\n"
   For each of 8 measurements:  sends "<distance_mm>\r\n"
   After all scans done:        sends "END\r\n"
	 
Wiring Summary 
	PH0-PH3 = Motor 
	PJ0 - PUR (active LO) start/stop scan 
	PJ1 - PUR (active low) arm/disarm  
	
	PN1 - D1 measurnment flash 
	PN0 - D2 flash for uart start 
	PF4 - D3 flash when motoro on   
	PF0 - D4 used for bus speed check 
	
	PB2 - SCL
	PB3 - SDA 
	PG0 - XSHUT 
	

*/ 
 
 // Include Header Files 
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
 
// I2C defines, unchanged  from lab) 
#define I2C_MCS_ACK     0x00000008 //data acknowledge enabled 
#define I2C_MCS_DATACK  0x00000008 //acknowledge daata
#define I2C_MCS_ADRACK  0x00000004//acknowledge address 
#define I2C_MCS_STOP    0x00000004 // start condition generate 
#define I2C_MCS_START   0x00000002// stop condition generated 
#define I2C_MCS_ERROR   0x00000002 // error flag  
#define I2C_MCS_RUN     0x00000001//i2c start 
#define I2C_MCS_BUSY    0x00000001 // start i2c communication   
#define I2C_MCR_MFE     0x00000010 //master function enable 
#define MAXRETRIES      5 //max number of entries before failing 

//ToF I2C address information, not changed from lab 
uint16_t dev = 0x29; //default address from DS 
int status = 0; //used for return values from ToF 0 = success, otherwise fail  



////////////////////////////////////////////////////////////////
////////////////////PORT INITIALIZATION ////////////////////////
////////////////////////////////////////////////////////////////


//DEN, 1 = enable, 0 =disable 
//DIR: 1=output, 0 = input 
//DATA 1 = HI (3.3v) and 0 = LO 
// AMSEL: 1= analog mode enabled 
//PUR: 1=actioviates pull up resistor 
		// Port H: stepper motor (PH3:0)
void PortH_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0) {}
    GPIO_PORTH_DIR_R   |=  0x0F;
    GPIO_PORTH_AFSEL_R &=  0xF0;
    GPIO_PORTH_DEN_R   |=  0x0F;
    GPIO_PORTH_AMSEL_R &=  0xF0;
}
 
			// Port J: onboard buttons PJ0 (start/stop) PJ1 (scan trigger), active-LOW with pull-up
void PortJ_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0) {}
    GPIO_PORTJ_DIR_R   &= ~0x03;
    GPIO_PORTJ_AFSEL_R &= ~0x03;
    GPIO_PORTJ_DEN_R   |=  0x03;
    GPIO_PORTJ_AMSEL_R &= ~0x03;
    GPIO_PORTJ_PUR_R   |=  0x03;// pull-up, active LO
}
 
			// Port N: PN1 = measurement LED, PN0 = UART Tx LED
void PortN_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0) {}
    GPIO_PORTN_DIR_R   |=  0x03;
    GPIO_PORTN_AFSEL_R &= ~0x03;
    GPIO_PORTN_DEN_R   |=  0x03;
    GPIO_PORTN_AMSEL_R &= ~0x03;
    GPIO_PORTN_DATA_R  &= ~0x03;
}

			// Port F: PF4 is status LED (motor rotating), PF0 = e
void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {}
    GPIO_PORTF_DIR_R   |=  0x13;  // PF4, PF1, PF0 as output (was 0x11)
    GPIO_PORTF_AFSEL_R &= ~0x13;
    GPIO_PORTF_DEN_R   |=  0x13;
    GPIO_PORTF_AMSEL_R &= ~0x13;
    GPIO_PORTF_DATA_R  &= ~0x13;
}

			//Port G: XSHUT line for ToF reset (PG0)
void PortG_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0) {}
    GPIO_PORTG_DIR_R   &= 0x00; //output 
    GPIO_PORTG_AFSEL_R &= ~0x01; //low is reset 
    GPIO_PORTG_DEN_R   |=  0x01; //hi is release 
    GPIO_PORTG_AMSEL_R &= ~0x01;
}

			/////////I2C INIT FROM LAB 
void I2C_Init(void) {
    SYSCTL_RCGCI2C_R  |= SYSCTL_RCGCI2C_R0; //turn on clock  
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; //turn on port B 
    while ((SYSCTL_PRGPIO_R & 0x0002) == 0) {} //wait till B ready 
    GPIO_PORTB_AFSEL_R |= 0x0C; //PB2 and Pb3 
    GPIO_PORTB_ODR_R   |= 0x08; //PB3 SDA open drain 
    GPIO_PORTB_DEN_R   |= 0x0C;
    GPIO_PORTB_PCTL_R   = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200;
    I2C0_MCR_R  = I2C_MCR_MFE; //enable I2C master function  
    I2C0_MTPR_R = 0b0000000000000101000000000111011; // 100 kbps
}

		/////ToF XSHUT reset
void VL53L1X_XSHUT(void) {
    GPIO_PORTG_DIR_R  |=  0x01;
    GPIO_PORTG_DATA_R &= ~0x01;   // pull low
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R  &= ~0x01;   // release (HiZ , the sensor powers back up)
}


//////////////////////////////////////////////////////////////
////////////////////STEPPER MOTOR/////////////////////////////
//////////////////////////////////////////////////////////////

// 45 degrees = 256 full steps (2048 steps per 360 deg)
#define STEPS_PER_360DEG  2048

#define STEPS_PER_45DEG   256   // 2048/ 8 = 256

#define STEPS_PER_11DEG  64    // 2048/ 32 = 64


int stepIndex = 0; // variable to track index/current step in CW and CCW arrays 

//CW full-step sequence
const uint8_t CW_SEQ[4]  = {
	0b00000011, 0b00000110, 0b00001100, 0b00001001};

//CCW full-step sequence for untangling wires 
const uint8_t CCW_SEQ[4] = {
	0b00001001, 0b00001100, 0b00000110, 0b00000011};


// CW one step
void Step_CW(void) {
    GPIO_PORTH_DATA_R = CW_SEQ[stepIndex];
    stepIndex = (stepIndex + 1) % 4;
    SysTick_Wait10ms(1);
}

// CCW one step
void Step_CCW(void) {
    GPIO_PORTH_DATA_R = CCW_SEQ[stepIndex];
    stepIndex = (stepIndex + 1) % 4;  // reverse through index
    SysTick_Wait10ms(1);
}
////////////////////////////////////////////////////////////////
////////////////////BUTTONS ////////////////////////////////////
////////////////////////////////////////////////////////////////


//Button 1 (PJ0) functions 
//determine if PJ0 button pressed 
int PJ0_Pressed(void) {
    return ((GPIO_PORTJ_DATA_R & 0x01) == 0x00);
	
}
//debounce to ensure no false positives 
void WaitRelease_PJ0(void) {
    while (PJ0_Pressed()) {}
    SysTick_Wait10ms(2);
}

//Button 2 (PJ1) functions
//determine if PJ0 button pressed 
int PJ1_Pressed(void) {
    return ((GPIO_PORTJ_DATA_R & 0x02) == 0x00);
}
//debounce to ensure no false positives 

void WaitRelease_PJ1(void) {
    while (PJ1_Pressed()) {}
    SysTick_Wait10ms(2);
}

////////////////////////////////////////////////////////////////
////////////////////Tof Sensor//////////////////////////////////
////////////////////////////////////////////////////////////////
void ToF_Init(void) {
    uint8_t sensorState = 0;
    while (sensorState == 0) {
        status = VL53L1X_BootState(dev, &sensorState);
        SysTick_Wait10ms(10);
    }
    UART_printf("ToF Booted!\r\n");
    status = VL53L1X_ClearInterrupt(dev);
    status = VL53L1X_SensorInit(dev);
    Status_Check("SensorInit", status);
    status = VL53L1X_SetDistanceMode(dev, 2);
    status = VL53L1X_SetTimingBudgetInMs(dev, 100);
    status = VL53L1X_SetInterMeasurementInMs(dev, 200);
}

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
int main(void) {
	
	
		//initialization   
    PLL_Init();
    SysTick_Init();
    onboardLEDs_Init();
    I2C_Init();
    UART_Init();
    PortG_Init();
    PortH_Init();  //motor 
    PortJ_Init();
    PortN_Init();
    PortF_Init();
	
	
	////////////////////////////////
		// Bus speed test: toggle PF0 as fast as possible
		// At 32 MHz, each instruction ~31ns
	/*while(1) {
			GPIO_PORTF_DATA_R |=  0x02;  // high
			GPIO_PORTF_DATA_R &= ~0x02;  // low
	}*/
		
//////////////////////////
			//initialize ToF 
    VL53L1X_XSHUT();
    ToF_Init();
    int armed = 0;
    uint32_t displacement_mm = 0;

    UART_printf("Ready. PJ0 to arm, PJ1 to scan.\r\n");//send message through uart to communicate button functionality  
			
    while (1) {
			
			
        // PJ0: arm/disarm
        if ((GPIO_PORTJ_DATA_R & 0x01) == 0x00) { //if PJ0 pressed 
            armed = !armed; // toggle arm 
            if (armed) {
                GPIO_PORTN_DATA_R |=  0x02; // PN1 ON 
                UART_printf("ARMED\r\n"); //send uart mesaage 
            } 
						else {//btton pressed a second time so dissarm everythign 
                GPIO_PORTN_DATA_R &= ~0x02;   //PN1 OFF
                UART_printf("END\r\n");
                displacement_mm = 0;
            }
            while ((GPIO_PORTJ_DATA_R & 0x01) == 0x00) {}  // wait PJ0 release
            SysTick_Wait10ms(2);
        }
				
				

        // PJ1: one full scan
        if (armed && (GPIO_PORTJ_DATA_R & 0x02) == 0x00) {
            while ((GPIO_PORTJ_DATA_R & 0x02) == 0x00) {}  // wait release
            SysTick_Wait10ms(2); //debounce 

            // send scan header
            GPIO_PORTN_DATA_R |=  0x01;   // PN0 ON: UART Tx
            sprintf(printf_buffer, "SCAN,%lu\r\n", (unsigned long)displacement_mm);
            UART_printf(printf_buffer);
						SysTick_Wait10ms(2);
            GPIO_PORTN_DATA_R &= ~0x01;   // PN0 OFF

            status = VL53L1X_StartRanging(dev);

            // 32 measurements, 11.25 deg apart
            for (int i = 0; i < 32; i++) {
                //step 45 deg CW (256 steps), 2048/32 = 64 
                GPIO_PORTF_DATA_R |= 0x10;   // PF4 ON: motor moving
                for (int s = 0; s < 64 ; s++) {
                    Step_CW(); //one single step in 4 step sequence 
                }
								
                GPIO_PORTH_DATA_R  = 0x00;   // reste motor 
                GPIO_PORTF_DATA_R &= ~0x10;  // PF4 OFF
								
                //read ToF
                uint8_t  dataReady = 0;
                uint16_t Distance;
                while (dataReady == 0) {
                    status = VL53L1X_CheckForDataReady(dev, &dataReady); //is a new distance ready to read?
                    VL53L1_WaitMs(dev, 5);
                }
                status = VL53L1X_GetDistance(dev, &Distance);
                status = VL53L1X_ClearInterrupt(dev);

                // flash measurement LED
                GPIO_PORTN_DATA_R |=  0x02;  // PN1 flash
                SysTick_Wait10ms(1);
                GPIO_PORTN_DATA_R &= ~0x02;

                // send distance
                GPIO_PORTN_DATA_R |=  0x01;  // PN0 ON: UART Tx
                sprintf(printf_buffer, "%u\r\n", Distance);
                UART_printf(printf_buffer);
                GPIO_PORTN_DATA_R &= ~0x01;  // PN0 OFF
            }

            VL53L1X_StopRanging(dev);
						
						
						
						//prevent wire tangling
            //spin back CCW 360degrees /2048 steps
            GPIO_PORTF_DATA_R |= 0x10;   // PF4 ON
            for (int s = 0; s < 2048; s++) {
                Step_CCW();
            }
            GPIO_PORTH_DATA_R  = 0x00;
            GPIO_PORTF_DATA_R &= ~0x10;  // PF4 OFF

            displacement_mm += 100;  // next scan 10 cm further
        }
    }
}
 
