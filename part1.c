// Documentation
// CECS346 Project 3 Part 1 - Drag Race
// Description: Design a system based on a drag race. A 'Christmas Tree' is the light signal that drives us to
//              determine when to accelerate (ie start the race), which is simulated by our LEDs. A track has
//              multiple sensors to determine if a car is in the proper position ("staged"), simulated by pressing
//              our "lane sensor" buttons and IR sensor, one per lane (left lane and right lane).
// Student: Sean Masterson

// Input/Output:
//   PB7 - Yellow LED 1 right
//   PB6 - Yellow LED 2 right
//   PB5 - Green LED right
//   PB4 - Red LED right
//   PB3 - Yellow LED 1 left
//   PB2 - Yellow LED 2 left
//   PB1 - Green LED left
//   PB0 - Red LED left
//   PA3 - Reset input
//   PF4 - IR sensor (also connected to on-board switch 1)
//   PF0 - Switch on breadboard (also connected to on-board switch 2 )

// Preprocessor Directives
#include <stdint.h>
#include "tm4c123gh6pm.h"

// Function Prototypes (from startup.s)
void DisableInterrupts(); // Disable interrupts
void EnableInterrupts();  // Enable interrupts
void WaitForInterrupt();  // Go to low power mode while waiting for the next interrupt

// Function Prototypes
void EdgeCounter_InitF();  // Initialize edge trigger interrupt for PF4+0 (SW1+2) both edges
void EdgeCounter_InitA();  // Initialize edge trigger interrupt for PA3 level
void Switch_InitA(void);
void Switch_InitB(void);
void Switch_InitF(void);
void SysTick_Init();      // Initialize SysTick timer for 0.1s delay with interrupt enabled

void GPIOPortF_Handler(); // Handle GPIO Port F interrupts
void GPIOPortA_Handler(); // Handle GPIO Port A interrupts
void SysTick_Handler();   // Handle SysTick generated interrupts
	
#define LIGHT_PORTB	(*((volatile unsigned long *)0x400053FC)) // PB01234567
	
#define SENSOR_PORTA	(*((volatile unsigned long *)0x40004020)) // PA3
#define SENSOR_PORTF	(*((volatile unsigned long *)0x40025044)) // PF04

struct State {
	uint32_t Out;
	uint32_t Time; // 1 ms units
	uint32_t Next[4]; // list of next states
};
typedef const struct State STyp;

#define initialize      0
#define waitForStaging  1
#define countdownY1  		2
#define countdownY2 		3
#define go   						4
#define winLeft 				5
#define winRight 				6
#define winBoth 				7
#define falseStartLeft 	8
#define falseStartRight 9
#define falseStartBoth 	10

STyp FSM [11] = {
	{0xFF, 1000, {waitForStaging, waitForStaging, waitForStaging, waitForStaging}},	// initialize
	{0x00, 500, {waitForStaging, waitForStaging, waitForStaging, countdownY1}},	    // waitForStaging
	{0x88, 500, {falseStartBoth, falseStartLeft, falseStartRight, countdownY2}},		// countdownY1
	{0x44, 500, {falseStartBoth, falseStartLeft, falseStartRight, go}},							// countdownY2
	{0x22, 50, {winBoth, winLeft, winRight, go}},																		// go
	{0x20, 1000, {waitForStaging, waitForStaging, waitForStaging, waitForStaging}},	// winLeft 				
	{0x02, 1000, {waitForStaging, waitForStaging, waitForStaging, waitForStaging}},	// winRight
	{0x22, 1000, {waitForStaging, waitForStaging, waitForStaging, waitForStaging}},	// winBoth
	{0x10, 1000, {waitForStaging, waitForStaging, waitForStaging, waitForStaging}},	// falseStartLeft
	{0x01, 1000, {waitForStaging, waitForStaging, waitForStaging, waitForStaging}},	// falseStartRight
	{0x11, 1000, {waitForStaging, waitForStaging, waitForStaging, waitForStaging}}	// falseStartBoth
};

uint8_t S;     // index of current state
uint8_t Input; // input obtained from switches

int main(void) {
	SysTick_Init(); // initialize SysTick timer
	
	EdgeCounter_InitF();
	EdgeCounter_InitA();
	
	// Initialize GPIO on Ports A, B, F
	
	// Port A Init
	Switch_InitA();
	
	// Port B Init
	Switch_InitB();
	
	// Port F Init
	Switch_InitF();
	
	// Initial state
	S = initialize;
	LIGHT_PORTB = ((~FSM[S].Out & 0xF0)) + ((FSM[S].Out & 0x0F)); // set lights port b to initial state
	
	while(1) {
		WaitForInterrupt();
	}
}

// Subroutine to wait about 0.1 sec
// Inputs: None
// Outputs: None
// Notes: the Keil simulation runs slower than the real board

#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Counter load value

void Switch_InitA(void){ // input
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x01;      // 1) activate clock for Port A
	delay = SYSCTL_RCGC2_R;      // allow time for clock to start
															 // 2) no need to unlock GPIO Port A
	GPIO_PORTA_DIR_R &= ~0x08;   // 3) input on PA3
	GPIO_PORTA_AFSEL_R &= ~0x08; // 4) PCTL GPIO on PA3
	GPIO_PORTA_DEN_R |= 0x08;    // 5) enable PA3
}

void Switch_InitB(void){ // output
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x02;      // 1) activate clock for Port B
	delay = SYSCTL_RCGC2_R;      // allow time for clock to start
															 // 2) no need to unlock GPIO Port B
	GPIO_PORTB_DIR_R |= 0xFF;    // 3) output PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7
	GPIO_PORTB_AFSEL_R &= ~0xFF; // 4) PCTL GPIO on PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7
	GPIO_PORTB_DEN_R |= 0xFF;    // 5) enable PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7
}

void Switch_InitF(void){ // input
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x20;           // 1) activate clock for Port F
	delay = SYSCTL_RCGC2_R;           // allow time for clock to start
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF (for PF0) 
  GPIO_PORTF_CR_R |= 0x01;          //    allow changes to PF0 (PF4-1 unlocked by default)
	GPIO_PORTF_DIR_R &= ~0x11;        // 3) input on PF4, PF0
	GPIO_PORTF_AFSEL_R &= ~0x11;      // 4) PCTL GPIO on PF4, PF0
	GPIO_PORTF_PUR_R |= 0x11;         // 4.5) Enable pull up resistor
	GPIO_PORTF_DEN_R |= 0x11;        // 5) enable PF4, PF0
}

// Initialize SysTick timer for 0.1s delay with interrupt enabled
void SysTick_Init() {
	NVIC_ST_CTRL_R = 0;
	NVIC_ST_RELOAD_R = 16000000 - 1; // 1 s / (1 / 16 MHz)
	NVIC_ST_CURRENT_R = 0;
	// priority 3
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x60000000;
	// enable SysTick with core clock and interrupts
	NVIC_ST_CTRL_R |= 0x07; // 0111
	EnableInterrupts();
}


// Initialize edge trigger interrupt for PF4, PF0 (SW1+2) both edges
void EdgeCounter_InitF() { // must be friendly
	SYSCTL_RCGC2_R |= 0x20; 																// (a) actiate clock for port F
	GPIO_PORTF_DIR_R &= ~0x11; 															// (c) make PF4, PF0 in (built-in button)
	GPIO_PORTF_AFSEL_R &= ~0x11; 														// disable alt funct on PF4, PF0
	GPIO_PORTF_DEN_R |= 0x11; 															// enable digital I/O on PF4, 0
	GPIO_PORTF_PCTL_R &= ~0x000F000F;												// configure PF4, 0 as GPIO
	GPIO_PORTF_AMSEL_R &= ~0x11; 													  // disable analog functionality on PF4, 0
	GPIO_PORTF_PUR_R |= 0x11; 															// enable weak pull-up on PF4, 0
	GPIO_PORTF_IS_R &= ~0x11; 															// (d) PF4, 0 is edge-sensitive
	GPIO_PORTF_IBE_R |= 0x11; 															// PF4, 0 is both edges
	//GPIO_PORTF_IEV_R |= 0x11; 														// PF4, 0 rising edge event
	GPIO_PORTF_ICR_R = 0x11; 																// (e) clear flag0, 4
	GPIO_PORTF_IM_R |= 0x11; 																// (f) arm interrupt on PF0, 4
	NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00400000;	// (g) priority 2
	NVIC_EN0_R = 0x40000000;																// (h) enable interrupt 30 in NVIC
	EnableInterrupts(); 																		// (i) Clears the I bit
}

// Initialize level trigger interrupt for PA3
void EdgeCounter_InitA() { // must be friendly
	SYSCTL_RCGC2_R |= 0x01; 																// (a) actiate clock for port A
	GPIO_PORTA_DIR_R &= ~0x08; 															// (c) make PA3 input
	GPIO_PORTA_AFSEL_R &= ~0x08; 														// disable alt funct on PA3
	GPIO_PORTA_DEN_R |= 0x08; 															// enable digital I/O on PA3
	GPIO_PORTA_PCTL_R &= ~0x0000F000;												// configure PF3 as GPIO
	GPIO_PORTA_AMSEL_R &= ~0x08; 													  // disable analog functionality on PA3
	GPIO_PORTA_IS_R |= 0x08; 															  // (d) PA3 is level-sensitive
	GPIO_PORTA_IBE_R &= ~0x08; 														  // PA3 is not both edges
	GPIO_PORTA_IEV_R |= 0x08; 															// PA3 level high
	GPIO_PORTA_ICR_R = 0x08; 																// (e) clear flag3
	GPIO_PORTA_IM_R |= 0x08; 																// (f) arm interrupt on PA3
	NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFFFF00) | 0x00000020;	// (g) priority 1
	NVIC_EN0_R = 0x00000001;																// (h) enable interrupt 0 in NVIC
	EnableInterrupts(); 																		// (i) Clears the I bit
}

// Handle GPIO Port F interrupts. When Port F interrupt triggers, go to next state
void GPIOPortF_Handler() {
	GPIO_PORTF_ICR_R = 0x11; // Clear flag for port F pins 0 and 4.
	Input = ((~SENSOR_PORTF & 0x10 ) >> 3) + (~SENSOR_PORTF & 0x01); // read sensors and shift bits
}

// Handle GPIO Port A interrupts. When Port A interrupt triggers, reset
void GPIOPortA_Handler() {
	GPIO_PORTA_ICR_R = 0x08; // Clear flag for port A pin 3.
	S = initialize;
	LIGHT_PORTB = 0x0F; // set lights port b. Note pins 7-4 are negative logic and 3-0 positive.
	SysTick_Init(); // Reset timer value to that of initial state
}

// Handle SysTick generated interrupts. When timer interrupt triggers, do what's necessary
void SysTick_Handler() {
	S = FSM[S].Next[Input];
	LIGHT_PORTB = ((~FSM[S].Out & 0xF0)) + ((FSM[S].Out & 0x0F)); // set lights port b
	// Reload timer with new value
	NVIC_ST_CTRL_R = 0; // stop timer
	NVIC_ST_RELOAD_R = (16000 * FSM[S].Time) - 1; // Wait FSM[S].Time milliseconds (16000 ticks = 1 ms)
	NVIC_ST_CURRENT_R = 0; // delete count value
	NVIC_ST_CTRL_R |= 0x07; // 0111, start timer
}
