// Documentation
// CECS346 Project 3 Part 2 - Car
// Description: Connect two stepper motors to TM4C.
// Student: Sean Masterson

// Input/Output:
//   PA3 - IR sensor
//   PB7 - Right stepper motor IN4
//   PB6 - Right stepper motor IN3
//   PB5 - Right stepper motor IN2
//   PB4 - Right stepper motor IN1
//   PB3 - Left stepper motor IN4
//   PB2 - Left stepper motor IN3
//   PB1 - Left stepper motor IN2
//   PB0 - Left stepper motor IN1
//   PF4 - On-board button 1
//   PF0 - On-board button 2

// Preprocessor Directives
#include <stdint.h>
#include "tm4c123gh6pm.h"

// Function Prototypes (from startup.s)
void DisableInterrupts(); // Disable interrupts
void EnableInterrupts();  // Enable interrupts
void WaitForInterrupt();  // Go to low power mode while waiting for the next interrupt

// Function Prototypes
void EdgeCounter_InitA();  // Initialize edge trigger interrupt for PA3 level
void EdgeCounter_InitF();  // Initialize edge trigger interrupt for PF4+0 (SW1+2) both falling edge

void Switch_InitA(void);
void Switch_InitB(void);
void Switch_InitF(void);
void SysTick_Init();      // Initialize SysTick timer for 1 ms delay with interrupt enabled

void GPIOPortA_Handler(); // Handle GPIO Port A interrupts
void GPIOPortF_Handler(); // Handle GPIO Port F interrupts
void SysTick_Handler();   // Handle SysTick generated interrupts

#define SENSOR_PORTA	(*((volatile unsigned long *)0x40004020)) // PA3
#define PORTB					(*((volatile unsigned long *)0x400053FC)) // PB01234567
#define SENSOR_PORTF	(*((volatile unsigned long *)0x40025044)) // PF04

struct State {
	uint32_t Out;     // output
	uint32_t Time;    // 1 ms units
	uint32_t Next[8]; // list of next states
};
typedef const struct State STyp;

#define phase1 0
#define phase2 1
#define phase3 2
#define phase4 3

// Input == move, cw, ccw

STyp FSM [4] = {
	// Since the left stepper is facing the opposite direction as the right stepper motor, they need to turn in opposite directions.
	{0x33, 5, {phase1, phase1, phase1, phase1, phase1, phase4, phase2, phase1}}, // phase1
	{0x96, 5, {phase2, phase2, phase2, phase2, phase2, phase1, phase3, phase2}}, // phase2
	{0xCC, 5, {phase3, phase3, phase3, phase3, phase3, phase2, phase4, phase3}}, // phase3
	{0x69, 5, {phase4, phase4, phase4, phase4, phase4, phase3, phase1, phase4}}, // phase4
};

uint8_t S;               // index of current state
uint8_t Input;           // input obtained from switches
uint32_t stepsRemaining; // steps remaining

int main(void) {
	SysTick_Init(); // initialize SysTick timer
	
	EdgeCounter_InitA();
	EdgeCounter_InitF();
	
	// Initialize GPIO on Ports A, B, F
	// Port A Init
	Switch_InitA();
	
	// Port B Init
	Switch_InitB();
	
	// Port F Init
	Switch_InitF();
	
	// Initial state
	S = phase1;
	
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

// Initialize SysTick timer for 1 ms delay with interrupt enabled
void SysTick_Init() {
	NVIC_ST_CTRL_R = 0;
	NVIC_ST_RELOAD_R = 16000 - 1; // 1 s / (1 / 16 MHz)
	NVIC_ST_CURRENT_R = 0;
	// priority 3
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x60000000;
	// enable SysTick with core clock and interrupts
	NVIC_ST_CTRL_R |= 0x07; // 0111
	EnableInterrupts();
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
	GPIO_PORTA_IEV_R &= ~0x08; 															// PA3 level low
	GPIO_PORTA_ICR_R = 0x08; 																// (e) clear flag3
	GPIO_PORTA_IM_R |= 0x08; 																// (f) arm interrupt on PA3
	NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFFFF00) | 0x00000020;	// (g) priority 1
	NVIC_EN0_R = 0x00000001;																// (h) enable interrupt 0 in NVIC
	EnableInterrupts(); 																		// (i) Clears the I bit
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
	GPIO_PORTF_IS_R &= ~0x11; 															  // (d) PF4, 0 is edge-sensitive
	GPIO_PORTF_IBE_R &= ~0x11; 															// PF4, 0 is NOT both edges
	GPIO_PORTF_IEV_R &= ~0x11; 														  // PF4, 0 falling edge event
	GPIO_PORTF_ICR_R = 0x11; 																// (e) clear flag0, 4
	GPIO_PORTF_IM_R |= 0x11; 																// (f) arm interrupt on PF0, 4
	NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00400000;	// (g) priority 2
	NVIC_EN0_R = 0x40000000;																// (h) enable interrupt 30 in NVIC
	EnableInterrupts(); 																		// (i) Clears the I bit
}

// Handle GPIO Port A interrupts. When Port A interrupt triggers, reset
void GPIOPortA_Handler() {
	GPIO_PORTA_ICR_R = 0x08; // Clear flag for port A pin 3.
	Input &= ~0x04; // move disable
}

// Handle GPIO Port F interrupts. When Port F interrupt triggers, go to next state
void GPIOPortF_Handler() {
	GPIO_PORTF_ICR_R = 0x11; // Clear flag for port F pins 0 and 4.
	Input |= 0x04; // move enable
	
	if ((~SENSOR_PORTF & 0x01) == 0x01) { // PF0 (SW2) pressed
		Input &= ~0x01;
		Input |= 0x02;
		stepsRemaining = 8 * 64; // 64:1 gear ratio
	}
	else if (((~SENSOR_PORTF & 0x10 ) >> 3) == 0x02) { // PF4 (SW1) pressed
		Input &= ~0x02;
		Input |= 0x01;
		stepsRemaining = 143 * 64; // 64:1 gear ratio
	}
	else {
		Input &= ~0x03;
		stepsRemaining = 0;
	}
}

// Handle SysTick generated interrupts. When timer interrupt triggers, do what's necessary
void SysTick_Handler() {
	
	if (stepsRemaining > 0) {
		S = FSM[S].Next[Input];
		PORTB = FSM[S].Out; // set stepper motors on port B
		--stepsRemaining;
	}
	
	// Reload timer with new value
	NVIC_ST_CTRL_R = 0; // stop timer
	NVIC_ST_RELOAD_R = (16000 * FSM[S].Time) - 1; // Wait FSM[S].Time milliseconds (16000 ticks = 1 ms)
	NVIC_ST_CURRENT_R = 0; // delete count value
	NVIC_ST_CTRL_R |= 0x07; // 0111, start timer
	
}
