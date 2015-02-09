/**
*
* @file p2.c
*
*
* @author Mark Ronay (ronay@pdx.edu)
*
* @copyright Portland State University, 2014, 2015
*
* This program detect a generated pwm signal with both software and hardware.   The hardware for PWM is done with
* a Xilinx Timer/Counter module set in PWM mode.   The PWM library builds on the Timer/Counter drivers
* provided by Xilinx and encapsulates common PWM functions.  The program also provides a working example
* of how to use the xps_s3eif driver to control the buttons, switches, rotary encoder, and display.
*
* The program uses the rotary encoder and switches to choose a PWM frequency and duty cycle.  The selected
* frequency and duty cycle are displayed on line 1 of the LCD. The detected signals are displayed on line 2.
* The detection mode is given by RBDled 1, which is red for hardware, blue for software. The program also illustrates
* the use of a Xilinx fixed interval timer module to generate a periodic interrupt for handling time-based
*  (maybe) and/or sampled inputs/outputs
*
* NOTE: This program used code extensively from Roy Kravitz's pwm_test.c file
* 	 				 
* <pre>
* MODIFICATION HISTORY:
*
* Ver 1  Who Mark Ronay Date 1/29/2015    
* ----- ---- -------- -----------------------------------------------

*
* @note
* The minimal hardware configuration for this program is a Microblaze-based system with at least 32KB of memory,
* an instance of Nexys4IO, an instance of the PMod544IOR2, an instance of an axi_timer, an instance of an axi_gpio
* and an instance of an axi_uartlite (used for xil_printf() console output)
*
******************************************************************************/

/************************ Include Files **************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "xparameters.h"
#include "xintc.h"
#include "xtmrctr.h"
#include "xgpio.h"
#include "mb_interface.h"
#include "platform.h"
#include "Nexys4IO.h"
#include "PMod544IOR2.h"
#include "pwm_tmrctr.h"


/************************** Constant Definitions ****************************/

// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// PWM and pulse detect timer parameters
#define PWM_TIMER_DEVICE_ID		XPAR_TMRCTR_0_DEVICE_ID

// Nexys4IO parameters
#define NX4IO_DEVICE_ID			XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR			XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR			XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Pmod544IO parameters
#define PMDIO_DEVICE_ID			XPAR_PMOD544IOR2_0_DEVICE_ID
#define PMDIO_BASEADDR			XPAR_PMOD544IOR2_0_S00_AXI_BASEADDR
#define PMDIO_HIGHADDR			XPAR_PMOD544IOR2_0_S00_AXI_HIGHADDR

// GPIO parameters
#define GPIO_0_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_1_DEVICE_ID			XPAR_AXI_GPIO_1_DEVICE_ID
//#define GPIO_INPUT_CHANNEL		1
//#define GPIO_OUTPUT_CHANNEL		2

// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR
#define PWM_TIMER_INTERRUPT_ID	XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_0_INTERRUPT_INTR

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		40000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			40	

// PWM selected frequencies in Hz
#define PWM_FREQ_10HZ			10
#define PWM_FREQ_100HZ			100
#define PWM_FREQ_1KHZ			1000
#define PWM_FREQ_5KHZ			5000
#define PWM_FREQ_10KHZ			10000
#define PWM_FREQ_50KHZ			50000
#define PWM_FREQ_100KHZ			100000
#define PWM_FREQ_200KHZ			200000
#define PWM_FREQ_500KHZ			500000
#define PWM_FREQ_800KHZ			800000

#define INITIAL_FREQUENCY		PWM_FREQ_1KHZ
#define INITIAL_DUTY_CYCLE		50
#define DUTY_CYCLE_CHANGE		5

#define	PWM_SIGNAL_MSK			0x01
#define CLKFIT_MSK				0x01
#define PWM_FREQ_MSK			0x23
#define MODE_SELECT_MASK		0x08
#define PWM_DUTY_MSK			0xFF

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/
#define MIN(a, b)  ( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  ( ((a) >= (b)) ? (a) : (b) )

/************************** Variable Definitions ****************************/	
// Microblaze peripheral instances
XIntc 	IntrptCtlrInst;						// Interrupt Controller instance
XTmrCtr	PWMTimerInst;						// PWM timer instance
XGpio	GPIOInst0;							// GPIO instance
XGpio	GPIOInst1;							// GPIO instance
// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler
// "clkfit" toggles each time the FIT interrupt handler is called so its frequency will
// be 1/2 FIT_CLOCK_FREQ_HZ.  timestamp increments every 1msec and is used in delay_msecs()
volatile unsigned int	clkfit;				// clock signal is bit[0] (rightmost) of gpio 0 output port									
volatile unsigned long	timestamp;			// timestamp since the program began

volatile u32			gpio_in;			// GPIO input port
volatile u32 			hardware_detector_high;
volatile u32 			hardware_detector_low;
volatile u32 			software_detector_high; // im not positive i need to declare these here, could probably just declare
												// them localy to the function
volatile u32 			software_detector_low;
// The following variables are shared between the functions in the program
// such that they must be global
int						pwm_freq;			// PWM frequency
int						pwm_duty;			// PWM duty cycle
int						high_count;
int						low_count;
int						total_count;

int						software_pwm_up_count;
int						software_pwm_down_count;
volatile unsigned int software_up_counter;
volatile unsigned int software_down_counter;
bool					new_perduty;		// new period/duty cycle flag
bool					mode_select;








//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
						--Below are the constants, variables, functions and routines in main for the
												pushbutton functional specification --
*/

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



#define	P				0
#define I				1
#define	D				2
#define OFFSET			3

 
bool					push_button_center;		// flag to go off if center push button is pressed for selecting gains on PID
int						push_button_up;			// increments from 0 to 3, each of which selects either P I or D or OFFSET

int						push_button_left;		//decrements gain value for selected P I or D
int						push_button_right;		//increments gain value for selected P I or D
int						PID_variable_value;				//a catch all value for any of the PID variables, will be divided by 100 for gain values
int						selected_PID;
volatile int			P_GAIN;
volatile int			I_GAIN;
volatile int			D_GAIN;
volatile int			OFFSET_VALUE;

//in initialize

push_button_center = false;

//in main function



if (NX4IO_isPressed(BTNC))
{
	push_button_center = ~push_button_center;
}

if(push_button_center)

{
	PMDIO_LCD_wrstring("PID PARAM SELECT");	//write PID PARAM SELECT on LCD
	PID_PARAM_SELECT();	//Call PID SELECT FUNCTION
}


//********************************************************

void PID_PARAM_SELECT(void)
{
	//format LCD here
											//when a test is not running P, I, D, OFFSET, SET POINT should be formatted on the LCD with the cursor indicating which is to be changed
	if (NX4IO_isPressed(BTNC))
	{
		push_button_center = ~push_button_center;
		return;
		}
	else if (NX4IO_isPressed(BTNU))
		push_button_up++;
		{
			if (push_button_up >3)
				push_button_up = 0;
				}
		}
	else if (NX4IO_isPressed(BTND))
		push_button_up--;
		{
			if (push_button_up >3 | <0)  //this is kind of clunky but maybe functional? decrementing this isnt in the design specs anyway
				push_button_up = 3; 
				}
		}
		
		
	else if (NX4IO_isPressed(BTNR))
		PID_variable_value++;
		{
			if (PID_variable_value >1000)
				PID_variable_value = 0;
				}
		}	
		
	else if (NX4IO_isPressed(BTNL))
		PID_variable_value--;
		{
			if (PID_variable_value >1000 | <0)
				PID_variable_value = 1000;
				}
		}	
		
		
		
	switch (push_button_up)
		{
			case 0x00:	selected_PID = P;	break;
			case 0x01:	selected_PID = I;	break;
			case 0x02:	selected_PID = D;	break;
			case 0x03:	selected_PID = OFFSET;	break;
		}
		
	if (selected_PID = P)
	{
	//set cursor to P
	P_GAIN = PID_variable_value/100
	}
	
	if (selected_PID = I)
	{
	//set cursor to I
	I_GAIN = PID_variable_value/100
	}
	
	if (selected_PID = D)
	{
	D_GAIN = PID_variable_value/100
	//set cursor to D
	}
	
	if (selected_PID = OFFSET)
	{
	OFFSET_VALUE = PID_variable_value
	//set cursor to OFFSET
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*
						--Here ends the pushbutton functional spec code
						
						
*/
//********************************************************










//hardware mode is 0, software mode is 1
volatile unsigned int		dutycycle; //i dont know why one of these is a u32 and the other an int, but it works fine
volatile u32 		freq;

				
/*---------------------------------------------------------------------------*/					
int						debugen = 0;		// debug level/flag
/*---------------------------------------------------------------------------*/
		
/*****************************************************************************/	
	

/************************** Function Prototypes ******************************/
int				do_init(void);											// initialize system
void			delay_msecs(unsigned int msecs);						// busy-wait delay for "msecs" miliseconds
void			voltstostrng(float v, char* s);							// converts volts to a string
void			update_lcd(int freq, int dutycyle, u32 linenum);		// update LCD display
				
void			FIT_Handler(void);										// fixed interval timer interrupt handler


/************************** MAIN PROGRAM ************************************/
int main()
{
	XStatus 	status;
	u16			sw, oldSw =0xFFFF;		// 0xFFFF is invalid - makes sure the PWM freq is updated the first time
	u16			sw_freq, sw_detect_mode;
	int			rotcnt, oldRotcnt = 0x1000;	
	bool		done = false;
	
	init_platform();

	// initialize devices and set up interrupts, etc.
 	status = do_init();
 	if (status != XST_SUCCESS)
 	{
 		PMDIO_LCD_setcursor(1,0);
 		PMDIO_LCD_wrstring("****** ERROR *******");
 		PMDIO_LCD_setcursor(2,0);
 		PMDIO_LCD_wrstring("INIT FAILED- EXITING");
 		exit(XST_FAILURE);
 	}
 	
	// initialize the global variables
	timestamp = 0;							
	pwm_freq = INITIAL_FREQUENCY;
	pwm_duty = INITIAL_DUTY_CYCLE;
	clkfit = 0;
	new_perduty = false;
    mode_select = true;
	// start the PWM timer and kick of the processing by enabling the Microblaze interrupt
	PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);	
	PWM_Start(&PWMTimerInst);
    microblaze_enable_interrupts();
    
	// display the greeting   
    PMDIO_LCD_setcursor(1,0);
    PMDIO_LCD_wrstring("544: Project 1");
	PMDIO_LCD_setcursor(2,0);
	PMDIO_LCD_wrstring(" by Mark Ronay ");
	NX4IO_setLEDs(0x0000FFFF);
	delay_msecs(2000);
	NX4IO_setLEDs(0x00000000);
		
   // write the static text to the display
    PMDIO_LCD_clrd();
    PMDIO_LCD_setcursor(2,0);
    PMDIO_LCD_wrstring("DFR:      DC:  %");
    PMDIO_LCD_setcursor(1,0);
    PMDIO_LCD_wrstring("GFR:      DC:  %");

    // turn off the LEDs and clear the seven segment display
    NX4IO_setLEDs(0x00000000);
    NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
    NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
      
    // main loop
	do
	{ 
		// check rotary encoder pushbutton to see if it's time to quit
		if (PMDIO_ROT_isBtnPressed())
		{
			done = true;
		}
		else
		{
			new_perduty = false;
			// get the switches and mask out all but the switches that determine the PWM timer frequency
			sw = NX4IO_getSwitches();
			sw_freq = NX4IO_getSwitches();
			sw_freq &= PWM_FREQ_MSK;

			// get the switches and mask out all but the switches that determine the detection mode
			sw_detect_mode = NX4IO_getSwitches();
			sw_detect_mode &= MODE_SELECT_MASK;

			//updates the lcd with selected and detected frequency and duty cycle
			update_lcd(freq, dutycycle, 2);
			update_lcd(pwm_freq, pwm_duty, 1);
			if (sw != oldSw)
			{





				switch (sw_freq)
				{
					case 0x00:	pwm_freq = PWM_FREQ_100HZ;	break;
					case 0x01:	pwm_freq = PWM_FREQ_1KHZ;	break;
					case 0x02:	pwm_freq = PWM_FREQ_5KHZ;	break;
					case 0x03:	pwm_freq = PWM_FREQ_10KHZ;	break;
					case 0x04:	pwm_freq = PWM_FREQ_10KHZ;	break;
					case 0x05:	pwm_freq = PWM_FREQ_100KHZ;	break;
					case 0x06:	pwm_freq = PWM_FREQ_200KHZ;	break;
					//extra cases to make up for a physically broken switch 2 on my board
					case 0x16:	pwm_freq = PWM_FREQ_5KHZ;	break;
					case 0x17:	pwm_freq = PWM_FREQ_100KHZ;	break;

					case 0x23:	pwm_freq = PWM_FREQ_500KHZ;	break;

				}
				if (sw_detect_mode ==0x08)
				{
					mode_select = true;
				}

				else
				{
					mode_select = false;
				}

				oldSw = sw;
				new_perduty = true;

			}
		
			// read rotary count and handle duty cycle changes
			// limit duty cycle to 0% to 99%
			PMDIO_ROT_readRotcnt(&rotcnt);
			if (rotcnt != oldRotcnt)
			{
				// show the rotary count in hex on the seven segment display
				NX4IO_SSEG_putU16Hex(SSEGLO, rotcnt);

				// change the duty cycle
				pwm_duty = MAX(0, MIN(rotcnt, 99));
				oldRotcnt = rotcnt;
				new_perduty = true;
			}

			// update generated frequency and duty cycle	
			if (new_perduty)
			{

			
				// set the new PWM parameters - PWM_SetParams stops the timer
				status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
				if (status == XST_SUCCESS)
				{


					PWM_Start(&PWMTimerInst);
				}
			}
		}
	} while (!done);
	
	// wait until rotary encoder button is released		
	do
	{
		delay_msecs(10);
	} while (PMDIO_ROT_isBtnPressed());

	// we're done,  say goodbye
	xil_printf("\nThat's All Folks!\n\n");
	PMDIO_LCD_wrstring("Thx its been fun!");
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
	delay_msecs(5000);
	PMDIO_LCD_clrd();
	cleanup_platform();
	exit(0);
 }


/**************************** HELPER FUNCTIONS ******************************/
		
/****************************************************************************/
/**
* initialize the system
* 
* This function is executed once at start-up and after resets.  It initializes
* the peripherals and registers the interrupt handler(s)
*****************************************************************************/
int do_init(void)
{
	int status;				// status from Xilinx Lib calls
	
	// initialize the Nexys4IO and Pmod544IO hardware and drivers
	// rotary encoder is set to increment from 0 by DUTY_CYCLE_CHANGE 
 	status = NX4IO_initialize(NX4IO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	
	status = PMDIO_initialize(PMDIO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	
	// successful initialization.  Set the rotary encoder
	// to increment from 0 by DUTY_CYCLE_CHANGE counts per
	// rotation.
 	PMDIO_ROT_init(DUTY_CYCLE_CHANGE, true);
	PMDIO_ROT_clear();
	
	
	// initialize the first GPIO instance
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO channel 1 is an 8-bit input port.  bit[7:1] = reserved, bit[0] = PWM output (for duty cycle calculation)
	// GPIO channel 2 is an 8-bit output port.  bit[7:1] = reserved, bit[0] = FIT clock
	XGpio_SetDataDirection(&GPIOInst0, 2, 0xFE);


	// initialize the second GPIO instance
		status = XGpio_Initialize(&GPIOInst1, GPIO_1_DEVICE_ID);
		if (status != XST_SUCCESS)
		{
			return XST_FAILURE;
		}
		//both channels should default as input.



	// initialize the PWM timer/counter instance but do not start it
	// do not enable PWM interrupts.  Clock frequency is the AXI clock frequency
	status = PWM_Initialize(&PWMTimerInst, PWM_TIMER_DEVICE_ID, false, AXI_CLOCK_FREQ_HZ);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	
	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
    if (status != XST_SUCCESS)
    {
       return XST_FAILURE;
    }

	// connect the fixed interval timer (FIT) handler to the interrupt
    status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
                           (XInterruptHandler)FIT_Handler,
                           (void *)0);
    if (status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }
 
	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
    status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
    if (status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

	// enable the FIT interrupt
    XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);

    // set the duty cycles for RGB1.  The channels will be enabled/disabled
    // in the FIT interrupt handler.  Red and Blue make purple
    NX4IO_RGBLED_setDutyCycle(RGB1, 64, 64, 64);
    NX4IO_RGBLED_setChnlEn(RGB1, false, false, false);
//add the second led to look at hardware signal
    NX4IO_RGBLED_setDutyCycle(RGB2, 64, 64, 64);
    NX4IO_RGBLED_setChnlEn(RGB2, false, false, false);
	return XST_SUCCESS;
}
		

/****************************************************************************/
/**
* delay execution for "n" msecs
* 
* Uses a busy-wait loop to delay execution.  Timing is approximate but we're not 
*  looking for precision here, just a uniform delay function.  The function uses the 
*  global "timestamp" which is incremented every msec by FIT_Handler().
*
* @note
* Assumes that this loop is running faster than the fit_interval ISR 
*
* @note
* If your program seems to hang it could be because the function never returns
* Possible causes for this are almost certainly related to the FIT timer.  Check
* your connections...is the timer clocked?  is it stuck in reset?  is the interrupt 
* output connected? You would not be the first student to face this...not by a longshot 
*****************************************************************************/
void delay_msecs(unsigned int msecs)
{
	unsigned long target;

	if ( msecs == 0 )
	{
		return;
	}
	target = timestamp + msecs;
	while ( timestamp != target )
	{
		// spin until delay is over
	}
}

/****************************************************************************/
/**
* converts volts to a fixed format string
* 
* accepts an float voltage reading and turns it into a 5 character string
* of the following format:
*		(+/-)x.yy 
* where (+/-) is the sign, x is the integer part of the voltage and yy is
* the decimal part of the voltage.
*
* @param	v is the voltage to convert
*
* @param	s is a pointer to the buffer receiving the converted string
*	
* @note
* Assumes that s points to an array of at least 6 bytes. No error checking is done	
* This is deprecated in the current program, but left in because it could be useful in the future
*****************************************************************************/
 void	voltstostrng(float v, char* s)
 {
	float	dpf, ipf;
	u32		dpi;	
	u32		ones, tenths, hundredths;

	 // form the fixed digits 
	 dpf = modff(v, &ipf);
	 dpi = dpf * 100;
	 ones = abs(ipf) + '0';
	 tenths = (dpi / 10) + '0';
	 hundredths = (dpi - ((tenths - '0') * 10)) + '0';
	 
	 // form the string and return
	 if (ipf == 0 && dpf == 0)
	 {
	 	*s++ = ' ';
	 }
	 else	
	 { 
	 	*s++ = ipf >= 0 ? '+' : '-';
	 }
	 *s++ = (char) ones;
	 *s++ = '.';
	 *s++ = (char) tenths;
	 *s++ = (char) hundredths;
	 *s   = 0;
	 return;
 };
 
 
/****************************************************************************/
/**
 * update the frequency/duty cycle LCD display
 * 
 * writes the frequency and duty cycle to the specified line.  Assumes the
 * static portion of the display is already written and the format of each
 * line of the display is the same.
 *
 * @param	freq is the  PWM frequency to be displayed
 *
 * @param	dutycycle is the PM duty cycle to be displayed
 *
 * @param	linenum is the line (1 or 2) in the display to update
 *****************************************************************************/
void update_lcd(int freq, int dutycycle, u32 linenum)
{
	PMDIO_LCD_setcursor(linenum, 4);
	PMDIO_LCD_wrstring("   ");
	PMDIO_LCD_setcursor(linenum, 4);
	if (freq < 1000) // display Hz if frequency < 1Khz
	{
		PMDIO_LCD_putnum(freq, 10);
		PMDIO_LCD_wrstring(" ");
	}
	else   // display frequency in KHz
	{
		PMDIO_LCD_putnum((freq / 1000), 10);
		PMDIO_LCD_wrstring("K ");
	}

	PMDIO_LCD_setcursor(linenum, 13);
	PMDIO_LCD_wrstring("  %");
	PMDIO_LCD_setcursor(linenum, 13);
	PMDIO_LCD_putnum(dutycycle, 10);
}


	
/**************************** INTERRUPT HANDLERS ******************************/

/****************************************************************************/
/**
* Fixed interval timer interrupt handler 
*  
* updates the global "timestamp" every millisecond.  "timestamp" is used for the delay_msecs() function
* and as a time stamp for data collection and reporting.  Toggles the FIT clock which can be used as a visual
* indication that the interrupt handler is being called.  Also makes RGB1 a PWM duty cycle indicator
*
* @note
* software pwd is implemented below, as is the calculator for creating frequency and duty cycle display numbers

 *****************************************************************************/
void FIT_Handler(void)
{
		
	static	int			ts_interval = 0;			// interval counter for incrementing timestamp

	// toggle FIT clock
	clkfit ^= 0x01;
	XGpio_DiscreteWrite(&GPIOInst0, 2, clkfit);

	// update timestamp	
	ts_interval++;	
	if (ts_interval > FIT_COUNT_1MSEC)
	{
		timestamp++;
		ts_interval = 1;
	}

	// Use an RGB LED (RGB1) as a PWM duty cycle indicator
	// we can read the current state of PWM out on GPIO[0] because we
	// fed it back around in the top level of our hardware design.
	// Note that this won't work well as the PWM frequency approaches
	// or exceeds 10KHz

	gpio_in = XGpio_DiscreteRead(&GPIOInst0, 1);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
				SOFTWARE PWD 
			two counters are implemented, one for high and one for low
			if the incoming signal is high the high counter is incremented
			while the low counter is reset and its value written to a variable for
			later calculation
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

*/
	if (gpio_in & 0x00000001) //if pwm singal is high
	{
		software_up_counter++;
		if (software_down_counter != 0)
			{

			software_pwm_down_count = software_down_counter;
			software_down_counter = 0;
			}
	}

	if (~gpio_in & 0x00000001)
	{
		software_down_counter++;
			if (software_up_counter !=0)

			{
				software_pwm_up_count = software_up_counter;

			software_up_counter = 0;
			}
	}


	//	}
	if (~mode_select)///software mode << assigns counts gathered by software to variables used for calculation and display
	{
		NX410_SSEG_setAllDigits(SSEGHI, CC_1, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
		NX4IO_RGBLED_setChnlEn(RGB1, false, false, true);
	high_count = software_pwm_up_count;
	low_count = software_pwm_down_count;
	total_count = software_pwm_up_count + software_pwm_down_count;
	dutycycle = 100*high_count/(total_count);
	freq = 100000000/(2500*(high_count+low_count)); //divide by 2500 because this block only executes when FIT causes interrupt every 2500 cycles



	}



	if (mode_select) ///hardware mode << assigns counts gathered by hardware module to variables used for calculation and display
	{

		NX410_SSEG_setAllDigits(SSEGHI, CC_2, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
		NX4IO_RGBLED_setChnlEn(RGB1, true, false, false);
	high_count = XGpio_DiscreteRead(&GPIOInst1, 1);
	low_count = XGpio_DiscreteRead(&GPIOInst1, 2);
	total_count = high_count + low_count;
	dutycycle = 100*high_count/(total_count);
	freq = 100000000/((high_count+low_count));
	}
// DEBUG/DISPLAY LED sets RGB2 to go turquoise if PWM duty cycle exceeds 50%
	if (high_count > low_count)
	{
				NX4IO_RGBLED_setChnlEn(RGB2, false, true, true);
					}
	else
	{
				NX4IO_RGBLED_setChnlEn(RGB2, false, false, false);
					}

}
//END OF FILE
