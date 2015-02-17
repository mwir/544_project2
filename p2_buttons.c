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
bool					run;
bool						write;
bool						written;





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
volatile float			P_GAIN;
volatile float			I_GAIN;
volatile float			D_GAIN;
volatile float			OFFSET_VALUE;

/****************************** typedefs and structures **********************/
typedef enum {STANDBY = 0x0, PID = 0x01, BANGBANG = 0x02,
				FUZZY = 0x03, TEST_INVALID = 0xFF} Test_t;

/************************************************************************/
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
void			update_lcd(int freq, int dutycyle, u32 linenum);		// update LCD display
void			FIT_Handler(void);										// fixed interval timer interrupt handler
void			PID_PARAM_SELECT(void);
void			PID_FUNCTION(void);
void			BANGBANG_FUNCTION(void);
void			FUZZY_FUNCTION(void);
void			PID_OPTIMIZER(void);
/************************** MAIN PROGRAM ************************************/
int main()
{
	XStatus 	status;
	u16			algo_select_sw, old_algo_select_sw = 0xFFFF;
	u16			test_select_sw;

	push_button_center = false;
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

	push_button_center = false;
	run = true;
	clkfit = 0;
	write = true;
	written = false;
	D_GAIN = 0;
	P_GAIN = 0;
	I_GAIN = 0;
	OFFSET_VALUE = 0;
	PID_variable_value = 0;

	// start the PWM timer and kick of the processing by enabling the Microblaze interrupt
	PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
	PWM_Start(&PWMTimerInst);
    microblaze_enable_interrupts();

	// display the greeting
    PMDIO_LCD_setcursor(1,0);
    PMDIO_LCD_wrstring("544: UI TEST");
	PMDIO_LCD_setcursor(2,0);
	PMDIO_LCD_wrstring(" by Mark Ronay ");
	NX4IO_setLEDs(0x0000FFFF);
	delay_msecs(2000);
	NX4IO_setLEDs(0x00000000);
	PMDIO_LCD_setcursor(1,0);
	PMDIO_LCD_wrstring("                ");
	PMDIO_LCD_setcursor(2,0);
	PMDIO_LCD_wrstring("               ");


    // turn off the LEDs and clear the seven segment display
    NX4IO_setLEDs(0x00000000);
    NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
    NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);

    // main loop
	do
	{
		algo_select_sw = NX4IO_getSwitches() & 0x03;
		test_select_sw = NX4IO_getSwitches() & 0xF000;
		//we want to create a way to set a set point, probably with the rotary knob.pushing the knob will transmit
		//data. perhaps we can have more functionality with a different switches mask
		if(test_select_sw ==0x8000)
		{
			PID_OPTIMIZER();
		}

		if (test_select_sw == 0)
		{
			if (NX4IO_isPressed(BTNC))
			{
				push_button_center = !push_button_center;
				PMDIO_LCD_setcursor(1,0);
				PMDIO_LCD_wrstring("PID PARAM SELECT");	//write PID PARAM SELECT on LCD
				written = false;
				write = true;
				delay_msecs(700);


			}

			if(push_button_center)
			{
				if(write)
				{
					PMDIO_LCD_setcursor(1,0);
					// write the static text to the display
				    PMDIO_LCD_clrd();
				    PMDIO_LCD_setcursor(2,0);
				    PMDIO_LCD_wrstring("D:     OFF:    ");

				    PMDIO_LCD_setcursor(1,0);
				    PMDIO_LCD_wrstring("P:     I:   ");

					write=false;
					written = true;
				}
			if (written);
			{
			NX4IO_RGBLED_setChnlEn(RGB1, false, false, false);
			PID_PARAM_SELECT();	//Call PID SELECT FUNCTION

			}
			if (NX4IO_isPressed(BTNC))
				{
					push_button_center = !push_button_center;
					NX4IO_RGBLED_setChnlEn(RGB1, true, false, false);
					PMDIO_LCD_setcursor(1,0);
					PMDIO_LCD_wrstring("RETURN          ");	//write PID PARAM SELECT on LCD
					PMDIO_LCD_setcursor(2,0);
					PMDIO_LCD_wrstring("                ");	//write PID PARAM SELECT on LCD

					delay_msecs(700);

					}


			}

			if( !push_button_center)
			{
				NX4IO_RGBLED_setChnlEn(RGB1, false, false, true);
							if (algo_select_sw == STANDBY)
							{

								NX4IO_RGBLED_setChnlEn(RGB2, false, false, false);
								PMDIO_LCD_setcursor(1,0);
								PMDIO_LCD_wrstring("STANDBY          ");
								PMDIO_LCD_setcursor(2,0);
								PMDIO_LCD_wrstring("                  ");

							}

							if (algo_select_sw == PID)
							{
								PMDIO_LCD_setcursor(1,0);
								PMDIO_LCD_wrstring("PID          ");
								PMDIO_LCD_setcursor(2,0);
								PMDIO_LCD_wrstring("                  ");
								PID_FUNCTION();
							}

							if (algo_select_sw == BANGBANG)
							{
								PMDIO_LCD_setcursor(1,0);
								PMDIO_LCD_wrstring("BANGBANG          ");
								PMDIO_LCD_setcursor(2,0);
								PMDIO_LCD_wrstring("                  ");
								BANGBANG_FUNCTION();
							}

							if (algo_select_sw == FUZZY)
							{
								PMDIO_LCD_setcursor(1,0);
								PMDIO_LCD_wrstring("FUZZY          ");
								PMDIO_LCD_setcursor(2,0);
								PMDIO_LCD_wrstring("                  ");
								FUZZY_FUNCTION();
							}

			}
		}
	}while (run);



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


	//+++++++++++++++++++++++++++++++++++++++

	/*
	 * 			INITIALIZE CUSTOM IP HERE
	 *
	 *
	 *
	 *
	 */
	//++++++++++++++++++++++++++++++++++++++


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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	/*

										*/
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void PID_PARAM_SELECT(void)
{
	


	//when a test is not running P, I, D, OFFSET, SET POINT should be formatted on the LCD with the cursor indicating which is to be changed



	if (NX4IO_isPressed(BTNU))
	{
		delay_msecs(50);
		push_button_up++;
	NX4IO_RGBLED_setChnlEn(RGB1, false, true, false);

			if (push_button_up >3)
			{
				push_button_up = 0;
				}
		}
	 if (NX4IO_isPressed(BTND))
	 {
		push_button_up--;
	 NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);

			if ((push_button_up >3) | (push_button_up<0))  //this is kind of clunky but maybe functional? decrementing this isnt in the design specs anyway
			{
				push_button_up = 3; 
				}
		}
		
		


	switch (push_button_up)
		{
			case 0x00:	selected_PID = P;
			PID_variable_value = P_GAIN;
					 break;
			case 0x01:	selected_PID = I;
			PID_variable_value = I_GAIN;
						 break;
			case 0x02:	selected_PID = D;
			PID_variable_value = D_GAIN;
						 break;
			case 0x03:	selected_PID = OFFSET;
			PID_variable_value = OFFSET_VALUE/10;
				 break;
		}



	if (NX4IO_isPressed(BTNR))
	{
		delay_msecs(50);
		PID_variable_value++;
	NX4IO_RGBLED_setChnlEn(RGB1, true, true, false);

			if (PID_variable_value >100)
			{
				PID_variable_value = 0;
				}
		}	
		
	if (NX4IO_isPressed(BTNL))
	{
		delay_msecs(50);
		PID_variable_value--;
	NX4IO_RGBLED_setChnlEn(RGB1, true, false, true);

			if ((PID_variable_value >100) | (PID_variable_value <0))
			{
				PID_variable_value = 100;
				}

		
	}



	if (selected_PID == P)
	{
		delay_msecs(50);
		NX4IO_setLEDs(0x0001);
		P_GAIN = PID_variable_value;

	PMDIO_LCD_setcursor(1,3);
	PMDIO_LCD_wrstring("    ");
	PMDIO_LCD_setcursor(1,3);
	PMDIO_LCD_putnum(P_GAIN, 10);		//write the proportional gain value
	}
	
	if (selected_PID == I)
	{
		delay_msecs(50);
		NX4IO_setLEDs(0x0002);
		I_GAIN = PID_variable_value;

	PMDIO_LCD_setcursor(1,12);
	PMDIO_LCD_wrstring("    ");
	PMDIO_LCD_setcursor(1,12);
	PMDIO_LCD_putnum(I_GAIN, 10);		//write the proportional gain value
	}
	
	if (selected_PID == D)
	{
		delay_msecs(50);
		NX4IO_setLEDs(0x0004);

		D_GAIN = PID_variable_value;
	PMDIO_LCD_setcursor(2,3);
	PMDIO_LCD_wrstring("    ");
	PMDIO_LCD_setcursor(2,3);
	PMDIO_LCD_putnum(D_GAIN, 10);		//write the proportional gain value
	}
	
	if (selected_PID == OFFSET)
	{
		delay_msecs(50);
		NX4IO_setLEDs(0x0008);

		OFFSET_VALUE = 10*PID_variable_value;
	PMDIO_LCD_setcursor(2,11);
	PMDIO_LCD_wrstring("    ");
	PMDIO_LCD_setcursor(2,11);
	PMDIO_LCD_putnum(OFFSET_VALUE, 10);		//write the proportional gain value
	}
	//delay_msecs(500);
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*
						--Here ends the pushbutton functional spec code
						
						
*/
//********************************************************


void PID_FUNCTION(void)
{
	NX4IO_RGBLED_setChnlEn(RGB2, false, true, false);
}

void BANGBANG_FUNCTION(void)
{
	NX4IO_RGBLED_setChnlEn(RGB2, true, false, false);
}


void FUZZY_FUNCTION(void)
{
	NX4IO_RGBLED_setChnlEn(RGB2, false, false, true);
}

void PID_OPTIMIZER(void)
{
	int i = 100;
	PMDIO_LCD_setcursor(1,0);
	PMDIO_LCD_wrstring("OPTIMIZING!!    ");
	PMDIO_LCD_setcursor(2,0);
	PMDIO_LCD_wrstring("                  ");

	/*
	 * psuedocode goes here
	 *
	 */
	do
	{
		NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);
		delay_msecs(100);
		NX4IO_RGBLED_setChnlEn(RGB2, false,false, false);
				delay_msecs(100);
		i=i-1;
	}
	while(i>0);
	PMDIO_LCD_setcursor(1,0);
	PMDIO_LCD_wrstring("OPTIMIZING DONE!!");
	PMDIO_LCD_setcursor(2,0);
	PMDIO_LCD_wrstring("                  ");
	delay_msecs(1000);
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


}
//END OF FILE
