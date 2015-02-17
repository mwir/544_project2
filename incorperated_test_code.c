/**
*
* @file incorperated_test_code.c
*
*
* @author Mark Ronay (ronay@pdx.edu)
--------------------------

*
* @note
* The minimal hardware configuration for this program is a Microblaze-based system with at least 32KB of memory,
* an instance of Nexys4IO, an instance of the PMod544IOR2, an instance of an axi_timer, an instance of an axi_gpio
* and an instance of an axi_uartlite (used for xil_printf() console output)
*
******************************************************************************/

/********** Include Files ***********/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "xparameters.h"
#include "xbasic_types.h"
#include "xtmrctr.h"
#include "xintc.h"
#include "xgpio.h"
#include "Nexys4IO.h"
#include "PMod544IOR2.h"
#include "pwm_tmrctr.h"
#include "mb_interface.h"

//ECE544 Students: 
//#include "YOUR_PERIPHERAL_DRIVER.h"

/************************** Macros and Constants ******************************/

//ECE544 Students: Change these to match your xparameters.h

// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ
#define PWM_TIMER_FREQ_HZ		XPAR_AXI_TIMER_0_CLOCK_FREQ_HZ

// GPIO parameters
#define GPIO_DEVICE_ID			XPAR_XPS_GPIO_0_DEVICE_ID
#define GPIO_BASEADDR			XPAR_XPS_GPIO_0_BASEADDR
#define GPIO_HIGHADDR			XPAR_XPS_GPIO_0_HIGHADDR
#define GPIO_OUTPUT_CHANNEL		1

// Nexys4IO and Pmod544IO parameters
#define NX4IO_DEVICE_ID			XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR			XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR			XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

#define PMD544IO_DEVICE_ID		XPAR_PMOD544IOR2_0_DEVICE_ID
#define PMD544IO_BASEADDR		XPAR_PMOD544IOR2_0_S00_AXI_BASEADDR
#define PMD544IO_HIGHADDR		XPAR_PMOD544IOR2_0_S00_AXI_HIGHADDR

// PWM timer parameters
// Set PWM frequency = 10KHz, duty cycle increments by 5%
#define PWM_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define PWM_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define PWM_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define PWM_FREQUENCY			10000	
#define PWM_VIN					3.3	
#define DUTY_CYCLE_CHANGE		2

// Min and Max duty cycle for step and characterization tests
#define STEPDC_MIN				1
#define STEPDC_MAX				99					
		
// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_XPS_INTC_0_DEVICE_ID
#define INTC_BASEADDR			XPAR_AXI_INTC_0_BASEADDR
#define INTC_HIGHADDR			XPAR_AXI_INTC_0_HIGHADDR
#define TIMER_INTERRUPT_ID		XPAR_AXI_INTC_0_XPS_TIMER_0_INTERRUPT_INTR
#define FIT_INTERRUPT_ID		XPAR_AXI_INTC_0_FIT_TIMER_0_INTERRUPT_INTR 
				
// Fixed Interval timer - 100MHz input clock, 5KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	AXI_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		5000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			(FIT_CLOCK_FREQ_HZ / 1000)	

// Light Sensor Peripheral parameters
// Add whatever constants you need to use your Light Sensor peripheral driver

// sample settings	
#define NUM_FRQ_SAMPLES			250	
//For updating PID variables via pushbutton - actual values in lower case, actual gains called P_GAIN etc
#define	P				0
#define I				1
#define	D				2
#define OFFSET			3

		
// macro functions
#define MIN(a, b)  ( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  ( ((a) >= (b)) ? (a) : (b) )

/***********************************************************************
/************************** Variable Definitions ****************************/	
// Microblaze peripheral instances
XIntc 	IntrptCtlrInst;						// Interrupt Controller instance
XTmrCtr	PWMTimerInst;						// PWM timer instance
XGpio	GPIOInst0;							// GPIO instance
/*88888888888888888888888888888888888888888888888888888
		--put in custom periph --
		
*///888888888888888888888888888888888888888888888888888

/****************************** typedefs and structures **********************/
typedef enum {TEST_TRACKING = 0x0, TEST_STEPLOHI = 0x01, TEST_STEPHILO = 0x02, 
				TEST_CHARACTERIZE = 0x03, TEST_INVALID = 0xFF} Test_t;

/************************************************************************/




// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler
volatile unsigned long	timestamp;					// timestamp since the program began
volatile u32			gpio_port = 0;				// GPIO port register - maintained in program


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
// The following variables are shared between the functions in the program
// such that they must be global
u16						sample[NUM_FRQ_SAMPLES];	// sample array 	
int						smpl_idx;					// index into sample array
int						frq_smple_interval;			// approximate sample interval			

int						pwm_freq;					// PWM frequency 
int						pwm_duty;			// PWM duty cycle
int						total_count;
// Light Sensor Peripheral parameters
// Add whatever global variables you need to use your Light Sensor peripheral driver


bool					new_perduty;		// new period/duty cycle flag
bool					mode_select;
bool					run;
bool					write;				//to track display updates in PID param selects
bool					written;


/************************** Function Prototypes ******************************/
XStatus 		DoTest_Track(void);											// Perform Tracking test
XStatus			DoTest_Step(int dc_start);									// Perform Step test
XStatus			DoTest_Characterize(void);									// Perform Characterization test
			
XStatus			do_init(void);												// initialize system
void			delay_msecs(u32 msecs);										// busy-wait delay for "msecs" milliseconds
void			voltstostrng(Xfloat32 v, char* s);							// converts volts to a string
void			update_lcd(int vin_dccnt, short vout_frqcnt);				// update LCD display

void			FIT_Handler(void);											// fixed interval timer interrupt handler
/*****************************************************************************/


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
						--Below are the constants, variables, functions and routines in main for the
												pushbutton functional specification --
*/

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


/*********************************************/
/*              Main Program                 */
/*********************************************/		
int main()
{
	XStatus 	Status;
	u16			sw, old_sw = 0xFFFF;	
	int			rotcnt, old_rotcnt = 0x1000;
	Test_t		test, next_test;
	
	// initialize devices and set up interrupts, etc.
 	Status = do_init();
 	if (Status != XST_SUCCESS)
 	{
 		PMDIO_LCD_setcursor(1,0);
 		PMDIO_LCD_wrstring("****** ERROR *******");
 		PMDIO_LCD_setcursor(2,0);
 		PMDIO_LCD_wrstring("INIT FAILED- EXITING");
 		exit(XST_FAILURE);
 	}

	// initialize the variables
	timestamp = 0;							
	pwm_freq = PWM_FREQUENCY;
	pwm_duty = STEPDC_MIN;
	next_test = TEST_INVALID;

	microblaze_enable_interrupts();
	 	  	
 	// display the greeting   
    PMDIO_LCD_setcursor(1,0);
    PMDIO_LCD_wrstring("PmodCtlSys Test ");
	PMDIO_LCD_setcursor(2,0);
	PMDIO_LCD_wrstring("R3.0 by Robin M.");
	NX4IO_setLEDs(0x0000FFFF);
	//Run the LED characterization routine to establish sensor min's and max's
    DoTest_Characterize();
	NX4IO_setLEDs(0x00000000);
       
    // main loop - there is no exit except by hardware reset
	while (1)
	{ 
		// read sw[1:0] to get the test to perform.
		sw = NX4IO_getSwitches() & 0x03;			
		if (sw == TEST_TRACKING)  // Test 0 = Track PWM voltage
		{
			// write the static info to display if necessary
			if (sw != next_test)
			{
				PMDIO_LCD_clrd();
				PMDIO_LCD_setcursor(1,0);
				PMDIO_LCD_wrstring("| TRK| Vi: sx.xx");
				PMDIO_LCD_setcursor(2,0);
				PMDIO_LCD_wrstring("Vo:sx.xx C:sxxxx");
			}
				 						
			// read rotary count and handle duty cycle changes
			// limit duty cycle to between STEPDC_MIN and STEPDC_MAX
			// PWM frequency does not change in this test
			PMDIO_ROT_readRotcnt(&rotcnt);
			if (rotcnt != old_rotcnt)
			{
				pwm_duty = MAX(STEPDC_MIN, MIN(rotcnt, STEPDC_MAX));
				old_rotcnt = rotcnt;
			}
			DoTest_Track();
			next_test = TEST_TRACKING;
		}   // Test 0 = Track PWM voltage
		else if ((sw == TEST_STEPHILO) || (sw == TEST_STEPLOHI))  // Test 1 & 2 - Step response 
		{
			Xfloat32	v;
			char		s[20];	
			
			// write the static info to the display if necessary
			if (sw != next_test)
			{
				if (sw == TEST_STEPHILO)
				{
					strcpy(s, "|HILO|Press RBtn");
				}
				else
				{
					strcpy(s, "|LOHI|Press RBtn");
				}
				
				PMDIO_LCD_clrd();
				PMDIO_LCD_setcursor(1,0);
				PMDIO_LCD_wrstring(s);
				PMDIO_LCD_setcursor(2,0);
				PMDIO_LCD_wrstring("LED OFF-Release ");
			}
			
			// start the test on the rising edge of the Rotary Encoder button press
			// the test will write the light detector samples into the global "sample[]"
			// the samples will be sent to stdout when the Rotary Encoder button
			// is released
			//
			
			// test whether the rotary encoder button has been pressed
			if (PMDIO_ROT_isBtnPressed())  // do the step test and dump data  
			{										
				// light "Run" (rightmost) LED to show the test has begun
				// and do the test.  The test will return when the measured samples array
				// has been filled.  Turn off the rightmost LED after the data has been 
				// captured to let the user know he/she can release the button
				NX4IO_setLEDs(0x00000001);
				if (sw == TEST_STEPHILO)  // perform High->Low step test
				{				
					DoTest_Step(STEPDC_MAX);
				}
				else  // perform Low->High step
				{
					DoTest_Step(STEPDC_MIN);
				}
				NX4IO_setLEDs(0x00000000);
				
				// wait for the Rotary Encoder button to be released
				// and then send the sample data to stdout
				do
				(
					delay_msecs(10);
				} while (PMDIO_ROT_isBtnPressed());
				
				// light "Transfer" LED to indicate that data is being transmitted
				// Show the traffic on the LCD
				NX4IO_setLEDs(0x00000002);
				PMDIO_LCD_clrd();
				PMDIO_LCD_setcursor(1, 0);
				PMDIO_LCD_wrstring("Sending Data....");
				PMDIO_LCD_setcursor(2, 0);
				PMDIO_LCD_wrstring("S:    DATA:     ");

				// print the descriptive heading followed by the data
				if (sw == TEST_STEPHILO)
				{
					xil_printf("\n\rHigh->Low Test Data\t\tAppx. Sample Interval: %d msec\n\r", frq_smple_interval);
				}
				else
				{
					xil_printf("\n\rLow->High Test Data\t\tAppx. Sample Interval: %d msec\n\r", frq_smple_interval);
				}
				
				// trigger the serial charter program)
				xil_printf("===STARTPLOT===\n");

				// start with the second sample.  The first sample is not representative of
				// the data.  This will pretty-up the graph a bit								
				for (smpl_idx = 1; smpl_idx < NUM_FRQ_SAMPLES; smpl_idx++)
				{
					u16 count;
					
					count = sample[smpl_idx];
					
					//ECE544 Students:
                    //Convert from count to 'volts'
                    //v = YOUR_FUNCTION(count);
					
					voltstostrng(v, s);
					xil_printf("%d\t%d\t%s\n\r", smpl_idx, count, s);
					
					PMDIO_LCD_setcursor(2, 2);
					PMDIO_LCD_wrstring("   ");
					PMDIO_LCD_setcursor(2, 2);
					PMDIO_LCD_putnum(smpl_idx, 10);
					PMDIO_LCD_setcursor(2, 11);
					PMDIO_LCD_wrstring("     ");
					PMDIO_LCD_setcursor(2, 11);
					PMDIO_LCD_putnum(count, 10);
				}
				
				// stop the serial charter program				
				xil_printf("===ENDPLOT===\n");
				
				NX4IO_setLEDs(0x00000000);								
				next_test = TEST_INVALID;			
			}  // do the step test and dump data
			else
			{
				next_test = test;
			}
		} // Test 1 & 2 - Step response
		else if (sw == TEST_CHARACTERIZE)  // Test 3 - Characterize Response
		{
			if (sw != next_test)
			{				
				PMDIO_LCD_clrd();
				PMDIO_LCD_setcursor(1,0);
				PMDIO_LCD_wrstring("|CHAR|Press RBtn");
				PMDIO_LCD_setcursor(2,0);
				PMDIO_LCD_wrstring("LED OFF-Release ");
			}
			// start the test on the rising edge of the Rotary Encoder button press
			// the test will write the samples into the global "sample[]"
			// the samples will be sent to stdout when the Rotary Encoder button
			// is released
			// test whether the rotary encoder button has been pressed
			if (PMDIO_ROT_isBtnPressed())  // do the step test and dump data  
			{	
				// light "Run" (rightmost) LED to show the test has begun
				// and do the test.  The test will return when the measured samples array
				// has been filled.  Turn off the rightmost LED after the data has been
				// captured to let the user know he/she can release the button
				NX4IO_setLEDs(0x00000001);			
				DoTest_Characterize();
				NX4IO_setLEDs(0x00000000);
				
				// wait for the Rotary Encoder button to be released
				// and then send the sample data to stdout
				do
				(
					delay_msecs(10);
				} while (PMDIO_ROT_isBtnPressed());
				
				// light "Transfer" LED to indicate that data is being transmitted
				// Show the traffic on the LCD
				NX4IO_setLEDs(0x00000002);
				PMDIO_LCD_clrd();
				PMDIO_LCD_setcursor(1, 0);
				PMDIO_LCD_wrstring("Sending Data....");
				PMDIO_LCD_setcursor(2, 0);
				PMDIO_LCD_wrstring("S:    DATA:     ");

				xil_printf("\n\rCharacterization Test Data\t\tAppx. Sample Interval: %d msec\n\r", frq_smple_interval);

				// trigger the serial charter program)
				xil_printf("===STARTPLOT===\n\r");
				
				for (smpl_idx = STEPDC_MIN; smpl_idx <= STEPDC_MAX; smpl_idx++)
				{
					u16 		count;
					Xfloat32	v;
					char		s[10]; 
					
					count = sample[smpl_idx];
					
					//ECE544 Students:
                    //Convert from count to 'volts' //there is a 1-1 correspondence between illumination and freq from the detector,
					//									but, bc the light source is not direct i ont know what the volts will be, we
					//									might have to actually take empirical data. Or, are we supposed to use this 
					//									program to find the freq to volts on our system? Yes - I think so.
                    //v = YOUR_FUNCTION(count);
					
					voltstostrng(v, s);
					xil_printf("%d\t%d\t%s\n\r", smpl_idx, count, s);

					PMDIO_LCD_setcursor(2, 2);
					PMDIO_LCD_wrstring("   ");
					PMDIO_LCD_setcursor(2, 2);
					PMDIO_LCD_putnum(smpl_idx, 10);
					PMDIO_LCD_setcursor(2, 11);
					PMDIO_LCD_wrstring("     ");
					PMDIO_LCD_setcursor(2, 11);
					PMDIO_LCD_putnum(count, 10);
				}

				// stop the serial charter program				
				xil_printf("===ENDPLOT===\n\r");
				
				NX4IO_setLEDs(0x00000000);								
				next_test = TEST_INVALID;
			}  // do the step test and dump data
			else
			{
				next_test = test;
			}
		} // Test 3 - Characterize Response
		else  // outside the current test range - blink LED's and hang
		{
			// should never get here but just in case
			NX4IO_setLEDs(0x000000FF);
			delay_msecs(2000);
			NX4IO_setLEDs(0x00);
		}
		// wait a bit and start again
		

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
			delay_msecs(100);
	}  // while(1) loop
 }  // end main()

 
/*********************************************/
/*             Test Functions                */
/*********************************************/

/*****
 * DoTest_Track() - Perform the Tracking test
 * 
 * This function uses the global "pwm_freq" and "pwm_duty" values to adjust the PWM
 * duty cycle and thus the intensity of the LED.  The function displays
 * the light detector reading as it tracks changes in the
 * LED intensity.  This test runs continuously until a different test is selected.
 * Returns XST_SUCCESS since this test can't fail.  Returns approximate sample interval
 * in the global variable "frq_sample_interval"
 *****/ 
XStatus DoTest_Track(void)
{
	static int		old_pwm_freq = 0;			// old pwm_frequency and duty cycle
	static int		old_pwm_duty = 200;			// these values will force the initial display	
	u16				frq_cnt;					// light detector counts to display
	XStatus			Status;						// Xilinx return status
	unsigned		tss;						// starting timestamp			

	if ((pwm_freq != old_pwm_freq) || (pwm_duty != old_pwm_duty))
	{	
		// set the new PWM parameters - PWM_SetParams stops the timer
		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
		if (Status == XST_SUCCESS)
		{							
			PWM_Start(&PWMTimerInst);
		}

		tss = timestamp;	
		
		//ECE544 Students:
        //make the light sensor measurement
		//frq_cnt = YOUR FUNCTION HERE;
		
		delay_msecs(1);
		frq_smple_interval = timestamp - tss;
				
		// update the display and save the frequency and duty
		// cycle for next time
		update_lcd(pwm_duty, frq_cnt);
		old_pwm_freq = pwm_freq;
		old_pwm_duty = pwm_duty;
	}
	return XST_SUCCESS;
}



/*****
 * DoTest_Step() - Perform the Step test
 * 
 * This function stabilizes the duty cycle at "dc_start" for
 * about a second and a half and then steps the duty cycle from min to max or
 * max to min depending on the test. NUM_FRQ_SAMPLES are collected
 * into the global array sample[].  An approximate sample interval
 * is written to the global variable "frq_smpl_interval"
 *****/ 
XStatus DoTest_Step(int dc_start)
{	
	XStatus		Status;					// Xilinx return status
	unsigned	tss;					// starting timestamp
	u16			frq_cnt;				// measured counts to display
		
	// stabilize the PWM output (and thus the lamp intensity) before
	// starting the test
	Status = PWM_SetParams(&PWMTimerInst, pwm_freq, dc_start);
	if (Status == XST_SUCCESS)
	{							
		PWM_Start(&PWMTimerInst);
	}
	else
	{
		return XST_FAILURE;
	}
	//Wait for the LED output to settle before starting
    delay_msecs(1500);
		
	if (dc_start > STEPDC_MAX / 2)
	{
		 Status = PWM_SetParams(&PWMTimerInst, pwm_freq, STEPDC_MIN); 
	}
	else
	{
		 Status = PWM_SetParams(&PWMTimerInst, pwm_freq, STEPDC_MAX); 
	}		
	if (Status == XST_SUCCESS)
	{							
		PWM_Start(&PWMTimerInst);
		pwm_duty = dc_start;
	}
	else
	{
		return XST_FAILURE;
	}
	
	// gather the samples
	smpl_idx = 0;
	tss = timestamp;
	while (smpl_idx < NUM_FRQ_SAMPLES)
	{
	
		//ECE544 Students:
        //make the light sensor measurement
		//sample[smpl_idx++] = YOUR FUNCTION HERE;
		
	}		
	frq_smple_interval = (timestamp - tss) / NUM_FRQ_SAMPLES;
	return XST_SUCCESS;
}
	
	
/*****
 * DoTest_Characterize() - Perform the Characterization test
 * 
 * This function starts the duty cycle at the minimum duty cycle and
 * then sweeps it to the max duty cycle for the test.
 * Samples are collected into the global array sample[].  
 * The function toggles the TEST_RUNNING signal high for the duration
 * of the test as a debug aid and adjusts the global "pwm_duty"
 *
 * The test also sets the global frequency count min and max counts to
 * help limit the counts to the active range for the circuit
 *****/
XStatus DoTest_Characterize(void)
{
	XStatus		Status;					// Xilinx return status
	unsigned	tss;					// starting timestamp
	u16			frq_cnt;				// counts to display
	int			n;						// number of samples
	Xuint32		freq, dutyfactor;		// current frequency and duty factor


	// stabilize the PWM output (and thus the lamp intensity) at the
	// minimum before starting the test
	pwm_duty = PWM_STEPDC_MIN;
	Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
	if (Status == XST_SUCCESS)
	{							
		PWM_Start(&PWMTimerInst);
	}
	else
	{
		return -1;
	}
	//Wait for the LED output to settle before starting
    delay_msecs(1500);
		
	// sweep the duty cycle from STEPDC_MIN to STEPDC_MAX
	smpl_idx = PWM_STEPDC_MIN;
	n = 0;
	tss = timestamp;
	while (smpl_idx <= PWM_STEPDC_MAX)
	{
		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, smpl_idx);
		if (Status == XST_SUCCESS)
		{							
			PWM_Start(&PWMTimerInst);
		}
		else
		{
			return -1;
		}
		
        //ECE544 Students:
        // make the light sensor measurement
		//sample[smpl_idx++] = YOUR FUNCTION HERE;
		
		n++;
		delay_msecs(50);
	}		
	frq_smple_interval = (timestamp - tss) / smpl_idx;

    //ECE544 Students:
    //Find the min and max values and set the scaling/offset factors to use for your convert to 'voltage' function.
    //NOTE: It may also be useful to scale the actual 'count' values to a range of 0 - 4095 for the SerialCharter application to work correctly 
    // FRQ_max_cnt = ?
    // FRQ_min_cnt = ?
	// YOUR_FUNCTION(FRQ_min_cnt,FRQ_max_cnt);
	
    return n;
}

	
	
/*********************************************/
/*            Support Functions              */
/*********************************************/

/*****
 * do_init() - initialize the system
 * 
 * This function is executed once at start-up and after a reset.  It initializes
 * the peripherals and registers the interrupt handlers
 *****/
XStatus do_init(void)
{
	XStatus 	Status;				// status from Xilinx Lib calls	
	
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
 	PMDIO_ROT_init(DUTY_CYCLE_CHANGE, true);
	PMDIO_ROT_clear();

	// initialize the GPIO instance
	Status = XGpio_Initialize(&GPIOInst, GPIO_DEVICE_ID);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO channel 1 is an 8-bit output port that your application can
	// use.  None of the bits are used by this program
	XGpio_SetDataDirection(&GPIOInst, GPIO_OUTPUT_CHANNEL, 0xF0);
	XGpio_DiscreteWrite(&GPIOInst, GPIO_OUTPUT_CHANNEL, gpio_port);
	
			
	// initialize the PWM timer/counter instance but do not start it
	// do not enable PWM interrupts
	Status = PWM_Initialize(&PWMTimerInst, PWM_TIMER_DEVICE_ID, false);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	
    //ECE544 Students:
    //Initialize your peripheral here
			
	// initialize the interrupt controller
	Status = XIntc_Initialize(&IntrptCtlrInst,INTC_DEVICE_ID);
    if (Status != XST_SUCCESS)
    {
       return XST_FAILURE;
    }

	// connect the fixed interval timer (FIT) handler to the interrupt
    Status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
                           (XInterruptHandler)FIT_Handler,
                           (void *)0);
    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }
 
 	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts, specifically real mode so that
	// the the  FIT can cause interrupts thru the interrupt controller.
    Status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    } 
      
 	// enable the FIT interrupt
    XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);	
	return XST_SUCCESS;
}
		
		

/*****
 * delay_msecs() - delay execution for "n" msecs
 * 
 *		timing is approximate but we're not looking for precision here, just
 *		a uniform delay function.  The function uses the global "timestamp" which
 *		is incremented every msec by FIT_Handler().
 *
 * NOTE:  Assumes that this loop is running faster than the fit_interval ISR (every msec)
 *****/
void delay_msecs(u32 msecs)
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


/*****
 * voltstostrng() - converts volts to a fixed format string
 * 
 * accepts an Xfloat32 voltage reading and turns it into a 5 character string
 * of the following format:
 *		(+/-)x.yy 
 * where (+/-) is the sign, x is the integer part of the voltage and yy is
 * the decimal part of the voltage.
 *	
 * NOTE:  Assumes that s points to an array of at least 6 bytes.	
 *****/
 void	voltstostrng(Xfloat32 v, char* s)
 {
	Xfloat32	dpf, ipf;
	Xuint32		dpi;	
	Xuint32		ones, tenths, hundredths;

	 // form the fixed digits 
	 dpf = modff(v, &ipf);
	 dpi = dpf * 100;
	 ones = abs(ipf) + '0';
	 tenths = (dpi / 10) + '0';
	 hundredths = (dpi - ((tenths - '0') * 10)) + '0';
	 
	 // form the string and return
	 *s++ = ipf == 0 ? ' ' : (ipf > 0 ? '+' : '-');
	 *s++ = (char) ones;
	 *s++ = '.';
	 *s++ = (char) tenths;
	 *s++ = (char) hundredths;
	 *s   = 0;
	 return;
 }
	  
	 
 /*****
  * update_lcd() - update the LCD display with a new count and voltage
  *
  * writes the display with new information.  "vin_dccnt" is the  unsigned PWM duty
  * cycle and "frqcnt" is the signed frq_count.  The function assumes that the
  * static portion of the display has been written and that the dynamic portion of
  * the display is the same for all tests
  *****/
 void update_lcd(int vin_dccnt, short frqcnt)
 {
 	Xfloat32	v;
 	char		s[10];

 	// update the PWM data
 	v = vin_dccnt * .01 * PWM_VIN;
 	voltstostrng(v, s);
 	PMDIO_LCD_setcursor(1, 11);
 	PMDIO_LCD_wrstring("      ");
 	PMDIO_LCD_setcursor(1, 11);
 	PMDIO_LCD_wrstring(s);

 	// update the data
    // ECE544 Students: Convert frequency count to 'volts'
    // v = YOUR_FUNCTION(frqcnt);
 	voltstostrng(v, s);
 	PMDIO_LCD_setcursor(2, 3);
 	PMDIO_LCD_wrstring("     ");
 	PMDIO_LCD_setcursor(2, 3);
 	PMDIO_LCD_wrstring(s);
 	PMDIO_LCD_setcursor(2, 11);
 	PMDIO_LCD_wrstring("     ");
 	PMDIO_LCD_setcursor(2, 11);
 	PMDIO_LCD_putnum(frqcnt, 10);
 	return;
 }

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//			PARAM_SELECT
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
 
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
 
 
 
/*********************************************/
/*            Interrupt Handlers             */
/*********************************************/

/*****
 * FIT_Handler() - Fixed interval timer interrupt handler 
 *  
 * updates the global "timestamp" every millisecond.  "timestamp" is used for the delay_msecs() function
 * and as a time stamp for data collection and reporting
 *****/
void FIT_Handler(void)
{	
	static	int			ts_interval = 0;			// interval counter for incrementing timestamp
			
	// update timestamp	
	ts_interval++;	
	if (ts_interval > FIT_COUNT_1MSEC)
	{
		timestamp++;
		ts_interval = 1;
	}
}	


