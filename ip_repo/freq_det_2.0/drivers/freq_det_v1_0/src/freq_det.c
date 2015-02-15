

/***************************** Include Files *******************************/
#include "freq_det.h"
#define clock 		100000000
/************************** Function Definitions ***************************/


//read period count - period out is tied to slave register 0 


int FrequencyOut(void)

 Period_count = FREQ_DET_mReadReg(BaseAddress, RegOffset)
 Frequency = clock/Period_count
 
 //divisor is tied to slave register 1 [7:0] - if set to 0 update every period
 
 //frequency to watt light intensity. 
 
 //light intensity to voltage supplied (actually, we dont know this information, we just need to push more voltage if we are under
 //the desired intensity. So there is a linear relationship between irradiance and kHz, ranging from .001 to 1000, per the spec. sheet.