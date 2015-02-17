#define setpoint 100
#define pwm_full_on 99
#define pwm_full_off 1

while(1)
{
	//delay function (1ms) //wait for next sample
	//read sensor
	//
	if (feq_det < setpoint) 
	{
	PWM(pwm_full_on);
	}
	else
	{
		PWM(pwm_full_off);
	}
}

