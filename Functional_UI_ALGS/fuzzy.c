
//we have 5 output possibilities, each one corresponding to an additional drive on the PWM signal

#define Z 0
#define SP .5
#define LP  1
#define SN  -.5
#define LN  -1

//the above values  can be fiddled with to make sense.

// macro functions
#define MIN(a, b)  ( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  ( ((a) >= (b)) ? (a) : (b) )





//the following structs return mu values for different error and delta values we can get from our reading. We have 4 outputs we can give,
//small positive (sp), large positive (lp), small negative (sn), large negative (ln) and zero (z). 

float sp[50] = {
		 0.6,//1
		 0.130,//2
		 0.2,0,//3
		 0.270,//4
		 0.340,//5
		 0.430,//6
		 0.5,0,//7
		 0.570,//8
		 0.640,//9
		 0.710,//10
		 0.780,//11
		 0.850,//12
		 0.920,//13
		 1,	//14
		 0.920,//15
		 0.850,//16
		 0.780,//17
		 0.710,//18
		 0.640,//19
		 0.570,//20
		 0.50,//21
		 0.430,//22
		 0.340,//23
		 0.270,//24
		 0.20,//25
		 0.130,//26
		 0.60,//27
		 0,//28
		 0,//29
		 0,//30
		 0,//31
		 0,//34
		 0,//32
		 0,//33
		 0,//34
		 0,//35
		 0,//36
		 0,//37
		 0,//38
		 0,//39
		 0,//40
		 0,//41
		 0,//44
		 0,//42
		 0,//43
		 0,//44
		 0,//45
		 0,//46
		 0,//47
		 0,//48
		 0,//49
		 0//50
		
};

float lp[50]={
		 1,	//1
		 0.92//2
		 0.85//3
		 0.78//4
		 0.71//5
		 0.64//6
		 0.57//7
		 0.5,//8
		 0.43//9
		 0.34//10
		 0.27//11
		 0.2,//12
		 0.130,//13
		 0.6,//14
		 0,//15
		 0,//16
		 0,//17
		 0,//18
		 0,//19
		 0,//20
		 0,//21
		 0,//22
		 0,//23
		 0,//24
		 0,//25
		 0,//26
		 0,//27
		 0,//28
		 0,//29
		 0,//30
		 0,//31
		 0,//34
		 0,//32
		 0,//33
		 0,//34
		 0,//35
		 0,//36
		 0,//37
		 0,//38
		 0,//39
		 0,//40
		 0,//41
		 0,//44
		 0,//42
		 0,//43
		 0,//44
		 0,//45
		 0,//46
		 0,//47
		 0,//48
		 0,//49
		 0//50
}
float ln[50]={
		0,//1
		 0,//2
		 0,//3
		 0,//4
		 0,//5
		 0,//6
		 0,//7
		 0,//8
		 0,//9
		 0,//10
		 0,//11
		 0,//12
		 0,//13
		 0,//14
		 0,//15
		 0,//16
		 0,//17
		 0,//18
		 0,//19
		 0,//20
		 0,//21
		 0,//22
		 0,//23
		 0,//24
		 0,//25
		 0,//26
		 0,//27
		 0,//28
		 0,//29
		 0,//30
		 0,//31
		 0,//34
		 0,//32
		 0,//33
		 0,//34
		 0,//35
		 0,//36
		 0,//37
		 0.6,///38
		 0.13,//39
		 0.2,///40
		 0.27,//41
		 0.34,//44
		 0.43,//42
		 0.5,///43
		 0.57,//44
		 0.64,//45
		 0.71,//46
		 0.78,//47
		 0.85,//48
		 0.92,//49
		 1	//50
}  ;         




float sn[50]={
		 0,//1
		 0,//2
		 0,//3
		 0,//4
		 0,//5
		 0,//6
		 0,//7
		 0,//8
		 0,//9
		 0,//10
		 0,//11
		 0,//12
		 0,//13
		 0,//14
		 0,//15
		 0,//16
		 0,//17
		 0,//18
		 0,//19
		 0,//20
		 0,//21
		 0,//22
		 0,//23
		 0,//24
		 0.6,///25
		 0.13,//26
		 0.2,///27
		 0.27,//28
		 0.34,//29
		 0.43,//30
		 0.5,///31
		 0.57,//34
		 0.64,//32
		 0.71,//33
		 0.78,//34
		 0.85,//35
		 0.92,//36
		 1,	///37
		 0.92,//38
		 0.85,//39
		 0.78,//40
		 0.71,//41
		 0.64,//44
		 0.57,//42
		 0.5,///43
		 0.43,//44
		 0.34,//45
		 0.27,//46
		 0.2,///47
		 0.13,//48
		 0.6,//49
		  0 //50
};
float z[50]={
		 0,//1
		 0,//2
		 0,//3
		 0,//4
		 0,//5
		 0,//6
		 0,//7
		 0,//8
		 0,//9
		 0,//10
		 0,//11
		 0.6,//12
		 0.13,//13
		 0.2,//14
		 0.27,//15
		 0.34,//16
		 0.43,//17
		 0.5,//18
		 0.57,//19
		 0.64,//20
		 0.71,//21
		 0.78,//22
		 0.85,//23
		 0.92,//24
		 1,	//25
		 0.92,//26
		 0.85,//27
		 0.78,//28
		 0.71,//29
		 0.64,//30
		 0.57,//31
		 0.5,//34
		 0.43,//32
		 0.34,//33
		 0.27,//34
		 0.2,//35
		 0.13,//36
		 0.6,//37
		 0,//38
		 0,//39
		 0,//40
		 0,//41
		 0,//44
		 0,//42
		 0,//43
		 0,//44
		 0,//45
		 0,//46
		 0,//47
		 0,//48
		 0,//49
		 0//50
};

//so, we have a resolution of 50. So these will be static, because they correspond to errors and deltas. So 25 is 0 error, 0 delta. 
//these structs are not 100 symmetrical but whatever.


//need a way to cast the continuous period counts into discreet errors and deltas. 
//so, take voltage that we get out of the convert. 330/6 = 55 so, take an incoming value, scale by 100, divide by 6. If 6 is an int then
//even if volt is a float we should get back an int. So this will work to break it up. We can cast any value larger than 50 onto 50. Or, we could
//rescale by dividing by 1.1 Then it gives a nice breakdown from 1-50. 

unscaled_voltage = get_voltage();
scaled_voltage = ((unscaled_voltage/1.1)*50)/(int 6);

scaled_set_point = ((unscaled_set_point/1.1)*50)/(int 6);

error = scaled_voltage -scaled_set_point ;//this value can go from -25 to plus 25, so we cant just look up the corresponding index on the table yet

prev_error = /*previous error value coming in from outside */// I dont know what any of these things are called outside the code so they will just
						//need to be brought in and scaled inside this function so we can index to the arrays. 

prev_scaled_error = 

delta = error - prev_error //

if (delta <0)
{
	delta_lookup=delta+25;
}
else if (delta >=0)
{
	delta_lookup=delta;
}

if (error <0)
{
	error_lookup=error+25;
]
else if (error >=0)
{
	error_lookup=error;
}


//so now we should be able to grab values from each of the tables



lp_delta_mu =  lp[delta_lookup];
sp_delta_mu = sp[delta_lookup];
ze_delta_mu = ze [delta_lookup];
sn_delta_mu = sn[delta_lookup];
ln_delta_mu = ln[delta_lookup];


lp_error_mu =  lp[error_lookup];
sp_error_mu = sp[error_lookup];
ze_error_mu = ze[error_lookup];
sn_error_mu = sn[error_lookup];
ln_error_mu = ln[error_lookup];



//Now we have to reference our rules:

//RULE1=========> IF error = ZE AND delta = ZE THEN output = ZE
rule1_mu = MIN(ze_delta_mu, ze_error_mu);
rule1_output = Z;
//...


//RULE2=========>IF error = ZE AND delta = SP THEN output = SN

rule2_mu = MIN(sp_delta_mu, ze_error_mu);
rule2_output = SN;
//...


 //RULE3=======> IF error = SN AND delta = SN THEN output = LP
rule3_mu = MIN(sn_delta_mu, sn_error_mu);
rule3_output = LP;
//...


//RULE4==========> IF error = LP OR  delta = LP THEN output = LN
rule4_mu = MAX(lp_delta_mu, lp_error_mu);
rule4_output = LN;

//Find centroid

centroid=(rule1_mu*rule1_output + rule2_mu*rule2_output + rule3_mu*rule3_output +rule4_mu+rule4_output)/(rule1_mu+rule2_mu+rule3_mu+rule4_mu);

//the output will ultimately be a voltage which will convert into a pwm signal. So, this number should be something to add or subtract from 
//the current output. PWM_OUT = PWM_OUT + centroid


PWM_OUT+=centroid;





