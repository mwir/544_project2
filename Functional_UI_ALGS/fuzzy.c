
//we have 5 output possibilities, each one corresponding to an additional drive on the PWM signal

#define Z 0
#define LIGHTEN .5
#define BRIGHTEN  1
#define DIM  -.5
#define DARKEN  -1

//the above output values  can be fiddled with to make sense. Even though these are hard values here they will only be used in fuzzy calculations.
// the idea behind what will happen is the following: A hard value comes in as the error and change in error from the detector. These value
//is then assigned a series of 'mu' values, telling how similar the hard value is to certain fuzzily defined sets. We  have called these
//sets by the comparative adjectives  'darker, dimmer, z, lighter, brighter' for our LED model. 

//The responses to these cases are given by the corresponding verbs. 

// macro functions
#define MIN(a, b)  ( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  ( ((a) >= (b)) ? (a) : (b) )

//DARKEN<DIM<Z<LIGHTEN<BRIGHTEN



//the following structs return mu values for different error and delta values we can get from our reading. We have 4 outputs we can give,
//small positive (lighter), large positive (brighter), small negative (dimmer), large negative (darker) and zero (z). 

float lighter[50] = {
		 0.06,//1
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
		 0.060,//27
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

float brighter[50]={
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
		 0.06,//14
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
float darker[50]={
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
		 0.06,///38
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




float dimmer[50]={
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
		 0.06,///25
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
		 0.06,//49
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
		 0.06,//12
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
		 0.06,//37
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
//these arrays are not 100 symmetrical but whatever.


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

prev_scaled_error = //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
		+++++^^^^^^^^^^^^^^^NEED TO MATCH VARIABLES
		
*/ //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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



brighter_delta_mu =  brighter[delta_lookup];
lighter_delta_mu = lighter[delta_lookup];
z_delta_mu = z[delta_lookup];
dimmer_delta_mu = dimmer[delta_lookup];
darker_delta_mu = darker[delta_lookup];


brighter_error_mu =  brighter[error_lookup];
lighter_error_mu = lighter[error_lookup];
z_error_mu = z[error_lookup];
dimmer_error_mu = dimmer[error_lookup];
darker_error_mu = darker[error_lookup];



//Now we have to reference our rules:

//RULE1=========> IF error = z AND delta = z THEN output = Z
rule1_mu = MIN(z_delta_mu, z_error_mu);
rule1_output = Z;
//...

//RULE2=========>IF error = z AND delta = lighter THEN output = DIM

rule2_mu = MIN(lighter_delta_mu, z_error_mu);
rule2_output = DIM;
//...

 //RULE3=======> IF error = dimmer AND delta = dimmer THEN output = BRIGHTEN
rule3_mu = MIN(dimmer_delta_mu, dimmer_error_mu);
rule3_output = LIGHTEN;
//...

//RULE4==========> IF error = brighter OR  delta = brighter THEN output = DARKEN
rule4_mu = MAX(brighter_delta_mu, brighter_error_mu);
rule4_output = DARKEN;

//RULE5==========> IF error = darker OR  delta = darker THEN output = BRIGHTEN
rule5_mu = MAX(darker_delta_mu, darker_error_mu);
rule5_output = BRIGHTEN;

//RULE6=========>IF error = z AND delta = darker THEN output = LIGHTEN
rule6_mu = MIN(darker_delta_mu, z_error_mu);
rule6_output = LIGHTEN;
//...

 //RULE7=======> IF error = lighter AND delta = lighter THEN output = BRIGHTEN
rule7_mu = MIN(lighter_delta_mu, lighter_error_mu);
rule7_output = DIM;
//...

//Find centroid

centroid=(rule1_mu*rule1_output + rule5_mu*rule5_output + rule2_mu*rule2_output + rule3_mu*rule3_output +rule4_mu+rule4_output + rule6_mu*rule6_output+rule7_mu*rule7_output)/(rule1_mu+rule2_mu+rule3_mu+rule4_mu + rule5_mu +rule6_mu+rule7_mu);

//the output will ultimately be a voltage which will convert into a pwm signal. So, this number should be something to add or subtract from 
//the current output. PWM_OUT = PWM_OUT + centroid


PWM_OUT+=centroid;





