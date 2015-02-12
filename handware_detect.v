///This is a hardware pwd module, to be called by the nexys4 top level. It should take a reading out of the AXI timer and then give an input to the microblaze to be put out to the lcd and leds
// first draft will attmept a static out put to the seven seg through the microblaze. Apparently the hardware detect has to go through the microblaze :/ can directly hook up to the peripherals. 
//I guess hook up to the gpios? 

module PWM_detect #(	// parameters


	parameter integer	RESET_POLARITY_LOW		= 1,
	
	parameter integer	SIMULATE				= 0,
	parameter integer	SIMULATE_FREQUENCY_CNT	= 5
)
	(
input sensor_input,
input pwd_clk,
input sysreset,
input averaging_number,
output reg [31:0] output_up_count,	
output reg [31:0]output_down_count,
output reg [31:0]average_out 

);

	// Clock counters
	reg [31:0] 	down_count=32'b0,//pulse_width_up_counter = 32'b0,
				up_count=32'b0,//pulse_width_down_counter = 32'b0;
				up_count_next=32'b0,
				down_count_next=32'b0,
		          averaging_register = 32'b0;
	
    reg[8:0]    averaging_counter = 32'b0;
always @(posedge pwd_clk) begin
   if (sysreset) begin
   down_count_next<=0;
	down_count<=0;
	output_up_count<=0;
	averaging_register<=0;
	up_count_next<=0;
	end
    end
 always @(posedge pwd_clk) begin 
    if (sensor_input) begin 
    output_down_count<= down_count_next;
	averaging_register <= averaging_register+down_count_next;
    down_count<=0;
	
    up_count_next=up_count+1'b1;
    up_count<=up_count_next;  
	
	averaging_counter = averaging_counter+1'b1;
	
    
	end
	
	else if (~sensor_input) begin
	 
	averaging_register <= averaging_register+up_count_next;
    up_count<=0;
    down_count_next=down_count+1'b1;
    down_count<=down_count_next; 
	up_count<=up_count;
	average_out <=average_out ;
	output_up_count<=output_up_count;
	output_down_count<=output_down_count;
	end

	else begin
	down_count_next<=down_count_next;
	down_count<=down_count;
	 output_up_count<=output_up_count;
	 averaging_register<=averaging_register;
	up_count_next<=up_count_next;
	end
	 
	
	if (averaging_counter == averaging_number) begin //this should give the average count over 16 cycles, smoothing out noise. 
		averaging_counter<=0;
		averaging_register <= averaging_register >> 5; //should be dividing by 16
		average_out <= averaging_register; //check blocking vs non-blocking here. 
		averaging_register <=1'b0;
		end
	else begin
		averaging_register<=averaging_register;
		average_out<=average_out;
		end
	end	
endmodule		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		