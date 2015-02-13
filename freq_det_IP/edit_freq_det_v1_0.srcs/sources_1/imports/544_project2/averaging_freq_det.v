`timescale 1ns / 1ps

// Created by Colten Nye

module freq_det(
    input clk,
    input reset_n,
    input freq_signal,
    output reg [31:0] result,
    input [7:0] divisor
);
    
    reg [31:0] sum = 0;
    reg [7:0]  curr_per = 0;
	reg [31:0] counter = 0;
	reg counting_high = 0;			// Flag used for determining if the signal just transitioned
	
	always @(posedge clk) begin
		if (reset_n == 1'b0) begin
            sum <= 0;
            curr_per <= 0;
            result <= 0;
			counter <= 0;
            counting_high <= 0;
		end
		
		// Signal is high
		else if (freq_signal) begin
			// Signal just switched from low
			if (~counting_high) begin
				counting_high <= 1'b1;			// Toggle the flag
                sum <= sum + counter;
                counter <= 1;
                curr_per <= curr_per + 1'b1;
			end
            else begin
                if ( curr_per == ( 1 << divisor) ) begin
                    result <= ( sum >> divisor );
                    sum <= 0;
                    curr_per <= 8'h00;
                end
                counter <= counter + 1'b1;	// Increment high counter
            end
		end
		
		// Signal is low
		else begin
			// Signal just switched from high
			if (counting_high) begin
				counting_high <= 1'b0;			// Toggle the flag
			end
			counter <= counter + 1'b1;	// Increment low counter
		end	
	end
endmodule
