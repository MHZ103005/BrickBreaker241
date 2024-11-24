`timescale 1ns / 1ps

module testbench ( );

	parameter CLOCK_PERIOD = 10;

    reg clock, length, resetn, inEnable;
    reg [2:0] state;
	reg [7:0] in;
    wire [7:0] paddleX;

	initial begin
        clock <= 1'b0;
	end // initial
	
	always @ (*)
	begin : Clock_Generator
		#((CLOCK_PERIOD) / 2) clock <= ~clock;
	end
	
	initial begin
        resetn <= 1'b0;
        #10 resetn <= 1'b1;
	end // initial

	initial begin
        state <= 3'b000;
		length <= 20;
		in <= 8'b0;
		inEnable <= 1'b1;
		
		#20 state <= 3'b001;
		#20 in <= 8'b11111111;
		#20 in <= 8'b00000001;
		#20 state <= 3'b000;
		#10 state <= 3'b001; inEnable <= 1'b0; in <= 8'b11111111;
	end // initial
	paddle U1 (clock, in, inEnable, length, paddleX, state, resetn);

endmodule
