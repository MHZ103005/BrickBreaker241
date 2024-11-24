`timescale 1ns / 1ps

module testbench ( );

	parameter CLOCK_PERIOD = 10;

    reg clock, reset, cX, cY, in;
    reg [2:0] state;
	wire signed [2:0] vX, vY;
	wire [7:0] nextX, nextY;
	wire move;

	initial begin
        clock <= 1'b0;
	end // initial
	
	always @ (*)
	begin : Clock_Generator
		#((CLOCK_PERIOD) / 2) clock <= ~clock;
	end
	
	initial begin
        reset <= 1'b0;
        #10 reset <= 1'b1;
	end // initial

	initial begin
        state <= 3'b001;
		in <= 1'b0;
		cX <= 1'b0;
		cY <= 1'b0;
		
		#10 in <= 1'b1;
		#20 cX <= 1'b1;
		#10 cX <= 1'b0; cY <= 1'b1;
		#10 cY <= 1'b0;
	end // initial
	ball U1 (clock, reset, state, cX, cY, nextX, nextY, vX, vY, in);

endmodule
