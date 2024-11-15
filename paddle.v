module paddle(clock, in,inEnable, length, paddleX, state, resetn, VGA_R, VGA_G,
			  VGA_B, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK);
    parameter A = 3'b000, B = 3'b001, C = 3'b010, D = 3'b011; 
    parameter E = 3'b100, F = 3'b101, G = 3'b110, H = 3'b111;
    input length, clock, resetn, inEnable;
    input [2:0]state;
    input [7:0] in; //Double Check size of input
    output reg [7:0] paddleX;
    reg [2:0] y_Q, Y_D;
    wire [7:0] VGA_X;
	wire [6:0] VGA_Y;
	reg [2:0] VGA_COLOR;


    always@(posedge clock) // we are going to need a slower counter for this
        begin
        if(!resetn ) paddleX <= 8'b0;/*default*/
        else if((state == 3'b001 | state = 3'b010) & inEnable)
            begin
            if(in == 8'b0;) // change to right input
                if(paddleX + length < 160) paddleX <= paddleX + 1;
            else if(in == 8'b1) // change to left input
                begin
                    if(paddleX > 0) paddleX <= paddleX - 1;
                end
            end
        end
    
    always @ (*)
        case (y_Q)

        endcase

    assign VGA
    vga_adapter VGA (
			.resetn(reset),
			.clock(clock),
			.colour(VGA_COLOR),
			.x(ballX),
			.y(ballY),
			.plot(1'b1),
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK_N(VGA_BLANK_N),
			.VGA_SYNC_N(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";

endmodule