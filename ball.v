module main(CLOCK_50, KEY, VGA_R, VGA_G, VGA_B,
				VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK);
    input CLOCK_50;
	input [3:0] KEY;

    wire [7:0]ballX, ballY, paddleX, paddleY;
    wire [2:0] vX, vY;
    wire [2:0] cX, cY;

    collision U1 (ballX, ballY, paddleX, paddleY, vX, vY, cX, cY);
    ball U2 (CLOCK_50, KEY[0], cX, cY, vX, vY, VGA_R, VGA_G, VGA_B, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK);

endmodule

module ball(clock, reset, /*state*/, cX, cY, ballX, ballY, vX, vY, /*in*/, VGA_R, VGA_G, VGA_B,
				VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK); //velocity may be required, or we can just hard code it
    input clock, reset, cX, cY;
    //input in;
    //input [2:0]state;
    output reg [2:0] vX, vY;
    output reg [7:0] ballX, ballY;
    reg plot;
    wire move;
    reg y_Q, Y_D;
    parameter draw = 1'b0, erase = 1'b1;
    wire tick;
	reg [2:0] VGA_COLOR; // for DESim VGA

    DelayCounter D1 (clock, reset, tick);

    // //FSM for ball state
    // always @ (*)
    //     case(state)
    //         mainMenu: move = 1'b0;
    //         level1: if(in == /* space*/) move = 1'b1;
    //         endScreen: move = 1'b0;
    //     endcase
    assign move = 1'b1;
    // FSM for animation
    always @ (*)
        case (y_Q)
            draw: if (draw) Y_D = erase;
                  else Y_D = draw;
            erase: if (erase) Y_D = draw;
                   else Y_D = erase;
        endcase
    // animation FSM outputs
    always @ (*)
    begin
        plot = 1'b1;
        case (y_Q)
            draw: VGA_COLOR = 3'b111;
            erase: VGA_COLOR = 3'b000;
        endcase
    end

    always @(posedge clock) // will need slower counter for this
        if(move)
            begin
            if(cX /*| cBrickX*/) vX[3] <= ~vX[3];
            if(cY /*| cBrickY*/) vY[3] <= ~vY[3];
            if(~vX[3]) ballX <= ballX +vX;
            if(vX[3]) ballX <= ballX - vX;
            if(~vY[3]) ballY <= ballY +vY;
            if(vY[3]) ballY <= ballY - vY;
            end
        else    
            begin
            vX <= 3'b001;
            vY <= 3'b001;
            ballX <= 20; // temporary
            ballY <= 20; // temporary
            end

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