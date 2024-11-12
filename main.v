module main(CLOCK_50, PS2_CLK, PS2_DAT);
input CLOCK_50;

reg[4:0]y, Y;

//wires inputs and outputs
wire clock;
wire reset;
wire startGame; //Assign to space bar
//assigns for IO
assign clock = CLOCK_50;

//Paddle variables
wire [7:0]paddleX, paddleY;
//Ball Variables
wire[7:0] ballX, ballY;
wire[2:0] vX, vY;
//Game Variables
wire[15:0]score;
wire[1:0]life;

//Keyboard modules
wire[7:0]keyP;
wire isPressed;

ps2input P1 (clock, reset, PS2_CLK, PS2_DAT, keyP, isPressed);

//FSM for game states
parameter mainMenu = 3'b0, level1 = 3'b001, endScreen = 3'b111;

always @ (*)
    case(y)
        mainMenu:
            if(startGame) Y <= level1;
            else Y <= mainMenu;
        level1: 
            if(life == 0) Y <= endScreen;
            else if // condition to move on to second level -> can determined by score or number of bricks
            else Y <= level1;
        endScreen:
    endcase

//FSM outputs
    always @ (*)
        VGA_COLOR = 3'b111, 

//FF update
always @(posedge clock)
    if(!resetn)
        y <= mainMenu;
    else
        y <= Y;


endmodule

module ps2input(clock, reset, PS2_CLK, PS2_DAT, keyD, keyP);
    input clock, reset, PS2_CLK, PS2_DAT;
    output reg [7:0] keyD;
    output keyP;

    wire [7:0]key_data;

    PS2_Controller PS2 (
	// Inputs
	.CLOCK_50(clock),
	.reset(reset),
	// Bidirectionals
	.PS2_CLK(PS2_CLK),
 	.PS2_DAT(PS2_DAT),
	// Outputs
	.received_data(key_data),
	.received_data_en(keyP)
    );
    //FF
    always @ (posedge clock)
        begin
        if(!resetn)
            keyD <= 8'h0;
        else if(keyP == 1'b1)
            keyD <= key_data;
        end

endmodule

module mainVGA(clock, VGA_X, VGA_Y, VGA_COLOR, plot, reset);
    input clock, reset;

    output [6:0] HEX3, HEX2, HEX1, HEX0;
	output [7:0] VGA_X;                     // for DESim VGA
	output [6:0] VGA_Y;                     // for DESim VGA
	output [2:0] VGA_COLOR;                 // for DESim VGA
	output plot;  

    always @ (*)


endmodule

module scorelife()
    
endmodule
