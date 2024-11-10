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
	.CLOCK_50				(clock),
	.reset				    (reset),

	// Bidirectionals
	.PS2_CLK			(PS2_CLK),
 	.PS2_DAT			(PS2_DAT),

	// Outputs
	.received_data		(key_data),
	.received_data_en	(keyP)
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

module paddle(clock, in, length, paddleX, state, resetn);
    input length, clock, resetn;
    input [2:0]state;
    input [7:0] in; //Double Check size of input
    output reg [7:0] paddleX;
    always@(posedge clock) // we are going to need a slower counter for this
        begin
        if(!resetn ) paddleX <= 8'b0;/*default*/
        else if(state == 3'b001 | state = 3'b010)
            begin
            if(in == 8'b0;)
                if(paddleX + length < 160) paddleX <= paddleX + 1;
            else if(in == 8'b1)
                begin
                if(paddleX > 0) paddleX <= paddleX - 1;
                end
            end
        end
endmodule

module ball(clock, reset, state, cX, cY, nextX, nextY, vX, vY, in); //velocity may be required, or we can just hard code it
    input clock, reset, cX, cY;
    input in;
    input [2:0]state;
    output reg signed [2:0] vX, vY;
    output reg [7:0] nextX, nextY;

    reg move;


    //FSM for ball state
    always @ (*)
        case(state)
            mainMenu: move = 1'b0;
            level1: if(in == /* space*/) move = 1'b1;
            endScreen: move = 1'b0;
        endcase

    always @(posedge clock) // will need slower counter for this
        if(move)
            begin
            if(cX | cBrickX) vX <= -vX;
            if(cY | cBrickY) vY <= -vY;
            ballX <= ballX + vX;
            ballY <= ballY + vY;
            end
        else    
            begin
            vX <= 3'b001;
            vY <= 3'b001;
            ballX <= /*default x and y*/;
            ballY <= ;
            end

endmodule

module collision(ballX, ballY, paddleX, paddleY, /*brick positions somehow*/, vX, vY, cPaddleX, cPaddleY, cBrickX, cBrickY, cWall, cRoof);
    input [7:0]ballX, ballY, paddleX, paddleY;
    input [2:0]vX, vY;
    output reg cX, cY, cBrickX, cBrickY

    always @(*)
        begin
        cX = 0;
        cY = 0;
        //Boundary Collision
        if(ballX <= 0 | ballX > 120) cX = 1; // left right wall
        if(ballY <= 0) cY = 1; // roof
        //Paddle collision
        if((ballX + vX >= paddleX) & (ballX + vX <= paddleX + length) & ballY + vY >= paddleY) cY = 1; // can add side collision for paddle
        //Collision with bricks
        
        end
endmodule

module scorelife()
    
endmodule