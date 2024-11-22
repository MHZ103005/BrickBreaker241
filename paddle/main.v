module main(SW, CLOCK_50, PS2_CLK, PS2_DAT, LEDR, KEY, VGA_R, VGA_G, VGA_B, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK, HEX5, HEX4, HEX2);
input CLOCK_50;
inout PS2_DAT, PS2_CLK;
input [3:0]KEY;
input[9:0]SW;

output [6:0]HEX5, HEX4, HEX2;
output[9:0]LEDR;

output [7:0] VGA_R;
output [7:0] VGA_G;
output [7:0] VGA_B;
output VGA_HS;
output VGA_VS;
output VGA_BLANK_N;
output VGA_SYNC_N;
output VGA_CLK;	


//wires inputs and outputs
wire clock;
wire reset;
wire startGame; //Assign to space bar
//assigns for IO
assign clock = CLOCK_50;
assign reset = KEY[0];

//Paddle variables
wire [7:0]paddleX, paddleY;
//Ball Variables
wire[7:0] ballX, ballY;
wire[2:0] vX, vY;
//Game Variables
wire[3:0]scored;
wire[1:0]life;
scoreHealth S1 (clock, reset, KEY[1], KEY[2], 7'd30, scored, life); 
//seg7 seg1 ({2'b0, scored[5:4]}, HEX5);
seg7 seg2 (scored, HEX4);
seg7 seg3 ({2'b0, life}, HEX2);

//Counter for paddle
wire tick;
DelayCounter d2 (clock, reset, tick);


//Keyboard modules
reg[7:0]keyP;
wire keyPressed;
parameter goo=1'b1, stop = 1'b0;
wire [7:0]key_data;
reg state;

PS2_Controller PS2 (.CLOCK_50(clock),.reset(~KEY[0]),.PS2_CLK(PS2_CLK),.PS2_DAT(PS2_DAT),	.received_data(key_data),.received_data_en(keyPressed));
//FF
always @ (posedge clock)
    begin
    if(!reset)
        begin
        state <= stop;
        keyP <= 8'h0;
        end
    else if(keyPressed == 1'b1)
    begin
        case(state)
        stop:
            if(key_data == 8'hF0)
                state <= goo;
            else begin 
                keyP <= key_data; 
                state <= stop; 
                end
        goo: 
            begin
            if(key_data == keyP) keyP <= 8'b0; 
            state <= stop;
            end
        endcase
    end
    end
keyboardtest k1 (keyP, LEDR[6:0]);
//VGA variables
parameter A = 3'b000, B = 3'b001, C = 3'b010, D = 3'b011; 
    parameter E = 3'b100, F = 3'b101, G = 3'b110, ERASE = 3'b111; 
    parameter XSCREEN = 160, YSCREEN = 120;
    parameter XDIM = 20, YDIM = 1;
    parameter X0 = 8'd39;
    parameter ALT = 3'b000; // alternate object color
    parameter K = 20; // animation speed: use 20 for hardware, 2 for ModelSim

wire [7:0] VGA_X; 
wire [6:0] VGA_Y;  
reg [2:0] VGA_COLOR;
reg plot;

wire [2:0] colour;
wire [7:0] X, Z;
wire [6:0] Y;
wire [7:0] XC;
wire [6:0] YC;
wire [K-1:0] slow;
wire go, sync;
reg Ly, Ey, Lxc, Lyc, Exc, Eyc;
wire Xdir;
reg Tdir;
reg [2:0] y_Q, Y_D;
wire [7:0] eraseX;


assign colour = SW[2:0];

UpDn_count U3 (8'd0, CLOCK_50, KEY[0], Exc, Lxc, 1'b1, XC);
        defparam U3.n = 8;
UpDn_count U5 ({K{1'b0}}, CLOCK_50, KEY[0], 1'b1, 1'b0, 1'b1, slow);
        defparam U5.n = K;
    assign sync = (slow == 0);
counter160 C12 (clock, reset, eraseX, Tdir);

updatePosition D1(clock, X, keyP, tick);

always @ (*)
        case (y_Q)
            A:  if (!go || !sync) Y_D = A;
                else Y_D = B; 
            B:  if (XC != XDIM-1) Y_D = B;    // draw
                else Y_D = C; 
            C:  if (YC != YDIM-1) Y_D = B;
                else Y_D = D;
            D:  if (!sync) Y_D = D;
                else Y_D = E;
            // ERASE: if(eraseX != XDIM - 1) Y_D = ERASE;
            //     else Y_D = E;
            E:  if (eraseX != XDIM-1) Y_D = E;    // erase
                else Y_D = F;
            F:  if (YC != YDIM-1) Y_D = E;
                else Y_D = G;
            G:  Y_D = B;
        endcase

always @ (*)
    begin
        // default assignments
        Lxc = 1'b0; Exc = 1'b0; VGA_COLOR = colour; plot = 1'b0; Tdir = 1'b0;
        case (y_Q)
            A:  begin Lxc = 1'b1; end
            B:  begin Exc = 1'b1; plot = 1'b1; end   // color a pixel
            C:  begin Lxc = 1'b1; end
            D:  Lyc = 1'b1;
            E:  begin Tdir = 1'b1; VGA_COLOR = ALT; plot = 1'b1; end   // color a pixel
            F:  begin Lxc = 1'b1;  end
        endcase
    end

    always @(posedge CLOCK_50)
        if (!KEY[0])
            y_Q <= 1'b0;
        else
            y_Q <= Y_D;

    assign go = ~KEY[3];

    assign VGA_X = X + XC + eraseX;
    assign VGA_Y = 7'd100;
    // connect to VGA controller
    vga_adapter VGA (.resetn(KEY[0]),.clock(CLOCK_50),.colour(VGA_COLOR),.x(VGA_X),.y(VGA_Y),.plot(plot),.VGA_R(VGA_R),.VGA_G(VGA_G),.VGA_B(VGA_B),.VGA_HS(VGA_HS),.VGA_VS(VGA_VS),.VGA_BLANK_N(VGA_BLANK_N),.VGA_SYNC_N(VGA_SYNC_N),.VGA_CLK(VGA_CLK));
        defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif"; 
//FSM for game states
parameter mainMenu = 3'b000, level1 = 3'b001, endScreen = 3'b111;

// always @ (*)
//     case(y)
//         mainMenu:
//             if(startGame) Y <= level1;
//             else Y <= mainMenu;
//         level1: 
//             if(life == 0) Y <= endScreen;
//             else if // condition to move on to second level -> can determined by score or number of bricks
//             else Y <= level1;
//         endScreen:
//     endcase

// //FSM outputs
//     always @ (*)
//         VGA_COLOR = 3'b111, 

// //FF update
// always @(posedge clock)
//     if(!resetn)
//         y <= mainMenu;
//     else
//         y <= Y;


endmodule

module ps2input(clock, reset, PS2_CLK, PS2_DAT, keyD, keyP, lights);
    input clock, reset;
    inout PS2_CLK, PS2_DAT;
    output reg [7:0] keyD;
    output keyP;
    output [6:0]lights;

    parameter go=1'b1, stop = 1'b0;

    wire [7:0]key_data;
    reg state;

    PS2_Controller PS2 (.CLOCK_50(clock),.reset(reset),.PS2_CLK(PS2_CLK),.PS2_DAT(PS2_DAT),	.received_data(key_data),.received_data_en(keyP));
    //FF
    always @ (posedge clock)
        begin
        if(!reset)
            begin
            state <= stop;
            keyD <= 8'h0;
            end
        else if(keyP == 1'b1)
        begin
            case(state)
            stop:
                if(key_data == 8'hF0)
                    state <= go;
                else begin 
                    keyD <= key_data; 
                    state <= stop; 
                    end
            go: 
                begin
                if(key_data == keyD) keyD <= 8'b0; 
                state <= stop;
                end
            endcase
        end
        end
endmodule

module UpDn_count (R, Clock, Resetn, E, L, UpDn, Q);
    parameter n = 8;
    input [n-1:0] R;
    input Clock, Resetn, E, L, UpDn;
    output reg [n-1:0] Q;

    always @ (posedge Clock)
        if (Resetn == 0)
            Q <= 0;
        else if (L == 1)
            Q <= R;
        else if (E)
            if (UpDn == 1)
                Q <= Q + 1;
            else
                Q <= Q - 1;
endmodule

module updatePosition(clock, X, keyPressed, tick);
    input clock, tick;
    input [7:0] keyPressed;
    output reg [7:0]X;

    always @ (posedge clock)
    begin
    if(keyPressed == 8'h23 & tick ) X <= X + 1;
    else if(keyPressed == 8'h1C & tick ) X <= X - 1;
    if(X < 1) X <= 1;
    if(X > 8'd119) X <= 8'd119;
    end
endmodule

module keyboardtest(input wire [7:0]storedLetterCode, output reg [6:0]lights);
always@(*)
begin
if(storedLetterCode == 8'h23) lights = 7'b1000000;
else if(storedLetterCode == 8'h1C) lights = 7'b0100000;
else lights = 7'b0;
end
endmodule

module DelayCounter(clock, reset, Q);
    input clock, reset;
    output reg Q;

    wire [25:0] slowcount;
    slowCounter S1 (clock, reset, slowcount);
    always @ (posedge clock)
    begin
        if(!reset) Q <= 0;
        if(slowcount == 26'd625000) Q <= Q + 1;
        else Q <= 0;
    end
endmodule

module slowCounter(clock, resetn, slowcount);
    input clock, resetn;
    output reg [25:0]slowcount;


    always @ (posedge clock)
    begin
        if(!resetn)
            slowcount <= 26'b0;
        else if(slowcount > 26'd625000)
            slowcount <= 26'b0;
        else
            slowcount <= slowcount + 1;
    end
endmodule

module counter160 (clock, reset, Q, enable);
    input clock, reset, enable;
    output reg [7:0]Q;

    always @(posedge clock)
        begin
        if(!reset) Q <= 8'b0;
        if(enable) Q <= Q + 1;
        end
endmodule

module scoreHealth(clock, reset, cBrickX, cBrickY, ballY, score, life); // add move and set it to 0 when ball goes below 0
    input clock, reset, cBrickX, cBrickY;
    input [6:0] ballY;
    output reg [3:0] score;
    output reg[1:0] life;
    always @ (posedge clock)
    begin
        if(!reset)
            score <= 4'b0000;
            life <= 2'b11;
        if(cBrickX | cBrickY) score <= score + 1;
        if(ballY < 20) life <= life -1;
    end
endmodule

module seg7(inputNum, Display);

input [3:0] inputNum;
output reg [6:0] Display;

    always @(*)
    begin
        if (inputNum == 4'b0000)
            Display = 7'b1000000;  // 0
        else if (inputNum == 4'b0001)
            Display = 7'b1111001;  // 1
        else if (inputNum == 4'b0010)
            Display = 7'b0100100;  // 2
        else if (inputNum == 4'b0011)
            Display = 7'b0110000;  // 3
        else if (inputNum == 4'b0100)
            Display = 7'b0011001;  // 4
        else if (inputNum == 4'b0101)
            Display = 7'b0010010;  // 5
        else if (inputNum == 4'b0110)
            Display = 7'b0000010;  // 6
        else if (inputNum == 4'b0111)
            Display = 7'b1111000;  // 7
        else if (inputNum == 4'b1000)
            Display = 7'b0000000;  // 8
        else if (inputNum == 4'b1001)
            Display = 7'b0010000;  // 9
        else if (inputNum == 4'b1010)
            Display = 7'b0001000;  // A
        else if (inputNum == 4'b1011)
            Display = 7'b0000011;  // B
        else if (inputNum == 4'b1100)
            Display = 7'b1000110;  // C
        else if (inputNum == 4'b1101)
            Display = 7'b0100001;  // D
        else if (inputNum == 4'b1110)
            Display = 7'b0000110;  // E
        else if (inputNum == 4'b1111)
            Display = 7'b0001110;  // F
        else
            Display = 7'b1111111;  // All Display off
    end
endmodule