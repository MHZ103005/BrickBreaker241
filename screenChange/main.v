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

//counter for paddle
wire tick;
DelayCounter d2 (clock, reset, tick);

//FSM for background drawing
parameter state1 = 3'b0, paddleDraw = 3'b001, state3 = 3'b010, drawBall = 3'b011, paddleDraw2 = 3'b100, drawBall2 = 3'b101;
reg [2:0] currentDraw, ns;
reg doneP, doneB;
always @(*)
    case(currentDraw)
        state1: if(address == 15'd19199) ns = paddleDraw;
                else ns = state1;
        paddleDraw: if(address == 15'd19199) ns = drawBall;
                else ns = paddleDraw;
        drawBall: if(address == 15'd19199) ns = state3;
                else ns = drawBall;
        state3: if(address == 15'd19199) ns = paddleDraw;
                else ns = state3;
        // waitState: if(fps60 == 20'd0 |fps60 == 20'd208333 | fps60 == 20'd416666 | fps60 == 20'd625000) ns = paddleDraw;
        //             else ns = waitState;
    endcase

always @(posedge clock)
    if(!reset) currentDraw <= state1;
    else currentDraw <= ns;

always @(*)
    case(currentDraw)
        state1: begin
                plot <= 1'b1;
                VGA_Xalt <= Xb;
                VGA_Yalt <= Yb; 
                VGA_COLOR <= currentScene; end 
        paddleDraw: begin 
                plot <= 1'b1;
                VGA_Xalt <= X + XC;
                VGA_Yalt <= 7'd100; 
                VGA_COLOR <= 3'b101;end
        drawBall:begin VGA_Xalt <= ballX;
                VGA_Yalt <= ballY;
                VGA_COLOR <= 3'b111; end
        state3: begin
                VGA_Xalt = Xb;
                VGA_Yalt <= Yb;
                plot <= 1'b1;
                VGA_COLOR <= 3'b000;
                end
    endcase

parameter A = 3'b000, B = 3'b001, C = 3'b010, D = 3'b011; 
parameter E = 3'b100, F = 3'b101, G = 3'b110, ERASE = 3'b111; 
parameter XSCREEN = 160, YSCREEN = 120;
parameter K = 20; // animation speed: use 20 for hardware, 2 for ModelSim

wire [7:0] VGA_X; 
wire [6:0] VGA_Y;  
reg [2:0] VGA_COLOR;
reg plot;

wire [2:0] colour;
wire [7:0] X, Z;
wire [6:0] Y;
wire [4:0] XC1, XC2;
wire [7:0] XC;
wire [7:0] YC;
wire [K-1:0] slow;
wire go, sync;
reg Ly, Ey, Lxc, Lyc, Exc, Eyc;
wire Xdir;
reg Tdir;
reg [2:0] y_Q, Y_D;
wire [7:0] eraseX;

reg [7:0] VGA_Xalt; 
reg [6:0] VGA_Yalt;  

wire [2:0] colour1, colour2, colour3, currentScene;
wire [1:0] scene;

MainMenu rom1 (address, clock, colour1);
win rom2 (address, clock, colour2);
losegame rom3 (address, clock, colour3);

muxScreens mux1 (colour1, colour2, colour3, scene, currentScene);

assign scene = SW[9:8];

wire [7:0] Xb;
wire [6:0] Yb;
wire[14:0] address;
//for background
xyscroll x1 (clock, reset, Xb, Yb, 1'b1);
addressScroll x2 (clock, reset, address, 1'b1);

//for paddle
XCcounter x3 (clock, reset, currentDraw == paddleDraw | currentDraw == paddleDraw2, XC);
UpDn_count U5 ({K{1'b0}}, CLOCK_50, KEY[0], 1'b1, 1'b0, 1'b1, slow);
        defparam U5.n = K;
assign sync = (slow == 0);
assign go = ~KEY[3];
updatePosition D1(clock, X, keyP, tick, 1'b1);

always @ (*)
        if(currentDraw == paddleDraw)
        case (y_Q)
            A:  if (!go || !sync) Y_D = A;
                else Y_D = B; 
            B:  if (XC != 20) Y_D = B;    // draw
                else Y_D = C; 
            C: Y_D = G;
            G: begin end
        endcase

always @ (*)
    if(currentDraw == paddleDraw)
    begin
        // default assignments
        Lxc = 1'b0; Exc = 1'b0; Tdir = 1'b0; Eyc = 1'b0; doneP = 1'b0;
        case (y_Q)
            A:  begin Lxc = 1'b1; end
            B:  begin Exc = 1'b1; end   // color a pixel
            C:  begin Lxc = 1'b1; Eyc = 1'b1;end
            G:  doneP = 1'b1;
        endcase
    end

always @(posedge CLOCK_50)
        if (!KEY[0])
            y_Q <= 1'b0;
        else
            y_Q <= Y_D;

//Ball drawing
wire [7:0] ballX;
wire [6:0] ballY;
wire [7:0] preX;
wire [6:0] preY;
wire [2:0] vX, vY;
wire cX, cY;
reg [2:0] y_Qball, Y_Dball;
wire move;
assign move = SW[7];

wire [19:0] fps60;
frame60 F1 (CLOCK_50, KEY[0], fps60);
ballMove B1 (CLOCK_50, fps60==20'd0, ballX, ballY, vX, vY, cX, cY, preX, preY, move, X);

// FSM state table
always @ (*)
    case (y_Qball)
        A:  if (!go || !sync) Y_Dball = A;
            else Y_Dball = B;
        B:  if (XC != 1) Y_Dball = B;    // draw
            else Y_Dball = C;
        C:  if (YC != 1) Y_Dball = B;
            else Y_Dball = G;
        G:  begin end
    endcase
// FSM outputs
always @ (*)
begin
    // default assignments
    Lxc = 1'b0; Lyc = 1'b0; Exc = 1'b0; Eyc = 1'b0; doneB = 1'b0;
    case (y_Qball)
        A:  begin Lxc = 1'b1; Lyc = 1'b1; end
        B:  begin Exc = 1'b1;  end   // color a pixel
        C:  begin Lxc = 1'b1; Eyc = 1'b1; end
        G:  doneB = 1'b1;
    endcase
end

always @(posedge clock)
    if (!KEY[0])
        y_Qball <= 1'b0;
    else
        y_Qball <= Y_Dball;

//Brick
wire [3:0] brickW;
wire [2:0] brickH;
wire[7:0]brickX;
wire[6:0]brickY;
reg [3:0] WC;
reg [2:0] HC;
reg Read_address;
reg [39:0] brickedUP;

// count through read addresses
always @ (posedge Clock)
        if (reset == 0 || Read_address == 39) Read_address <= 0;
        else Read_address <= Read_address + 1;

// instantiate memory module
ram40x22 BG1 (Clock, Read_address, DataOut);

// get the data of a brick from memory
assign {brickX, brickY, brickW, brickH} = DataOut;

always @ (posedge clock)
begin
    if (reset) begin WC <= 0; HC <= 0; brickedUP <= {40{1'b1}}; end
    else
        WC <= WC + 1;
        if (WC == brickW)
        begin
            WC <= 0;
            if (HC == brickH) HC <= 0;
            else HC <= HC + 1;
        end

    //outX <= brickX + WC;
   // outY <= brickY + HC;
    //outCOLOR <= 3'b000;
   // plot <= 1'b1;
    if (brickedUP[Read_address])
        if ((((vY[2] & ballY + vY[0] == brickY) | (!vY[2] & ballY - vY[0] == brickY + brickH)) & ((ballX + vX[0] > brickX & !vX[2] & ballX +vX[0] < brickX + brickW) | (ballX - vX[0] > brickX & vX[2] & ballX < brickX + brickW))) | (((vX[2] & ballX + vX[0] == brickX) | (!vX[2] & ballX - vX[0] == brickX + brickW)) & ((ballY + vY[0] > brickY & !vY[2] & ballY +vY[0] < brickY + brickH) | (ballY - vY[0] > brickY & vY[2] & ballY < brickY + brickH))) && brickedUP[Read_address])            begin
            brickedUP[Read_address] = 0;
            cX <= 1;
            cY <= 1;
        end

end



assign VGA_X = VGA_Xalt;
assign VGA_Y = VGA_Yalt;





    // connect to VGA controller
vga_adapter VGA (
        .resetn(KEY[0]),
        .clock(CLOCK_50),
        .colour(VGA_COLOR),
        .x(VGA_X),
        .y(VGA_Y),
        .plot(plot ),
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
endmodule

module muxScreens(c1, c2, c3, scene, Q);
    input [2:0]c1, c2, c3;
    input [1:0] scene;
    output reg [2:0]Q;

    always @ (*)
    begin
        if(scene == 2'b00)
            Q <= c1;
        else if(scene == 2'b01)
            Q <= c2;
        else if(scene == 2'b10)
            Q <= c3;
    end
endmodule

module xyscroll(clock, resetn, X, Y, enable);
    output reg [8:0] X;
    output reg [7:0] Y;
    input clock, resetn, enable;
    always @(posedge clock)
        if(!resetn)
        begin
            X <= 0;
            Y <= 0;
            end
        else if(enable)
        if(Y == 7'd119 && X == 8'd159)
            begin
            X <= 0;
            Y <= 0;
            end
        else if(X == 8'd159)
            begin
            X <= 0;
            Y <= Y + 1'b1;
            end
        else 
            X <= X + 1'b1;
endmodule

//cycles through 320x240 addresses since address_translator is currently under debug
module addressScroll(clock, resetn, address, enable);
    output reg [14:0] address;
    input clock, resetn, enable;
    always @(posedge clock)
        if(!resetn)
            address <= 0;
        else if(enable)
            if(address == 15'd19199)
            address <= 0;
            else
            address <= address + 1'b1;
endmodule

module updatePosition(clock, X, keyPressed, tick, enable);
    input clock, tick, enable;
    input [7:0] keyPressed;
    output reg [7:0]X;

    always @ (posedge clock)
    begin
    if(keyPressed == 8'h23 & tick ) X <= X + 1;
    else if(keyPressed == 8'h1C & tick ) X <= X - 1;
    if(X < 1) X <= 1;
    if(X  > 8'd139) X <= 8'd139;
    end
endmodule

module XCcounter(clock, reset, E, Q);
    input clock, reset, E;
    output reg [7:0]Q;

    always @(posedge clock)
    begin
        if(!reset)
            Q <= 0;
        else if(E) begin
            Q <= Q+1;
            if(Q > 19) Q <= 19; end
        else Q <= 0;
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

module frame60 (input clock, input reset, output reg [19:0] FPS);
    always @ (posedge clock)
    begin
        if (!reset || FPS == 20'd833332) FPS <= 0; // 20'd833332
        else FPS <= FPS + 1;
    end
endmodule

module ballMove (input clock, input moveEN, output reg [7:0] ballX, output reg [6:0] ballY, output reg [2:0] vX, vY, output cX, cY, output reg [7:0] preX, output reg [6:0] preY, input move, input [7:0]paddleX);
    assign cX = (((ballX-vX[0] == 0) || (ballX-vX[0] == 159)) && vX[2]) || (((ballX+vX[0] == 0) || (ballX+vX[0] == 159)) && ~vX[2]);
    assign cY = (((ballY-vY[0] == 0) || (ballY-vY[0] == 119)) && vY[2]) || (((ballY+vY[0] == 0) || (ballY+vY[0] == 119)) && ~vY[2]) || (ballY + vY[0] == 7'd100 & !vY[2]) & ((ballX + vX[0] > paddleX & !vX[2] & ballX +vX[0] < paddleX + 19) | (ballX - vX[0] > paddleX & vX[2] & ballX < paddleX + 19 ));
    always @ (posedge clock)
    begin
        if (!move) begin ballX <= 8'd79; ballY <= 7'd99; preX <= 8'd79; preY <= 7'd99; vX <= 3'b101; vY <= 3'b101; end
        if (moveEN)
        begin
            if (cX)
            begin
                if (ballX >=  150) ballX <= ballX - 1;
                vX[2] <= vX[2]^cX;
            end
            else if (cY)
            begin
                if (ballY >=  110) ballY <= ballY - 1;
                vY[2] <= vY[2]^cY;
            end
            preX <= ballX;
            preY <= ballY;
            if (vX[2])
                ballX <= ballX-vX[0];
            else
                ballX <= ballX+vX[0];
            if (vY[2])
                ballY <= ballY-vY[0];
            else
                ballY <= ballY+vY[0];
        end
    end
endmodule

module brick (input clock, input [7:0] ballX, input [6:0] ballY, output wire [7:0] brickX, output wire [6:0] brickY, output reg cX, cY, output reg [7:0] outX, output reg [6:0] outY, output reg [2:0] outCOLOR, input resetn);
    wire [3:0] brickW;
    wire [2:0] brickH;
    reg [3:0] WC;
    reg [2:0] HC;
    reg Read_address;
    reg [39:0] brickedUP;
    
    // count through read addresses
    always @ (posedge Clock)
            if (Resetn == 0 || Read_address == 39) Read_address <= 0;
            else Read_address <= Read_address + 1;

    // instantiate memory module
    ram40x22 BG1 (Clock, Read_address, DataOut);

    // get the data of a brick from memory
    assign {brickX, brickY, brickW, brickH} = DataOut;

    always @ (posedge clock)
    begin
        if (Resetn) begin WC <= 0; HC <= 0; brickedUP <= {40{1'b1}}; end
        else
            WC <= WC + 1;
            if (WC == brickW)
            begin
                WC <= 0;
                if (HC == brickH) HC <= 0;
                else HC <= HC + 1;
            end

        outX <= brickX + WC;
        outY <= brickY + HC;
        outCOLOR <= 3'b000;
        plot <= 1'b1;

        if (brickedUP[Read_address])
            if ((((vY[2] & ballY + vY[0] == brickY) | (!vY[2] & ballY - vY[0] == brickY + brickH)) & ((ballX + vX[0] > brickX & !vX[2] & ballX +vX[0] < brickX + brickW) | (ballX - vX[0] > brickX & vX[2] & ballX < brickX + brickW))) | (((vX[2] & ballX + vX[0] == brickX) | (!vX[2] & ballX - vX[0] == brickX + brickW)) & ((ballY + vY[0] > brickY & !vY[2] & ballY +vY[0] < brickY + brickH) | (ballY - vY[0] > brickY & vY[2] & ballY < brickY + brickH))) && brickedUP[Read_address])            begin
                brickedUP[Read_address] = 0;
                cX <= 1;
                cY <= 1;
            end
    end
endmodule