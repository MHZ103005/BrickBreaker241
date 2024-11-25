module main(SW, CLOCK_50, PS2_CLK, PS2_DAT, LEDR, KEY, VGA_R, VGA_G, VGA_B, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK, HEX5, HEX4, HEX2, AUD_XCK, AUD_DACDAT, AUD_BCLK, AUD_ADCLRCK, AUD_DACLRCK, FPGA_I2C_SCLK, FPGA_I2C_SDAT);
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

output AUD_XCK;      
output AUD_DACDAT;    
inout AUD_BCLK;       
inout AUD_ADCLRCK;     
inout AUD_DACLRCK;      
output FPGA_I2C_SCLK;  
inout FPGA_I2C_SDAT; 


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

//Game FSM
parameter mainMenu = 3'b0, game1 = 3'b001, loss = 3'b010, win = 3'b011, noMotion = 3'b100;
reg [2:0]gameState, ngState;

always @(*)
    case(gameState)
        mainMenu: if(keyP == 8'h29) ngState = noMotion;
                    else ngState = mainMenu;
        game1: begin //if(score == 40) ngState = win;
               if(ballY > 100) ngState = noMotion;
                else ngState = game1; end
        noMotion: begin if(life==0) ngState = loss;
                    else if(keyP == 8'h29) ngState = game1;
                    else ngState = noMotion; end
    endcase

always @(posedge clock)
    if(!reset) gameState <= mainMenu;
    else gameState <= ngState;

//score
wire [5:0]score;
wire [1:0]life;

seg7 score1 ({2'b0, score[5:4]}, HEX5);
seg7 score2 (score[3:0], HEX4);
seg7 health1 ({2'b0, life}, HEX2);

scoreHealth score3 (clock, reset, cX, cY, ballY, score);

//FSM for background drawing
parameter state1 = 3'b0, paddleDraw = 3'b001, state3 = 3'b010, drawBall = 3'b011, erasePaddle = 3'b100, eraseBall = 3'b101, waitState = 3'b110;
reg [2:0] currentDraw, ns;
reg doneP, doneB;
always @(*)
    case(currentDraw)
        state1: if(gameState == mainMenu | gameState == loss) ns = state1;
                else if(address == 15'd19199) ns = paddleDraw;
                else ns = state1;
        paddleDraw: if(address == 15'd19199) ns = drawBall;
                else ns = paddleDraw;
        drawBall: if(address == 15'd19199) ns = erasePaddle;
                else ns = drawBall;
        erasePaddle:begin if(!move) ns = waitState;  
                    else if(erasePaddleCount == 160) ns = eraseBall;
                    else ns = erasePaddle;end
        eraseBall:  if(ballY < 30) ns = state3;
                    else ns = paddleDraw; 
        state3: if(address == 15'd19199) ns = state1;
                else ns = state3;
        waitState:  if(erasePaddleCount == 160) ns = state3;
                    else ns = waitState;
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
        erasePaddle:
                begin VGA_Xalt <= erasePaddleCount;
                    VGA_Yalt <= 7'd100;
                    VGA_COLOR = 3'b000;
                end
        eraseBall: begin VGA_Xalt = preX;
                    VGA_Yalt = preY; end
        state3: begin
                VGA_Xalt <= Xb;
                VGA_Yalt <= Yb;
                plot <= 1'b1;
                VGA_COLOR <= 3'b000;
                end
        waitState: begin
                    VGA_Xalt <= erasePaddleCount;
                    VGA_Yalt <= 7'd98;
                    VGA_COLOR <= 3'b000; end
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

//background changing
wire [2:0] colour1, colour2, colour3, colour4, currentScene;

MainMenu rom1 (address, clock, colour1); //memory for all backgrounds
win rom2 (address, clock, colour2);
losegame rom3 (address, clock, colour3);
level1 rom4(address, clock, colour4);

muxScreens mux1 (colour1, colour2, colour3, colour4, gameState, currentScene);

wire [7:0] Xb;
wire [6:0] Yb;
wire[14:0] address;
wire[7:0] erasePaddleCount;
//for background
xyscroll x1 (clock, reset, Xb, Yb, 1'b1);
addressScroll x2 (clock, reset, address, 1'b1);
counter160 x3 (clock, reset, erasePaddleCount, currentDraw == erasePaddle | currentDraw == waitState);

//erasing while not moving
wire [7:0]Xerase;
wire [6:0]Yerase;
nonMovingErase non1(clock, reset, Xerase, Yerase, currentDraw == waitState);

//for paddle
XCcounter x23 (clock, reset, currentDraw == paddleDraw, XC);
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
//assign move = SW[7];

wire [19:0] fps60;
frame60 F1 (CLOCK_50, KEY[0], fps60);
ballMove B1 (CLOCK_50, fps60==20'd0, ballX, ballY, vX, vY, cX, cY, preX, preY, move, X);
moveEnabling move1 (clock, reset, keyP, gameState, move, ballY, life);

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

//Assigning VGA variables
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


//AUDIO CONTROLLER
reg wave;
reg [31:0]countAudio;
wire[31:0]audioOut;

integer period;
always@(*) 
    if(cX | cY) period = 56818;
    else (gameState == loss) period = 1775;
    else period = 500000000;

always @(posedge clock)
    if(!reset) begin countAudio <= 0; wave <= 0; end
    else if (countAudio >= period) begin countAudio <= 0; wave = ~wave; end
    else countAudio <= countAudio + 1;

assign audioOut = wave ? 32'd1000000000: -32'd1000000000;

Audio_Controller Audio_Controller ( // Audio controller takes audio_out and manages it
        .CLOCK_50(clock),
        .reset(~KEY[0]),
        .clear_audio_in_memory(),
        .clear_audio_out_memory(),
        .left_channel_audio_out(audioOut),
        .right_channel_audio_out(audioOut),
        .write_audio_out(1'b1),     // Always write audio out
        .AUD_ADCDAT(1'b0),
        .AUD_BCLK(AUD_BCLK),
        .AUD_ADCLRCK(AUD_ADCLRCK),
        .AUD_DACLRCK(AUD_DACLRCK),
        .audio_in_available(),
        .audio_out_allowed(),
        .AUD_XCK(AUD_XCK),
        .AUD_DACDAT(AUD_DACDAT)
    );

    // Audio configuration module
    avconf #(.USE_MIC_INPUT(1)) avc (
        .FPGA_I2C_SCLK(FPGA_I2C_SCLK),
        .FPGA_I2C_SDAT(FPGA_I2C_SDAT),
        .CLOCK_50(CLOCK_50),
        .reset(~KEY[0])
    );
endmodule

module muxScreens(c1, c2, c3, c4, state, Q);
    input [2:0]c1, c2, c3, c4;
    input [2:0] state;
    output reg [2:0]Q;

    always @ (*)
    begin
        if(state == 3'b000)
            Q <= c1;
        else if(state == 3'b011)
            Q <= c2;
        else if(state == 3'b010)
            Q <= c3;
        else if(state == 3'b001 | 3'b100)
            Q <= c4;
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
        if (!move) begin ballX <= paddleX + 9; ballY <= 7'd98; preX <= paddleX + 9; preY <= 7'd98; vX <= 3'b101; vY <= 3'b101; end
        else 
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

module scoreHealth(clock, reset, cBrickX, cBrickY, ballY, score); // add move and set it to 0 when ball goes below 0
    input clock, reset, cBrickX, cBrickY;
    input [6:0] ballY;
    output reg [3:0] score;
    always @ (posedge clock)
    begin
        if(!reset)
            score <= 4'b0000;
        if(cBrickX | cBrickY) score <= score + 1;
    end
endmodule

module counter160 (clock, reset, Q, enable);
    input clock, reset, enable;
    output reg [7:0]Q;

    always @(posedge clock)
        begin
        if(!reset) Q <= 8'b0;
        if(enable) Q <= Q + 1;
        else Q <= 0;
        end
endmodule

module nonMovingErase (clock, reset, X, Y, enable);
    input clock, reset, enable;
    output reg [6:0]Y;
    output reg [7:0]X;
    
    always @ (posedge clock)begin
        if(!reset) begin X <= 0; Y <= 7'd118; end
        if(enable) 
            if(X == 8'd160) begin X <= 0; Y <= Y+1;end 
            else X <= X+1;
        else begin X <= 0; Y <= 7'd118; end
    end 
endmodule 

module moveEnabling (clock, reset, keyin, state, move, ballY, life);
    input clock, reset;
    input [7:0]keyin;
    input [2:0]state;
    input [6:0] ballY;
    output reg move; 
    output reg [1:0]life;

    always @(posedge clock)
        if(!reset) begin move <= 1'b0; life <= 2'b11; end
        else 
            if(keyin == 8'h29 && state == 3'b100) move <= 1'b1;
            else if(ballY > 100) if(move) begin life <= life - 1; move<= 1'b0; end
endmodule