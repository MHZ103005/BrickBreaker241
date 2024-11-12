module ball(clock, reset, state, cX, cY, nextX, nextY, vX, vY, in); //velocity may be required, or we can just hard code it
    input clock, reset, cX, cY;
    input in;
    input [2:0]state;
    output signed [2:0] vX, vY;
    output reg [7:0] nextX, nextY;

    reg move;
    wire tick;
    DelayCounter D2 (clock, reset, tick);

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
            if(cX | cBrickX) vX[3] <= ~vX[3];
            if(cY | cBrickY) vY[3] <= ~vY[3];
            if(~vX[3] & tick) ballX <= ballX + vX;
            if(vX[3] & tick) ballX <= ballX - vX;
            if(~vY[3] & tick) ballY <= ballY +vY;
            if(vY[3] & tick) ballY <= ballY - vY;
            end
        else    
            begin
            vX <= 3'b001;
            vY <= 3'b001;
            ballX <= 20; // temporary
            ballY <= 20; // temporary
            end

endmodule