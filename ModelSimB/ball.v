module ball(clock, reset, state, cX, cY, nextX, nextY, vX, vY, in); //velocity may be required, or we can just hard code it
    input clock, reset, cX, cY;
    input in;
    input [2:0]state;
    output reg [2:0] vX, vY;
    output reg [7:0] nextX, nextY;

    reg move;

    //FSM for ball state
    always @ (*)
        case(state)
            3'b000: move = 1'b0;
            3'b001: if(in == 1'b1) move = 1'b1;
            // endScreen: move = 1'b0;
        endcase

    always @(posedge clock) // will need slower counter for this
        if(move)
            begin
            if(cX /*| cBrickX*/) vX[2] <= ~vX[2];
            if(cY /*| cBrickY*/) vY[2] <= ~vY[2];
            if(~vX[2]) nextX <= nextX + vX[0];
            if(vX[2]) nextX <= nextX - vX[0];
            if(~vY[2]) nextY <= nextY +vY[0];
            if(vY[2]) nextY <= nextY - vY[0];
            end
        else    
            begin
            vX <= 3'b001;
            vY <= 3'b001;
            nextX <= 20;
            nextY <= 20;
            end

endmodule