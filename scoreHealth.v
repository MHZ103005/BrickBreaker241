module scoreHealth(clock, reset, cBrickX, cBrickY, ballY, score, life); // add move and set it to 0 when ball goes below 0
    input clock, reset, cBrickX, cBrickY;
    input [6:0] ballY;
    output reg [5:0] score;
    output reg[1:0] life;
    always @ (posedge clock)
    begin
        if(!reset)
            score <= 0;
            life <= 2'b11;
        if(cBrickX | cBrickY) score <= score + 1;
        if(ballY < 20) life <= life -1;
    end
endmodule