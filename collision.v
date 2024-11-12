module collision(clock, ballX, ballY, paddleX, paddleY, length, vX, vY, cBrickX, cBrickY, cWall, cRoof);
    input clock;
    input[4:0]length;
    input [7:0]ballX, ballY, paddleX, paddleY;
    input [2:0]vX, vY;
    output reg cX, cY, cBrickX, cBrickY;

    reg writeEnable;

    reg [7:0]brickX, brickY;
    reg[3:0]brickW, brickH;
    reg brickActive;
    reg [4:0]index;

    brick_memory B1 (clock, index, brickX, brickY, brickW, brickH, brickActive);

    always @(posedge clock)
        begin
        cX = 0;
        cY = 0;
        //Boundary Collision
        if(ballX <= 0 | ballX > 159) cX = 1; // left right wall
        if(ballY <= 0) cY = 1; // roof
        //Paddle collision -> fix velocity doesnt account for signed
        if((ballX + vX[0] >= paddleX) & (ballX + vX[0] <= paddleX + length) & ballY + vY[0] >= paddleY) cY = 1; // can add side collision for paddle
        end
    //brick collision
    always @ (posedge clock)
        if(!reset)
            begin
            cBrickX <= 0;
            cBrickY <= 0;
            index <= 0;
            writeEnable <= 0;
            end
        else 
            begin
            cBrickX = 0;
            cBrickY = 0;
            writeEnable = 0;
            for(index = 0; index < 40; index = index + 1)
                if(brickActive) 
                    if((ballX > brickX) & (ballX < brickX + brickW) & ballY + vY[0] <= brickY + brickW & ballY + vY[0] > brickY) 
                        brickY <= 1'b1;
                        writeEnable = 1'b1;
                    else if((ballY > brickY) & (ballY <= brickY + brickW) & (ballX + vX[0] > brickX) & (ballX + vX[0] < brickX + brickW)) 
                        brickX <= 1'b1;
                        writeEnable <= 1'b1;
            end
endmodule