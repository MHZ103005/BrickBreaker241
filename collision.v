module collision(ballX, ballY, paddleX, paddleY /*brick positions somehow*/, vX, vY, cBrickX, cBrickY, cWall, cRoof);
    input [7:0]ballX, ballY, paddleX, paddleY;
    input [2:0]vX, vY;
    output reg cX, cY, cBrickX, cBrickY;

    always @(*)
        begin
        cX = 0;
        cY = 0;
        //Boundary Collision
        if(ballX <= 0 | ballX > 159) cX = 1; // left right wall
        if(ballY <= 0) cY = 1; // roof
        //Paddle collision
        if((ballX + vX >= paddleX) & (ballX + vX <= paddleX + length) & ballY + vY >= paddleY) cY = 1; // can add side collision for paddle
        //Collision with bricks
        end
endmodule