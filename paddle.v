module paddle(clock, in,inEnable, length, paddleX, state, reset);
    input clock, reset, inEnable;
    input [4:0]length;
    input [2:0]state;
    input [7:0] in; //Double Check size of input
    output reg [7:0] paddleX;
    wire tick;
    DelayCounter D3 (clock, reset, tick);
    always@(posedge clock) // we are going to need a slower counter for this
        begin
        if(!reset) paddleX <= 8'b0;/*default*/
        else if((state == 3'b001 | state = 3'b010) & inEnable)
            begin
            if(in == 8'b0 & tick) // change to right input
                if(paddleX + length < 160) paddleX <= paddleX + 1;
            else if(in == 8'b1 & tick) // change to left input
                begin
                if(paddleX > 0) paddleX <= paddleX - 1;
                end
            end
        end
endmodule