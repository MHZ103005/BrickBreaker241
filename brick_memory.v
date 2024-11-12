module brick_memory(clk, brick_index, brickX, brickY, brickW, brickH, brickActive, reset, writeEnable);
    parameter n = 6;//number of bricks
    input clk, reset, writeEnable;
    input [n-1 : 0]brick_index;
    output reg [7:0]brickX, brickY;
    output reg [3:0] brickW, brickH; //change brick width and height accordingly
    output reg brickActive;

    reg [24:0] brick_data [0:39] /*synthesis ram_init_file = ram32x4.mif */; //change synthesis file

    //initial begin for when testing outside of fpga board

    always @ (posedge clk)
        if(!reset)
            begin
                for(i = 0, i < numBricks, i = i+1)
                begin
                    brick_data[i][0] <= 1;
                end
            end
        {brickX, brickY, brickW, brickH, brickActive} <= brick_data[brick_index];

    always @ (posedge clock)
        if(writeEnable == 1)
            brick_data[brick_index] <= {brickX, brickY, brickW, brickH, 1'b0};
    
endmodule