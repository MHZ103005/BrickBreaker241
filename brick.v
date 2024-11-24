module brick (input clock, input [7:0] ballX, input [6:0] ballY, output reg cX, cY, output reg [7:0] outX, output reg [6:0] outY, output reg [2:0] outCOLOR, input resetn, input [2:0] vx, vy);
    wire [3:0] brickW;
    wire [2:0] brickH;
    reg [3:0] WC;
    reg [2:0] HC;
    reg Read_address;
    reg [39:0] brickedUP;
    wire [7:0] brickX;
    wire [6:0] brickY; 
    
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
            if ((((vY[2] & ballY + vY[0] == brickY) | (!vY[2] & ballY - vY[0] == brickY + brickH)) & ((ballX + vX[0] > brickX & !vX[2] & ballX +vX[0] < brickX + brickW) | (ballX - vX[0] > brickX & vX[2] & ballX < brickX + brickW))) | (((vX[2] & ballX + vX[0] == brickX) | (!vX[2] & ballX - vX[0] == brickX + brickW)) & ((ballY + vY[0] > brickY & !vY[2] & ballY +vY[0] < brickY + brickH) | (ballY - vY[0] > brickY & vY[2] & ballY < brickY + brickH))) && brickedUP[Read_address])
            begin
                brickedUP[Read_address] = 0;
                cX <= 1;
                cY <= 1;
            end
    end
endmodule

module hitSound(
    input CLOCK_50,         // 50 MHz clock from DE1-SoC
    input [3:0] KEY,        // Keys for reset control
    output AUD_XCK,         // Clock for the audio chip
    output AUD_DACDAT,      // Digital audio data output
    inout AUD_BCLK,         // Audio bit-stream clock
    inout AUD_ADCLRCK,      // Audio ADC LR clock
    inout AUD_DACLRCK,      // Audio DAC LR clock
    output FPGA_I2C_SCLK,   // I2C clock for audio configuration
    inout FPGA_I2C_SDAT,    // I2C data for audio configuration
    input [11:0] SW         // Keyboard input (use switches for testing, can be replaced with actual keyboard data)
);
    
endmodule