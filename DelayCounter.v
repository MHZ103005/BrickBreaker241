module DelayCounter(clock, reset, Q);
    input clock, reset;
    output reg Q;

    wire [25:0] slowcount;
    slowCounter S1 (clock, reset, slowcount);
    always @ (posedge clock)
        if(slowcount == 26'd50000000) Q <= Q + 1;
        else Q <= 0;
endmodule


module slowCounter(clock, resetn, slowcount);
    input clock, resetn;
    output reg [25:0]slowcount;


    always @ (posedge clock)
    begin
        if(!resetn)
            slowcount <= 26'b0;
            Q <= 1'b0;
        else if(slowcount > 26'd50000000)
            slowcount <= 26'b0;
        else
            slowcount <= slowcount + 1;
    end
endmodule