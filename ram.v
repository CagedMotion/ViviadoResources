`timescale 1ns / 1ps

module ram(
    output wire [9:0] rdata,
    input wire clk, we,
    input wire [9:0] address,
    input wire [9:0] wdata
    );
    reg [9:0] ram[1023:0];
    reg [9:0] address_reg;    // Address register

    always @ (posedge clk)
    begin
        if (we)
            ram[address] <= wdata;
        else
            address_reg <= address;
    end

    assign rdata = ram[address_reg];
endmodule

module tb_ram();
    wire [9:0] rdata;
    reg clk, we;
    reg [9:0] address;
    reg [9:0] wdata;
    
    ram dut(.rdata(rdata), .clk(clk), .we(we), .address(address), .wdata(wdata));
    
    parameter PERIOD = 10;
    initial clk = 1'b0;
    always #(PERIOD/2) clk = ~clk;
    
    initial begin
        wdata = 10'h01;
        address = 10'd0;
        we = 1'b1;
        #PERIOD;

        wdata = 10'h02;
        address = 10'd1;     
        #PERIOD;

        wdata = 10'h03;
        address = 10'd2;     
        #PERIOD;

        address = 10'd0;
        we = 1'b0;
        #PERIOD;

        address = 10'd1;
        #PERIOD;

        address = 10'd2;
        #PERIOD;

        wdata = 10'h04;
        address = 10'd1;
        we = 1'b1;
        #PERIOD;

        address = 10'd1;
        we = 1'b0;
        #PERIOD;
    end
endmodule