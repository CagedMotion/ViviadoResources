`timescale 1 ns/1 ps

module register_file(
    input  wire        clk,    // Clock signal
    input  wire        rst,    // Synchronous reset
    input  wire        we,     // Write enable signal
    input  wire [2:0]  waddr,  // 3-bit write address (0 to 7, but only 0-3 are writeable)
    input  wire [9:0]  wdata,  // 10-bit data to write
    input  wire [2:0]  raddr1, // 3-bit read address for read port 1 (can be 0-7)
    input  wire [2:0]  raddr2, // 3-bit read address for read port 2 (can be 0-7)
    output wire [9:0]  rdata1, // 10-bit output from read port 1
    output wire [9:0]  rdata2  // 10-bit output from read port 2
);

    // Declare an array of 8 registers, each 10 bits wide.
    reg [9:0] registers [7:0];
    integer i;
    
    // Synchronous reset and write operation.
    // On reset, all registers are cleared.
    // On a write, only registers with an address from 0 to 3 are updated.
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 8; i = i + 1)
                registers[i] <= 10'b0;
        end else if (we) begin
            // Only allow writes to registers 0-3 (the writeable subset)
            if (waddr < 3'd4)
                registers[waddr] <= wdata;
        end
    end

    // Asynchronous (combinational) reads.
    assign rdata1 = registers[raddr1];
    assign rdata2 = registers[raddr2];

endmodule

module tb_register_file;
    // Inputs
    reg clk;
    reg rst;
    reg we;
    reg [2:0] waddr;
    reg [9:0] wdata;
    reg [2:0] raddr1, raddr2;
    // Outputs
    wire [9:0] rdata1, rdata2;
    
    // Instantiate the register file module
    register_file dut (
       .clk(clk),
       .rst(rst),
       .we(we),
       .waddr(waddr),
       .wdata(wdata),
       .raddr1(raddr1),
       .raddr2(raddr2),
       .rdata1(rdata1),
       .rdata2(rdata2)
    );
    
    // Clock generation: 10 ns period
    initial begin
       clk = 0;
       forever #5 clk = ~clk;
    end
   
    parameter PERIOD = 10;
    // Test procedure
    initial begin
       // Apply reset
       rst = 1;
       we = 0;
       waddr = 3'b000;
       wdata = 10'd0;
       raddr1 = 3'b000;
       raddr2 = 3'b001;
       #PERIOD;
       rst = 0;
       
       // Write to register 1 (writeable, since 1 < 4)
       we = 1;
       waddr = 3'b001;  // Register 1
       wdata = 10'd55;
       #PERIOD;  // Data written on next clock edge
       we = 0;
       
       // Attempt to write to register 4 (not writeable, since 4 >= 4)
       #10;
       we = 1;
       waddr = 3'b100;  // Register 4
       wdata = 10'd100;
       #PERIOD;
       we = 0;
       
       // Read from register 1 and register 4
       raddr1 = 3'b001; // Expect 55 from register 1
       raddr2 = 3'b100; // Expect 0 from register 4 (write ignored)
       #10;
       $display("Read1: Register 1 = %d, Read2: Register 4 = %d", rdata1, rdata2);
       
       // Write to register 3 (writeable)
       we = 1;
       waddr = 3'b011; // Register 3
       wdata = 10'd200;
       #PERIOD;
       we = 0;
       
       // Read from register 3 and register 0
       raddr1 = 3'b011; // Expect 200 from register 3
       raddr2 = 3'b000; // Expect 0 from register 0 (if not written before)
       #PERIOD;
       $display("Read1: Register 3 = %d, Read2: Register 0 = %d", rdata1, rdata2);
       
       $finish;
    end
endmodule
