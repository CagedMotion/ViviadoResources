`timescale 1 ns/1 ps

module register_file(
    input  wire       clk,       // Global clock signal
    input  wire       rst,       // Synchronous reset
    input  wire       we,        // Write enable signal
    input  wire       bank_sel,  // Bank select: 0 = Bank0, 1 = Bank1
    input  wire [1:0] waddr,     // 2-bit write address (0 to 3)
    input  wire [9:0] wdata,     // 10-bit data to write
    input  wire [1:0] raddr1,    // 2-bit read address for read port 1 (0 to 3)
    input  wire [1:0] raddr2,    // 2-bit read address for read port 2 (0 to 3)
    output reg [9:0] rdata1,    // 10-bit output from read port 1
    output reg [9:0] rdata2     // 10-bit output from read port 2
);
    
    wire [9:0] r1, r2, r3, r4, r5, r6, r7, r8;
    reg [9:0] w1, w2, w3, w4, w5, w6, w7, w8; 
    //bank 0
    register_t0 t0(.clk(clk), .reset(rst), .en(we), .dout(r1), .din(w1));
    register_t1 t1(.clk(clk), .reset(rst), .en(we), .dout(r2), .din(w2));
    register_s0 s0(.clk(clk), .reset(rst), .en(we), .dout(r3), .din(w3));
    register_s1 s1(.clk(clk), .reset(rst), .en(we), .dout(r4), .din(w4));
    
    //bank1
    register_t2 t2(.clk(clk), .reset(rst), .en(we), .dout(r5), .din(w5));
    register_t3 t3(.clk(clk), .reset(rst), .en(we), .dout(r6), .din(w6));
    register_s2 s2(.clk(clk), .reset(rst), .en(we), .dout(r7), .din(w7));
    register_s3 s3(.clk(clk), .reset(rst), .en(we), .dout(r8), .din(w8));
    
    // Two banks, each with 4 registers of 10 bits
    wire [2:0] write_selector;
    wire [3:0] read_selector0, read_selector1;
    assign write_selector = {bank_sel,waddr[1:0]};
    assign read_selector0 = {bank_sel, raddr1[1:0]};
    assign read_selector1 = {bank_sel,raddr2[1:0]};
    
    // Synchronous reset and write operation.
    // On reset, clear both banks.
    // On a write, update the register in the selected bank.
    always @(posedge clk) begin
        if (rst == 1'b1) begin
            w1 = 10'b0;
            w2 = 10'b0;
            w3 = 10'b0;
            w4 = 10'b0;
            w5 = 10'b0;
            w6 = 10'b0;
            w7 = 10'b0;
            w8 = 10'b0;
            rdata1 = 10'b0;
            rdata2 = 10'b0; 
        end else if (we == 1'b1) begin
             case(write_selector)     
                3'b000 : w1 = wdata;
                3'b001 : w2 = wdata;
                3'b010 : w3 = wdata;
                3'b011 : w4 = wdata;
                3'b100 : w5 = wdata;
                3'b101 : w6 = wdata;
                3'b110 : w7 = wdata;
                3'b111 : w8 = wdata;
                endcase                  
            end
        else begin
            case(read_selector0)
                3'b000 : rdata1 = r1;
                3'b001 : rdata1 = r2;
                3'b010 : rdata1 = r3;
                3'b011 : rdata1 = r4;
                3'b100 : rdata1 = r5;
                3'b101 : rdata1 = r6;
                3'b110 : rdata1 = r7;
                3'b111 : rdata1 = r8;
            endcase
            case(read_selector1)
                3'b000 : rdata2 = r1;
                3'b001 : rdata2 = r2;
                3'b010 : rdata2 = r3;
                3'b011 : rdata2 = r4;
                3'b100 : rdata2 = r5;
                3'b101 : rdata2 = r6;
                3'b110 : rdata2 = r7;
                3'b111 : rdata2 = r8;
            endcase 
        end
end
    // Asynchronous (combinational) reads based on the bank selection.
    //assign rdata1 = (bank_sel == 1'b0) ? bank0[raddr1] : bank1[raddr1];
    //assign rdata2 = (bank_sel == 1'b0) ? bank0[raddr2] : bank1[raddr2];

endmodule
module tb_register_file;
    // Testbench signals
    reg         clk;
    reg         rst;
    reg         we;
    reg         bank_sel;      // Bank select signal: 0 for bank 0, 1 for bank 1
    reg  [1:0]  waddr;         // 2-bit write address (0 to 3)
    reg  [9:0]  wdata;
    reg  [1:0]  raddr1, raddr2; // 2-bit read addresses
    wire [9:0]  rdata1, rdata2;

    // Instantiate the register file with bank switching
    register_file dut (
       .clk(clk),
       .rst(rst),
       .we(we),
       .bank_sel(bank_sel),
       .waddr(waddr),
       .wdata(wdata),
       .raddr1(raddr1),
       .raddr2(raddr2),
       .rdata1(rdata1),
       .rdata2(rdata2)
    );
    
    // Clock generation: 10 ns period (5 ns high, 5 ns low)
    initial begin
       clk = 0;
       forever #5 clk = ~clk;
    end
   
    parameter PERIOD = 10;
    
    // Test procedure
    initial begin
       // Apply reset to initialize registers in both banks
       rst = 1;
       we = 0;
       bank_sel = 0;
       waddr = 2'b00;
       wdata = 10'd0;
       raddr1 = 2'b00;
       raddr2 = 2'b01;
       #PERIOD;
       rst = 0;
       
       // --- Test Bank 0 ---
       // Write to Bank 0, register 1
       we = 1;
       bank_sel = 0;      // Select bank 0
       waddr = 2'b01;     // Register 1 in bank 0
       wdata = 10'd55;
       #PERIOD;
       we = 0;
       
       // Write to Bank 0, register 3
       we = 1;
       bank_sel = 0;
       waddr = 2'b11;     // Register 3 in bank 0
       wdata = 10'd10;
       #PERIOD;
       we = 0;
       
       // Read from Bank 0: Expect register 1 = 55 and register 3 = 200
       bank_sel = 0;
       //raddr1 = 2'b01;    // Expect 55 from bank 0
       raddr2 = 2'b11;    // Expect 200 from bank 0
       #PERIOD;
       $display("Bank 0: rdata1 (Reg1) = %d, rdata2 (Reg3) = %d", rdata1, rdata2);
       
       // --- Test Bank 1 ---
       // Write to Bank 1, register 2
       we = 1;
       bank_sel = 1;      // Select bank 1
       waddr = 2'b10;     // Register 2 in bank 1
       wdata = 10'd100;
       #PERIOD;
       we = 0;
       
       // Write to Bank 1, register 0
       we = 1;
       waddr = 2'b00;     // Register 0 in bank 1
       wdata = 10'd150;
       #PERIOD;
       we = 0;
       
       // Read from Bank 1: Expect register 2 = 100 and register 0 = 150
       bank_sel = 1;
       raddr1 = 2'b10;    // Expect 100 from bank 1
       raddr2 = 2'b00;    // Expect 150 from bank 1
       #PERIOD;
       $display("Bank 1: rdata1 (Reg2) = %d, rdata2 (Reg0) = %d", rdata1, rdata2);
       
       // Confirm that switching banks keeps the data isolated:
       // Read from Bank 0, register 2 (which was never written) should be 0.
       bank_sel = 0;
       raddr1 = 2'b10;    // Expect 0 in bank 0, reg2
       #PERIOD;
       $display("Bank 0: rdata1 (Reg2) = %d (expected 0)", rdata1);
       
       $finish;
    end
endmodule