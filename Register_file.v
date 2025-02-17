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
    reg         bank_sel;      // 0 for Bank 0, 1 for Bank 1
    reg  [1:0]  waddr;         // 2-bit write address (0 to 3)
    reg  [9:0]  wdata;         // 10-bit data to write
    reg  [1:0]  raddr1, raddr2; // 2-bit read addresses (0 to 3)
    wire [9:0]  rdata1, rdata2;  // 10-bit read data outputs

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
        // -------- Initialization --------
        rst = 1;
        we = 0;
        bank_sel = 0;  // Default to Bank 0
        waddr = 2'b00;
        wdata = 10'd0;
        raddr1 = 2'b00;
        raddr2 = 2'b01;
        #PERIOD;
        rst = 0;
        #PERIOD;
        
        // -------- Test Bank 0: Write to multiple registers --------
        // Write 55 to Bank 0, Reg1 (waddr = 01)
        we = 1;
        bank_sel = 0;      
        waddr = 2'b01;
        wdata = 10'd55;
        #PERIOD;
        we = 0;
        #PERIOD;
        
        // Write 100 to Bank 0, Reg2 (waddr = 10)
        we = 1;
        bank_sel = 0;
        waddr = 2'b10;
        wdata = 10'd100;
        #PERIOD;
        we = 0;
        #PERIOD;
        
        // Write 75 to Bank 0, Reg3 (waddr = 11)
        we = 1;
        bank_sel = 0;
        waddr = 2'b11;
        wdata = 10'd75;
        #PERIOD;
        we = 0;
        #PERIOD;
        
        // Read back from Bank 0: 
        // Expect Reg1 = 55, Reg2 = 100, Reg3 = 75.
        bank_sel = 0;
        raddr1 = 2'b01;    // Read Bank0 Reg1
        raddr2 = 2'b10;    // Read Bank0 Reg2
        #PERIOD;
        $display("Time %0t: Bank0: rdata1 (Reg1) = %d, rdata2 (Reg2) = %d", $time, rdata1, rdata2);
        
        // Change read addresses to check Reg3 as well.
        raddr1 = 2'b11;    // Read Bank0 Reg3
        #PERIOD;
        $display("Time %0t: Bank0: rdata1 (Reg3) = %d (expected 75)", $time, rdata1);
        
        // -------- Test Bank 1: Write to multiple registers --------
        // Write 200 to Bank 1, Reg0 (waddr = 00)
        we = 1;
        bank_sel = 1;      
        waddr = 2'b00;
        wdata = 10'd200;
        #PERIOD;
        we = 0;
        #PERIOD;
        
        // Write 250 to Bank 1, Reg3 (waddr = 11)
        we = 1;
        bank_sel = 1;
        waddr = 2'b11;
        wdata = 10'd250;
        #PERIOD;
        we = 0;
        #PERIOD;
        
        // Read back from Bank 1:
        bank_sel = 1;
        raddr1 = 2'b00;    // Expect 200 from Bank1 Reg0
        raddr2 = 2'b11;    // Expect 250 from Bank1 Reg3
        #PERIOD;
        $display("Time %0t: Bank1: rdata1 (Reg0) = %d, rdata2 (Reg3) = %d", $time, rdata1, rdata2);
        
        // -------- Confirm Bank Isolation --------
        // Switch back to Bank 0 and read Reg2 (should remain 100)
        bank_sel = 0;
        raddr1 = 2'b10;    // Bank0 Reg2
        #PERIOD;
        $display("Time %0t: Bank0: rdata1 (Reg2) = %d (expected 100)", $time, rdata1);
        
        $finish;
    end
endmodule
