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
    reg [7:0] rwe;
    //bank 0
    register_t0 t0(.clk(clk), .reset(rst), .en(rwe[0]), .dout(r1), .din(wdata));
    register_t1 t1(.clk(clk), .reset(rst), .en(rwe[1]), .dout(r2), .din(wdata));
    register_s0 s0(.clk(clk), .reset(rst), .en(rwe[2]), .dout(r3), .din(wdata));
    register_s1 s1(.clk(clk), .reset(rst), .en(rwe[3]), .dout(r4), .din(wdata));
    
    //bank1
    register_t2 t2(.clk(clk), .reset(rst), .en(rwe[4]), .dout(r5), .din(wdata));
    register_t3 t3(.clk(clk), .reset(rst), .en(rwe[5]), .dout(r6), .din(wdata));
    register_s2 s2(.clk(clk), .reset(rst), .en(rwe[6]), .dout(r7), .din(wdata));
    register_s3 s3(.clk(clk), .reset(rst), .en(rwe[7]), .dout(r8), .din(wdata));
    
    // Two banks, each with 4 registers of 10 bits
    wire [2:0] write_selector;
    wire [2:0] read_selector0, read_selector1;
    assign write_selector = {bank_sel,waddr[1:0]};
    assign read_selector0 = {bank_sel,raddr1[1:0]};
    assign read_selector1 = {bank_sel,raddr2[1:0]};
    
    
    // Synchronous reset and write operation.
    // On reset, clear both banks.
    // On a write, update the register in the selected bank.
     // Synchronous write block.
    
    always @(write_selector, we) begin
        if (we == 1) begin    
            case(write_selector)
                3'b000: rwe = 8'b00000001;
                3'b001: rwe = 8'b00000010;
                3'b010: rwe = 8'b00000100;
                3'b011: rwe = 8'b00001000;
                3'b100: rwe = 8'b00010000;
                3'b101: rwe = 8'b00100000;
                3'b110: rwe = 8'b01000000;
                3'b111: rwe = 8'b10000000;
                default: rwe = 8'b00000000;
            endcase                  
        end 
        else
            rwe = 8'b00000000;
    end
    
    // Combinational read blocks.
    always @(negedge(clk)) begin
        case(read_selector0)
            3'b000: rdata1 = r1;
            3'b001: rdata1 = r2;
            3'b010: rdata1 = r3;
            3'b011: rdata1 = r4;
            3'b100: rdata1 = r5;
            3'b101: rdata1 = r6;
            3'b110: rdata1 = r7;
            3'b111: rdata1 = r8;
            default: rdata1 = 10'd0;
        endcase
    end

    always @(negedge(clk)) begin
        case(read_selector1)
            3'b000: rdata2 = r1;
            3'b001: rdata2 = r2;
            3'b010: rdata2 = r3;
            3'b011: rdata2 = r4;
            3'b100: rdata2 = r5;
            3'b101: rdata2 = r6;
            3'b110: rdata2 = r7;
            3'b111: rdata2 = r8;
            default: rdata2 = 10'd0;
        endcase 
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
  reg         bank_sel;
  reg  [1:0]  waddr;
  reg  [9:0]  wdata;
  reg  [1:0]  raddr1;
  reg  [1:0]  raddr2;
  wire [9:0]  rdata1;
  wire [9:0]  rdata2;
  
  // Instantiate the register_file module
  register_file uut(
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
  
    parameter PERIOD = 10;
    initial clk = 1'b0;
    always #(PERIOD/2) clk = ~clk;
    
  // Test sequence
  initial begin
    // --- Initialization ---
    rst      = 1;          // start with reset asserted
    we       = 0;          
    bank_sel = 0;
    waddr    = 2'b00;
    wdata    = 10'b0;
    raddr1   = 2'b00;
    raddr2   = 2'b00;    
    #PERIOD
    // --- Write to Bank 0 ---
    // Write to register 0 (bank0, addr 0)
    
    rst = 0;              // Deassert reset
    bank_sel = 0;
    we       = 1;
    waddr    = 2'b00;
    wdata    = 10'd15;     // example value
    #PERIOD

    // Write to register 1 (bank0, addr 1)
    //@(posedge clk);
        bank_sel = 0;
        we       = 1;
        waddr    = 2'b01;
        wdata    = 10'd23;
        #PERIOD

    // Write to register 2 (bank0, addr 2)
    //@(posedge clk);
    bank_sel = 0;
    we       = 1;
    waddr    = 2'b10;
    wdata    = 10'd37;
    #PERIOD

   // Write to register 3 (bank0, addr 3)
   bank_sel = 0;
   we       = 1;
   waddr    = 2'b11;
   wdata    = 10'd42;
        #PERIOD
        #PERIOD
//    // Disable writing so that read operations can occur
//    // --- Read from Bank 0 ---
//    // Read registers 0 and 1 using port1 and port2 respectively.
 
    we = 0;
    bank_sel = 0;
    raddr1   = 2'b00;    // should read register 0 (value 15)
    raddr2   = 2'b01;    // should read register 1 (value 23)
    #PERIOD      // wait for synchronous read
    $display("Time %t: Bank0 -> rdata1 (reg0) = %d, rdata2 (reg1) = %d", $time, rdata1, rdata2);
    #PERIOD
    
      // Read registers 2 and 3.
       raddr1   = 2'b10;   // should read register 2 (value 37)
       raddr2   = 2'b11;    // should read register 3 (value 42)
       #PERIOD
       $display("Time %t: Bank0 -> rdata1 (reg2) = %d, rdata2 (reg3) = %d", $time, rdata1, rdata2);
       #PERIOD
       
    // --- Write to Bank 1 ---
    // Write to register 0 in bank1
    bank_sel = 1;
    we       = 1;
    waddr    = 2'b00;
    wdata    = 10'd7;
    #PERIOD
    
    // Write to register 1 in bank 1
    bank_sel = 1;
    we       = 1;
    waddr    = 2'b01;
    wdata    = 10'd14;
    #PERIOD
    // Write to register 2 in bank1
    bank_sel = 1;
    we       = 1;
    waddr    = 2'b10;
    wdata    = 10'd28;
    #PERIOD
    // Write to register 3 in bank1
    bank_sel = 1;
    we       = 1;
    waddr    = 2'b11;
    wdata    = 10'd35;
    #PERIOD 
    #PERIOD
  
    // --- Read from Bank 1 ---
    // Read registers 0 and 1 
    we = 0;
    bank_sel = 1;
    raddr1   = 2'b00;    // should read register 0 (value 7)
    raddr2   = 2'b01;    // should read register 1 (value 14)
    #PERIOD
    $display("Time %t: Bank1 -> rdata1 (reg0) = %d, rdata2 (reg1) = %d", $time, rdata1, rdata2);
    #PERIOD
    // Read registers 2 and 3 
    raddr1   = 2'b10;    // should read register 2 (value 28)
    raddr2   = 2'b11;    // should read register 3 (value 35)
    #PERIOD
    $display("Time %t: Bank1 -> rdata1 (reg2) = %d, rdata2 (reg3) = %d", $time, rdata1, rdata2);
    #PERIOD

    $finish;
  end

endmodule

