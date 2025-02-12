`timescale 1 ns/1 ps
module register_s0(
    output wire [9:0] dout,
    input wire clk, reset, en,
    input wire [9:0] din );
    
    // Declare the internal wire for connecting the 10-bit register output.
    wire [9:0] internal_dout;
    
    register_10bit reg_inst(
        .clk(clk),
        .reset(reset),
        .en(en),
        .dout(internal_dout),
        .din(din)
    );
    
    assign dout = internal_dout;
endmodule

module tb_register_s0;
    // Declare testbench signals
    reg         clk;
    reg         reset;
    reg         en;
    reg  [9:0]  din;
    wire [9:0]  dout;

    // Instantiate the register_s0 module
    register_s0 dut (
        .dout(dout),
        .clk(clk),
        .reset(reset),
        .en(en),
        .din(din)
    );

    // Clock generation: a 10 ns period (5 ns high, 5 ns low)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    parameter PERIOD = 10;
    // Stimulus: drive reset, enable, and din
    initial begin
        // Initially, assert reset to clear the register
        reset = 1;
        en    = 0;    // Disable writing while in reset
        din   = 10'd0;
        #PERIOD;
        
        // Deassert reset and enable writing
        reset = 0;
        en    = 1;
        
        // Apply a new value to din and allow the register to capture it.
        din = 10'd25;
        #PERIOD;  // Wait for one clock edge
        $display("Time: %0t | en = %b | din = %d | dout = %d", $time, en, din, dout);
        
        // Change din to another value while en is active.
        din = 10'd50;
        #PERIOD;
        $display("Time: %0t | en = %b | din = %d | dout = %d", $time, en, din, dout);
        
        // Disable writing (en = 0); the register should hold the last value.
        en = 0;
        din = 10'd75;  // Change din, but register should not update
        #PERIOD;
        $display("Time: %0t | en = %b | din = %d | dout = %d", $time, en, din, dout);
        
        // Re-enable writing and apply a new value.
        en = 1;
        din = 10'd100;
        #PERIOD;
        $display("Time: %0t | en = %b | din = %d | dout = %d", $time, en, din, dout);
        
        // End simulation
        $finish;
    end
endmodule