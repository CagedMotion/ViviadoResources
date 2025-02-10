`timescale 1 ns/1 ps

module fetch_unit_with_reg(
    input  wire       clk,
    input  wire       reset,
    input  wire       branch,
    input  wire [9:0] branch_addr,
    output wire [9:0] pc_out
);

    // Internal signal to hold the computed next PC
    wire [9:0] next_pc;

    // Compute the next PC value:
    // If branch is asserted, choose branch_addr; otherwise, increment current PC by 1.
    // (The current PC will be available at pc_out.)
    assign next_pc = branch ? branch_addr : (pc_out + 10'd1);

    // Instantiate the pre-designed 10-bit register module.
    // It should have the interface: clk, reset, en, din, dout.
    register_10bit pc_reg (
        .clk(clk),
        .reset(reset),
        .en(1'b1),      
        .din(next_pc),
        .dout(pc_out)
    );

endmodule

module tb_fetch_unit_with_reg;
    reg clk;
    reg reset;
    reg branch;
    reg [9:0] branch_addr;
    wire [9:0] pc_out;
    
    // Instantiate the fetch unit.
    fetch_unit_with_reg dut (
        .clk(clk),
        .reset(reset),
        .branch(branch),
        .branch_addr(branch_addr),
        .pc_out(pc_out)
    );
    
    // Clock generation: Toggle every 5 ns for a 10 ns period.
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // Stimulus: Reset and then test branch and non-branch behavior.
    initial begin
        reset = 1; branch = 0; branch_addr = 10'd100;
        #15;  // Ensure reset is applied.
        reset = 0;
        #100;
        branch = 1;  // Test branch condition.
        #20;
        branch = 0;
        #100;
        $finish;
    end
endmodule

