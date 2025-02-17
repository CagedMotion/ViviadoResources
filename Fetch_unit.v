`timescale 1 ns/1 ps

module fetch_unit_with_reg(
    input  wire       clk,
    input  wire       reset,
    input  wire       branch,       // Branch signal: when true, use branch_addr
    input  wire       jump,         // Jump signal: when true, use jump_target
    input  wire [9:0] branch_addr,  // Branch target (PC+1 + immediate)
    input  wire [9:0] jump_target,  // Jump target (direct 10-bit address)
    output wire [9:0] pc_out        // Current PC output
);

    // Calculate PC+1
    wire [9:0] pc_plus_one;
    assign pc_plus_one = pc_out + 10'd1;
    
    // Next PC selection:
    // Priority: if jump is asserted, load jump_target;
    // otherwise if branch is asserted, load branch_addr;
    // else increment PC.
    wire [9:0] next_pc;
    assign next_pc = jump ? jump_target : (branch ? branch_addr : pc_plus_one);
    
    // Instantiate the 10-bit register that holds the PC.
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

