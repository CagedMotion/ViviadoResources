`timescale 1 ns/1 ps

module program_counter(
    input  wire       clk,         // Clock signal
    input  wire       reset,       // Reset signal
    input  wire       branch,      // Branch control signal
    input  wire [9:0] branch_addr, // Branch address if a branch is taken
    output wire [9:0] pc           // Current PC output
);

    // Internal signals
    wire [9:0] current_pc; // Holds the current PC (output of the register)
    wire [9:0] next_pc;    // Holds the next PC value computed by logic

    // Compute the next PC value:
    // If 'branch' is high, then load branch_addr; otherwise, increment by 1.
    assign next_pc = branch ? branch_addr : (current_pc + 10'd1);

    // Instantiate the 10-bit register to store the PC.
    // Here we assume the register updates on the rising edge of clk and resets when reset is asserted.
    register_10bit pc_reg (
        .clk(clk),
        .reset(reset),
        .en(1'b1),     // Always enabled in this example; you can add an enable signal if needed.
        .din(next_pc), // Feed the computed next_pc into the register.
        .dout(current_pc) // current_pc reflects the register's current value.
    );

    // Drive the module's output with the current PC value.
    assign pc = current_pc;

endmodule

module tb_program_counter;
    // Testbench signals
    reg         clk;
    reg         reset;
    reg         branch;
    reg  [9:0]  branch_addr;
    wire [9:0]  pc;

    // Instantiate the program_counter module (Device Under Test)
    program_counter dut (
        .clk(clk),
        .reset(reset),
        .branch(branch),
        .branch_addr(branch_addr),
        .pc(pc)
    );

    // Clock Generation: 10 ns period (5 ns high, 5 ns low)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    parameter PERIOD = 10;
    // Stimulus for the program counter
    initial begin
        // Initialize inputs
        reset = 1;
        branch = 0;
        branch_addr = 10'd0;

        // Apply reset for a couple of clock cycles
        #PERIOD;  // Wait 10 ns
        reset = 0;  // Deassert reset

        // Let the PC increment sequentially for a few cycles
        // At this point, since branch is 0, PC should increment by 1 each cycle.
        #PERIOD;  // After first rising edge post-reset
        $display("Time %0t: PC = %d (expected ~1)", $time, pc);
        #PERIOD;
        $display("Time %0t: PC = %d (expected ~2)", $time, pc);
        #PERIOD;
        $display("Time %0t: PC = %d (expected ~3)", $time, pc);

        // Now, test the branch functionality.
        // Assert the branch signal with a branch address.
        branch = 1;
        branch_addr = 10'd100;
        #PERIOD;  
        // On the next rising edge, PC should load branch_addr (100) rather than increment.
        $display("Time %0t: PC = %d (expected 100 due to branch)", $time, pc);
        branch = 0;  // Deassert branch for subsequent cycles

        // Let the PC increment normally from the branch address.
        #PERIOD;
        $display("Time %0t: PC = %d (expected 101)", $time, pc);
        #PERIOD;
        $display("Time %0t: PC = %d (expected 102)", $time, pc);

        // Test another branch later in the simulation.
        #PERIOD;
        branch = 1;
        branch_addr = 10'd200;
        #PERIOD;
        $display("Time %0t: PC = %d (expected 200 due to branch)", $time, pc);
        branch = 0;

        // Let the PC run a few more cycles to confirm normal operation resumes.
        #PERIOD;
        $display("Time %0t: PC = %d (expected 201)", $time, pc);
        #10;
        $display("Time %0t: PC = %d (expected 202)", $time, pc);

        // End simulation
        #PERIOD;
        $finish;
    end

endmodule