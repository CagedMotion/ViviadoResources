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
