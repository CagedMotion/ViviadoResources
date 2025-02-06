module fetch_unit(
    input  wire       clk,
    input  wire       reset,
    input  wire       en,
    input  wire       branch,
    input  wire [9:0] branch_addr,
    output wire [31:0] instruction  // Assuming a 32-bit wide instruction
);

    // The program counter
    wire [9:0] pc;
    program_counter pc_inst (
        .clk(clk),
        .reset(reset),
        .en(en),
        .branch(branch),
        .branch_addr(branch_addr),
        .pc_out(pc)
    );

    // Instruction memory: Here we assume a module "instruction_memory"
    // that takes a 10-bit address and outputs a 32-bit instruction.
    instruction_memory imem_inst (
        .addr(pc),
        .data_out(instruction)
    );

endmodule
