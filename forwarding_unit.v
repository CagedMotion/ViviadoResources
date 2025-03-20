module forwarding_unit (
    input  wire        exmem_wb_wr,   // Asserted if the instruction in the EX-MEM/WB Register is a write
    input  wire [2:0]  ex_dest_reg,   // Destination register of the Execute stage instruction.
    input  wire [9:0]  ex_dest_reg_value, // Value of the Destination register after the execute/memory stage'
    input  wire [2:0]  id_dest_reg,   // First source register from the Decode stage.
    input  wire [2:0]  id_src_reg,   // Second source register from the Decode stage.
    output reg         forwardA,          // Selector bit for Forward A Mux
    output reg         forwardB          // Selector bit for Forward B Mux

);

    always @(*) begin
        // If a load instruction in the execute stage writes to a register that is
        // needed by the instruction in the decode stage, assert stall.
        if (((exmem_wb_wr == 1'b1) & (ex_dest_reg_value == 10'b0)) & (ex_dest_reg == id_dest_reg)) begin
            forwardA = 1'b1;
        end else begin
            forwardA = 1'b0;
        end
        if (((exmem_wb_wr == 1'b1) & (ex_dest_reg_value == 10'b0)) & (ex_dest_reg == id_src_reg)) begin
            forwardB = 1'b1;
        end else begin
            forwardB = 1'b0;
        end
    end

endmodule