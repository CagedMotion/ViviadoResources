module hazard_detection_unit (
    input  wire        ex_mem_read,   // Asserted if the instruction in the Execute stage is a load.
    input  wire [1:0]  ex_dest_reg,   // Destination register of the Execute stage instruction.
    input  wire [1:0]  id_src_reg1,   // First source register from the Decode stage.
    input  wire [1:0]  id_src_reg2,   // Second source register from the Decode stage.
    output reg         stall          // Stall signal: asserted when a hazard is detected.
);

    always @(*) begin
        // If a load instruction in the execute stage writes to a register that is
        // needed by the instruction in the decode stage, assert stall.
        if (ex_mem_read && ((ex_dest_reg == id_src_reg1) || (ex_dest_reg == id_src_reg2))) begin
            stall = 1'b1;
        end else begin
            stall = 1'b0;
        end
    end

endmodule