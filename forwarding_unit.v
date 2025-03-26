module forwarding_unit (
    input  wire        exmem_wb_wr,      // Write-back enable from EX/MEM stage.
    input  wire        ex_is_load,       // Asserted if the EX/MEM instruction is a load.
    input  wire [2:0]  ex_dest_reg,      // Destination register in EX/MEM.
    //input  wire [9:0]  ex_dest_reg_value, // (Not used for hazard detection in this version)
    input  wire [2:0]  id_dest_reg,      // First source register from ID (decode) stage.
    input  wire [2:0]  id_src_reg,       // Second source register from ID stage.
    output reg         forwardA,         // Forward selector for operand A.
    output reg         forwardB,         // Forward selector for operand B.
    output reg         stall             // Stall signal for load-use hazard.
);

    always @(*) begin
        // Default values: no forwarding and no stall.
        forwardA = 1'b0;
        forwardB = 1'b0;
        stall    = 1'b0;
        
        // Check for a load hazard:
        // If the EX/MEM stage instruction is a load (ex_is_load is high)
        // and it writes to a register that is used by the instruction in ID stage,
        // then assert stall.
        if (exmem_wb_wr && ex_is_load &&
            ((ex_dest_reg == id_dest_reg) || (ex_dest_reg == id_src_reg))) begin
            stall = 1'b1;
        end else begin
            // Normal forwarding for non-load instructions:
            if (exmem_wb_wr && (ex_dest_reg == id_dest_reg))
                forwardA = 1'b1;
            if (exmem_wb_wr && (ex_dest_reg == id_src_reg))
                forwardB = 1'b1;
        end
    end

endmodule
//module forwarding_unit (
//    input  wire        exmem_wb_wr,   // Asserted if the instruction in the EX-MEM/WB Register is a write
//    input  wire [2:0]  ex_dest_reg,   // Destination register of the Execute stage instruction.
//    input  wire [9:0]  ex_dest_reg_value, // Value of the Destination register after the execute/memory stage'
//    input  wire [2:0]  id_dest_reg,   // First source register from the Decode stage.
//    input  wire [2:0]  id_src_reg,   // Second source register from the Decode stage.
//    output reg         forwardA,          // Selector bit for Forward A Mux
//    output reg         forwardB          // Selector bit for Forward B Mux

//);

//    always @(*) begin
//        // If a load instruction in the execute stage writes to a register that is
//        // needed by the instruction in the decode stage, assert stall.
//        if (((exmem_wb_wr == 1'b1) & (ex_dest_reg_value == 10'b0)) & (ex_dest_reg == id_dest_reg)) begin
//            forwardA = 1'b1;
//        end else begin
//            forwardA = 1'b0;
//        end
//        if (((exmem_wb_wr == 1'b1) & (ex_dest_reg_value == 10'b0)) & (ex_dest_reg == id_src_reg)) begin
//            forwardB = 1'b1;
//        end else begin
//            forwardB = 1'b0;
//        end
//    end

//endmodule