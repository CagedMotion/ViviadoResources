module hazard_unit(
    input  wire clk,
    input  wire rst,
    input  wire cache_ready,  // from the Cache module
    // You might also include other hazard signals such as from load hazards,
    // branch hazards etc.
    output reg  stall        // Global stall signal for the pipeline
);

    // Combinational logic to generate stall signal.
    always @(cache_ready) begin
        // If the cache is not ready, assert stall.
        if (!cache_ready)
            stall = 1'b1;
        else
            stall = 1'b0;
    end

endmodule
