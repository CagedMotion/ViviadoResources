module hazard_unit(
    input  wire cache_ready,       // 1 ⇒ cache is ready
    input  wire load_use_hazard,   // 1 ⇒ must stall for a load-use data hazard
    input  wire branch_hazard,     // 1 ⇒ must stall for branch resolution (optional)
    output wire stall              // 1 ⇒ pipeline must stall
);

    // stall if ANY hazard is present or the cache is not ready
    assign stall = (~cache_ready)
                 | load_use_hazard
                 | branch_hazard
                 // | other_hazard_flag
                 ;

endmodule

