`timescale 1 ns/1 ps

module fetch_unit(
    input  wire       clk,
    input  wire       reset,
    input  wire       halted,
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
    // If halted is asserted, hold the current PC.
    // Otherwise, follow jump > branch > increment.
    wire [9:0] next_pc;
    assign next_pc = (halted == 1'b1) ? pc_out :
                     (jump   ? jump_target : (branch ? branch_addr : pc_plus_one));
    
    // Instantiate the 10-bit register that holds the PC.
    register_10bit pc_reg (
        .clk(clk),
        .reset(reset),
        .en(1'b1),  // Always enabled
        .din(next_pc),
        .dout(pc_out)
    );
    
endmodule

module branch_predictor_1bit #(
    parameter NUM_ENTRIES = 16  // Can be 16, 32, or 64
)(
    input  wire         clk,
    input  wire         reset,
    input  wire [9:0]   pc,           // Current PC value from the fetch stage
    output reg          prediction,   // Prediction output: 1 = taken, 0 = not taken
    // Update interface: when branch outcome is resolved
    input  wire         update,       // Asserted when a branch outcome is available
    input  wire         actual_taken, // Actual branch outcome (1 = taken, 0 = not taken)
    input  wire [9:0]   update_pc     // PC of the branch being updated
);

  localparam INDEX_WIDTH = $clog2(NUM_ENTRIES);

  // 1-bit branch history table, indexed from 0 to NUM_ENTRIES-1
  reg BHT [0:NUM_ENTRIES-1];

  // Use the lower INDEX_WIDTH bits of the PC as index
  wire [INDEX_WIDTH-1:0] index       = pc[INDEX_WIDTH-1:0];
  wire [INDEX_WIDTH-1:0] update_index = update_pc[INDEX_WIDTH-1:0];

  integer i;
  // Reset and update logic
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      // Initialize BHT. To force "always not taken", set all bits to 0.
      // To force "always taken", set all bits to 1.
      for (i = 0; i < NUM_ENTRIES; i = i + 1)
        BHT[i] <= 1'b0;  // Change to 1'b1 for "always taken" behavior.
    end
    else if (update) begin
      // Update the entry at update_index with the actual outcome
      BHT[update_index] <= actual_taken;
    end
  end

  // Combinationally output the prediction for the current PC
  always @(*) begin
    prediction = BHT[index];
  end

endmodule

module branch_predictor_2bit #(
    parameter NUM_ENTRIES = 16  // Can be 16, 32, or 64
)(
    input  wire         clk,
    input  wire         reset,
    input  wire [3:0]   pc,           // Current PC value from the fetch stage
    output reg          prediction,   // Prediction output: 1 = taken, 0 = not taken
    // Update interface: when branch outcome is resolved
    input  wire         update,       // Asserted when branch outcome is available
    input  wire         actual_taken, // Actual branch outcome (1 = taken, 0 = not taken)
    input  wire [3:0]   update_pc     // PC of the branch being updated
);

  localparam INDEX_WIDTH = $clog2(NUM_ENTRIES);

  // 2-bit branch history table (array of 2-bit counters)
  reg [1:0] BHT [0:NUM_ENTRIES-1];

  // Indexing using lower bits of the PC
  wire [INDEX_WIDTH-1:0] index       = pc[INDEX_WIDTH-1:0];
  wire [INDEX_WIDTH-1:0] update_index = update_pc[INDEX_WIDTH-1:0];

  integer i;
  // Reset and update logic
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      // Initialize each counter to a weakly not taken state (e.g., 2'b01).
      for (i = 0; i < NUM_ENTRIES; i = i + 1)
        BHT[i] <= 2'b01;
    end
    else if (update) begin
      // Update the saturating counter for the branch at update_index
      if (actual_taken) begin
        // Increment the counter if not already at maximum (2'b11)
        if (BHT[update_index] != 2'b11)
          BHT[update_index] <= BHT[update_index] + 1;
      end
      else begin
        // Decrement the counter if not already at minimum (2'b00)
        if (BHT[update_index] != 2'b00)
          BHT[update_index] <= BHT[update_index] - 1;
      end
    end
  end

  // Combinationally produce prediction based on the counter value:
  // Predict taken if counter is 2'b10 or 2'b11.
  always @(*) begin
    prediction = (BHT[index] >= 2) ? 1'b1 : 1'b0;
  end

endmodule


module tb_fetch_unit;
    // Inputs to the fetch unit
    reg         clk;
    reg         reset;
    reg         branch;  
    reg         jump;         
    reg  [9:0]  branch_addr;  
    reg  [9:0]  jump_target;
    reg         halted;  
    // Output from the fetch unit
    //wire  [9:0]  instruction;     
    wire [9:0]  pc_out;

    // Instantiate the fetch unit under test
    fetch_unit dut (
        .clk(clk),
        .reset(reset),
        .branch(branch),
        .jump(jump),
        .halted(halted),
        .branch_addr(branch_addr),
        .jump_target(jump_target),
        .pc_out(pc_out)
    );
    
    // Clock generation: Toggle every 5 ns for a 10 ns period
    parameter PERIOD = 10;
    initial clk = 1'b0;
    always #(PERIOD/2) clk = ~clk;
    // Stimulus: test normal increments, then branch, then jump
    initial begin
        // -------- Reset Phase --------
        reset        = 1;
        branch       = 0;
        jump         = 0;
        branch_addr  = 10'd0;
        jump_target  = 10'd0;
        halted       = 1'b0;
//        branch_addr  = 10'd100;
//        jump_target  = 10'd500;
        
        // Assert reset for 15 ns
        #PERIOD;
        reset = 0;   // Deassert reset, PC should start incrementing from 0
        #PERIOD;         // Let it increment a couple cycles

        // -------- Test Branch --------
        // When branch is asserted, PC should load branch_addr on the next clock edge
        //$display("\n--- Asserting BRANCH to load PC with branch_addr = %d ---", branch_addr);
        //branch = 1;
        //#PERIOD;
        //branch = 0;  // Deassert branch
        #PERIOD;         // Wait a few cycles to observe increments again

        // -------- Test Jump --------
        // When jump is asserted, PC should load jump_target on the next clock edge
        //$display("\n--- Asserting JUMP to load PC with jump_target = %d ---", jump_target);
        //jump = 1;
        //#PERIOD;
        //jump = 0;    // Deassert jump
        #PERIOD;         // Wait a few cycles to observe increments again
        halted       = 1'b1;
        #PERIOD;#PERIOD;#PERIOD;#PERIOD;
        // End simulation
        $finish;
    end
endmodule


