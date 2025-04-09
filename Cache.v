`timescale 1ns/1ps

module Cache(
    input clk,
    input rst,
    input CPU_RW,              // 0: read, 1: write.
    input [9:0] cpu_data_in,
    input [9:0] cpu_address,
    input [9:0] mem_data_from_ram,  // data incoming from RAM (10 bits, one word)
    input mem_ready,
    output reg [9:0] cpu_data_out,
    output reg cache_ready,
    output reg [9:0] mem_addr,
    output reg [9:0] mem_data_to_ram, // data going to the RAM (one word at a time).
    output reg mem_rw,
    output reg mem_req
    );
    
    // Temporary storage for the data coming from memory.
    reg [9:0] new_block_lower;
    reg [9:0] new_block_upper;
    
    // Latched CPU write data for write misses.
    reg [9:0] latched_cpu_data;
    
    // *** Latches for the faulting address fields ***
    reg [3:0] latched_index;
    reg [4:0] latched_tag;
    reg       latched_offset;
    reg       latched_cpu_rw;
    
    // Cache arrays.
    reg valid [0:15];
    reg dirty [0:15];
    reg [4:0] cache_tag [0:15];
    reg [19:0] cache_data [0:15];
    
    // Address breakdown for the current CPU access (not necessarily the miss)
    wire [3:0] cpu_index = cpu_address[4:1];
    wire [4:0] cpu_tag   = cpu_address[9:5];
    wire       cpu_offset = cpu_address[0];    
    
    // State machine definitions.
    reg [1:0] state, next_state;
    // Use a one-bit phase to represent the two memory cycles.
    reg mem_phase;  // 0: first half, 1: second half.
     
    parameter IDLE_COMPARE = 2'b00;
    parameter WRITEBACK    = 2'b01;
    parameter ALLOCATION   = 2'b10;
    
    // Determine a hit based on the current access.
    // (Note: In a miss, we will latch the values so that later allocation is done correctly.)
    wire hit = valid[cpu_index] && (cache_tag[cpu_index] == cpu_tag);
        
    // Initializations.
    integer i;
    initial begin
        state <= IDLE_COMPARE;
        next_state <= IDLE_COMPARE;
        mem_phase <= 0;
        cache_ready = 1'b0;
        for (i = 0; i < 16; i = i + 1) begin
            valid[i] <= 1'b0;
            dirty[i] <= 1'b0;
            cache_tag[i] <= 5'b00000;
            cache_data[i] <= 20'd0;
        end
    end
  

    // Combinational block: Next state and output signals.
    always @(*) begin
        // Default assignments.
        next_state    = state;
        cache_ready   = 1'b0;
        mem_req       = 0;
        mem_rw        = 0;
        mem_addr      = 10'b0;
        mem_data_to_ram = 10'b0;
        cpu_data_out  = 10'b0;
        
        case (state)
            IDLE_COMPARE: begin
                cache_ready = 1'b1;
                if (cpu_address != 10'b0) begin
                    if (hit) begin
                        // On hit, immediately serve the CPU.
                        next_state = IDLE_COMPARE;
                        if (!CPU_RW) begin
                            // Read: pick the appropriate half from the cache.
                            cpu_data_out = (cpu_offset) ? cache_data[cpu_index][19:10]
                                                        : cache_data[cpu_index][9:0];
                        end else begin
                            // For a write hit, you might just echo back the CPU data.
                            cpu_data_out = cpu_data_in;
                        end
                    end else begin
                        // On miss, choose to either write-back (if dirty) or allocate.
                        next_state = (valid[cpu_index] && dirty[cpu_index]) ? WRITEBACK : ALLOCATION;
                    end
                end else begin
                    next_state = IDLE_COMPARE;
                end
            end
            
            WRITEBACK: begin
                // Write-back: write one half (word) per cycle.
                mem_req = 1'b1;
                mem_rw  = 1'b1;  // Write operation.
                // Note: using current cpu_index here assumes the miss address is valid.
                mem_addr = {cache_tag[cpu_index], cpu_index, mem_phase};
                if (mem_phase == 0)
                    mem_data_to_ram = cache_data[cpu_index][9:0];
                else
                    mem_data_to_ram = cache_data[cpu_index][19:10];
                // After both halves are written back, transition to ALLOCATION.
                if ((mem_phase == 1) && mem_ready)
                    next_state = ALLOCATION;
            end
            
            ALLOCATION: begin
                // Allocation: read one half (word) per cycle.
                mem_req = 1'b1;
                mem_rw  = 1'b0;  // Read operation.
                // Use the latched tag and index (from when the miss occurred)
                mem_addr = {latched_tag, latched_index, mem_phase};
                // Once both halves are read, finish allocation.
                if ((mem_phase == 1) && mem_ready)
                    next_state = IDLE_COMPARE;
            end
            
            default: next_state = IDLE_COMPARE;
        endcase
    end

    // Sequential block: state update and proper mem_phase toggling.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE_COMPARE;
            mem_phase <= 0;
        end else begin
            state <= next_state;
            // During WRITEBACK and ALLOCATION, toggle mem_phase per cycle when memory is ready.
            if ((state == WRITEBACK || state == ALLOCATION) && mem_ready) begin
                if (mem_phase == 0)
                    mem_phase <= 1;
                else
                    mem_phase <= 0;  // Reset phase after second half.
            end else if (state == IDLE_COMPARE) begin
                mem_phase <= 0;
            end
        end
    end

    // Latch the faulting address and CPU signals when a miss is detected.
    // This ensures the correct index, tag, offset and CPU_RW value are used later in allocation.
    always @(posedge clk) begin
        if (state == IDLE_COMPARE && (cpu_address != 10'b0) && !hit) begin
            latched_index   <= cpu_address[4:1];
            latched_tag     <= cpu_address[9:5];
            latched_offset  <= cpu_address[0];
            latched_cpu_rw  <= CPU_RW;
            // Also latch CPU write data if this is a write miss.
            if (CPU_RW)
                latched_cpu_data <= cpu_data_in;
        end
    end

    // Sequential block for capturing data from memory and updating the cache.
    always @(posedge clk) begin
        // During ALLOCATION, capture the incoming data from RAM.
        if (state == ALLOCATION && mem_ready) begin
            if (mem_phase == 0)
                new_block_lower <= mem_data_from_ram[9:0];
            else if (mem_phase == 1)
                new_block_upper <= mem_data_from_ram[9:0];
        end

        // When allocation finishes (after both halves have been fetched), update the cache.
        if (state == ALLOCATION && mem_ready && (mem_phase == 1)) begin
            // Use the latched index and tag rather than the current cpu_address fields.
            cache_tag[latched_index] <= latched_tag;
            valid[latched_index]     <= 1'b1;
            if (latched_cpu_rw) begin
                // Write miss: update the appropriate half using the latched CPU data.
                if (latched_offset) begin
                    // CPU intended to write to the upper half.
                    cache_data[latched_index] <= {latched_cpu_data, new_block_lower};
                end else begin
                    // CPU intended to write to the lower half.
                    cache_data[latched_index] <= {new_block_upper, latched_cpu_data};
                end
                dirty[latched_index] <= 1'b1;
            end else begin
                // Read miss: simply store both halves from memory.
                cache_data[latched_index] <= {new_block_upper, new_block_lower};
                dirty[latched_index] <= 1'b0;
            end
        end

        // For write hits (when the block is already in the cache), update the cache directly.
        if (state == IDLE_COMPARE && (cpu_address != 10'b0) && hit && CPU_RW) begin
            if (cpu_offset)
                cache_data[cpu_index][19:10] <= cpu_data_in;
            else
                cache_data[cpu_index][9:0]  <= cpu_data_in;
            dirty[cpu_index] <= 1'b1;
        end
    end
    
endmodule

module tb_Cache();
    // Signal declarations.
    reg clk;
    reg rst;
    reg CPU_RW;           // 0: read, 1: write.
    reg [9:0] cpu_data_in;
    reg [9:0] cpu_address;
    wire [9:0] cpu_data_out;
    wire cache_ready;
    
    // Memory interface signals between Cache and RAM.
    wire [9:0] mem_addr;
    wire [9:0] mem_data_out_to_ram;
    wire mem_rw;
    wire mem_req;
    
    // Connection between RAM and Cache for read data.
    wire [9:0] ram_rdata;
    reg mem_ready;

    // Instantiate the Cache module.
    Cache cache_inst (
        .clk(clk),
        .rst(rst),
        .CPU_RW(CPU_RW),
        .cpu_data_in(cpu_data_in),
        .cpu_address(cpu_address),
        .mem_data_from_ram(ram_rdata),       // Data coming from RAM.
        .mem_ready(mem_ready),
        .cpu_data_out(cpu_data_out),
        .cache_ready(cache_ready),
        .mem_addr(mem_addr),
        .mem_data_to_ram(mem_data_out_to_ram), // Data to be written back to RAM.
        .mem_rw(mem_rw),
        .mem_req(mem_req)
    );

    // Instantiate the RAM module.
    // When mem_rw = 1, the write enable is asserted.
    wire ram_we = mem_rw;
    ramtask2 ram_inst (
        .rdata(ram_rdata),
        .clk(clk),
        .we(ram_we),
        .address(mem_addr),
        .wdata(mem_data_out_to_ram)
    );

    // Clock generation: 10 ns period.
    parameter PERIOD = 10;
    initial begin
        clk = 1;
        forever #(PERIOD/2) clk = ~clk;
    end

    // Test sequence.
    initial begin
        // Initialization.
        rst = 1;
        CPU_RW = 0;
        cpu_data_in = 10'd0;
        cpu_address = 10'd0;
        mem_ready = 1;    // For simplicity, assume memory is always ready.
        #(2*PERIOD);
        rst = 0;
        #(PERIOD);

        //---------------------------------------------------
        // Test 1: Write to 4 distinct cache locations.
        //---------------------------------------------------
        // We choose addresses that map to different indices.
        
        // Write to address 50 (binary example).
        cpu_address = 10'b0000110010;  
        CPU_RW = 1; // Write mode.
         #(PERIOD);
        
        // Write to address 67.
        cpu_address = 10'b0001000011;  
        CPU_RW = 1;
        #(PERIOD);

        // Write to address 83.
        cpu_address = 10'd83;  
        CPU_RW = 1;
        cpu_data_in = 10'd300;
        #(PERIOD);
        
        // Write to address 95.
        cpu_address = 10'd95;  
        CPU_RW = 1;
        cpu_data_in = 10'd400;
        #(PERIOD);
        
        // Write to address 150.
        cpu_address = 10'b0010010110;
        CPU_RW = 1;
        cpu_data_in = 10'd100;
        #(PERIOD);

        //---------------------------------------------------
        // Test 2: Read back from those locations (read hits).
        //---------------------------------------------------
        cpu_address = 10'd50;  
        CPU_RW = 0; // Read mode.
        #(PERIOD);
        
        cpu_address = 10'd67;  
        CPU_RW = 0;
        #(PERIOD);
        
        cpu_address = 10'd83;  
        CPU_RW = 0;
        #(PERIOD);
        
        cpu_address = 10'd95;  
        CPU_RW = 0;
        #(PERIOD);

        //---------------------------------------------------
        // Test 3: Write hit - update one location.
        //---------------------------------------------------
        // Update address 50 with new data.
        cpu_address = 10'd50;
        CPU_RW = 1;
        cpu_data_in = 10'd150; 
        #(PERIOD);
        
        // Read back address 50.
        cpu_address = 10'd50;
        CPU_RW = 0;
        #(PERIOD);

        //---------------------------------------------------
        // Test 4: Eviction with Write-Back.
        //---------------------------------------------------
        // Same index but a different tag to force eviction.
        cpu_address = 10'd70;
        CPU_RW = 0; // Read mode triggers allocation.
        #(3*PERIOD);
        
        // Now, write new data into address 70.
        cpu_address = 10'd70;
        CPU_RW = 1;
        cpu_data_in = 10'd777;
        #(PERIOD);
        
        // Read back address 70.
        cpu_address = 10'd70;
        CPU_RW = 0;
        #(PERIOD);

        //---------------------------------------------------
        // Additional Checks.
        //---------------------------------------------------
        cpu_address = 10'd67;
        CPU_RW = 0;
        #(PERIOD);
        
        cpu_address = 10'd83;
        CPU_RW = 0;
        #(PERIOD);

        #(5*PERIOD);
        $finish;
    end
endmodule



