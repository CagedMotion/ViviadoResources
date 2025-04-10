`timescale 1ns/1ps

module Cache(
    input clk,
    input rst,
    input CPU_RW,              // 0: read, 1: write.
    input [9:0] cpu_address,
    input mem_ready,
    inout wire [19:0] mem_data_ram_bus,
    inout wire [9:0] cpu_data_bus,
    output reg cache_ready,
    output reg [9:0] mem_addr,
    output reg mem_rw,
    output reg mem_req
    );
    
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
     
    parameter IDLE_COMPARE = 2'b00;
    parameter WRITEBACK    = 2'b01;
    parameter ALLOCATION   = 2'b10;
    
    //bidir signals and signals for getting data around.
    reg [19:0] mem_data_ram_store_bus;
    reg [9:0] cpu_data_store_bus;
    wire [19:0] mem_data_ram_write, mem_data_ram_read;
    wire [9:0] cpu_data_write, cpu_data_read;
    assign mem_data_ram_bus = (mem_rw == 1'b1) ?
        mem_data_ram_read :
        20'bzzzzzzzzzz_zzzzzzzzzz;
        
    assign cpu_data_bus = (CPU_RW == 1'b1) ?
        10'bzzzzzzzzzz :
        cpu_data_read;
        
    assign cpu_data_read = cpu_data_store_bus;
    assign mem_data_ram_read = mem_data_ram_store_bus;
    assign cpu_data_write = cpu_data_bus;
    assign mem_data_ram_write = mem_data_ram_bus;
            
    // Determine a hit based on the current access.
    // (Note: In a miss, we will latch the values so that later allocation is done correctly.)
    reg hit;
//    valid[cpu_index] && (cache_tag[cpu_index] == cpu_tag);
        
    // Initializations.
    integer i;
    initial begin
        state <= IDLE_COMPARE;
        next_state <= IDLE_COMPARE;
        cache_ready = 1'b0;
        for (i = 0; i < 16; i = i + 1) begin
            valid[i] <= 1'b0;
            dirty[i] <= 1'b0;
            cache_tag[i] <= 5'b00000;
            cache_data[i] <= 20'd0;
        end
    end
    
    // Combinational block: Next state and output signals.
    always @(IDLE_COMPARE, WRITEBACK, ALLOCATION, state, cpu_address, hit, rst, mem_ready, cpu_index, CPU_RW, cpu_offset, cpu_tag) begin
        // Default assignments.
        next_state    = IDLE_COMPARE;
        cache_ready   = 1'b1;
        mem_rw        = 0;
        mem_req       = 1'b0;
        mem_addr      = 10'b0;
        mem_data_ram_store_bus = 20'b0;
        cpu_data_store_bus  = 10'b0;
        hit = 1'b0;
        
        case (state)
            IDLE_COMPARE: begin
                cache_ready = 1'b1;
                if (rst == 1'b0) begin
                    hit = valid[cpu_index] && (cache_tag[cpu_index] == cpu_tag);
                    if (hit == 1'b1) begin
                        // On hit, immediately serve the CPU.
                        if (dirty[cpu_index] == 1'b0) begin
                            next_state = IDLE_COMPARE;
                            if (!CPU_RW) begin
                                // Read: pick the appropriate half from the cache.
                                cpu_data_store_bus = (cpu_offset) ? cache_data[cpu_index][19:10]
                                                            : cache_data[cpu_index][9:0];
                            end else begin
                                // For a write hit, you might just echo back the CPU data.
                                cpu_data_store_bus = (cpu_offset) ? cache_data[cpu_index][19:10]
                                                            : cache_data[cpu_index][9:0];
                            end
                        end else begin
                            next_state = WRITEBACK;
                        end
                    end else begin
                        // On miss, choose to either write-back (if dirty) or allocate.
                        if (valid[cpu_index] && dirty[cpu_index]) begin
                            next_state = WRITEBACK;
                        end else begin
                            next_state = ALLOCATION;
                        end
                    end
                end else begin
                    next_state = IDLE_COMPARE;
                end
            end
            
            WRITEBACK: begin
                cache_ready = 1'b0;
                // Write-back: write one half (word) per cycle.
                mem_rw  = 1'b1;  // Write operation.
                mem_req = 1'b1;
                // Note: using current cpu_index here assumes the miss address is valid.
                mem_addr = {cache_tag[cpu_index], cpu_index, cpu_offset};
                mem_data_ram_store_bus = cache_data[cpu_index];
                next_state = ALLOCATION;
            end
            
            ALLOCATION: begin
                cache_ready = 1'b0;
                // Allocation: read one half (word) per cycle.
                mem_rw  = 1'b0;  // Read operation.
                mem_req = 1'b1;
                mem_addr = {cpu_tag, cpu_index, cpu_offset};
                if (mem_ready)
                    next_state = IDLE_COMPARE;
            end
            
            default: next_state = IDLE_COMPARE;
        endcase
    end

    // Sequential block: state update and proper mem_phase toggling.
    always @(posedge clk, posedge rst) begin
        if (rst) begin
            state <= IDLE_COMPARE;
        end else begin
            state <= next_state;
        end
    end

    // Latch the faulting address and CPU signals when a miss is detected.
    // This ensures the correct index, tag, offset and CPU_RW value are used later in allocation.
    // Sequential block for capturing data from memory and updating the cache.
    always @(posedge clk) begin
        // During ALLOCATION, capture the incoming data from RAM.

        // When allocation finishes (after both halves have been fetched), update the cache.
        if (state == ALLOCATION && mem_ready) begin
            // Use the latched index and tag rather than the current cpu_address fields.
            cache_tag[cpu_index] <= cpu_tag;
            valid[cpu_index]     <= 1'b1;
            if (CPU_RW) begin
                // Write miss: update the appropriate half using the latched CPU data.
                if (cpu_offset) begin
                    // CPU intended to write to the upper half.
                    cache_data[cpu_index][19:10] <= cpu_data_store_bus; //fix?
                end else begin
                    // CPU intended to write to the lower half.
                    cache_data[cpu_index][9:0] <= cpu_data_store_bus; //fix?
                end
                dirty[cpu_index] <= 1'b1;
            end else begin
                // Read miss: simply store both halves from memory.
                cache_data[cpu_index] <= mem_data_ram_write; //fix?
                dirty[cpu_index] <= 1'b0;
            end
        end

        // For write hits (when the block is already in the cache), update the cache directly.
        if (state == IDLE_COMPARE && hit && CPU_RW) begin
            if (cpu_offset)
                cache_data[cpu_index][19:10] <= cpu_data_write;
            else
                cache_data[cpu_index][9:0]  <= cpu_data_write;
            dirty[cpu_index] <= 1'b1;
        end
    end
    
endmodule

module tb_Cache();
    // Signal declarations.
    reg clk;
    reg rst;
    reg CPU_RW;           // 0: read, 1: write.
    reg [9:0] cpu_data_write;
    reg [9:0] cpu_address;
    wire cache_ready;
    
    // Memory interface signals between Cache and RAM.
    wire [9:0] mem_addr;
    wire mem_rw;
    
    wire [19:0] mem_data_ram_bus;
    wire [9:0] cpu_data_bus, cpu_data_read;
    
    // Connection between RAM and Cache for read data
    wire mem_ready;
    wire mem_req;
    
    // Instantiate the Cache module.
    Cache cache_inst (
        .clk(clk),
        .rst(rst),
        .CPU_RW(CPU_RW),
        .cpu_data_bus(cpu_data_bus),
        .cpu_address(cpu_address),
        .mem_data_ram_bus(mem_data_ram_bus),
        .mem_ready(mem_ready),
        .cache_ready(cache_ready),
        .mem_addr(mem_addr),
        .mem_rw(mem_rw),
        .mem_req(mem_req)
    );

    // Instantiate the RAM module.
    // When mem_rw = 1, the write enable is asserted.
    wire ram_we = mem_rw;
    ramtask2 ram_inst (
        .data(mem_data_ram_bus),
        .clk(clk),
        .we(ram_we),
        .mem_ready(mem_ready),
        .address(mem_addr),
        .mem_req(mem_req)
    );
        
    assign cpu_data_bus = (CPU_RW == 1'b1) ?
            cpu_data_write :
            10'bzzzzzzzzzz;
    
    // Clock generation: 10 ns period.
    parameter PERIOD = 10;
    initial clk = 1;
    always #(PERIOD/2) clk = ~clk;
    
    assign cpu_data_read = cpu_data_bus;
    
    // Test sequence.
    initial begin
        // Initialization.
        rst = 1;
        CPU_RW = 0;
        cpu_data_write = 10'd0;
        cpu_address = 10'd0;
        #(PERIOD);

        //---------------------------------------------------
        // Test 1: Write to 5 distinct cache locations.
        //---------------------------------------------------
        // We choose addresses that map to different indices.
        // the first two are cache misses which load from ram
        // Write to address 50 (binary example).
        rst = 0;
        cpu_address = 10'b0000110010;  
        CPU_RW = 0; // Write mode.
        cpu_data_write = 10'd0;
         #(2*PERIOD);
        
        // Write to address 67.
        cpu_address = 10'b0001000011;  
        CPU_RW = 0;
        cpu_data_write = 10'd0;
        #(2*PERIOD);

        //this part deals with the cpu_data_in part.
        // Write to address 84.
        cpu_address = 10'b0001010100;  
        CPU_RW =1;
        cpu_data_write = 10'd300;
        #(2*PERIOD);
        
        // Write to address 95.
        cpu_address = 10'b0001011111;  
        CPU_RW = 1;
        cpu_data_write = 10'd400;
        #(2*PERIOD);
        
        // Write to address 150.
        cpu_address = 10'b0010010110;
        CPU_RW = 1;
        cpu_data_write = 10'd100;
        #(3*PERIOD);

        //---------------------------------------------------
        // Test 2: Read back from those locations (read hits).
        //---------------------------------------------------
        cpu_address = 10'd50;  
        CPU_RW = 0; // Read mode.
        #(PERIOD);
        
        cpu_address = 10'd67;  
        CPU_RW = 0;
        #(PERIOD);
        
        cpu_address = 10'd84;  
        CPU_RW = 0;
        #(PERIOD);
        
        cpu_address = 10'd95;  
        CPU_RW = 0;
        #(PERIOD);
        
        cpu_address = 10'd150;
        CPU_RW = 0;
        #(PERIOD)
        
        //---------------------------------------------------
        // Test 3: Write hit - update one location.
        //---------------------------------------------------
        // Update address 50 with new data.
        cpu_address = 10'd84;
        CPU_RW = 1;
        cpu_data_write = 10'd150; 
        #(3*PERIOD);
        
        // Read back address 50.
        cpu_address = 10'd84;
        CPU_RW = 0;
        #(3*PERIOD);

        //---------------------------------------------------
        // Test 4: Eviction with Write-Back.
        //---------------------------------------------------
        // Same index but a different tag to force eviction.
        cpu_address = 10'd223;
        CPU_RW = 0; // Read mode triggers allocation.
        #(1*PERIOD);
        
        // Now, write new data into address 70.
        cpu_address = 10'd223;
        CPU_RW = 1;
        cpu_data_write = 10'd500;
        #(1*PERIOD);
        
        // Read back address 70.
        cpu_address = 10'd223;
        CPU_RW = 0;
        #(3*PERIOD);

        //---------------------------------------------------
        // Additional Checks.
        //---------------------------------------------------
        cpu_address = 10'd67;
        CPU_RW = 0;
        #(PERIOD);
        
        cpu_address = 10'd84;
        CPU_RW = 0;
        #(PERIOD);

        #(PERIOD);
        $finish;
    end
endmodule



