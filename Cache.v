`timescale 1ns/1ps

module Cache(
    input clk,
    input rst,
    input CPU_RW,
    input [19:0] cpu_data_in,
    input [9:0] cpu_address,
    input [19:0] mem_data_from_ram,  // data incoming from ram
    input mem_ready,
    output reg [19:0] cpu_data_out,
    output reg cache_ready,
    output reg [9:0] mem_addr,
    output reg [19:0] mem_data_to_ram, // data going to the ram.
    output reg mem_rw,
    output reg mem_req
    );
    
    //making sure I can index throught the blocks we have.
    reg valid [0:15];
    reg dirty [0:15];
    reg [4:0] cache_tag [0:15];
    reg [19:0] cache_data [0:15];
    
    wire [3:0] cpu_index = cpu_address[4:1];
    wire [4:0] cpu_tag   = cpu_address[9:5];
    wire cpu_offset = cpu_address[0];    
    
    reg [1:0] state;
    reg [1:0] next_state;
     reg [1:0] mem_phase;
     
    // start of the state machine for the cache to go from idle to compare to write back and allocation.  
    parameter IDLE_COMPARE      = 2'b00;
    parameter WRITEBACK         = 2'b01;
    parameter ALLOCATION        = 2'b10;
    
    
    wire hit = valid[cpu_index] && (cache_tag[cpu_index] == cpu_tag);
    
    // Temporary register to capture data fetched from memory.
    reg [19:0] new_block;
        
    // setting all the blocks in the cache to be 0 initially.
    integer i;
    initial begin
        next_state <= IDLE_COMPARE;
        cache_ready = 1'b0;
        for(i=0; i<16; i=i+1) begin
            valid[i] <= 1'b0;
            dirty[i] <= 1'b0;
            cache_tag[i] <= 5'b00000;
            cache_data[i] <= 20'd0;
        end
     end
  
    
    // Combinational Logic: Next State and Output Signals
    always @(state, hit, mem_phase, CPU_RW, mem_ready) begin
        // Default assignments.
        next_state    = state;
        cache_ready   = 0;
        mem_req       = 0;
        mem_rw        = 0;
        mem_addr      = 10'b0;
        mem_data_to_ram   = 20'b0;
        cpu_data_out  = 20'b0;
        
        case (state)
            IDLE_COMPARE: begin
                cache_ready = 1'b1;
                if (cpu_address != 10'b0) begin
                    if (hit) begin
                        // On hit, simply serve the CPU request.
                        next_state = IDLE_COMPARE;
                        if (!CPU_RW) begin
                            // Read: select the proper word.
                            cpu_data_out = (cpu_offset) ? cache_data[cpu_index][19:10] :
                                                          cache_data[cpu_index][9:0];
                        end else begin
                            // Write hit: output data to CPU (the cache update happens in sequential logic).
                            cpu_data_out = cpu_data_in;
                        end
                    end else begin
                        // On miss, if the cache line is valid and dirty, prepare for write-back;
                        // otherwise, move directly to allocation.
                        next_state = (valid[cpu_index] && dirty[cpu_index]) ? WRITEBACK : ALLOCATION;
                    end
                end else begin
                    next_state = IDLE_COMPARE;
                end
            end
            
            WRITEBACK: begin
                // Write-back phase: write one word per phase.
                mem_req = 1'b1;
                mem_rw  = 1'b1;  // Write operation.
                mem_addr = {cache_tag[cpu_index], cpu_index, mem_phase[0]};
                if (mem_phase == 0)
                    mem_data_to_ram = {10'b0, cache_data[cpu_index][9:0]};
                else
                    mem_data_to_ram = {10'b0, cache_data[cpu_index][19:10]};
                // After writing both words, proceed to allocation.
                if (mem_phase == 1 && mem_ready)
                    next_state = ALLOCATION;
            end
            
            ALLOCATION: begin
                // Allocation phase: read one word per phase.
                mem_req = 1'b1;
                mem_rw  = 1'b0;  // Read operation.
                mem_addr = {cpu_tag, cpu_index, mem_phase[0]};
                // When both words have been read, update cache line and return to idle.
                if (mem_ready && (mem_phase == 1)) begin
                    next_state  = IDLE_COMPARE;
                    cache_ready = 1'b1;
                end
            end
            
            default: next_state = IDLE_COMPARE;
        endcase
    end

    // Sequential Logic: State Update and Memory Phase
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= IDLE_COMPARE;
            mem_phase <= 0;
        end else begin
            state <= next_state;
            if ((state == WRITEBACK || state == ALLOCATION) && mem_ready)
                mem_phase <= mem_phase + 1;
            else if (state == IDLE_COMPARE)
                mem_phase <= 0;
        end
    end

    // Sequential Logic: Capturing Memory Data and Cache Updates
    always @(posedge clk or posedge rst) begin
    if (rst) begin
        new_block <= 20'd0;
    end else begin
        // Capture incoming memory data during ALLOCATION.
        if (state == ALLOCATION && mem_ready) begin
            if (mem_phase == 0)
                new_block[9:0]  <= mem_data_from_ram[9:0];   // Capture lower word.
            else if (mem_phase == 1)
                new_block[19:10] <= mem_data_from_ram[9:0];   // Capture upper word.
        end

        // Update cache for write hits.
        if (state == IDLE_COMPARE && cpu_address != 10'b0 && hit && CPU_RW) begin
            if (cpu_offset)
                cache_data[cpu_index][19:10] <= cpu_data_in;
            else
                cache_data[cpu_index][9:0]  <= cpu_data_in;
            dirty[cpu_index] <= 1'b1;
        end

        // After allocation completes (i.e., both words captured), update the cache line.
        if (state == ALLOCATION && mem_ready && (mem_phase == 1)) begin
            cache_data[cpu_index] <= new_block;
            cache_tag[cpu_index]  <= cpu_tag;
            valid[cpu_index]      <= 1'b1;
            if (CPU_RW) begin
                if (cpu_offset)
                    cache_data[cpu_index][19:10] <= cpu_data_in;
                else
                    cache_data[cpu_index][9:0]  <= cpu_data_in;
                dirty[cpu_index] <= 1'b1;
            end else begin
                dirty[cpu_index] <= 1'b0;
            end
        end
    end
end

endmodule


module tb_Cache;
    // Signal declarations.
    reg clk;
    reg rst;
    reg CPU_RW;           // 0: read, 1: write.
    reg [19:0] cpu_data_in;
    reg [9:0] cpu_address;
    wire [19:0] cpu_data_out;
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
        .mem_data_from_ram(ram_rdata), // Data coming from RAM.
        .mem_ready(mem_ready),
        .cpu_data_out(cpu_data_out),
        .cache_ready(cache_ready),
        .mem_addr(mem_addr),
        .mem_data_to_ram(mem_data_out_to_ram), // Data written back to RAM.
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
        // Initialize signals.
        rst = 1;
        CPU_RW = 0;       // Default to read.
        cpu_data_in = 20'd0;
        cpu_address = 10'd0;
        mem_ready = 1;    // Assume memory is always ready.
        # (2 * PERIOD);
        rst = 0;
        #PERIOD;

        // -------------------------------
        // Test 1: Read Miss then Read Hit
        // -------------------------------
        cpu_address = 10'd50; // Issue read at address 50.
        CPU_RW = 0;          // Read mode.
        #PERIOD;
        
        // Allow time for the cache to complete the fetch.
        # (2 * PERIOD);

        cpu_address = 10'd50; // Issue second read to the same address.
        CPU_RW = 0;          // Read mode.
        #PERIOD;

        // -------------------------------
        // Test 2: Write then Read Back
        // -------------------------------
        // Write a new value (e.g., 123) to address 50.
        cpu_address = 10'd50;
        CPU_RW = 1;          // Write mode.
        cpu_data_in = 20'd123;
        #PERIOD;

        // Now read back from address 50 expecting the new value.
        cpu_address = 10'd50;
        CPU_RW = 0;          // Read mode.
        #PERIOD;

        # (5 * PERIOD);
        $finish;
    end
endmodule

