`timescale 1ns/1ps

module Cache(
    input clk,
    input rst,
    input CPU_RW,
    input [19:0] cpu_data_in,
    input [9:0] cpu_address,
    input [19:0] mem_data_out,
    input mem_ready,
    output reg [19:0] cpu_data_out,
    output reg cache_ready,
    output reg [9:0] mem_addr,
    output reg [19:0] mem_data_in,
    output reg mem_rw,
    output reg mem_req
    );
    
    //making sure I can index throught the blocks we have.
    reg valid [0:15];
    reg dirty [0:15];
    reg [4:0] cache_tag [0:15];
    reg [19:0] cache_data [0:15];
    
    // setting all the blocks in the cache to be 0 initially.
    integer i;
    initial begin
        for(i=0; i<16; i=i+1) begin
            valid[i] <= 1'b0;
            dirty[i] <= 1'b0;
            cache_tag[i] <= 5'b00000;
            cache_data[i] <= 20'd0;
        end
     end
   
    // start of the state machine for the cache to go from idle to compare to write back and allocation.  
    parameter IDLE            = 2'b00;
    parameter COMPARE         = 2'b01;
    parameter WRBA = 2'b10;
    
    reg [1:0] state;
    
    reg [9:0] req_address;
    reg       req_rw;
    reg [19:0] req_data_in;
    
    // this is how the difference between the write back and allocation happens to make sure it works properly.
    reg sub_phase;
    
    wire [3:0] cpu_index = cpu_address[4:1];
    wire [4:0] cpu_tag   = cpu_address[9:5];
    wire [3:0] req_index = req_address[4:1];
    wire [4:0] req_tag   = req_address[9:5];
    
    // Combinational logic for next state and outputs
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state       <= IDLE;
            cache_ready   <= 1;
            mem_req     <= 0;
            sub_phase   <= 0;
        end else begin
            case (state)
                // IDLE: Wait for a CPU request and latch it.
                IDLE: begin
                    cache_ready   <= 1;
                    mem_req     <= 0;
                    req_address    <= cpu_address;
                    req_rw      <= CPU_RW;
                    req_data_in <= cpu_data_in;
                    state       <= COMPARE;
                end

                // COMPARE: Compare the request to the cache line.
                COMPARE: begin
                    cache_ready <= 0;
                    if ( valid[req_address[4:1]] && (cache_tag[req_address[3:0]] == req_address[9:5]) ) begin
                        // Hit: serve request immediately.
                        if (req_rw == 0) begin
                            cpu_data_out <= cache_data[req_address[4:1]];
                        end else begin
                            cache_data[req_address[3:0]] <= req_data_in;
                            dirty[req_address[3:0]]      <= 1;
                            cpu_data_out              <= req_data_in;
                        end
                        state <= IDLE;
                    end else begin
                        // Miss: decide whether a write-back is required.
                        if ( valid[req_address[3:0]] && dirty[req_address[3:0]] ) begin
                            // Victim is dirty: initiate write-back.
                            mem_req     <= 1;
                            mem_rw      <= 1;  // Write.
                            mem_addr    <= { cache_tag[req_address[3:0]], req_address[3:0] };
                            mem_data_in <= cache_data[req_address[3:0]];
                            sub_phase   <= 0;  // Write-back phase.
                        end else begin
                            // No write-back needed; issue a read.
                            mem_req     <= 1;
                            mem_rw      <= 0;  // Read.
                            mem_addr    <= req_address;
                            sub_phase   <= 1;  // Allocation phase.
                        end
                        state <= WRBA;
                    end
                end

                // WRBA: Handle write-back (if needed) and then allocate the new block.
                WRBA: begin
                    if (mem_ready) begin
                        if (sub_phase == 0) begin
                            // Write-back completed. Now issue read to fetch new block.
                            mem_req   <= 1;
                            mem_rw    <= 0;      // Read.
                            mem_addr  <= req_address;
                            sub_phase <= 1;
                        end else begin
                            // Read completed. Update cache.
                            mem_req                     <= 0;
                            cache_data[req_index]       <= mem_data_out;
                            valid[req_index]            <= 1;
                            cache_tag[req_index]        <= req_tag;
                            dirty[req_index]            <= 1'b0;
                            if (req_rw == 1) begin
                                cache_data[req_index]   <= req_data_in;
                                dirty[req_index]        <= 1;
                                cpu_data_out            <= req_data_in;
                            end else begin
                                cpu_data_out            <= mem_data_out;
                            end
                            cache_ready <= 1;
                            state     <= IDLE;
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule


module tb_Cache();



endmodule