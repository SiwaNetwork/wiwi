

module synchronizer (
    input wire clk,        // Clock input
    input wire async_in,   // Asynchronous GPIO input
    output wire sync_out   // Synchronized output
);

    // Intermediate signals for the two flip-flops
    reg sync_ff1;
    reg sync_ff2;

    // Synchronizer logic
    always @(posedge clk) begin
        sync_ff1 <= async_in;  // First stage flip-flop
        sync_ff2 <= sync_ff1;  // Second stage flip-flop
    end

    // Assign synchronized output
    assign sync_out = sync_ff2;

endmodule




module single_bit_fifo_cdc (
    input wire clk,       // Clock input
    input wire rst,       // Reset input
    input wire data_in,   // Single-bit data input (sampled on falling edge)
    output reg data_out   // Single-bit data output (available on rising edge)
);

    reg [1:0] fifo;       // 2-bit FIFO to hold the data
    reg rd_ptr, wr_ptr;   // Read and write pointers

    // Sample data on the falling edge of the clock and write to FIFO
    always @(negedge clk or posedge rst) begin
        if (rst) begin
            wr_ptr <= 1;
            fifo[0] <= 0;
            fifo[1] <= 0;
        end else begin
            fifo[wr_ptr] <= data_in;
            wr_ptr <= ~wr_ptr;
        end
    end

    // Read data from FIFO on the rising edge of the clock
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rd_ptr <= 0;
            data_out <= 0;
        end else begin
            data_out <= fifo[rd_ptr];
            rd_ptr <= ~rd_ptr;
        end
    end

endmodule







module data_synchronizer (
    input wire clk,       // Clock input
    input wire rst,       // Reset input
    input wire data_in,   // Data input
    output reg data_out   // Synchronized data output
);

    reg sampled_data;     // Data sampled on falling edge
    reg fifo_stage1, fifo_stage2;  // FIFO stages for synchronization

    // Sample data on the falling edge of the clock
    always @(negedge clk or posedge rst) begin
        if (rst) begin
            sampled_data <= 0;
        end else begin
            sampled_data <= data_in;
        end
    end

    // FIFO stages for synchronizing data to the rising edge
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            fifo_stage1 <= 0;
            fifo_stage2 <= 0;
            data_out <= 0;
        end else begin
            fifo_stage1 <= sampled_data;  // Capture sampled data in first stage
            fifo_stage2 <= fifo_stage1;   // Transfer to second stage
            data_out <= fifo_stage2;      // Output synchronized data
        end
    end

endmodule




module dio0_state_machine (
    input wire clk,            // Clock input
    input wire reset,          // Asynchronous reset
    input wire dio0_in,        // Asynchronous input signal 1
    input wire stm_toggle_in,  // Asynchronous input signal 2
    output reg output_signal   // Output signal
);

    // State encoding
    localparam RESET   = 2'b00;
    localparam STATE_1 = 2'b01;
    localparam STATE_2 = 2'b10;

    // State registers
    reg [1:0] current_state, next_state;

    // Synchronizer registers
    reg dio0_in_sync1, dio0_in_sync2;
    reg stm_toggle_in_sync1, stm_toggle_in_sync2;

    // Input edge detection registers
    reg dio0_in_d, stm_toggle_in_d;

    // State transition and output logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state <= RESET;
            dio0_in_sync1 <= 1'b0;
            dio0_in_sync2 <= 1'b0;
            stm_toggle_in_sync1 <= 1'b0;
            stm_toggle_in_sync2 <= 1'b0;
            dio0_in_d <= 1'b0;
            stm_toggle_in_d <= 1'b0;
        end else begin
            current_state <= next_state;

            // Synchronize dio0_in to the clock domain
            dio0_in_sync1 <= dio0_in;
            dio0_in_sync2 <= dio0_in_sync1;
            dio0_in_d <= dio0_in_sync2;

            // Synchronize stm_toggle_in to the clock domain
            stm_toggle_in_sync1 <= stm_toggle_in;
            stm_toggle_in_sync2 <= stm_toggle_in_sync1;
            stm_toggle_in_d <= stm_toggle_in_sync2;
        end
    end

    // Next state logic
    always @(*) begin
        // Default next state is the current state
        next_state = current_state;
        output_signal = 1'b1; // Default output is 1

        case (current_state)
            RESET: begin
                output_signal = 1'b1;
                if (dio0_in_sync2 && !dio0_in_d) begin // Detect rising edge on dio0_in
                    next_state = STATE_1;
                end
            end

            STATE_1: begin
                output_signal = 1'b0;
                if (stm_toggle_in_sync2 && !stm_toggle_in_d) begin // Detect rising edge on stm_toggle_in
                    next_state = STATE_2;
                end
            end

            STATE_2: begin
                output_signal = 1'b0;
                if (!stm_toggle_in_sync2 && stm_toggle_in_d) begin // Detect falling edge on stm_toggle_in
                    next_state = RESET;
                end
            end

            default: begin
                next_state = RESET;
            end
        endcase
    end
endmodule












module data_serializer (
    input wire clk,             // Clock input
    input wire rst,             // Reset input
    input wire [12:0] data_in,  // 13-bit parallel data input
    input wire data_strobe,     // Load signal to load data_in into the shift register
    output wire clk_out,         // Clock output for shifting data (conditionally enabled)
    output reg data_out         // Serial data output
);

    reg [15:0] shift_reg;
	reg clk_out_enable;
    reg [5:0] bit_counter;
	
	reg [15:0] test_counter; 
	
    // State encoding
    parameter STATE_A = 1'b0, STATE_B = 1'b1;
	// Current and next state variables
    reg state, next_state;
	reg data_strobe_d;  // Delayed data_strobe to detect rising edge
	
    // Next state logic
    always @(*) begin
        next_state = state; // Default is to stay in the current state
        if (data_strobe && !data_strobe_d) begin  // Detect rising edge of data_strobe
            case (state)
                STATE_A: next_state = STATE_B;
                STATE_B: next_state = STATE_A;
                default: next_state = STATE_A;
            endcase
        end
    end
	
    // State transition logic
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= STATE_A;
            data_strobe_d <= 0;
        end else begin
            state <= next_state;
            data_strobe_d <= data_strobe;  // Capture the data_strobe
        end
    end
	

	
	// clk_out_enable control 
	always @(negedge clk or posedge rst) begin
		if (rst) begin
			clk_out_enable <= 1'b0;
		//end else if ( bit_counter == 16 ) begin
		end else if ( bit_counter != 0 ) begin
			clk_out_enable <= 1'b1;
		//end else if ( bit_counter == 0 ) begin
		end else begin
			clk_out_enable <= 1'b0;
		end
	end
	

	assign clk_out = (clk_out_enable) ? clk : 0;	
	

    // Load data into the shift register on the data_strobe signal
	// shift data out on rising edge, use CPOL=0 / CPHA=1
	// SX1257 uses CPOL=0/CPHA=0 , so its different from SX1257
	// but makes it easier
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bit_counter <= 6'b0;
			data_out <= 0;
			shift_reg <= 16'b0;
			test_counter <= 16'b0;
        end else if (data_strobe) begin
            bit_counter <= 6'd16;
			//shift_reg <= { 3'd0  , data_in}; // just add leading zeroes
			
			
			shift_reg <= { { 3{ data_in[12] } }, data_in }; // sign extend -> THIS IS MAIN PATH
			//shift_reg <= test_counter;
			test_counter <= test_counter + 1;
			
			//

			/*
			if ( state == STATE_A ) begin
				shift_reg <= 16'hdead; // hack constant to verify getting to stm proper
			end else begin
				shift_reg <= 16'hbeef;
			end
			*/
        end else if ( bit_counter != 0 ) begin
			bit_counter <= bit_counter - 1'b1;
			data_out <= shift_reg[0];
            shift_reg <= {1'b0, shift_reg[15:1]};
		end else begin
			data_out <= 0;
		end
    end


endmodule




