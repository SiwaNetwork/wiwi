

module mux4to1(
    input wire [1:0] sel,  // 2-bit select input
    input wire in0,        // Input 0
    input wire in1,        // Input 1
    input wire in2,        // Input 2
    input wire in3,        // Input 3
    output reg out         // Output
);

always @* begin
    case (sel)
        2'b00: out = in0;
        2'b01: out = in1;
        2'b10: out = in2;
        2'b11: out = in3;
        default: out = in0; // Safe default
    endcase
end

endmodule







module led_blink_pattern(
    input wire clk,         // Clock input
    input wire rst,       // Active high reset
    output reg led          // LED output
);

// Define states using parameters
parameter BLINK_SHORT_ON = 0;
parameter BLINK_SHORT_OFF = 1;
parameter BLINK_LONG_ON = 2;
parameter BLINK_LONG_OFF = 3;

// Define time durations as parameters (adjust numbers based on your clock frequency)
localparam CLK_FREQ = 48_000_000;  // Clock frequency in Hz
localparam SHORT_BLINK = CLK_FREQ * 0.2;  // 200 ms blink
localparam LONG_BLINK = CLK_FREQ * 1.0;   // 1000 ms blink
localparam IDLE = 0;                      // Idle state for simplicity

// Current state variable
reg [1:0] current_state = BLINK_SHORT_OFF;

// Counter for timing blinks
reg [31:0] counter = 0;

// State machine logic
always @(posedge clk) begin
    if (rst) begin
        // Reset conditions
        led <= 1;
        counter <= 0;
        current_state <= BLINK_SHORT_OFF;
    end else begin
        case (current_state)
            BLINK_SHORT_OFF: begin
                led <= 1;
                if (counter < SHORT_BLINK) counter <= counter + 1;
                else begin
                    counter <= 0;
                    current_state <= BLINK_LONG_ON;
                end
            end
            BLINK_LONG_ON: begin
                led <= 0;
                if (counter < LONG_BLINK) counter <= counter + 1;
                else begin
                    counter <= 0;
                    current_state <= BLINK_LONG_OFF;
                end
            end
            BLINK_LONG_OFF: begin
                led <= 1;
                if (counter < SHORT_BLINK) counter <= counter + 1;
                else begin
                    counter <= 0;
                    current_state <= BLINK_SHORT_ON;
                end
            end
            BLINK_SHORT_ON: begin
                led <= 0;
                if (counter < SHORT_BLINK) counter <= counter + 1;
                else begin
                    counter <= 0;
                    current_state <= BLINK_SHORT_OFF;
                end
            end
            default: begin
                current_state <= BLINK_SHORT_OFF;
            end
        endcase
    end
end

endmodule


module mux2to1 (
    input wire a,
    input wire b,
    input wire sel,
    output wire out
);

    assign out = sel ? b : a;

endmodule

module mux_with_reset (
    input wire a,       // Input a
    input wire b,       // Input b
    input wire sel,     // Select input
    input wire reset,   // Reset signal
	input wire reset_val, 
    output reg out      // Output
);

    always @(*) begin
        if (reset) begin
            out = reset_val;  // Force output when reset is asserted
        end else begin
            if (sel) begin
                out = b;  // Select b if sel is 1
            end else begin
                out = a;  // Select a if sel is 0
            end
        end
    end

endmodule

	

