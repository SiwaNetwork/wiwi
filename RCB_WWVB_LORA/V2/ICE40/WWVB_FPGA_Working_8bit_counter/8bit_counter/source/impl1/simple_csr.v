/*
 * CSR Register Map:
 *
 * Base Address: BASE_ADDR
 *
 * Offset | Register         | Bit Fields    | R/W
 * --------------------------------------------------
 * 0x00   | Scratch Pad 0     | [7:0]         | R/W
 * 0x01   | Scratch Pad 1     | [7:0]         | R/W
 * 0x02   | Scratch Pad 2     | [7:0]         | R/W
 * 0x03   | Scratch Pad 3     | [7:0]         | R/W
 * 0x04   | Scratch Pad 4     | [7:0]         | R/W
 * 0x05   | Scratch Pad 5     | [7:0]         | R/W
 * 0x06   | Scratch Pad 6     | [7:0]         | R/W
 * 0x07   | Scratch Pad 7     | [7:0]         | R/W
 * 0x08   | Version Major     | [7:0]         | R
 * 0x09   | Version Minor     | [7:0]         | R
 */

module spi_csr_interface #(
    parameter BASE_ADDR = 8'h00  // Base address for the CSR block
)(
    input wire clk,
    input wire rst_n,
    input wire [7:0] sys_addr,      // System bus control registers address
    input wire [7:0] sys_data_in,   // System bus data input
    output reg [7:0] sys_data_out,  // System bus data output
    output reg sys_ack,             // System bus acknowledgement
    input wire sys_read_write,      // System read/write signal (0 = read, 1 = write)
    input wire sys_strobe           // System strobe signal
);

    // Scratch pad registers (8 x 8-bit registers)
    reg [7:0] scratch_pad [0:7];

    // Version registers (read-only)
    wire [7:0] version_major = 8'h01;
    wire [7:0] version_minor = 8'h00;

    // Internal address offset calculation
    wire [7:0] offset_addr = sys_addr - BASE_ADDR;

    // Address decoding and register access
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sys_ack <= 1'b0;
            sys_data_out <= 8'b0;
        end else if (sys_strobe) begin
            sys_ack <= 1'b1;
            if (sys_addr >= BASE_ADDR && sys_addr < (BASE_ADDR + 8'h0A)) begin
                case (offset_addr)
                    8'h00, 8'h01, 8'h02, 8'h03, 8'h04, 8'h05, 8'h06, 8'h07: begin
                        // Scratch pad registers
                        if (sys_read_write) begin
                            // Write operation
                            scratch_pad[offset_addr[2:0]] <= sys_data_in;
                        end else begin
                            // Read operation
                            sys_data_out <= scratch_pad[offset_addr[2:0]];
                        end
                    end
                    8'h08: begin
                        // Version Major register (read-only)
                        sys_data_out <= version_major;
                    end
                    8'h09: begin
                        // Version Minor register (read-only)
                        sys_data_out <= version_minor;
                    end
                    default: sys_data_out <= 8'h00;
                endcase
            end else begin
                sys_ack <= 1'b0;
                sys_data_out <= 8'h00;
            end
        end else begin
            sys_ack <= 1'b0;
        end
    end

endmodule
