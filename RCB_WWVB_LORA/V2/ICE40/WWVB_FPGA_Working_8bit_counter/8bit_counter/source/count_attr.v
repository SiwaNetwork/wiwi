// 8 bit counter with asynchronous reset
module top(	
	input tim1_ch1,
	inout i2c4_scl,
	inout i2c4_sda,
	input hrtim_chb1,
	input sx1276_dio0,
	output sx1257_sck,
	input stm_fpga_spare5, // 0 for SPI stream disable, 1 for SPI stream enable
	input wire sx1257_clk_out,
	input sx1257_clk1, // not used on board 
	output sx1257_q_in,
	output sx1257_i_in,
	input wire sx1257_q_out,
	input sx1257_miso,
	input wire sx1257_i_out,
	input hrtim_che2,
	output sx1257_mosi,
	output sx1257_nss,
	output spi6_miso,
	input spi6_mosi,
	input spi6_sck,
	input spi6_nss,
	output wire spi2_sck,
	output wire spi1_sck,
	output wire spi1_mosi,
	output spi2_mosi,
	input spi2_miso,
	input spi1_miso,
	input sx_rf_sw_ctrl,
	input hrtim_eev7,
	input tim8_c1,
	input fpga_clk2,
	input stm_fpga_spare1, // sx1257 multiplexer select 
	input stm_fpga_spare2, // ice40 internal spi slave chip select
	input stm_fpga_spare3,
	input wire stm_fpga_spare4, // use as reset line, active high
	input fpga_clk,
	output wire led0,
	output wire led1,
	output wire led2
	);
	
/*
// Internal 48MHz oscillator
wire hfosc_clk_48MHz;	
HSOSC
#(
 .CLKHF_DIV ("0b00")
) OSCInst0 (
 .CLKHFEN (1'b1),
 .CLKHFPU (1'b1),
 .CLKHF (hfosc_clk_48MHz)
);
*/



wire internal_led_signal; 
led_blink_pattern led_blink(
    .clk(sx1257_clk_out),
    .rst(stm_fpga_spare4),           // Convert active high reset to active low for the led_blink_pattern
    .led(internal_led_signal)
);
assign led2 = internal_led_signal;


assign led0 = 1'b1;
assign led1 = 1'b0;







// SX1257 management / config SPI 
// for now, ICE40 doesn't do anything, just pass it directly to SX1257
// spare4 is "reset", so gate this passthrough with that as well
// reset is active high, if high, send 1
// stm_fpga_spare
mux_with_reset sx1257_mgmt_sck_mux (
    .a(spi6_sck), // stm32 direct
    .b(1'b1), // not used for now
    .sel(1'b0),   
    .reset(stm_fpga_spare4),
	.reset_val(1'b1), 
    .out(sx1257_sck) 
);
mux_with_reset sx1257_mgmt_mosi_mux (
    .a(spi6_mosi), // stm32 direct
    .b(1'b1), // not used for now
    .sel(1'b0),   
    .reset(stm_fpga_spare4),
	.reset_val(1'b1), 
    .out(sx1257_mosi) 
);
mux_with_reset sx1257_mgmt_miso_mux (
    .a(sx1257_miso), // stm32 direct
    .b(1'b1), // not used for now
    .sel(1'b0),   
    .reset(stm_fpga_spare4),
	.reset_val(1'b1), 
    .out(spi6_miso) 
);
mux_with_reset sx1257_mgmt_nss_mux (
    .a(spi6_nss), // stm32 direct
    .b(1'b1), // not used for now
    .sel(1'b0),   
    .reset(stm_fpga_spare4),
	.reset_val(1'b1), 
    .out(sx1257_nss) 
);



/********************** Using Alex's CIC decimator ****************/
// Internal signals
wire [12:0] i_demodulated_data;
wire i_data_strobe;
wire i_input_tready;
wire i_output_tready;
wire i_out_synced;

wire [12:0] q_demodulated_data;
wire q_data_strobe;
wire q_input_tready;
wire q_output_tready;
wire q_out_synced;
wire stream_enable_synced;

wire gated_i_strobe;
wire gated_q_strobe;
wire cic_reset;



single_bit_fifo_cdc i_rising_sync (
	.clk(sx1257_clk_out),
	.rst(stm_fpga_spare4),
	.data_in(sx1257_i_out),
	.data_out(i_out_synced)
);
single_bit_fifo_cdc q_rising_sync (
	.clk(sx1257_clk_out),
	.rst(stm_fpga_spare4),
	.data_in(sx1257_q_out),
	.data_out(q_out_synced)
);

synchronizer stream_enable (
	.clk(sx1257_clk_out),
	.async_in(stm_fpga_spare5),
	.sync_out(stream_enable_synced)
);

/*
// have a state machine based on dio0 also gate the stream
wire dio0_state_output;
dio0_state_machine dio0_state (
	.clk(sx1257_clk_out),
	.reset(stm_fpga_spare4),
	.dio0_in(sx1276_dio0),
	.stm_toggle_in(stm_fpga_spare3),
	.output_signal(dio0_state_output)
);
*/



//assign cic_reset = stm_fpga_spare4 | !stream_enable_synced;
// using Alex's cic decimator
cic_decimator #(
	.WIDTH(1),
	.RMAX(32),
	.M(1),
	.N(3)
	//.REG_WIDTH(16)
) i_demod (
	.clk(sx1257_clk_out),
	.rst(stm_fpga_spare4),
	
	.input_tdata(i_out_synced),
	.input_tvalid(1'b1), // always valid, constant stream
	.input_tready(i_input_tready),
	
	.output_tdata(i_demodulated_data),
	.output_tvalid(i_data_strobe),
	.output_tready(1'b1), // always ready , lay it on me
	.rate(6'd32)  // Decimation rate
);
// Instantiate the data serializer

assign gated_i_strobe = i_data_strobe & stream_enable_synced;
data_serializer i_serializer (
	.clk(sx1257_clk_out),
	.rst(stm_fpga_spare4),
	.data_in(i_demodulated_data),
	//.data_in(13'head),
	//.data_strobe(i_data_strobe & stream_enable_synced & dio0_state_output),
	.data_strobe(gated_i_strobe),
	.clk_out(spi1_sck),
	.data_out(spi1_mosi)
);



cic_decimator #(
	.WIDTH(1),
	.RMAX(32),
	.M(1),
	.N(3)
	//.REG_WIDTH(16)
) q_demod (
	.clk(sx1257_clk_out),
	.rst(stm_fpga_spare4),
	
	.input_tdata(q_out_synced),
	.input_tvalid(1'b1), // always valid, constant stream
	.input_tready(q_input_tready),
	
	.output_tdata(q_demodulated_data),
	.output_tvalid(q_data_strobe),
	.output_tready(1'b1), // always ready, lay it on me
	.rate(6'd32)  // Decimation rate
);


assign gated_q_strobe = q_data_strobe & stream_enable_synced;
// Instantiate the data serializer
data_serializer q_serializer (
	.clk(sx1257_clk_out),
	.rst(stm_fpga_spare4),
	.data_in(q_demodulated_data), 
	//.data_in(13'head),
	//.data_strobe(q_data_strobe & stream_enable_synced & dio0_state_output),
	.data_strobe(gated_q_strobe),
	.clk_out(spi2_sck),
	.data_out(spi2_mosi)
);




//assign spi1_sck = 1'b0;
//assign spi1_mosi = 1'b0;
//assign spi2_sck = 1'b0;
//assign spi2_mosi = 1'b0;


// just drive the SDR with constant value, effectively constant tone if you enable TX
assign sx1257_q_in = 1'b1;
assign sx1257_i_in = 1'b1;


endmodule
