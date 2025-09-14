module testbench;

    reg clk;
    reg rst;
    reg data_in;
    reg input_tvalid;
    wire input_tready;
    wire [12:0] decimator_output;
    wire decimator_output_valid;
    reg decimator_output_ready;
    wire clk_out;
    wire data_out;
    reg [5:0] rate;  // Adjusted rate signal width to 6 bits

    // Instantiate the CIC decimator
    cic_decimator #(
        .WIDTH(1),
        .RMAX(32),
        .M(1),
        .N(3),
        .REG_WIDTH(13)  // 13-bit output
    ) decimator (
        .clk(clk),
        .rst(rst),
        .input_tdata(data_in),
        .input_tvalid(input_tvalid),
        .input_tready(input_tready),
        .output_tdata(decimator_output),
        .output_tvalid(decimator_output_valid),
        .output_tready(decimator_output_ready),
        .rate(rate)  // Decimation rate
    );

    // Instantiate the data serializer
    data_serializer #(
    ) serializer (
        .clk(clk),
        .rst(rst),
        .data_in(decimator_output),  // 13-bit input
        .data_strobe(decimator_output_valid),
        .clk_out(clk_out),
        .data_out(data_out)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        // Initialize signals
        clk = 0;
        rst = 1;
        data_in = 0;
        input_tvalid = 0;
        decimator_output_ready = 1;
        rate = 6'd31;  // Set the decimation rate to 31

        // Dump waveforms
        $dumpfile("test.vcd");
        $dumpvars(0, testbench);
        
        // Reset the DUT
        #10 rst = 0;

        // Generate a 50 percent duty cycle signal for testing
        repeat (1000) begin  // Generate 1000 cycles of 50 percent duty cycle signal
            #10 data_in = 1;
            input_tvalid = 1;
            #10 data_in = 0;
            input_tvalid = 1;
        end

        // Finish simulation after a certain period
        #20000 $finish;
    end

    // Monitor signals
    initial begin
        $monitor($time, " rst=%b, clk=%b, data_in=%b, decimator_output=%d, data_out=%b",
                 rst, clk, data_in, decimator_output, data_out);
    end

endmodule
