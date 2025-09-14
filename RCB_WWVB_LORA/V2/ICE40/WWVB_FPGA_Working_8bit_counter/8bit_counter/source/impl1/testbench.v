module cascaded_cic_filter_tb;

    reg clk;
    reg rst;
    reg data_in;
    wire signed [12:0] data_out;
    wire data_strobe;
    wire clk_out;
    wire serial_data_out;

    // Instantiate the cascaded CIC filter
    cascaded_cic_filter #(
        .DECIMATION_FACTOR(32),
        .NUM_STAGES(3),
        .BIT_WIDTH(16)
    ) cic_filter_inst (
        .clk(clk),
        .rst(rst),
        .data_in(data_in),
        .data_out(data_out),
        .data_strobe(data_strobe)
    );

    // Instantiate the data serializer
    data_serializer serializer_inst (
        .clk(clk),
        .rst(rst),
        .data_in(data_out),
        .data_strobe(data_strobe),
        .clk_out(clk_out),
        .data_out(serial_data_out)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        // Initialize signals
        clk = 0;
        rst = 1;
        data_in = 0;
        
        // Reset the DUT
        #10 rst = 0;

        // Provide input stimulus
        repeat (1024) begin
            #10 data_in = $random % 2;  // Random -1 or 1
        end

        // Finish simulation
        #100 $finish;
    end

    // Monitor signals
    initial begin
        $monitor($time, " data_in=%b, data_out=%d, data_strobe=%b, serial_data_out=%b", data_in, data_out, data_strobe, serial_data_out);
    end

endmodule
