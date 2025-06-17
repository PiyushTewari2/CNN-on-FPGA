module top_cnn #(
    parameter CHANNEL_IN         = 3,
    parameter PIXEL_WIDTH        = 16,
    parameter WEIGHT_WIDTH       = 9,
    parameter OUT_WIDTH          = 16,
    parameter IMG_DIM_IN         = 224,

    // Layer-wise filter counts
    parameter FILTERS1           = 4,
    parameter FILTERS2           = 8,
    parameter FILTERS3           = 16,
    parameter FILTERS4           = 16,
    parameter FILTERS5           = 8,
    parameter FILTERS6           = 8,  // Linear1 output size
    parameter FILTERS7           = 2,  // Final output size

    // Derived channel counts
    parameter CHANNEL1           = FILTERS1,
    parameter CHANNEL2           = FILTERS2,
    parameter CHANNEL3           = FILTERS3,
    parameter CHANNEL4           = FILTERS4,
    parameter CHANNEL5           = FILTERS5,
    parameter CHANNEL6           = FILTERS6,

    // Image dimensions post each pooling
    parameter IMG_DIM1           = 222,
    parameter IMG_DIM2           = 109,
    parameter IMG_DIM3           = 52,
    parameter IMG_DIM4           = 24,
    parameter IMG_DIM5           = 10,
    parameter IMG_DIM6           = 5,
    parameter IMG_DIM7           = 1
)(
    input wire clk,
    input wire reset,

    input wire [CHANNEL_IN*PIXEL_WIDTH-1:0] pixel_in,
    input wire data_on,

    input wire [WEIGHT_WIDTH-1:0] bias_in,
    input wire [WEIGHT_WIDTH-1:0] weight_in,
    input wire weight_data_on,
    input wire bias_data_on,
    input wire weight_reset,
    input wire bias_reset,

    output wire weights_done,
    output wire bias_done
);

    // Intermediate layer outputs and valids
    wire [CHANNEL1*OUT_WIDTH-1:0] conv1_out;
    wire conv1_valid;
    wire [CHANNEL1*OUT_WIDTH-1:0] pool1_out;
    wire pool1_valid;

    wire [CHANNEL2*OUT_WIDTH-1:0] conv2_out;
    wire conv2_valid;
    wire [CHANNEL2*OUT_WIDTH-1:0] pool2_out;
    wire pool2_valid;

    wire [CHANNEL3*OUT_WIDTH-1:0] conv3_out;
    wire conv3_valid;
    wire [CHANNEL3*OUT_WIDTH-1:0] pool3_out;
    wire pool3_valid;

    wire [CHANNEL4*OUT_WIDTH-1:0] conv4_out;
    wire conv4_valid;
    wire [CHANNEL4*OUT_WIDTH-1:0] pool4_out;
    wire pool4_valid;

    wire [CHANNEL5*OUT_WIDTH-1:0] conv5_out;
    wire conv5_valid;
    wire [CHANNEL5*OUT_WIDTH-1:0] pool5_out;
    wire pool5_valid;

    wire [CHANNEL6*OUT_WIDTH-1:0] conv6_out;
    wire conv6_valid;

    wire [FILTERS7*OUT_WIDTH-1:0] final_out;
    wire final_valid;

    wire w_done1, w_done2, w_done3, w_done4, w_done5, w_done6, w_done7;
    wire b_done1, b_done2, b_done3, b_done4, b_done5, b_done6, b_done7;

    // Layer 1
    convolution #(
        .CHANNEL(CHANNEL_IN), .IMAGE_DIM(IMG_DIM_IN), .STRIDE(1), .ZERO_PAD(0),
        .DATA_WIDTH_INPUT(PIXEL_WIDTH), .DATA_WIDTH_WEIGHT(WEIGHT_WIDTH),
        .FILTER_NUMBER(FILTERS1), .OUT_WIDTH(OUT_WIDTH)
    ) conv1 (
        .clk(clk), .reset(reset),
        .pixel_in(pixel_in), .data_on(data_on),
        .bias_in(bias_in), .weight_in(weight_in),
        .weight_data_on(weight_data_on), .bias_data_on(bias_data_on),
        .weight_reset(weight_reset), .bias_reset(bias_reset),
        .weights_done(w_done1), .bias_done(b_done1),
        .conv_out(conv1_out), .mul_do(conv1_valid)
    );

    maxpooling #(
        .CHANNEL(CHANNEL1), .IMAGE_DIM(IMG_DIM1), .STRIDE(2),
        .DATA_WIDTH(OUT_WIDTH)
    ) pool1 (
        .clk(clk), .reset(reset),
        .pixel_in(conv1_out), .data_on(conv1_valid),
        .pool_out(pool1_out), .pool_valid(pool1_valid)
    );

    // Layer 2
    convolution #(
        .CHANNEL(CHANNEL1), .IMAGE_DIM(IMG_DIM1/2), .STRIDE(1), .ZERO_PAD(0),
        .DATA_WIDTH_INPUT(PIXEL_WIDTH), .DATA_WIDTH_WEIGHT(WEIGHT_WIDTH),
        .FILTER_NUMBER(FILTERS2), .OUT_WIDTH(OUT_WIDTH)
    ) conv2 (
        .clk(clk), .reset(reset),
        .pixel_in(pool1_out), .data_on(pool1_valid),
        .bias_in(bias_in), .weight_in(weight_in),
        .weight_data_on(weight_data_on), .bias_data_on(bias_data_on),
        .weight_reset(weight_reset), .bias_reset(bias_reset),
        .weights_done(w_done2), .bias_done(b_done2),
        .conv_out(conv2_out), .mul_do(conv2_valid)
    );

    maxpooling #(
        .CHANNEL(CHANNEL2), .IMAGE_DIM(IMG_DIM2), .STRIDE(2),
        .DATA_WIDTH(OUT_WIDTH)
    ) pool2 (
        .clk(clk), .reset(reset),
        .pixel_in(conv2_out), .data_on(conv2_valid),
        .pool_out(pool2_out), .pool_valid(pool2_valid)
    );

    // Layer 3
    convolution #(
        .CHANNEL(CHANNEL2), .IMAGE_DIM(IMG_DIM2/2), .STRIDE(1), .ZERO_PAD(0),
        .DATA_WIDTH_INPUT(PIXEL_WIDTH), .DATA_WIDTH_WEIGHT(WEIGHT_WIDTH),
        .FILTER_NUMBER(FILTERS3), .OUT_WIDTH(OUT_WIDTH)
    ) conv3 (
        .clk(clk), .reset(reset),
        .pixel_in(pool2_out), .data_on(pool2_valid),
        .bias_in(bias_in), .weight_in(weight_in),
        .weight_data_on(weight_data_on), .bias_data_on(bias_data_on),
        .weight_reset(weight_reset), .bias_reset(bias_reset),
        .weights_done(w_done3), .bias_done(b_done3),
        .conv_out(conv3_out), .mul_do(conv3_valid)
    );

    maxpooling #(
        .CHANNEL(CHANNEL3), .IMAGE_DIM(IMG_DIM3), .STRIDE(2),
        .DATA_WIDTH(OUT_WIDTH)
    ) pool3 (
        .clk(clk), .reset(reset),
        .pixel_in(conv3_out), .data_on(conv3_valid),
        .pool_out(pool3_out), .pool_valid(pool3_valid)
    );

    // Layer 4
    convolution #(
        .CHANNEL(CHANNEL3), .IMAGE_DIM(IMG_DIM3/2), .STRIDE(1), .ZERO_PAD(0),
        .DATA_WIDTH_INPUT(PIXEL_WIDTH), .DATA_WIDTH_WEIGHT(WEIGHT_WIDTH),
        .FILTER_NUMBER(FILTERS4), .OUT_WIDTH(OUT_WIDTH)
    ) conv4 (
        .clk(clk), .reset(reset),
        .pixel_in(pool3_out), .data_on(pool3_valid),
        .bias_in(bias_in), .weight_in(weight_in),
        .weight_data_on(weight_data_on), .bias_data_on(bias_data_on),
        .weight_reset(weight_reset), .bias_reset(bias_reset),
        .weights_done(w_done4), .bias_done(b_done4),
        .conv_out(conv4_out), .mul_do(conv4_valid)
    );

    maxpooling #(
        .CHANNEL(CHANNEL4), .IMAGE_DIM(IMG_DIM4), .STRIDE(2),
        .DATA_WIDTH(OUT_WIDTH)
    ) pool4 (
        .clk(clk), .reset(reset),
        .pixel_in(conv4_out), .data_on(conv4_valid),
        .pool_out(pool4_out), .pool_valid(pool4_valid)
    );

    // Layer 5
    convolution #(
        .CHANNEL(CHANNEL4), .IMAGE_DIM(IMG_DIM4/2), .STRIDE(1), .ZERO_PAD(0),
        .DATA_WIDTH_INPUT(PIXEL_WIDTH), .DATA_WIDTH_WEIGHT(WEIGHT_WIDTH),
        .FILTER_NUMBER(FILTERS5), .OUT_WIDTH(OUT_WIDTH)
    ) conv5 (
        .clk(clk), .reset(reset),
        .pixel_in(pool4_out), .data_on(pool4_valid),
        .bias_in(bias_in), .weight_in(weight_in),
        .weight_data_on(weight_data_on), .bias_data_on(bias_data_on),
        .weight_reset(weight_reset), .bias_reset(bias_reset),
        .weights_done(w_done5), .bias_done(b_done5),
        .conv_out(conv5_out), .mul_do(conv5_valid)
    );

    maxpooling #(
        .CHANNEL(CHANNEL5), .IMAGE_DIM(IMG_DIM5), .STRIDE(2),
        .DATA_WIDTH(OUT_WIDTH)
    ) pool5 (
        .clk(clk), .reset(reset),
        .pixel_in(conv5_out), .data_on(conv5_valid),
        .pool_out(pool5_out), .pool_valid(pool5_valid)
    );

    // Layer 6: Linear
    LinearLayer #(
        .CHANNEL(CHANNEL5), .IMG_DIM(IMG_DIM6), .FILTER_NUMBER(FILTERS6),
        .DATA_WIDTH_INPUT(PIXEL_WIDTH), .DATA_WIDTH_WEIGHT(WEIGHT_WIDTH),
        .OUT_WIDTH(OUT_WIDTH)
    ) linear1 (
        .clk(clk), .reset(reset),
        .pixel_in(pool5_out), .data_on(pool5_valid),
        .bias_in(bias_in), .weight_in(weight_in),
        .weight_data_on(weight_data_on), .bias_data_on(bias_data_on),
        .weight_reset(weight_reset), .bias_reset(bias_reset),
        .weights_done(w_done6), .bias_done(b_done6),
        .conv_out(conv6_out), .mul_do(conv6_valid)
    );

    // Layer 7: Final linear
    LinearLayer #(
        .CHANNEL(CHANNEL6), .IMG_DIM(IMG_DIM7), .FILTER_NUMBER(FILTERS7),
        .DATA_WIDTH_INPUT(PIXEL_WIDTH), .DATA_WIDTH_WEIGHT(WEIGHT_WIDTH),
        .OUT_WIDTH(OUT_WIDTH)
    ) linear2 (
        .clk(clk), .reset(reset),
        .pixel_in(conv6_out), .data_on(conv6_valid),
        .bias_in(bias_in), .weight_in(weight_in),
        .weight_data_on(weight_data_on), .bias_data_on(bias_data_on),
        .weight_reset(weight_reset), .bias_reset(bias_reset),
        .weights_done(w_done7), .bias_done(b_done7),
        .conv_out(final_out), .mul_do(final_valid)
    );

    assign weights_done = w_done1 & w_done2 & w_done3 & w_done4 & w_done5 & w_done6 & w_done7;
    assign bias_done    = b_done1 & b_done2 & b_done3 & b_done4 & b_done5 & b_done6 & b_done7;

endmodule


/////////////////////////////////////////////////////////////////////////
//Linear Layer
/////////////////////////////////////////////////////////////////////////
module LinearLayer #(
    parameter IMG_DIM = 5,
    parameter CHANNEL = 8,
    parameter FILTER_NUMBER = 8,
    parameter DATA_WIDTH_INPUT = 16,
    parameter DATA_WIDTH_WEIGHT = 12,
    parameter OUT_WIDTH = 20
)(
    input clk,
    input reset,

    input [DATA_WIDTH_WEIGHT-1:0] weight_in,
    input [DATA_WIDTH_WEIGHT-1:0] bias_in,
    input weight_data_on,
    input bias_data_on,
    input weight_reset,
    input bias_reset,

    input [CHANNEL*DATA_WIDTH_INPUT-1:0] pixel_in,
    input data_on,

    output reg weights_done,
    output reg bias_done,
    output reg [FILTER_NUMBER*OUT_WIDTH-1:0] conv_out,
    output reg mul_do
);

    localparam TOTAL_INPUTS = IMG_DIM * IMG_DIM;
    localparam TOTAL_WEIGHTS = TOTAL_INPUTS * CHANNEL * FILTER_NUMBER;
    localparam ADDR_WIDTH_WEIGHT = $clog2(TOTAL_WEIGHTS);
    localparam ADDR_WIDTH_BIAS = $clog2(FILTER_NUMBER);

    reg [DATA_WIDTH_WEIGHT-1:0] weight_mem [0:TOTAL_WEIGHTS-1];
    reg [ADDR_WIDTH_WEIGHT-1:0] weight_wr_addr;

    reg [DATA_WIDTH_WEIGHT-1:0] bias_mem [0:FILTER_NUMBER-1];
    reg [ADDR_WIDTH_BIAS-1:0] bias_wr_addr;

    reg signed [OUT_WIDTH-1:0] acc [0:FILTER_NUMBER-1];

    reg [$clog2(TOTAL_INPUTS)-1:0] input_count;

    reg signed [DATA_WIDTH_INPUT-1:0] pixel;
    reg signed [DATA_WIDTH_WEIGHT-1:0] weight;
    reg signed [OUT_WIDTH-1:0] product;

    integer f, c;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            weights_done <= 0;
            bias_done <= 0;
            weight_wr_addr <= 0;
            bias_wr_addr <= 0;
            input_count <= 0;
            mul_do <= 0;
            conv_out <= 0;
            for (f = 0; f < FILTER_NUMBER; f = f + 1)
                acc[f] <= 0;
        end else begin
            // Weight Loading
            if (weight_reset) begin
                weight_wr_addr <= 0;
                weights_done <= 0;
            end else if (weight_data_on && !weights_done) begin
                weight_mem[weight_wr_addr] <= weight_in;
                weight_wr_addr <= weight_wr_addr + 1;
                if (weight_wr_addr == TOTAL_WEIGHTS - 1)
                    weights_done <= 1;
            end

            // Bias Loading
            if (bias_reset) begin
                bias_wr_addr <= 0;
                bias_done <= 0;
            end else if (bias_data_on && !bias_done) begin
                bias_mem[bias_wr_addr] <= bias_in;
                bias_wr_addr <= bias_wr_addr + 1;
                if (bias_wr_addr == FILTER_NUMBER - 1)
                    bias_done <= 1;
            end

            // Computation
            if (data_on) begin
                mul_do <= 1;
                for (f = 0; f < FILTER_NUMBER; f = f + 1) begin
                    for (c = 0; c < CHANNEL; c = c + 1) begin
                        pixel = pixel_in[c*DATA_WIDTH_INPUT +: DATA_WIDTH_INPUT];
                        weight = weight_mem[(input_count * CHANNEL * FILTER_NUMBER) + (f * CHANNEL + c)];
                        product = pixel * weight;
                        acc[f] <= acc[f] + (product >>> 4);
                    end
                end
                input_count <= input_count + 1;
                if (input_count == TOTAL_INPUTS - 1) begin
                    for (f = 0; f < FILTER_NUMBER; f = f + 1) begin
                        acc[f] <= acc[f] + bias_mem[f];
                        conv_out[f*OUT_WIDTH +: OUT_WIDTH] <= acc[f];
                        acc[f] <= 0;
                    end
                    input_count <= 0;
                end
            end else begin
                mul_do <= 0;
            end
        end
    end
endmodule



////////////////////////////////////////////////////////////////////////
//MAXPOOLING
////////////////////////////////////////////////////////////////////////

module maxpooling #(
    parameter IMAGE_DIM = 32,
    parameter STRIDE = 2,
    parameter ZERO_PAD = 0,
    parameter DATA_WIDTH = 8,
    parameter CHANNEL = 3
)(
    input wire clk,
    input wire reset,
    input wire [CHANNEL*DATA_WIDTH-1:0] pixel_in,
    input wire data_on,
    output wire [CHANNEL*DATA_WIDTH-1:0] pool_out,
    output wire pool_valid
);

    wire [CHANNEL*4*DATA_WIDTH-1:0] window_pixels;

    image_window_extractor_2x2 #(
        .IMAGE_DIM(IMAGE_DIM),
        .ZERO_PAD(ZERO_PAD),
        .DATA_WIDTH(DATA_WIDTH),
        .CHANNEL(CHANNEL),
        .STRIDE(STRIDE)
    ) win_extractor (
        .clk(clk),
        .reset(reset),
        .data_on(data_on),
        .pixel_in(pixel_in),
        .window_out(window_pixels),
        .mul_do(pool_valid)
    );

    genvar ch;
    generate
        for (ch = 0; ch < CHANNEL; ch = ch + 1) begin : pool_ch
            wire [DATA_WIDTH-1:0] p0, p1, p2, p3;
            assign p0 = window_pixels[(ch*4+0)*DATA_WIDTH +: DATA_WIDTH];
            assign p1 = window_pixels[(ch*4+1)*DATA_WIDTH +: DATA_WIDTH];
            assign p2 = window_pixels[(ch*4+2)*DATA_WIDTH +: DATA_WIDTH];
            assign p3 = window_pixels[(ch*4+3)*DATA_WIDTH +: DATA_WIDTH];

            wire [DATA_WIDTH-1:0] max0 = (p0 > p1) ? p0 : p1;
            wire [DATA_WIDTH-1:0] max1 = (p2 > p3) ? p2 : p3;
            wire [DATA_WIDTH-1:0] max_final = (max0 > max1) ? max0 : max1;

            assign pool_out[ch*DATA_WIDTH +: DATA_WIDTH] = max_final;
        end
    endgenerate

endmodule

////////////////////////////////////////////////////////////////////////
//CONVOLUTION
////////////////////////////////////////////////////////////////////////

module convolution #(
    parameter IMAGE_DIM = 32,
    parameter STRIDE = 1,
    parameter FILTER_SIZE = 3,
    parameter ZERO_PAD = 1,
    parameter DATA_WIDTH_INPUT = 30,
    parameter DATA_WIDTH_WEIGHT = 21,
    parameter CHANNEL = 3,
    parameter FILTER_NUMBER = 8,
    parameter OUT_WIDTH = 30
)(
    input wire clk,
    input wire reset,

    input wire [CHANNEL*DATA_WIDTH_INPUT-1:0] pixel_in,
    input wire data_on,

    input wire [DATA_WIDTH_WEIGHT-1:0] bias_in,
    input wire [DATA_WIDTH_WEIGHT-1:0] weight_in,
    input wire weight_data_on,
    input wire bias_data_on,
    input wire weight_reset,
    input wire bias_reset,

    output wire weights_done,
    output wire bias_done,
    output wire [FILTER_NUMBER*OUT_WIDTH-1:0] conv_out,
    output wire mul_do
);

    localparam WINDOW_SIZE = 9;
    localparam TOTAL_WEIGHTS = CHANNEL * WINDOW_SIZE * FILTER_NUMBER;
    localparam ADDR_WIDTH_WEIGHT = $clog2(TOTAL_WEIGHTS);
    localparam MULT_WIDTH = DATA_WIDTH_INPUT + DATA_WIDTH_WEIGHT;
    localparam FRAC_WIDTH_INPUT = 21;
    localparam FRAC_WIDTH_WEIGHT = 21;
    localparam FRAC_WIDTH_OUTPUT = 21;
    localparam TRUNCATE_LSB = 21;

    // Bias memory
    wire [FILTER_NUMBER*DATA_WIDTH_WEIGHT-1:0] flat_bias;

    // Bias Loader
    bias_loader #(
        .FILTER_NUMBER(FILTER_NUMBER),
        .DATA_WIDTH_WEIGHT(DATA_WIDTH_WEIGHT)
    ) bias_loader_inst (
        .clk(clk),
        .bias_reset(bias_reset),
        .bias_data_on(bias_data_on),
        .bias_in(bias_in),
        .bias_done(bias_done),
        .flat_bias(flat_bias)
    );

    // Weight Loader with shift registers
    weight_loader_conv #(
        .CHANNEL(CHANNEL),
        .FILTER_NUMBER(FILTER_NUMBER)
    ) weight_loader_inst (
        .clk(clk),
        .weight_reset(weight_reset),
        .weight_data_on(weight_data_on),
        .weights_done(weights_done)
    );

    // Shift register chain to hold weights
    wire [DATA_WIDTH_WEIGHT-1:0] shift_reg_out [0:TOTAL_WEIGHTS-1];
    wire [DATA_WIDTH_WEIGHT-1:0] shift_reg_in [0:TOTAL_WEIGHTS-1];

    genvar i;
    generate
        for (i = 0; i < TOTAL_WEIGHTS; i = i + 1) begin : shift_chain
            if (i == 0) begin
                assign shift_reg_in[i] = weight_in;
            end else begin
                assign shift_reg_in[i] = shift_reg_out[i-1];
            end

            shift_register #(
                .N(DATA_WIDTH_WEIGHT)
            ) sr_inst (
                .clk(clk),
                .reset(weight_reset),
                .data_on(weight_data_on && !weights_done),
                .data_in(shift_reg_in[i]),
                .data_out(shift_reg_out[i])
            );
        end
    endgenerate

    // Window extractor
    wire [CHANNEL*WINDOW_SIZE*DATA_WIDTH_INPUT-1:0] window_pixels;

    image_window_extractor #(
        .IMAGE_DIM(IMAGE_DIM),
        .ZERO_PAD(ZERO_PAD),
        .DATA_WIDTH(DATA_WIDTH_INPUT),
        .CHANNEL(CHANNEL),
        .STRIDE(STRIDE)
    ) image_window_extractor_inst (
        .clk(clk),
        .reset(reset),
        .data_on(data_on),
        .pixel_in(pixel_in),
        .window_out(window_pixels),
        .mul_do(mul_do)
    );

    // MAC logic directly inside this module
    genvar f, c, w;
    generate
        for (f = 0; f < FILTER_NUMBER; f = f + 1) begin : filter_loop
            wire signed [OUT_WIDTH-1:0] partial_sum [0:CHANNEL*WINDOW_SIZE-1];

            for (c = 0; c < CHANNEL; c = c + 1) begin : channel_loop
                for (w = 0; w < WINDOW_SIZE; w = w + 1) begin : win_loop
                    wire signed [DATA_WIDTH_INPUT-1:0] pixel = window_pixels[(c*WINDOW_SIZE + w)*DATA_WIDTH_INPUT +: DATA_WIDTH_INPUT];
                    wire signed [DATA_WIDTH_WEIGHT-1:0] weight = shift_reg_out[(f*CHANNEL*WINDOW_SIZE) + (c*WINDOW_SIZE + w)];

                    wire signed [MULT_WIDTH-1:0] product_full = pixel * weight;
                    wire signed [MULT_WIDTH-TRUNCATE_LSB-1:0] product_trunc = product_full[MULT_WIDTH-1:TRUNCATE_LSB];
                    wire signed [OUT_WIDTH-1:0] product_ext = {{(OUT_WIDTH-(MULT_WIDTH-TRUNCATE_LSB)){product_trunc[MULT_WIDTH-TRUNCATE_LSB-1]}}, product_trunc};

                    assign partial_sum[c*WINDOW_SIZE + w] = product_ext;
                end
            end

            // Accumulate all products
            wire signed [OUT_WIDTH-1:0] acc_temp [0:CHANNEL*WINDOW_SIZE];
            assign acc_temp[0] = partial_sum[0];
            for (w = 1; w < CHANNEL*WINDOW_SIZE; w = w + 1) begin : acc_loop
                assign acc_temp[w] = acc_temp[w-1] + partial_sum[w];
            end

            // Add bias
            wire signed [DATA_WIDTH_WEIGHT-1:0] bias_raw = flat_bias[f*DATA_WIDTH_WEIGHT +: DATA_WIDTH_WEIGHT];
            wire signed [OUT_WIDTH-1:0] bias_ext = {{(OUT_WIDTH-DATA_WIDTH_WEIGHT){bias_raw[DATA_WIDTH_WEIGHT-1]}}, bias_raw};
            wire signed [OUT_WIDTH-1:0] acc_with_bias = acc_temp[CHANNEL*WINDOW_SIZE-1] + bias_ext;

            // ReLU
            wire [OUT_WIDTH-1:0] relu_out = acc_with_bias[OUT_WIDTH-1] ? {OUT_WIDTH{1'b0}} : acc_with_bias;

            // Assign to output
            assign conv_out[f*OUT_WIDTH +: OUT_WIDTH] = relu_out;
        end
    endgenerate

endmodule


////////////////////////////////////////////////////////////////////////
//BIAS_LOADER
////////////////////////////////////////////////////////////////////////

module bias_loader #(
    parameter FILTER_NUMBER = 8,
    parameter DATA_WIDTH_WEIGHT = 9
)(
    input wire clk,
    input wire bias_reset,
    input wire bias_data_on,
    input wire [DATA_WIDTH_WEIGHT-1:0] bias_in,
    output reg bias_done,
    output reg [FILTER_NUMBER*DATA_WIDTH_WEIGHT-1:0] flat_bias
);

    reg [$clog2(FILTER_NUMBER):0] bias_ptr;

    always @(posedge clk or posedge bias_reset) begin
        if (bias_reset) begin
            bias_ptr <= 0;
            flat_bias <= 0;
            bias_done <= 0;
        end else if (bias_data_on && !bias_done) begin
            flat_bias[bias_ptr*DATA_WIDTH_WEIGHT +: DATA_WIDTH_WEIGHT] <= bias_in;
            bias_ptr <= bias_ptr + 1;
            if (bias_ptr == FILTER_NUMBER-1)
                bias_done <= 1;
        end
    end
endmodule

////////////////////////////////////////////////////////////////////////
//WEIGHT_LOADER_CONV
////////////////////////////////////////////////////////////////////////

module weight_loader_conv #(
    parameter CHANNEL = 3,
    parameter FILTER_NUMBER = 8
)(
    input wire clk,
    input wire weight_reset,
    input wire weight_data_on,
    output reg weights_done
);

    localparam TOTAL_WEIGHTS = CHANNEL * 9 * FILTER_NUMBER;
    localparam ADDR_WIDTH = $clog2(TOTAL_WEIGHTS);


    // Write pointer and weights_done logic
    reg [ADDR_WIDTH-1:0] write_ptr;
    always @(posedge clk or posedge weight_reset) begin
        if (weight_reset) begin
            write_ptr <= 0;
            weights_done <= 0;
        end else if (weight_data_on && !weights_done) begin
            write_ptr <= write_ptr + 1;
            if (write_ptr == TOTAL_WEIGHTS - 1) begin
                weights_done <= 1;
            end
        end
    end

endmodule

////////////////////////////////////////////////////////////////////////
//IMAGE_WINDOW_EXTRACTOR_3*3
////////////////////////////////////////////////////////////////////////

module image_window_extractor_2x2 #(
    parameter IMAGE_DIM = 32,
    parameter ZERO_PAD = 0,
    parameter DATA_WIDTH = 8,
    parameter CHANNEL = 3,
    parameter STRIDE = 2
)(
    input wire clk,
    input wire reset,
    input wire data_on,
    input wire [CHANNEL*DATA_WIDTH-1:0] pixel_in,
    output wire [CHANNEL*4*DATA_WIDTH-1:0] window_out,
    output reg mul_do
);

    localparam ROW_LEN = IMAGE_DIM + 2*ZERO_PAD;

    genvar ch, i;
    generate
        for (ch = 0; ch < CHANNEL; ch = ch + 1) begin : channel_block
            wire [DATA_WIDTH-1:0] row_buffer [0:ROW_LEN-1]; // one line buffer per channel

            // Shift registers for the row buffer
            shift_register #(.N(DATA_WIDTH)) sr_row0 (
                .clk(clk),
                .reset(reset),
                .data_on(data_on),
                .data_in(pixel_in[ch*DATA_WIDTH +: DATA_WIDTH]),
                .data_out(row_buffer[0])
            );
            for (i = 1; i < ROW_LEN; i = i + 1) begin : row_buf
                shift_register #(.N(DATA_WIDTH)) sr_row (
                    .clk(clk),
                    .reset(reset),
                    .data_on(data_on),
                    .data_in(row_buffer[i-1]),
                    .data_out(row_buffer[i])
                );
            end

            // Registers to hold current row pixels (streamed in)
            reg [DATA_WIDTH-1:0] curr_pix0, curr_pix1;
            always @(posedge clk) begin
                if (data_on) begin
                    curr_pix0 <= pixel_in[ch*DATA_WIDTH +: DATA_WIDTH];
                    curr_pix1 <= row_buffer[0];
                end
            end

            // Extract 2x2 window: [prev_row[i], prev_row[i+1], curr_row[i], curr_row[i+1]]
            assign window_out[ch*4*DATA_WIDTH +: 4*DATA_WIDTH] = {
                row_buffer[1], row_buffer[0], // previous row pixels
                curr_pix1, curr_pix0          // current row pixels
            };
        end
    endgenerate

    // Valid signal logic
    reg [$clog2(ROW_LEN)-1:0] x_pos;
    reg [$clog2(IMAGE_DIM + 2*ZERO_PAD)-1:0] y_pos;
    reg [$clog2(ROW_LEN*2):0] data_count;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            x_pos <= 0;
            y_pos <= 0;
            data_count <= 0;
        end else if (data_on) begin
            data_count <= data_count + 1;
            if (x_pos == ROW_LEN - 1) begin
                x_pos <= 0;
                y_pos <= y_pos + 1;
            end else begin
                x_pos <= x_pos + 1;
            end
        end
    end

    always @(*) begin
        // We need 2 rows filled to form valid 2x2 window
        if (data_count >= ROW_LEN + 1) begin
            mul_do = ((x_pos % STRIDE == 0) && (y_pos % STRIDE == 0));
        end else begin
            mul_do = 0;
        end
    end

endmodule



////////////////////////////////////////////////////////////////////////
//IMAGE_WINDOW_EXTRACTOR
////////////////////////////////////////////////////////////////////////

module image_window_extractor #(
    parameter IMAGE_DIM = 32,
    parameter ZERO_PAD = 1,
    parameter DATA_WIDTH = 8,
    parameter CHANNEL = 3,
        parameter STRIDE = 1
)(
    input wire clk,
    input wire reset,
    input wire data_on,
    input wire [CHANNEL*DATA_WIDTH-1:0] pixel_in,
    output wire [CHANNEL*9*DATA_WIDTH-1:0] window_out,
    output reg mul_do
);

    localparam ROW_LEN = IMAGE_DIM + 2*ZERO_PAD;

    // Shift Register Module (as provided)
    // module shift_register #(parameter N = 8)(...); // Assume already defined elsewhere

    genvar ch, i;

    // For each channel
    generate
        for (ch = 0; ch < CHANNEL; ch = ch + 1) begin : channel_block
            // --- First horizontal chain ---
            wire [DATA_WIDTH-1:0] row1_chain [0:ROW_LEN-1];
            // --- Second horizontal chain ---
            wire [DATA_WIDTH-1:0] row2_chain [0:ROW_LEN-1];

            // First element of row1_chain gets pixel_in for this channel
            shift_register #(.N(DATA_WIDTH)) sr_row1_0 (
                .clk(clk),
                .reset(reset),
                .data_on(data_on),
                .data_in(pixel_in[ch*DATA_WIDTH +: DATA_WIDTH]),
                .data_out(row1_chain[0])
            );
            // Rest of row1_chain
            for (i = 1; i < ROW_LEN; i = i + 1) begin : row1_chain_gen
                shift_register #(.N(DATA_WIDTH)) sr_row1 (
                    .clk(clk),
                    .reset(reset),
                    .data_on(data_on),
                    .data_in(row1_chain[i-1]),
                    .data_out(row1_chain[i])
                );
            end

            // row2_chain[0] gets last of row1_chain
            shift_register #(.N(DATA_WIDTH)) sr_row2_0 (
                .clk(clk),
                .reset(reset),
                .data_on(data_on),
                .data_in(row1_chain[ROW_LEN-1]),
                .data_out(row2_chain[0])
            );
            // Rest of row2_chain
            for (i = 1; i < ROW_LEN; i = i + 1) begin : row2_chain_gen
                shift_register #(.N(DATA_WIDTH)) sr_row2 (
                    .clk(clk),
                    .reset(reset),
                    .data_on(data_on),
                    .data_in(row2_chain[i-1]),
                    .data_out(row2_chain[i])
                );
            end

            // --- Vertical shift registers (depth 3) for each of 3 columns ---
            wire [DATA_WIDTH-1:0] col0 [0:2];
            wire [DATA_WIDTH-1:0] col1 [0:2];
            wire [DATA_WIDTH-1:0] col2 [0:2];

            // col0: pixel_in chain
            shift_register #(.N(DATA_WIDTH)) sr_col0_0 (
                .clk(clk),
                .reset(reset),
                .data_on(data_on),
                .data_in(pixel_in[ch*DATA_WIDTH +: DATA_WIDTH]),
                .data_out(col0[0])
            );
            shift_register #(.N(DATA_WIDTH)) sr_col0_1 (
                .clk(clk),
                .reset(reset),
                .data_on(data_on),
                .data_in(col0[0]),
                .data_out(col0[1])
            );
            shift_register #(.N(DATA_WIDTH)) sr_col0_2 (
                .clk(clk),
                .reset(reset),
                .data_on(data_on),
                .data_in(col0[1]),
                .data_out(col0[2])
            );

            // col1: row1_chain[ROW_LEN-1] chain
            shift_register #(.N(DATA_WIDTH)) sr_col1_0 (
                .clk(clk),
                .reset(reset),
                .data_on(data_on),
                .data_in(row1_chain[ROW_LEN-1]),
                .data_out(col1[0])
            );
            shift_register #(.N(DATA_WIDTH)) sr_col1_1 (
                .clk(clk),
                .reset(reset),
                .data_on(data_on),
                .data_in(col1[0]),
                .data_out(col1[1])
            );
            shift_register #(.N(DATA_WIDTH)) sr_col1_2 (
                .clk(clk),
                .reset(reset),
                .data_on(data_on),
                .data_in(col1[1]),
                .data_out(col1[2])
            );

            // col2: row2_chain[ROW_LEN-1] chain
            shift_register #(.N(DATA_WIDTH)) sr_col2_0 (
                .clk(clk),
                .reset(reset),
                .data_on(data_on),
                .data_in(row2_chain[ROW_LEN-1]),
                .data_out(col2[0])
            );
            shift_register #(.N(DATA_WIDTH)) sr_col2_1 (
                .clk(clk),
                .reset(reset),
                .data_on(data_on),
                .data_in(col2[0]),
                .data_out(col2[1])
            );
            shift_register #(.N(DATA_WIDTH)) sr_col2_2 (
                .clk(clk),
                .reset(reset),
                .data_on(data_on),
                .data_in(col2[1]),
                .data_out(col2[2])
            );

            // --- Concatenate the 3x3 window for this channel ---
            assign window_out[ch*9*DATA_WIDTH +: 9*DATA_WIDTH] = {
                col2[2], col2[1], col2[0], // bottom row
                col1[2], col1[1], col1[0], // middle row
                col0[2], col0[1], col0[0]  // top row
            };
        end
    endgenerate

    // --- mul_do: Asserted when enough data has been loaded to form a valid window ---
    // This is a simple example: you may want a more precise condition
    reg [$clog2(ROW_LEN)-1:0] x_pos;
    reg [$clog2(IMAGE_DIM + 2*ZERO_PAD)-1:0] y_pos;
    reg [$clog2(ROW_LEN*3):0] data_count;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            x_pos <= 0;
            y_pos <= 0;
        end else if (data_on) begin
            if (x_pos == ROW_LEN - 1) begin
                x_pos <= 0;
                y_pos <= y_pos + 1;
            end else begin
                x_pos <= x_pos + 1;
            end
        end
    end


    always @(*) begin
        // We need at least 3 rows filled to form valid window
        if (data_count >= ROW_LEN * 2 + 2) begin
            mul_do = ((x_pos % STRIDE == 0) && (y_pos % STRIDE == 0));
        end else begin
            mul_do = 0;
        end
    end

endmodule

////////////////////////////////////////////////////////////////////////
//SHIFT_REGISTER
////////////////////////////////////////////////////////////////////////

module shift_register #(
    parameter N = 8
)(
    input wire clk,
    input wire reset,
    input wire data_on,
    input wire [N-1:0] data_in,
    output reg [N-1:0] data_out
);
    always @(posedge clk) begin
        if (reset)
            data_out <= 0;
        else if (data_on)
            data_out <= data_in;
    end
endmodule                                                                                                                 
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  