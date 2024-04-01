`timescale 1ns / 1ps

module conv_core_reg#(parameter N = 16, S = 2, K = 3) //where N is size of input image and weights, S is stride, and K is the size of convolution
                     (input [N*S*K-1:0]input_layer, 
                      input [N*K-1:0]weights, 
                      input weight_buffer_en, 
                      input input_buffer_en,
                      input weight_en,
                      input input_en,
                      input clk,
                      output [N-1:0]product_result,
                      output one_cell_done,
                      output one_conv_done); //here we have two options between hardare and speed, now we can have single weight and single image at a time and use only one accumulator at a time, or can have multiple output and multiple multipler based on hardware

wire [N-2:0] in1;
wire [N-2:0] in2;
wire [N-2:0]product_obtained;
wire product_done; //valid from product calculated till next input changes

reg [N*K-1:0] input_buffer[K-1:0];
reg [N*K-1:0] weight_buffer[K-1:0];
reg [N*K-1:0] input_buffer_operation;
reg [N*K-1:0] weight_buffer_operation;
reg operation_flag1 = 0;
reg operation_flag2 = 0;


reg write_weight_done = 1'b0;
reg write_input_done = 1'b0;

reg [N-2:0] in1_temp = 0;
reg [N-2:0] in2_temp = 0;
reg [N-2:0] in1_temp_final = 0;
reg [N-2:0] in2_temp_final = 0;

reg [$clog2(N)-1:0] k = 0;
reg [$clog2(K)-1:0] line = 0; 
reg [$clog2(K)-1:0] cell_counter = 0;
reg wait_flag = 1'b0;

reg [N-1:0] zeros = 0;
reg mul_send_done = 1'b0;

reg sign_in1;
reg sign_in2;


//weight_enable would be high when new data is there else it would be low
//as soon as weight enable is high weight_buffer which is an array of registers each of 16 bit, starts filling together(generated)
//once the count is 3 hence it stops and makes the flag of write_weight_done as 1 and hence now it is ready to multiply from weight side
//similar code for input channel as well
genvar i;

generate
    for (i = 0; i < K; i = i+1)
    begin
        always @ (posedge clk)
        begin
            if (weight_buffer_en)    //this signal whould be high when one whole buffer is being written, this is to show whether the buffer is still being written or not
            begin
                if (weight_en)      //this will be high when specific colunmn is being written, this is based on the controller
                begin
                    mul_send_done <= 1'b0;
                    operation_flag1 <= 1'b1;
                    weight_buffer[i] <= {weights[N*(i+1)-1:N*i], weight_buffer[i][K*N-1:N]}; 
                    write_weight_done <= 1'b0;
                end
            end
            else
            begin
                if (operation_flag1 == 1)
                    begin
                        weight_buffer_operation <= weight_buffer[0];
                        operation_flag1 <= 1'b0;
                    end              
                write_weight_done <= 1'b1;
            end
        end
    end
endgenerate

genvar j;

generate
    for (j = 0; j < K; j = j+1)
    begin
        always @ (posedge clk)
        begin
            if (input_buffer_en)
            begin
                if (input_en)
                begin
                    mul_send_done <= 1'b0; 
                    operation_flag2 <= 1'b1;
                    input_buffer[j] <= {input_layer[S*N*(j+1)-1:S*N*j], input_buffer[j][K*N-1:N*S]};  
                    write_input_done <= 1'b0;
                end
            end
            else
            begin
                if (operation_flag2 == 1)
                begin
                    input_buffer_operation <= input_buffer[0];
                    operation_flag2 <= 1'b0;
                end
                write_input_done <= 1'b1;
            end
        end
    end
endgenerate

//once both the registers are written, for input and the weights hence now we can go for multiplication or finding the result
//once multiplication willbe done we will go for accumulation and once mul and add bothe are completed we will generate a signal called one cycle done, hence the other data which is ready can be used
// now since mul enable is high hence I can send it to multiplier block
always @ (posedge clk)
begin
    if (write_input_done == 1'b1 && write_weight_done == 1'b1)
    begin
        if (k == N-1)
        begin 
            if (product_done == 1'b1)
            begin
                k <= 0;
                wait_flag <= 1'b0;
                cell_counter <= cell_counter+1;
                
                in1_temp_final <= in1_temp;
                in2_temp_final <= in2_temp;
                
                sign_in1 <= input_buffer_operation[k];
                sign_in2 <= weight_buffer_operation[k];
                
                if (cell_counter == K-1)
                begin
                    if (line == K-1)
                    begin
                        line <= 0;
                        mul_send_done <= 1'b1;
                        cell_counter <= 0;
                        write_input_done <= 1'b0;
                        write_weight_done <= 1'b0;
                        input_buffer_operation <= 0;
                        weight_buffer_operation <= 0;
                    end
                    else 
                    begin
                        mul_send_done <= 1'b0;
                        line <= line+1;
                        input_buffer_operation <= input_buffer [line+1];
                        weight_buffer_operation <= weight_buffer [line+1];
                        cell_counter <= 0;
                    end
                end
                else
                begin
                    weight_buffer_operation <= {zeros,weight_buffer_operation[K*N-1:N]};
                    input_buffer_operation <= {zeros,input_buffer_operation[K*N-1:N]}; 
                end
                
                in1_temp <= 0;
                in2_temp <= 0;
            end
            else
                begin
                wait_flag <= 1'b1;
                k <= k;
                end
        end
        else
        begin
            in1_temp <= in1_temp + (input_buffer_operation[k]<<k);
            in2_temp <= in2_temp + (weight_buffer_operation[k]<<k);
            if (wait_flag ==  1'b1)        
                k <= k;
            else
                k <= k+1;
        end
    end 
end

always @ (posedge clk)
begin
    if (mul_send_done == 1)
        begin
        write_input_done <= 1'b0;
        write_weight_done <= 1'b0;
        end
end

assign in1 = in1_temp_final;
assign in2 = in2_temp_final;
assign product_result = {(sign_in1^sign_in2),product_obtained};
assign one_cell_done = product_done;
assign one_conv_done = mul_send_done;

product #(.N(N-1)) multiplication (.a(in1),.b(in2),.c(product_obtained),.clk(clk),.result_flag(product_done));

endmodule
