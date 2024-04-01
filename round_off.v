`timescale 1ns / 1ps

module round_off #(parameter N = 16, M = 32)(input [M-1:0] mul_large, output [N-1:0] mul_small, input clock, output result_ready);
     
reg [$clog2(M)-1:0]digit_counter = 0;
reg [M-1:0] remaining_digits;
reg stop = 1'b0;
reg [M-1:0] mul_small_temp;
reg [N-1:0] mul_small_final;
reg [$clog2(M)-1:0]num_digit_of_N;
reg [$clog2(M)-1:0]i = 0;
reg begin_condition = 1'b0;
reg result_flag = 1'b1;

always @ (mul_large)
begin
    result_flag = 1'b0;
    remaining_digits = mul_large;
    mul_small_temp = mul_large;
    begin_condition = 1'b1;
    if ((N-((N/10)*10)) < 4)
        num_digit_of_N = ((N/10)*3)+1;
    else if ((N-((N/10)*10)) < 7)
        num_digit_of_N = ((N/10)*3)+2;
    else
        num_digit_of_N = ((N/10)*3)+3; 
end

always @ (posedge clock)
begin
    if (begin_condition)
    begin
        if (remaining_digits != 0)
        begin
            remaining_digits <= remaining_digits/10;
            digit_counter <= digit_counter + 1;
            stop <= 1'b0;
        end
        else
            stop <= 1'b1;    
    end
end

always @ (posedge clock)
begin
    if (stop)
    begin
        begin_condition <= 1'b0;
        if (digit_counter <  num_digit_of_N)
        begin
            mul_small_final <= mul_small_temp;
            result_flag <= 1'b1;
            stop <= 1'b0;
            digit_counter <= 0;
        end
        else
        begin
            i <= i+1;
            if (i > (digit_counter-num_digit_of_N-1))
            begin
                if (mul_small_temp < (2**N))
                begin
                    result_flag <= 1'b1;               
                    mul_small_final <= mul_small_temp;
                end
                else
                begin
                    result_flag <= 1'b1;
                    mul_small_final <= (mul_small_temp/10); 
                end
                stop <= 1'b0;
                i <= 1'b0;
                digit_counter <= 0;
    
            end
            else
                mul_small_temp <= mul_small_temp/10;
        end  
    end
end    


assign mul_small = mul_small_final;
assign result_ready = result_flag;

endmodule
