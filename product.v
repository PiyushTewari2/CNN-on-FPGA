`timescale 1ns / 1ps

module product #(parameter N = 16)(input [N-1:0]a, input [N-1:0]b, output [N-1:0]c, input clk, output result_flag);

wire [2*N-1:0] imm_mul;
wire [N-1:0] final_mul;

round_off #(.N(N), .M(2*N)) round (.mul_large(imm_mul),.mul_small(final_mul),.clock(clk),.result_ready(result_flag));

assign imm_mul = a*b;
assign c = final_mul;

endmodule
