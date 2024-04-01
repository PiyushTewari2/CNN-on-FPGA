`timescale 1ns / 1ps

module line_buffer#(parameter IMAGE_DIM = 224, STRIDE = 2, DATA_WIDTH = 16, FILTER_SIZE = 3, ZERO_PAD_ORI = 0, ZERO_PAD_AR = 1) 
                   (input clk,
                    input load_next,
                    input reset,
                    input [IMAGE_DIM*STRIDE-1:0] inp,
                    output valid);
                    
reg [DATA_WIDTH*(IMAGE_DIM+ZERO_PAD_ORI)-1:0] line_buffer [FILTER_SIZE-2:0];   
reg [DATA_WIDTH*FILTER_SIZE-1:0] line_buffer_final [FILTER_SIZE-1:0];
reg [$clog2(FILTER_SIZE-1)-1:0] line = 0; 
reg [$clog2((IMAGE_DIM+ZERO_PAD_ORI)/STRIDE)-1:0] h_counter = 0;
reg [$clog2(FILTER_SIZE)-1:0]  v_counter = 0;

parameter initial_state = 3'b000;
parameter normal_transmission = 3'b001;
parameter line_change_transmission = 3'b010;
parameter original_zero_padding = 3'b011;
parameter pause = 3'b100;

reg [2:0] current_state = initial_state;


genvar i;
generate
    for (i = 1; i < FILTER_SIZE-1; i = i+1)
    begin
        always @ (posedge clk)
        begin
            if (current_state == initial_state)    //this signal whould be high when one whole buffer is being written, this is to show whether the buffer is still being written or not
            begin
                coun
                line_buffer[0] <= {inp, line_buffer[0][(IMAGE_DIM+ZERO_PAD_ORI)*DATA_WIDTH-1:DATA_WIDTH*STRIDE]}; 
                line_buffer[i] <= {line_buffer[i-1][STRIDE*DATA_WIDTH-1:0], line_buffer[i][(IMAGE_DIM+ZERO_PAD_ORI)*DATA_WIDTH-1:DATA_WIDTH*STRIDE]}; 
                line_buffer_final[0] <= {inp, line_buffer_final[0][FILTER_SIZE*DATA_WIDTH-1:DATA_WIDTH*STRIDE]};
                line_buffer_final[i] <= {line_buffer[i-1][STRIDE*DATA_WIDTH-1:0], line_buffer_final[i][FILTER_SIZE*DATA_WIDTH-1:DATA_WIDTH*STRIDE]};
                line_buffer_final[FILTER_SIZE-1] <= {line_buffer[FILTER_SIZE-2][STRIDE*DATA_WIDTH-1:0], line_buffer_final[FILTER_SIZE-1][FILTER_SIZE*DATA_WIDTH-1:DATA_WIDTH*STRIDE]};
            end
        end
    end
endgenerate




always @(posedge clk)
begin
    if (reset) 
    begin
        current_state <= initial_state;
    end
    else
    begin
        case (current_state)
        
            initial_state : 
            begin
               if ()
                  <state> <= <next_state>;
               else if (<condition>)
                  <state> <= <next_state>;
               else
                  <state> <= <next_state>;
               line_buffer [v_counter] <=  {inp,line_buffer[v_counter][IMAGE_DIM+ZERO_PAD_ORI-1:STRIDE]};
            end
            
            normal_transmission : 
            begin
               if (<condition>)
                  <state> <= <next_state>;
               else if (<condition>)
                  <state> <= <next_state>;
               else
                  <state> <= <next_state>;
               <outputs> <= <values>;
            end
            
            line_change_transmission : 
            begin
               if (<condition>)
                  <state> <= <next_state>;
               else if (<condition>)
                  <state> <= <next_state>;
               else
                  <state> <= <next_state>;
               <outputs> <= <values>;
            end
            original_zero_padding : 
            begin
               if (<condition>)
                  <state> <= <next_state>;
               else if (<condition>)
                  <state> <= <next_state>;
               else
                  <state> <= <next_state>;
               <outputs> <= <values>;
            end
            pause : 
            begin
               if (<condition>)
                  <state> <= <next_state>;
               else if (<condition>)
                  <state> <= <next_state>;
               else
                  <state> <= <next_state>;
               <outputs> <= <values>;
            end
         endcase
     end
end		

					
										
endmodule
