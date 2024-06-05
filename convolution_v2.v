`timescale 1ns / 1ps

//Always remember that here we have used fixed point representation and everywhere you will find the fractional part precision to be of 8 bits, therefore after multiplicationwe will be truncating 8 bits of LSB
module convolution#(parameter IMAGE_DIM = 224, STRIDE = 2, FILTER_SIZE = 3, ZERO_PAD = 0, DATA_WIDTH_INPUT = 9, DATA_WIDTH_WEIGHT = 9) 
                   (
				   input clk,
				   input reset,
				   input [DATA_WIDTH_WEIGHT*FILTER_SIZE*FILTER_SIZE-1:0] input_weight //This is as follows: {w33,w32,w31,w23,...,w11}. NOTE: Please send the input in the ways previously mentioned
                   input [DATA_WIDTH_INPUT-1:0] input_stream,	//Input stream is the input bus width, to increase the speed we can use DATA_WIDTH*STRIDE                  
                   output [19:0] conv_out,	//OUTPUT = {c11+c12+c13+b11+b12+b13+a11+a12+a13}; The output is the summation of all the values the length used is 20 bits so that there is no overflow observed; 1 bit is for sign, 8 bit for fractional part and the rest 11 bits for integer part
                   output memory_command,	//0: it means wait, or do not send. 1: means carry on, i.e. let the data come
                   output convolution_done	//flag signal to show that convolution is ready, ACTIVE_HIGH	   
				   );      

//Declaring registers and memories
reg [FILTER_SIZE*FILTER_SIZE-1:0] zero_pad_signal;	//this signal will be used to connect to each conv cell and to select which needs to be zero and which needs to be 1. Eg. if this signal is 9'b000000000, then no cell needs zero padding; if 9'b101000011, then cells of position 0,1,6, and 8 needs to be zero instead of the value                    
reg [DATA_WIDTH_INPUT*(IMAGE_DIM+2*ZERO_PAD)-1:0] line_buffer [FILTER_SIZE-2:0]; 
reg signed [DATA_WIDTH_INPUT*FILTER_SIZE-1:0] computation_block [FILTER_SIZE-1:0];  //This is as follows: [(0):{a33,a32,a31},(1):{a23,a22,a21},(2):{a13,a12,a11}]
reg memory_command_signal = 0; 
reg convolution_done_reg;
reg [$clog2(IMAGE_DIM+2*ZERO_PAD)-1:0] h_counter = 0;
reg [$clog2(IMAGE_DIM+2*ZERO_PAD)-1:0] horizontal_filter = 0;
reg [$clog2(IMAGE_DIM+2*ZERO_PAD)-1:0] vertical_filter = 0;
//reg [$clog2(FILTER_SIZE)-1:0]  v_counter = 0;
reg [$clog2(FILTER_SIZE)-1:0]fill_buffer_counter = 0;
reg [$clog2(IMAGE_DIM+2*ZERO_PAD)-1:0] line_counter = 0;
reg signed [DATA_WIDTH_WEIGHT*FILTER_SIZE-1:0] weight_matrix [FILTER_SIZE-1:0]; //This is as follows: [(0):{w33,w32,w31},(1):{w23,w22,w21},(2):{w13,w12,w11}]
reg weight_flag = 0;
reg signed [($clog2(FILTER_SIZE*FILTER_SIZE)+DATA_WIDTH_INPUT+DATA_WIDTH_WEIGHT-1)-1:0] mul_result [FILTER_SIZE-1:0][FILTER_SIZE-1:0];  //SIZE IS SUCH THAT THERE IS NO OVERFLOW
reg [$clog2(IMAGE_DIM)-1:0] count_correct_computation = 0;
reg input_stream_ready;
reg mul_result_flag;
reg signed [($clog2(FILTER_SIZE*FILTER_SIZE)+DATA_WIDTH_INPUT+DATA_WIDTH_WEIGHT-1):0] out_result;
reg signed [19:0]conv_out_reg;

//State definition
parameter initial_state = 3'b000;	//The state of the machine in the starting will be this state only; In this state we need to fill the buffer completely, before starting the computation (once we get the reset signal)
parameter normal_transmission = 3'b001;	//This is for normal transmission when the buffer is filled 
parameter line_change_transmission = 3'b010;  //During a line change we need to change the line for that we need to skip a complete row. Which will happen in this state
parameter computation_state = 3'b011;	//Computation state or the pause state where the flow will be paused and the resilt will be computed 
parameter done = 3'b100; //do nothing in this state, just sit ideal and send the signal

//assignment to the output
assign conv_out = conv_out_reg;
assign memory_command = memory_command_signal;
assign convolution_done = convolution_done_reg;

//when reset is done
always @ (posedge reset)
begin
	memory_command_signal <= 0;
	current_state <= initial_state;
	weight_flag <= 1;
end

//This will be controlling the incoming weights, weights will be loaded to the weight matrix(an array XD) in one cycle
genvar j;
generate
    for (j = 0; j < FILTER_SIZE; j = j+1)
    begin
        always @ (posedge clk)
        begin
			if (weight_flag)
			begin
				weight_matrix[j] = {input_weight[FILTER_SIZE*DATA_WIDTH_WEIGHT*(FILTER_SIZE-j)-1:FILTER_SIZE*DATA_WIDTH_WEIGHT*(FILTER_SIZE-j-1)]};  //CHECK IF CORRECT(YES IT WILL BE)
				weight_flag = 0;
			end
        end
    end
endgenerate

//for initial_state
genvar i;
generate
    for (i = 1; i < FILTER_SIZE-1; i = i+1)
    begin
        always @ (posedge clk)
        begin
			//In the initial state all the buffers are empty we need to initialise them so that normal transmission can happen
            if (current_state == initial_state)    
            begin
				//{
				//This is the control block which controls the filling of the line buffer
				//This will be implemented using a priority encoder over the line_counter variable
				if (line_counter < ZERO_PAD)      //initially the line counter will be zero and if padding is there then we need to fill the lower buffers with zero 
				begin
					memory_command_signal <= 0;   //will ask the memory to pause, not needed redundant
					h_counter <= h_counter + 1;   
					//here we are filling the line buffers with zero initially, it depends on how big is the zero padding
					//{
					line_buffer[0] <= {DATA_WIDTH_INPUT{0}, line_buffer[0][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};    
					line_buffer[i] <= {line_buffer[i-1][DATA_WIDTH_INPUT-1:0], line_buffer[i][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]}; 
					computation_block[0] <= {DATA_WIDTH_INPUT{0}, computation_block[0][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
					computation_block[i] <= {line_buffer[i-1][DATA_WIDTH_INPUT-1:0], computation_block[i][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
					computation_block[FILTER_SIZE-1] <= {line_buffer[FILTER_SIZE-2][DATA_WIDTH_INPUT-1:0], computation_block[FILTER_SIZE-1][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};	
					//}
					//when the line is filled then increment the line_counter, and reset the h_counter
					if (h_counter == (IMAGE_DIM+2*ZERO_PAD-1))
					begin
						line_counter <= line_counter + 1;
						h_counter <= 0;
					end
				end
				// now as soon as the line_counter is filled with zero or does not have zero padding present hence we will 
				else if (line_counter < FILTER_SIZE-1)
				begin
					//priority encoder implementation
					//now we also need to check the horizontal walls
					if (h_counter < ZERO_PAD)
					begin
						memory_command_signal <= 0;  //no need, redundant
						h_counter <= h_counter + 1;
						line_buffer[0] <= {DATA_WIDTH_INPUT{0}, line_buffer[0][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};    
						line_buffer[i] <= {line_buffer[i-1][DATA_WIDTH_INPUT-1:0], line_buffer[i][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]}; 
						computation_block[0] <= {DATA_WIDTH_INPUT{0}, computation_block[0][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
						computation_block[i] <= {line_buffer[i-1][DATA_WIDTH_INPUT-1:0], computation_block[i][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
						computation_block[FILTER_SIZE-1] <= {line_buffer[FILTER_SIZE-2][DATA_WIDTH_INPUT-1:0], computation_block[FILTER_SIZE-1][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};	
						if (h_counter == (ZERO_PAD-1))
						begin
							memory_command_signal <= 1;  //now just before one clock make this 1 therefore this will assign the output to the previous module, the memory
						end					
					end
					//now this is for normal input without pad and all
					else if (h_counter <(IMAGE_DIM+ZERO_PAD-1))
					begin
						h_counter <= h_counter + 1;
						line_buffer[0] <= {input_stream, line_buffer[0][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};    
						line_buffer[i] <= {line_buffer[i-1][DATA_WIDTH_INPUT-1:0], line_buffer[i][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]}; 
						computation_block[0] <= {input_stream, computation_block[0][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
						computation_block[i] <= {line_buffer[i-1][DATA_WIDTH_INPUT-1:0], computation_block[i][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
						computation_block[FILTER_SIZE-1] <= {line_buffer[FILTER_SIZE-2][DATA_WIDTH_INPUT-1:0], computation_block[FILTER_SIZE-1][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};	
						if (h_counter == (IMAGE_DIM+ZERO_PAD-1))
					end	
					//again the horizontal wall
					else if (h_counter <(IMAGE_DIM+2*ZERO_PAD-1))
					begin
						h_counter <= h_counter + 1;
						line_buffer[0] <= {DATA_WIDTH_INPUT{0}, line_buffer[0][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};    
						line_buffer[i] <= {line_buffer[i-1][DATA_WIDTH_INPUT-1:0], line_buffer[i][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]}; 
						computation_block[0] <= {DATA_WIDTH_INPUT{0}, computation_block[0][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
						computation_block[i] <= {line_buffer[i-1][DATA_WIDTH_INPUT-1:0], computation_block[i][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
						computation_block[FILTER_SIZE-1] <= {line_buffer[FILTER_SIZE-2][DATA_WIDTH_INPUT-1:0], computation_block[FILTER_SIZE-1][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
					end
					//now this is for line change and reset the horizontal counter
					else
					begin
						line_counter <= line_counter + 1;
						h_counter <= 0;
					end
				//when the line counter becomes FILTER_SIZE-1 then we are done with initial
				else if (line_counter == FILTER_SIZE-1)
				begin
					//now we just need to fill the last line of computation block, for this we will have a fill buffer counter
					if (fill_buffer_counter == FILTER_SIZE)
					begin
						h_counter <= 0; 
						fill_buffer_counter <= 0;    
						current_state <= computation_state;
						//memory_command_signal <= 0;       //Next one will be already loaded in this cycle
						horizontal_filter <= 2; //the index is as follows: 1, 2, 3, 4,..., Image_dim+2*Stride; since the first one will be already computed after this operation hence we will make it 2.
						vertical_filter <= 1;
					end
					else
					begin
						fill_buffer_counter <= fill_buffer_counter + 1;	
						line_buffer[0] <= {input_stream, line_buffer[0][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};    
						line_buffer[i] <= {line_buffer[i-1][DATA_WIDTH_INPUT-1:0], line_buffer[i][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]}; 
						computation_block[0] <= {input_stream, computation_block[0][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
						computation_block[i] <= {line_buffer[i-1][DATA_WIDTH_INPUT-1:0], computation_block[i][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
						computation_block[FILTER_SIZE-1] <= {line_buffer[FILTER_SIZE-2][DATA_WIDTH_INPUT-1:0], computation_block[FILTER_SIZE-1][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};							
					end 
				end
			end
		end           
    end
endgenerate

//for normal_transmission state, this design can be pipelined, since the compuattion takes only one cycle. But this design is non pipelined right now 
genvar j;
generate
    for (j = 1; j < FILTER_SIZE-1; j = j+1)
    begin
        always @ (posedge clk)
        begin
			//for zero padding
			if (horizontal_filter < (ZERO_PAD+1))  //Horizontal_filter has the index of the block which is going to be computed
			begin
				horizontal_filter <= horizontal_filter + 1;
				line_buffer[0] <= {DATA_WIDTH_INPUT{0}, line_buffer[0][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};    
				line_buffer[j] <= {line_buffer[j-1][DATA_WIDTH_INPUT-1:0], line_buffer[j][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]}; 
				computation_block[0] <= {DATA_WIDTH_INPUT{0}, computation_block[0][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
				computation_block[j] <= {line_buffer[j-1][DATA_WIDTH_INPUT-1:0], computation_block[j][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
				computation_block[FILTER_SIZE-1] <= {line_buffer[FILTER_SIZE-2][DATA_WIDTH_INPUT-1:0], computation_block[FILTER_SIZE-1][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};	
				//Here we will need to send the output to computation state if and only if it is valid, by default the value 1 will be valid and then 1+stride which is 3 will be valid, hence (horizontal_filter-1)%STRIDE must be equal to zero
				if ((horizontal_filter-1)%STRIDE == 0)
				begin
					current_state <= computation_state;
					memory_command_signal <= 0; 					
				end
				else
				begin
					current_state <= normal_transmission;
				end
			end
			//let us start from the default location
			else if (horizontal_filter <(IMAGE_DIM+ZERO_PAD+1))   //cause when computing 226 it should be 226 hence, upper value 227
			begin
				horizontal_filter <= horizontal_filter + 1;
				line_buffer[0] <= {input_stream, line_buffer[0][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};    
				line_buffer[j] <= {line_buffer[j-1][DATA_WIDTH_INPUT-1:0], line_buffer[j][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]}; 
				computation_block[0] <= {input_stream, computation_block[0][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
				computation_block[j] <= {line_buffer[j-1][DATA_WIDTH_INPUT-1:0], computation_block[j][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
				computation_block[FILTER_SIZE-1] <= {line_buffer[FILTER_SIZE-2][DATA_WIDTH_INPUT-1:0], computation_block[FILTER_SIZE-1][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};		
				if (((horizontal_filter-1)%STRIDE == 0) && (horizontal_filter < (IMAGE_DIM+2*STRIDE+1)-FILTER_SIZE+1))
				begin
					memory_command_signal <= 0;
					current_state <= computation_state;					
				end
				else if (horizontal_filter
				else
				begin
					current_state <= normal_transmission;
				end				
			end	
			//for zero padding final wall
			else if (horizontal_filter < (IMAGE_DIM+2*ZERO_PAD+1))
			begin
				horizontal_filter <= horizontal_filter + 1;
				line_buffer[0] <= {DATA_WIDTH_INPUT{0}, line_buffer[0][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};    
				line_buffer[j] <= {line_buffer[j-1][DATA_WIDTH_INPUT-1:0], line_buffer[j][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]}; 
				computation_block[0] <= {DATA_WIDTH_INPUT{0}, computation_block[0][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
				computation_block[j] <= {line_buffer[j-1][DATA_WIDTH_INPUT-1:0], computation_block[j][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
				computation_block[FILTER_SIZE-1] <= {line_buffer[FILTER_SIZE-2][DATA_WIDTH_INPUT-1:0], computation_block[FILTER_SIZE-1][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};	
				//Here we will need to send the output to computation state if and only if it is valid, by default the value 1 will be valid and then 1+stride which is 3 will be valid, hence (horizontal_filter-1)%STRIDE must be equal to zero
				if ((horizontal_filter-1)%STRIDE == 0 && (horizontal_filter < (IMAGE_DIM+2*STRIDE+1)-FILTER_SIZE+1))
				begin
					current_state <= computation_state;
					memory_command_signal <= 0; 					
				end
				else
				begin
					current_state <= normal_transmission;
				end
			end
			//for line change and controlling the vertical shift, padding bhi karni padegi bhool gaya tha
			else
			begin 
				if (vertical_filter%STRIDE == 0)
				begin
					//for memory_command_signal control
					//{
					if (ZERO_PAD == 0)
					begin
						memory_command_signal <= 1;
					end
					else
					begin
						memory_command_signal <= 0; 
					end
					//}
					//for controlling the vertical of the convolution
					//{
					if (vertical_filter == ((IMAGE_DIM+2*STRIDE+1)-FILTER_SIZE))
					begin
						memory_command_signal <= 0;
						horizontal_filter <= 1;
						convolution_done_reg <= 1;
						current_state <= done;
					end
					else 
					begin
						vertical_filter <= vertical_filter + 1;
						horizontal_filter <= 1;
						current_state <= normal_transmission;
					end
					//}
				end
				else
				begin
					vertical_filter <= vertical_filter + 1;
					horizontal_filter <= 1;
					current_state <= line_change_transmission;
					memory_command_signal <= 0;
				end
			end
        end
    end
endgenerate

//for line_change_transmission state 
genvar k;
generate
    for (k = 1; k < FILTER_SIZE-1; k = k+1)
    begin
        always @ (posedge clk)
        begin
			//for zero padding
			if (horizontal_filter < (ZERO_PAD+1))  //Horizontal_filter has the index of the block which is going to be computed
			begin
				horizontal_filter <= horizontal_filter + 1;
				line_buffer[0] <= {DATA_WIDTH_INPUT{0}, line_buffer[0][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};    
				line_buffer[k] <= {line_buffer[k-1][DATA_WIDTH_INPUT-1:0], line_buffer[j][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]}; 
				computation_block[0] <= {DATA_WIDTH_INPUT{0}, computation_block[0][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
				computation_block[k] <= {line_buffer[k-1][DATA_WIDTH_INPUT-1:0], computation_block[k][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
				computation_block[FILTER_SIZE-1] <= {line_buffer[FILTER_SIZE-2][DATA_WIDTH_INPUT-1:0], computation_block[FILTER_SIZE-1][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};	
			end
			//let us start from the default location
			else if (horizontal_filter <(IMAGE_DIM+ZERO_PAD+1))   //cause when computing 226 it should be 226 hence, upper value 227
			begin
				horizontal_filter <= horizontal_filter + 1;
				line_buffer[0] <= {input_stream, line_buffer[0][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};    
				line_buffer[k] <= {line_buffer[k-1][DATA_WIDTH_INPUT-1:0], line_buffer[k][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]}; 
				computation_block[0] <= {input_stream, computation_block[0][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
				computation_block[k] <= {line_buffer[k-1][DATA_WIDTH_INPUT-1:0], computation_block[k][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
				computation_block[FILTER_SIZE-1] <= {line_buffer[FILTER_SIZE-2][DATA_WIDTH_INPUT-1:0], computation_block[FILTER_SIZE-1][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};		
			end	
			//for zero padding final wall
			else if (horizontal_filter < (IMAGE_DIM+2*ZERO_PAD+1))
			begin
				horizontal_filter <= horizontal_filter + 1;
				line_buffer[0] <= {DATA_WIDTH_INPUT{0}, line_buffer[0][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};    
				line_buffer[k] <= {line_buffer[k-1][DATA_WIDTH_INPUT-1:0], line_buffer[k][(IMAGE_DIM+2*ZERO_PAD)*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]}; 
				computation_block[0] <= {DATA_WIDTH_INPUT{0}, computation_block[0][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
				computation_block[k] <= {line_buffer[k-1][DATA_WIDTH_INPUT-1:0], computation_block[k][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};
				computation_block[FILTER_SIZE-1] <= {line_buffer[FILTER_SIZE-2][DATA_WIDTH_INPUT-1:0], computation_block[FILTER_SIZE-1][FILTER_SIZE*DATA_WIDTH_INPUT-1:DATA_WIDTH_INPUT]};	
			end
			//for line change and controlling the vertical shift, padding bhi karni padegi bhool gaya tha
			else
			begin 
				if (vertical_filter%STRIDE == 0)
				begin
					//for memory_command_signal control
					//{
					if (ZERO_PAD == 0)
					begin
						memory_command_signal <= 1;
					end
					else
					begin
						memory_command_signal <= 0; 
					end
					//}
					//for controlling the vertical of the convolution
					//{
					if (vertical_filter == ((IMAGE_DIM+2*STRIDE+1)-FILTER_SIZE))
					begin
						memory_command_signal <= 0;
						horizontal_filter <= 1;
						convolution_done_reg <= 1;
						current_state <= done;
					end
					else 
					begin
						vertical_filter <= vertical_filter + 1;
						horizontal_filter <= 1;
						current_state <= normal_transmission;
					end
					//}
				end
				else
				begin
					current_state <= line_change_transmission;
					memory_command_signal <= 0;
				end
			end
        end
    end
endgenerate

//for computation state
genvar n,o;
generate
	for (n = 0; n < FILTER_SIZE; n = n+1)
	begin
		for (o = 0; o < FILTER_SIZE; o = o+1)
		begin
			always @ (posedge clk)
			begin
				if (current_state == computation_state)
				begin
					mul_result[n][o] <= computation_block[n][(o+1)*(DATA_WIDTH_INPUT-1):o*DATA_WIDTH_INPUT]*weight_matrix[n][(o+1)*(DATA_WIDTH_WEIGHT-1):o*DATA_WIDTH_WEIGHT]
					mul_result_flag <= ~mul_result_flag;
				end
			end
		end
	end
endgenerate	

//for adding all the weights result
//as soon as the multiplication result will be ready it will be sent here where the final output will be computed
genvar p, q;
generate
	for (p = 0; p < FILTER_SIZE; p = p+1)
	begin
		for (q = 0; q < FILTER_SIZE; q = q+1)
		begin
			always @ (mul_result_flag)   
			begin
				out_result = out_result + mul_result[p][q];
				out_result_flag = ~out_result_flag;
			end
		end	
	end
endgenerate

//sending the output as tructated result and also changing the state
always @ (out_result_flag)
begin
	conv_out_reg = {(20-(($clog2(FILTER_SIZE*FILTER_SIZE)+DATA_WIDTH_INPUT+DATA_WIDTH_WEIGHT-1)-8)){out_result[($clog2(FILTER_SIZE*FILTER_SIZE)+DATA_WIDTH_INPUT+DATA_WIDTH_WEIGHT-1)]},out_result[($clog2(FILTER_SIZE*FILTER_SIZE)+DATA_WIDTH_INPUT+DATA_WIDTH_WEIGHT-1)-1:8]}; //leaving the 8 bit of the LSB and truncating for 20 bits
	current_state = normal_transmission;
	if (horizontal_filter < ZERO_PAD+1)
	begin
		memory_command_signal = 0;
	end
	else if (horizontal_filter > IMAGE_DIM+ZERO_PAD)
	begin
		memory_command_signal = 0;
	end
	else
	begin
		memory_command_signal = 1;
	end
end

always @ (posedge clk)
begin
	convolution_done_reg <= 1;
end
	
endmodule
