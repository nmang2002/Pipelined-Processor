// Suhani Jain and Navya Mangipudi
// 04/07/2023
// EE 469
// Lab #1, Task 2

// reg_file is the top-level module for the 16x32 register system implemented in Task 2.
// This module implements a 16x32 register file that has 2 read ports, 1 write port, and is
// asynchronous. It has 16 addresses and each one stores a 32-bit word. This module inputs
// a clock, 1-bit wr_en, which enables write, 32-bit write_data, which writes in data,
// 4-bit write_addr, which writes addresses, and 4-bit read_addr1 and read_addr2, which
// read in data. It outputs 32-bit read_data1 and read_data2, which read out the data.

module reg_file(input logic clk, wr_en,
	input logic [31:0] write_data,
	input logic [3:0] write_addr, input logic [3:0]
	read_addr1, read_addr2, output logic [31:0]
	read_data1, read_data2);
	
	logic[15:0][31:0] memory; // makes a 16 by 32 array to store memory
	
	// On the negative edge of the clock, 
	// if wr_en is enabled, then values are written to memory
	
	always_ff @(negedge clk) begin
		if (wr_en) begin
			memory[write_addr] <= write_data;
		end
	end
	
	// read_data1 and read_data2 are assigned to values in memory to be read out
	
	assign read_data1 = memory[read_addr1];
	assign read_data2 = memory[read_addr2];
endmodule
