// Suhani Jain and Navya Mangipudi
// 04/07/2023
// EE 469
// Lab #1, Task 3

// alu is the top-level module for the alu system implemented in Task 3.
// This module uses the outputs from the 32-bit adder submodule and implements
// addition, subtraction, ANDing, and ORing of 32-bit numbers. This module inputs
// 32-bit a and b, upon which the operations are conducted, and 2-bit ALUControl, which
// tells which operation to perform. It outputs 32-bit Result, the output of the operation
// between a and b, and 4-bit ALUFlags, which represents different conditions of the operation. 

module alu (input logic [31:0] a, b, input logic [1:0] ALUControl,
output logic [31:0] Result, output logic [3:0] ALUFlags);

	logic [31:0] sum; // result from adder
	logic cout; // carry out from adder
	
	// adder op takes a, ALUControl[0] ? ~b : b, and ALUControl[0] as inputs to parameters
	// a, b, and cin respectively. It returns Result and out as sum and cout respectively.
	// This submodule is the 32-bit adder that returns the sum and carry out of the operation.
	
	adder op(.a(a), .b(ALUControl[0] ? ~b : b), .cin(ALUControl[0]), .Result(sum), .out(cout));
	
	// Performs the specific operation for each ALUControl case and outputs it to Result
	// Sets each bit of the ALUFlags equal to the correct value
	
	always_comb begin
		case (ALUControl)
			
			2'b00: Result = sum; // addition
			2'b01: Result = a - b; // subtraction
			2'b10: Result = a & b; // ANDing
			2'b11: Result = a | b; // ORing
		
		endcase
			
		ALUFlags[3] = Result[31]; // negative flag
		ALUFlags[2] = (Result == 32'h00000000); // zero flag
		if (ALUControl == 2'b00 || ALUControl == 2'b01) begin
			ALUFlags[1] = cout; // carry flag for addition and subtraction
		end else begin
			ALUFlags[1] = 0; // carry flag is zero for AND and OR
		end
		ALUFlags[0] = ~(a[31] ^ b[31]) & (a[31] ^ Result[31]); // overflow flag
		
	end
endmodule