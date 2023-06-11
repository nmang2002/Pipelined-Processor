
// Navya Mangipudi & Suhani Jain 
// 05/05/23
// EE 469
// Lab #3, ARM Pipelined Processor

// hazard unit deals with any possible hazards that come up, such as data and control hazards,
// by data forwarding and stalling.

// rst - system reset
// WA3M, WA3W, WA3E - stage registers for memory, writeback, and execute respectively
// RA1D, RA2D, RA1E, RA2E - input addresses for the register file
// PCSrcD, PCSrcE, PCSrcM, PCSrcW - pc source control signal for each stage: decode, execute, memory, and writeback respectively
// RegWriteM, RegWriteE, RegWriteW - control signals for writing of data to reg file for memory, execute, writeback stages
// MemToRegE - control signal for source of value being written to register file (execute stage)
// BranchTakenE - If branch instruction in execute stage is taken or not
// StallF, StallD, FlushD, FlushE - control signal for whether a certain stage of pipeline should be stalled
// ForwardAE, ForwardBE - control signals for data forwarding

module hazard_unit(
	input logic  rst,
	input logic  [3:0] WA3M, WA3W,
	input logic  [3:0] RA1E, RA2E, RA1D, RA2D, WA3E,
	input logic  PCSrcD, PCSrcE, PCSrcM, PCSrcW,
	input logic  RegWriteM, RegWriteE, RegWriteW, MemtoRegE,
	input logic  BranchTakenE,
	output logic StallF, StallD, FlushD, FlushE, 
	output logic [1:0] ForwardAE, ForwardBE
   );
	 
	// logic for checking matching between stages
	logic Match_1E_M, Match_1E_W, Match_2E_M, Match_2E_W;
	logic Match_12_D_E, ldrStallD, PCWrPendingF;
	 
	// Data Forwarding
	// Execute stage register matches Memory stage register
	assign Match_1E_M = (RA1E == WA3M);
	assign Match_2E_M = (RA2E == WA3M);
	
	// Execute stage register matches Writeback stage register
	assign Match_1E_W = (RA1E == WA3W);
	assign Match_2E_W = (RA2E == WA3W);
	
	// Forward Result if it matches
	always_comb begin
		if (Match_1E_M & RegWriteM) ForwardAE = 2'b10;
		else if (Match_1E_W & RegWriteW) ForwardAE = 2'b01;
		else ForwardAE = 2'b00;
		
		
		if (Match_2E_M & RegWriteM) ForwardBE = 2'b10;
		else if (Match_2E_W & RegWriteW) ForwardBE = 2'b01;
		else ForwardBE = 2'b00;
	end	
	
	// Stalling Logic
	// Determines if source register in Decode stage is the same as in the execute stage
	always_comb begin
		if (rst) begin // set all signals to zero
			Match_12_D_E = 0;
			ldrStallD = 0;
			PCWrPendingF = 0;
			StallF = 0;
			FlushD = 0;
			FlushE = 0;
			StallD = 0;
		end
		else begin // control stalling logic
			Match_12_D_E = (RA1D == WA3E) | (RA2D == WA3E);
			ldrStallD = Match_12_D_E & MemtoRegE;
			
			PCWrPendingF = PCSrcD | PCSrcE | PCSrcM; // PCWrPendingF if PC written to in D,C,M stages
			StallF = ldrStallD | PCWrPendingF; // stall fetch if PCWrPendingF
			// flush decode if PCWrPending or PC written in writeback or if branch is taken
			FlushD = PCWrPendingF | PCSrcW | BranchTakenE; 
			FlushE = ldrStallD | BranchTakenE; // flush execute if branch is taken
			StallD = ldrStallD; // stall decode if IdrStallD
		end
	end
    
endmodule
