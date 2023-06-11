// Navya Mangipudi & Suhani Jain 
// 05/05/23
// EE 469
// Lab #3, Pipelined Processor

/* arm is the spotlight of the show and contains the bulk of the datapath and control logic. This module is split into two parts, the datapath and control. 
*/

// clk - system clock
// rst - system reset
// InstrF - incoming 32 bit instruction from imem, contains opcode, condition, addresses and or immediates
// ReadData - data read out of the dmem
// WriteDataM - data to be written to the dmem
// MemWriteM - write enable to allowed WriteData to overwrite an existing dmem word
// PCF - the current program count value, goes to imem to fetch instruction
// ALUOutM - result of the ALU operation, sent as address to the dmem


module arm (
    input  logic        clk, rst,
    input  logic [31:0] InstrF,
    input  logic [31:0] ReadDataM,
    output logic [31:0] WriteDataM, 
    output logic [31:0] PCF, ALUOutM,
    output logic        MemWriteM
);

    // datapath buses and signals
    logic [31:0] PCPrime, PCPlus4F, PCPlus8D, PCPlus8E, PCPrime2; // pc signals
    logic [ 3:0] RA1D, RA2D, RA1E, RA2E;                  // regfile input addresses
    logic [31:0] RD1, RD2, RD1E, RD2E;                  // raw regfile outputs
    logic [ 3:0] ALUFlags;                  // alu combinational flag outputs
    logic [31:0] ExtImmE, ExtImmD, SrcA, SrcB;        // immediate and alu inputs 
    logic [31:0] ResultW;                    // computed or fetched value to be written into regfile or pc
	 logic [31:0] InstrD;							// instruction signal from fetch register
	 logic [3:0]  FlagsReg;						   // register to store flags from most recent CMP command
	 logic [31:0] ALUOutW;							// alu result
	 logic [3:0]  WA3D, WA3E, WA3M, WA3W;		// regfile write address (pipeline stages)
	 logic [31:0] WriteDataE, WriteDataW;		// pipeline reg write data
	 logic [31:0] ReadDataW;						// pipeline register read data
	 logic [31:0] ALUResultE;						// BTA selected from ALUResultE

    // control signals
	 logic PCSrcM, MemtoRegM, ALUSrcM, RegWriteM;
	 logic PCSrcW, MemtoRegW, ALUSrcW, RegWriteW;
    logic [1:0] RegSrcD, ImmSrcD, ALUControlD;
	 logic [1:0] RegSrcE, ImmSrcE, ALUControlE;
	 logic FlagWriteE, FlagWriteD; // if flags held
	 logic PCSrcD, MemWriteD, RegWriteD, MemtoRegD, ALUSrcD, BranchD;
	 logic PCSrcE, MemWriteE, RegWriteE, MemtoRegE, ALUSrcE, BranchE;
	 logic PCSrcE2;
	 
	 logic [31:0] SrcAE, SrcBE, SrcAE2, SrcBE2;
	 // CondE & CondExe for conditional execution
	 logic [3:0] CondE, FlagsE, FlagsPrime; // FlagsE & FlagsPrime stores from aluflags
	 logic CondExe;
	 logic [1:0] ForwardAE, ForwardBE; // data forwarding 
	 
	 
	 // hazard control
	 logic StallF, StallD, FlushD, FlushE;
	 logic BranchTakenE;
	
	 // hazard_unit h1 takes rst, WA3M, WA3W, RA1E, RA2E, RA1D, RA2D, WA3E, PCSrcD, PCSrcE, PCSrcM, PCSrcW,
	 // RegWriteM, RegWriteE, RegWriteW, MemToRegE, BranchTakenE as inputs. It returns
	 // StallF, StallD, FlushD, FlushE, ForwardAE, ForwardBE respectively.
	 // This submodule is the hazard unit which deals with possible hazards from the 	
	 // Pipelined Processor
	 hazard_unit h1 (
		  .rst				(rst),
		  .WA3M     		(WA3M), 
		  .WA3W     		(WA3W),
		  .RA1E				(RA1E),
		  .RA2E				(RA2E),
		  .RA1D				(RA1D),
		  .RA2D				(RA2D),
		  .WA3E				(WA3E),
		  .PCSrcD			(PCSrcD),
		  .PCSrcE			(PCSrcE),
		  .PCSrcM			(PCSrcM),
		  .PCSrcW			(PCSrcW),
		  .RegWriteM		(RegWriteM),
		  .RegWriteE		(RegWriteE),
		  .RegWriteW		(RegWriteW),
		  .MemtoRegE		(MemtoRegE),
		  .BranchTakenE	(BranchTakenE),
		  .StallF			(StallF),
		  .StallD			(StallD),
		  .FlushD			(FlushD),
		  .FlushE			(FlushE),
		  .ForwardAE		(ForwardAE),
		  .ForwardBE		(ForwardBE)	

    );
	  
	 
    /* The datapath consists of a PC as well as a series of muxes to make decisions about which data words to pass forward and operate on. It is 
    ** noticeably missing the register file and alu, which you will fill in using the modules made in lab 1. To correctly match up signals to the 
    ** ports of the register file and alu take some time to study and understand the logic and flow of the datapath.
    */
    //-------------------------------------------------------------------------------
    //                                      DATAPATH
    //-------------------------------------------------------------------------------


    assign PCPrime2 = PCSrcW ? ResultW : PCPlus4F;  // mux, use either default or newly computed value
	 assign PCPrime = BranchTakenE ? ALUResultE : PCPrime2; // mux, use either default or newly computed value
    assign PCPlus4F = PCF + 32'd4;                  // default value to access next instruction

    // update the PC, at rst initialize to 0
    always_ff @(posedge clk) begin
        if (rst) PCF <= '0;
		  else if (~StallF) PCF <= PCPrime; // if stall not active, then not stalling
        else     PCF <= PCF; // if stall active, then stalling
    end



	 // Fetch Stage
	 
	 always_ff @(posedge clk) begin
		if (rst) begin // reset
			InstrD <= 32'b0;
			PCPlus8D <= 32'b0;
		end
		else if (FlushD) begin // if flush active, then flush signals
			InstrD <=32'b0;
			PCPlus8D <= 32'b0;
		end
		else if (~StallD) begin // if stall notactive, do not stall
			InstrD <= InstrF;
			PCPlus8D <= PCPlus4F + 32'd4;
		end
		else begin // if stall active, then initiate stalling
			InstrD <= InstrD;
			PCPlus8D <= PCPlus8D;
		end
	end
	
	// if branch instruction (InstrD[19:16]) then RegSrcD[0]
   // if memory instruction (InstrD[15:12]) then RegSrcD[1]
	 assign RA1D = RegSrcD[0] ? 4'd15        : InstrD[19:16];
    assign RA2D = RegSrcD[1] ? InstrD[15:12] : InstrD[ 3: 0];
	 

    // reg_file u_reg_file takes clk, RegWriteW, ResultW, WA3W, RA1D, and RA2D as inputs to the 
	 //parameters clk, wr_en, write_data, write_addr, read_addr1, & read_addr2 respectively. It returns RD1 and RD2 
	 // respectively. This submodule is the register system which returns 32-bit RD1 & RD2 which read out the data.  
    reg_file u_reg_file (
        .clk       (clk), 
        .wr_en     (RegWriteW),
        .write_data(ResultW),
        .write_addr(WA3W),
        .read_addr1(RA1D), 
        .read_addr2(RA2D),
        .read_data1(RD1), 
        .read_data2(RD2)
    );
	 
	 
	 // two muxes, put together into an always_comb for clarity
    // determines which set of instruction bits are used for the immediate
    always_comb begin
        if      (ImmSrcD == 'b00) ExtImmD = {{24{InstrD[7]}},InstrD[7:0]};          // 8 bit immediate - reg operations
        else if (ImmSrcD == 'b01) ExtImmD = {20'b0, InstrD[11:0]};                 // 12 bit immediate - mem operations
        else                      ExtImmD = {{6{InstrD[23]}}, InstrD[23:0], 2'b00}; // 24 bit immediate - branch operation
    end

	 // Pipeline from decode to execute
	 always_ff @(posedge clk) begin
		if (rst) begin // reset
			PCSrcE <= 1'b0;
			RegWriteE <= 1'b0;
			MemtoRegE <= 1'b0;
			MemWriteE <= 1'b0;
			ALUControlE <= 2'b0;
			BranchE <= 1'b0;
			ALUSrcE <= 1'b0;
			FlagWriteE <= 1'b0;
			CondE <= 4'b0;
			FlagsE <= 4'b0;
			ExtImmE <= 'b0;
			WA3E <= 4'b0;
			RD1E <= 32'b0;
			RD2E <= 32'b0;
			PCPlus8E <= 32'b0;
			RA2E <= 4'b0;
			RA1E <= 4'b0;
			
		end 
		if (FlushE) begin // if flush active, flushing
			PCSrcE <= 1'b0;
			RegWriteE <= 1'b0;
			MemtoRegE <= 1'b0;
			MemWriteE <= 1'b0;
			ALUControlE <= 2'b0;
			BranchE <= 1'b0;
			ALUSrcE <= 1'b0;
			FlagWriteE <= 1'b0;
			CondE <= 4'b0;
			FlagsE <= 4'b0;
			ExtImmE <= 'b0;
			WA3E <= 4'b0;
			RD1E <= 32'b0;
			RD2E <= 32'b0;
			PCPlus8E <= 32'b0;
			RA2E <= 4'b0;
			RA1E <= 4'b0;	
		end
		else begin // if neither reset nor flush active, continue
			PCSrcE <= PCSrcD;
			RegWriteE <= RegWriteD;
			MemtoRegE <= MemtoRegD;
			MemWriteE <= MemWriteD;
			ALUControlE <= ALUControlD;
			BranchE <= BranchD;
			ALUSrcE <= ALUSrcD;
			ExtImmE <= ExtImmD;
			FlagWriteE <= FlagWriteD;
			CondE <= InstrD[31:28];
			FlagsE <= FlagsPrime;
			WA3E <= InstrD[15:12];
			RD1E <= RD1;
			RD2E <= RD2;
			PCPlus8E <= PCPlus8D;
			RA2E <= RA2D;
			RA1E <= RA1D;
		end
	 end


	 
    // SrcBE2 and SrcAE2 are direct outputs of the register file, wheras SrcB is chosen between reg file output and the immediate
    assign SrcBE2 = (RA2E == 'd15) ? PCPlus8E : RD2E;           // substitute the 15th regfile register for PC 
    assign SrcAE2 = (RA1E == 'd15) ? PCPlus8E : RD1E;           // substitute the 15th regfile register for PC 
   
		// first mux, uses fwding values to select signal for fwding for SrcAE
		always_comb begin
			if (~ForwardAE[0] & (~ForwardAE[1]))		SrcAE = SrcAE2;
			else if ((~ForwardAE[1]) & ForwardAE[0])	SrcAE = ResultW;
			else SrcAE = ALUOutM;
		end
		
		// first mux, uses fwding values to select signal for fwding for SrcAE
		always_comb begin
			if ((~ForwardBE[0]) & (~ForwardBE[1]))		WriteDataE = SrcBE2;
			else if ((~ForwardBE[1]) & ForwardBE[0])	WriteDataE = ResultW;
			else WriteDataE = ALUOutM;
		end
		
	  assign SrcBE      = ALUSrcE        ? ExtImmE  : WriteDataE;     // determine alu operand to be either from reg file or from immediate
	
	// alu u_alu takes SrcAE, SrcBE, and ALUControlE as inputs to parameters a, b, and ALUControl respectively.   
	// It returns ALUResultE and ALUFlags as Result and ALUFlags respectively.
	// This submodule is the alu which returns the 32 bit result (ALUResultE) which is the output of the operation conducted 
	// between SrcAE and SrcBE based on ALUControlE. Also returns 4-bit ALUFlags which represents different conditions of 
	// the operation.
    alu u_alu (
        .a          (SrcAE), 
        .b          (SrcBE),
        .ALUControl (ALUControlE),
        .Result     (ALUResultE),
        .ALUFlags   (ALUFlags)
    );
	 
	 
//	 cond unit
	always_comb begin 
		case(CondE)
			4'b1110: CondExe = 1; //unnconditional
			4'b0000: begin //equal
				if (FlagsE[2]) begin 
					CondExe = FlagsE[2];
				end
				else begin
					CondExe = 0;
				end
			end
			4'b0001: begin // not equal 
				if (~FlagsE[2]) begin
					CondExe = 1;
				end
				else begin
					CondExe = 0;
				end			
			end
			4'b1010: begin // greater/equal 
				if (FlagsE[2] || ~FlagsE[3]) begin
					CondExe = 1;
				end
				else begin
					CondExe = 0;
				end
			end
			4'b1100: begin // greater
				if (~FlagsE[3]) begin
					CondExe = 1;
				end
				else begin
					CondExe = 0;
				end
			end
			4'b1101: begin // less or equal 
				if (FlagsE[2] || FlagsE[3]) begin
					CondExe = 1;
				end
				else begin
					CondExe = 0;
				end
			end
			4'b1011: begin // less
				if (FlagsE[3]) begin
					CondExe = 1;
				end
				else begin
					CondExe = 0;
				end
			end
			default: begin // defualt
				CondExe = 1;
			end
		endcase
	end
			
	logic RegWriteE2, MemWriteE2;
	
	// and gates after cond unit, signals and-ed with CondExe
	 assign PCSrcE2 = CondExe & PCSrcE;
	 assign RegWriteE2 = CondExe & RegWriteE;
	 assign MemWriteE2 = CondExe & MemWriteE;
	 assign BranchTakenE = BranchE & CondExe;
	 
	 // execute register
	 always_ff @(posedge clk) begin
		if (rst) begin // reset
			PCSrcM <= 0;
			RegWriteM <= 0;
			MemtoRegM <= 0;
			MemWriteM <= 0;
			WA3M <= 4'b0;
			ALUOutM <= 32'b0;
			WriteDataM <= 32'b0;
		end else begin
			PCSrcM <= PCSrcE2;
			RegWriteM <= RegWriteE2;
			MemtoRegM <= MemtoRegE;
			MemWriteM <= MemWriteE2;
			WA3M <= WA3E;
			ALUOutM <= ALUResultE;
			WriteDataM <= WriteDataE;
		end
	end
	
	// memory unit/register
	always_ff @(posedge clk) begin
		if (rst) begin // reset
			PCSrcW <= 0;
			RegWriteW <= 0;
			MemtoRegW <= 0;
			WA3W <= 4'b0;
			ALUOutW <= 32'b0;
			WriteDataW <= 32'b0;
		end else begin
			PCSrcW <= 0;
			RegWriteW <= RegWriteM;
			MemtoRegW <= MemtoRegM;
			WA3W <= WA3M;
			ALUOutW <= ALUOutM;
			WriteDataW <= WriteDataM;
		end
	end
			
    // determine the result to run back to PC or the register file based on whether we used a memory instruction
   assign ResultW = MemtoRegW ? ReadDataW : ALUOutW;    // determine whether final writeback result is from dmemory or alu
		  
	 
    /* The control conists of a large decoder, which evaluates the top bits of the instruction and produces the control bits 
    ** which become the select bits and write enables of the system. The write enables (RegWrite, MemWrite and PCSrc) are 
    ** especially important because they are representative of your processors current state. 
    */
    //-------------------------------------------------------------------------------
    //                                      CONTROL
    //-------------------------------------------------------------------------------
    
	 // whether or not to write to FlagsPrime, if flagwrite true, assign aluflags to flagsprime
	 assign FlagsPrime = FlagWriteE ? ALUFlags : 4'b0;
	 
	 always_comb begin
        casez (InstrD[27:20])

            // ADD (Imm or Reg)
            8'b00?_0100_0 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we add
                PCSrcD    = 0;
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = InstrD[25]; // may use immediate
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00; 
                ALUControlD = 'b00;
					 FlagWriteD = 0;
					 BranchD = 0;
					 
            end

            // SUB (Imm or Reg)
            8'b00?_0010_0 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we sub
                PCSrcD    = 0; 
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = InstrD[25]; // may use immediate
                RegWriteD = 1;   //I dont know about this one
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00; 
                ALUControlD = 'b01;
					 FlagWriteD = InstrD[20];
					 BranchD = 0;
            end
				
				
            // CMP (Imm or Reg)
            8'b00?_0010_1 : begin   // note that we use wildcard "?" in bit 25. That bit decides whether we use immediate or reg, but regardless we sub
                PCSrcD    = 0; 
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = InstrD[25]; // may use immediate
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00; 
                ALUControlD = 'b01;
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // AND
            8'b000_0000_0 : begin
                PCSrcD    = 0; 
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = 0; 
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00;    // doesn't matter
                ALUControlD = 'b10;  
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // ORR
            8'b000_1100_0 : begin
                PCSrcD    = 0; 
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = 0; 
                RegWriteD = 1;
                RegSrcD   = 'b00;
                ImmSrcD   = 'b00;    // doesn't matter
                ALUControlD = 'b11;
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // LDR
            8'b010_1100_1 : begin
                PCSrcD    = 0; 
                MemtoRegD = 1; 
                MemWriteD = 0; 
                ALUSrcD   = 1;
                RegWriteD = 1;
                RegSrcD   = 'b10;    // msb doesn't matter
                ImmSrcD   = 'b01; 
                ALUControlD = 'b00;  // do an add
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // STR
            8'b010_1100_0 : begin
                PCSrcD    = 0; 
                MemtoRegD = 0; // doesn't matter
                MemWriteD = 1; 
                ALUSrcD   = 1;
                RegWriteD = 0;
                RegSrcD   = 'b10;    // msb doesn't matter
                ImmSrcD   = 'b01; 
                ALUControlD = 'b00;  // do an add
					 FlagWriteD = 0;
					 BranchD = 0;
            end

            // B
            8'b1010_???? : begin
               // Based on flag bits, conditions defined are computed for branch execution
					// If conditions are not met, instruction is completely ignored & control logic deasserted
					 PCSrcD    = 1; 
                MemtoRegD = 0; // doesn't matter
                MemWriteD = 0; 
                ALUSrcD   = 1;
                RegWriteD = 0;
                RegSrcD   = 'b01;    // msb doesn't matter
                ImmSrcD   = 'b10; 
                ALUControlD = 'b00;  // do an add
					 FlagWriteD = 0;
					 BranchD = 1;	 
				end

			default: begin
				  PCSrcD    = 0; 
				  MemtoRegD = 0; // doesn't matter
				  MemWriteD = 0; 
				  ALUSrcD   = 0;
				  RegWriteD = 0;
				  RegSrcD   = 'b00;
				  ImmSrcD   = 'b00; 
				  ALUControlD = 'b00;  // do an add
				  FlagWriteD = 0;
				  BranchD = 0;	
			end
		endcase
    end


endmodule