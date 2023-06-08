// Computer Architecture (CO224) - Lab 05
// Design: Testbench of Integrated CPU of Simple Processor
//Group : 22

`include "alu.v"
`include "reg_file.v"


module cpu_tb;

    reg CLK, RESET;
    wire [31:0] PC;
    reg [31:0] INSTRUCTION;
    
    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    */
    
    // TODO: Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
    reg [7:0]instr_mem [1023:0];
    // TODO: Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    //       (make sure you include the delay for instruction fetching here)
    always @(PC) begin
		#2
		INSTRUCTION = {instr_mem[PC+3],instr_mem[PC+2],instr_mem[PC+1],instr_mem[PC]};
	end
        
    initial
    begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
        
        //loading instr_mem content from instr_mem.mem file
        $readmemb("instr_mem.mem", instr_mem);
    end
    
    /* 
    -----
     CPU
    -----
    */
    cpu mycpu(PC, INSTRUCTION, CLK, RESET);

    initial
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, cpu_tb);
        
        CLK = 1'b0;
        RESET = 1'b0;
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        RESET = 1'b1;
        #8
        RESET = 1'b0;
        // finish simulation after some time
        #500
        $finish;
        
    end
	
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule

//module to caculate twos complement
module twosComp(TWOSIN,TWOSOUT);
	input[7:0]TWOSIN;     //8 bit input
	output[7:0]TWOSOUT;   //8 bit output
	
	assign #1 TWOSOUT = ~TWOSIN + 8'b00000001;  //twos complement of the input assigned to output
endmodule //twosComp

//module for a mux

module mux08(MUXIN1,MUXIN2,MUXSELECT,MUXOUT);
	input [7:0]MUXIN1;       //8 bit input
	input [7:0]MUXIN2;       //8 bit input
	input MUXSELECT;         //input
	output reg [7:0]MUXOUT;  //8 bit output register
	
	//Get executed whenever the values of SELECT,IN1,IN2 changes
	always @(MUXSELECT,MUXIN1,MUXIN2) begin
	//Choose the output according to the value of select 
	case(MUXSELECT)
		1'b0 : MUXOUT = MUXIN1;
		1'b1 : MUXOUT = MUXIN2;
	endcase
	end
endmodule 

module mux32(MUXIN1,MUXIN2,MUXSELECT,MUXOUT);
	input [31:0]MUXIN1;       //32 bit input
	input [31:0]MUXIN2;       //32 bit input
	input MUXSELECT;         //input
	output reg [31:0]MUXOUT;  //32 bit output register
	
	//Get executed whenever the values of SELECT,IN1,IN2 changes
	always @(MUXSELECT,MUXIN1,MUXIN2) begin
	//Choose the output according to the value of select 
	case(MUXSELECT)
		1'b1 : MUXOUT = MUXIN1;
		1'b0 : MUXOUT = MUXIN2;
	endcase
	end
endmodule 

//module for a adder to increment the pc value
module PCADDER (PCINPUT,PCNEXT);
	input [31:0] PCINPUT;    //32 bit input
	output [31:0] PCNEXT; //32 bit output register
	
	//get executed whenever the value of PCINPUT changes
	assign #1 PCNEXT = PCINPUT + 4;  //has a delay of 1 unit
	
endmodule //PCADDER

//module to determine PC value in a j/beq instruction
module ADDER(PCINPUT2,INCREMENT,PCNEXT2);
	input signed [31:0] PCINPUT2;    //32 bit input
	input signed [7:0] INCREMENT;
	output signed [31:0] PCNEXT2; //32 bit output register
	
	//get executed whenever the value of PCINPUT changes
	assign #2 PCNEXT2 = PCINPUT2 + (INCREMENT<<2);  //has a delay of 2 unit
endmodule

//module for the control unit
module control(OP,WRITEENABLE,ISMINUS,ISIMM,ALUOP,JUMP,BRANCH);
	input[7:0] OP ;                            //8 bit input
	output reg[0:0] WRITEENABLE,ISMINUS,ISIMM,BRANCH,JUMP; //3 output registers
	//output reg[1:0] OFFSET; //3 output registers
	output reg [2:0] ALUOP;                    //3 bit output register

    //Get executed whenever the OP changes
	always @(OP) begin
		//Generating the signals according to the opcode
		//with a delay of 1 unit
		#1 case(OP)
			8'b0000_0000 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH} = 8'b000_1_0_1_0_0;	//loadi
			8'b0000_0001 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH} = 8'b000_1_0_0_0_0;	//mov
			8'b0000_0010 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH} = 8'b001_1_0_0_0_0;	//add
			8'b0000_0011 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH} = 8'b001_1_1_0_0_0;	//sub
			8'b0000_0100 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH} = 8'b010_1_0_0_0_0;	//and
			8'b0000_0101 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH} = 8'b011_1_0_0_0_0;	//or
			8'b0000_0110 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH} = 8'bxxx_0_0_0_1_0;	//j
			8'b0000_0111 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH} = 8'b100_0_1_0_0_1;	//beq
		
		endcase
	end	
endmodule 

//module for CPU
module cpu(PC,INSTRUCTION,CLK,RESET);
	input [31:0]INSTRUCTION; //32 bit input
	input  CLK,RESET;        //2 inputs
	output reg[31:0]PC;      //32 bit output reg
	
	reg [7:0]OPCODE,IMMEDIATE;
  reg signed [7:0] INCREMENT;
	reg[2:0]WRITEREG,READREG1,READREG2;                                    
	wire[7:0]ALURESULT,REGOUT1,REGOUT2,MUX1OUT,MUX2OUT,TWOSRESULT;     //6 8-bit wires
	wire[2:0]ALUOP;                                                    //3 bit wire
	wire ISMINUS,ISIMM,WRITEENABLE,ZERO,JUMP,BRANCH,BSEL;              //7 wires
	wire [31:0] PCRESULT,OFFSET,PCnext,PCVAL;                          //32 bit wire
	wire [1:0] SEL;
 
    
    and (BSEL,BRANCH,ZERO);  //BSEL = BRANCH AND ZERO 
	//Instantiate mux32 module to choose between jump and normal increment of pc
	mux32 mux01(.MUXIN1(OFFSET),.MUXIN2(PCRESULT),.MUXSELECT(JUMP),.MUXOUT(PCVAL));
	//Instantiate mux32 module to choose between beq and jump/normal increment
	mux32 mux02(.MUXIN1(OFFSET),.MUXIN2(PCVAL),.MUXSELECT(BSEL),.MUXOUT(PCnext));
	
	//Get executed at the positive edge of the clock
	always @(posedge CLK) begin
		//if RESET is high assign 0 to the PC
		if(RESET) begin
			#1 PC = 0;
		//else assign the incremented value to the PC
		end else begin
			#1 PC = PCnext;
		end
	end
	
 	//instantiate PCADDER module
	PCADDER adder1(.PCINPUT(PC),.PCNEXT(PCRESULT)); 
    //instantiate adder to increase the value in j/beq instruction	
	ADDER adder2(.PCINPUT2(PCRESULT),.INCREMENT(INCREMENT),.PCNEXT2(OFFSET));  
	
	//decoding
	always @(INSTRUCTION) begin
		OPCODE = INSTRUCTION[31:24];        //24-31 bits -> Opcode
		WRITEREG = INSTRUCTION[18:16];      //16-18 bits -> Destination register
		READREG1 = INSTRUCTION[10:8];       //8-10 bits -> Source register 1
		READREG2 = INSTRUCTION[2:0];        //0-7 bits -> Source register 2 
		IMMEDIATE = INSTRUCTION[7:0];       //0-7 bits -> Immediate value
		INCREMENT = INSTRUCTION[23:16];     //16-23 bits -> Increment value
	end
	
	//instantiate control module
	control controlUnit1(.OP(OPCODE),.WRITEENABLE(WRITEENABLE),.ALUOP(ALUOP),.ISMINUS(ISMINUS),.ISIMM(ISIMM),.BRANCH(BRANCH),.JUMP(JUMP));
	//instantiate reg_file module
	reg_file regfile1(.IN(ALURESULT),.OUT1(REGOUT1),.OUT2(REGOUT2),.INADDRESS(WRITEREG),.OUT1ADDRESS(READREG1),.OUT2ADDRESS(READREG2),.WRITE(WRITEENABLE),.CLK(CLK),.RESET(RESET));
	//instantiate alu model
	alu alu1(.DATA1(REGOUT1),.DATA2(MUX2OUT),.ZERO(ZERO),.RESULT(ALURESULT),.SELECT(ALUOP));
	//instantiate mux08 module for twos complement
	mux08 mux1twos(.MUXIN1(REGOUT2),.MUXIN2(TWOSRESULT),.MUXOUT(MUX1OUT),.MUXSELECT(ISMINUS));
	//instantiate mux08 module for immidiate value
	mux08 mux2imm(.MUXIN1(MUX1OUT),.MUXIN2(IMMEDIATE),.MUXOUT(MUX2OUT),.MUXSELECT(ISIMM));
	//instantiate twosComp module
	twosComp twosComp1(.TWOSIN(REGOUT2),.TWOSOUT(TWOSRESULT));	
	
endmodule //cpu

