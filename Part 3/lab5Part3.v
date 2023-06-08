// Computer Architecture (CO224) - Lab 05
// Design: Testbench of Integrated CPU of Simple Processor

//Group: 22
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
        
        // METHOD 1: manually loading instructions to instr_mem
        //{instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]} = 32'b00000000000001000000000000000101;
        //{instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]} = 32'b00000000000000100000000000001001;
        //{instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]} = 32'b00000010000001100000010000000010;
        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
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
module mux(MUXIN1,MUXIN2,MUXSELECT,MUXOUT);
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

//module for a adder to increment the pc value
module PCADDER (PCINPUT,PCNEXT);
	input [31:0] PCINPUT;    //32 bit input
	output [31:0] PCNEXT; //32 bit output register
	
	//get executed whenever the value of PCINPUT changes
	assign #1 PCNEXT = PCINPUT + 4;  //has a delay of 1 unit
	
endmodule //PCADDER

//module for the control unit
module control(OP,WRITEENABLE,ISMINUS,ISIMM,ALUOP);
	input[7:0] OP ;                            //8 bit input
	output reg[0:0] WRITEENABLE,ISMINUS,ISIMM; //3 output registers
	output reg [2:0] ALUOP;                    //3 bit output register

    //Get executed whenever the OP changes
	always @(OP) begin
		//Generating the signals according to the opcode
		//with a delay of 1 unit
		case(OP)
			//loadi
			8'b00000000 : 
			begin
			#1
				WRITEENABLE =1'b1;
				ISMINUS = 1'b0;
				ISIMM = 1'b1;
				ALUOP = 3'b000;
			end 
			
			//mov
			8'b00000001 : 
			begin
			#1
				WRITEENABLE = 1'b1;
				ISMINUS = 1'b0;
				ISIMM = 1'b0;
				ALUOP = 3'b000;
			end
			
			//add
			8'b00000010 :
			begin
			#1
				WRITEENABLE =1'b1;
				ISMINUS = 1'b0;
				ISIMM = 1'b0;
				ALUOP = 3'b001;
			end
			
			//sub
			8'b00000011 :
			begin
			#1
				WRITEENABLE =1'b1;
				ISMINUS = 1'b1;
				ISIMM = 1'b0;
				ALUOP = 3'b001;
			end
			
			//and
			8'b00000100 :
			begin
			#1
				WRITEENABLE =1'b1;
				ISMINUS = 1'b0;
				ISIMM = 1'b0;
				ALUOP = 3'b010;
			end
				
			//or
			8'b00000101 :
			begin
			#1
				WRITEENABLE =1'b1;
		        ISMINUS = 1'b0;
				ISIMM = 1'b0;
				ALUOP = 3'b011;
			end
		endcase
	end	
endmodule //control

//module for CPU
module cpu(PC,INSTRUCTION,CLK,RESET);
	input [31:0]INSTRUCTION; //32 bit input
	input  CLK,RESET;        //2 inputs
	output reg[31:0]PC;      //32 bit output reg
	
	reg [7:0]OPCODE,IMMEDIATE;
	reg[2:0]WRITEREG,READREG1,READREG2;                                    
	wire[7:0]ALURESULT,REGOUT1,REGOUT2,MUX1OUT,MUX2OUT,TWOSRESULT;   //6 8-bit wires
	wire[2:0]ALUOP;                                                    //3 bit wire
	wire ISMINUS,ISIMM,WRITEENABLE;                                    //3 wires
	wire [31:0] PCRESULT;                                              //32 bit wire
	
	//instantiate PCADDER module
	PCADDER adder1(.PCINPUT(PC),.PCNEXT(PCRESULT));
	//Get executed at the positive edge of the clock
	always @(posedge CLK) begin	
		//if RESET is high assign 0 to the PC
		if(RESET) begin
			#1 PC = 0;
		//else assign the incremented value to the PC
	end else begin	
		#1 PC = PCRESULT;
	end
	end
	
	//decoding 
	always @(INSTRUCTION) begin
		OPCODE = INSTRUCTION[31:24];    //24-31 bits -> Opcode
		WRITEREG = INSTRUCTION[18:16];      //16-18 bits -> Destination register
		READREG1 = INSTRUCTION[10:8];       //8-10 bits -> Source register 1
		READREG2 = INSTRUCTION[2:0];        //0-7 bits -> Source register 2 / Immediate value
		IMMEDIATE = INSTRUCTION[7:0];
	end
	
	//instantiate control module
	control controlUnit1(.OP(OPCODE),.WRITEENABLE(WRITEENABLE),.ALUOP(ALUOP),.ISMINUS(ISMINUS),.ISIMM(ISIMM));
	//instantiate reg_file module
	reg_file regfile1(.IN(ALURESULT),.OUT1(REGOUT1),.OUT2(REGOUT2),.INADDRESS(WRITEREG),.OUT1ADDRESS(READREG1),.OUT2ADDRESS(READREG2),.WRITE(WRITEENABLE),.CLK(CLK),.RESET(RESET));
	//instantiate alu model
	alu alu1(.DATA1(REGOUT1),.DATA2(MUX2OUT),.RESULT(ALURESULT),.SELECT(ALUOP));
	//instantiate mux module for twos complement
	mux mux1twos(.MUXIN1(REGOUT2),.MUXIN2(TWOSRESULT),.MUXOUT(MUX1OUT),.MUXSELECT(ISMINUS));
	//instantiate mux module for immidiate value
	mux mux2imm(.MUXIN1(MUX1OUT),.MUXIN2(IMMEDIATE),.MUXOUT(MUX2OUT),.MUXSELECT(ISIMM));
	//instantiate twosComp module
	twosComp twosComp1(.TWOSIN(REGOUT2),.TWOSOUT(TWOSRESULT));	
	
endmodule //cpu

//Register file
module reg_file(IN,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,WRITE,CLK,RESET);
	//port declaration
	input[7:0] IN;						//8-bit input (the data input)
	input[2:0]INADDRESS;					//3-bit input (reg no to store data IN )
	input [2:0] OUT1ADDRESS,OUT2ADDRESS; 			//2-3-bit input(where data retrieved from)
	input CLK,RESET,WRITE;					//3 inputs for synchronization and enable write
	output [7:0] OUT1,OUT2;					//2 8-bit parallel data outputs
	reg [7:0] r[0:7]; 					//array of 8 8-bit registers

	integer i;

	//triggered at the positive edge of the clock
	always @(posedge CLK)begin
	//if RESET is high
	if(RESET) begin
		#1  		//delay of 1 unit for writing	to the registers
		//All registered cleared(written 0)
		for(i=0;i<8;i++)
		r[i] = 8'd0;	
	end
        
	if(WRITE) begin
        //Write to the register given by the address with the value given
        #1 r[INADDRESS] = IN;   //has 1 unit delay when writing to the registers
    end
	end 

//Get executed whenever the register values or addresses changes
//always @(OUT1ADDRESS or OUT2ADDRESS or r[OUT1ADDRESS] or r[OUT2ADDRESS]) begin

	//read the value in the register with the given OUT1ADDRESS and load to OUT1
	 
	assign #2 OUT1 = r[OUT1ADDRESS];	//has 2 unit delay when reading from the registers
	//read the value in the register with the given OUT2ADDRESS and load to OUT2
	assign #2 OUT2 = r[OUT2ADDRESS];	//has 2 unit delay when reading from the registers
//end
 // For testing ----------------------------------------------------------------------------------------------------------------------------------------------------
    initial
    begin
        // monitor change in reg file content and print 
        #5;
        $display("\n\t\t\t=================================================");
        $display("\t\t\t Change of Register Content starting from Time #5");
        $display("\t\t\t=================================================\n");
        $display("\t\ttime\treg0\treg1\treg2\treg3\treg4\treg5\treg6\treg7");
        $display("\t\t--------------------------------------------------------------------");
        $monitor($time, "\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d",r[0],r[1], r[2],r[3], r[4], r[5], r[6], r[7]);
    end
    // -----
endmodule	

//ALU
module alu(DATA1,DATA2,RESULT,SELECT);
	//Port declaration
	input [7:0] DATA1;					//8 bit unsigned input 
	input signed [7:0] DATA2;			//8 bit signed input
	input [2:0] SELECT;					//3 bit unsigned input
	output reg signed[7:0] RESULT;		//8 bit signed input register
	
	wire [7:0] FORWARD_RESULT,AND_RESULT,OR_RESULT,ADD_RESULT; //8 bit wires to collect design outputs

	//instantiate FORWARD module
	FORWARD myforward(DATA2,FORWARD_RESULT);
	//instantiate AND module	
	AND myand(DATA1,DATA2,AND_RESULT);
	//instantiate OR module
	OR myor(DATA1,DATA2,OR_RESULT);
	//instantiate ADD module
	ADD myadd(DATA1,DATA2,ADD_RESULT);

	//Get executed whenever the values SELECT,FORWARD_RESULT,AND_RESULT,ADD_RESULT and OR_RESULT changes
	always @(SELECT or FORWARD_RESULT or AND_RESULT or ADD_RESULT or OR_RESULT)
	//Select the result based on the value of the selecct signal
	begin
		case(SELECT)
			3'b000 : RESULT = FORWARD_RESULT; 
			3'b001 : RESULT = ADD_RESULT;
			3'b010 : RESULT = AND_RESULT;
			3'b011 : RESULT = OR_RESULT;
			default:RESULT = 8'b0;
		endcase
	end
endmodule	

//module for FORWARD function
module FORWARD(IN,OUT);
	input [7:0] IN;		//8 bit input
	output [7:0] OUT;	//8 bit output
	
	assign #1 OUT = IN; //assign input to output with 1 unit delay
endmodule	//FORWARD

//module for AND function
module AND(IN1,IN2,OUT);
	input [7:0] IN1,IN2; //8 bit inputs
	output [7:0] OUT;	 //8 bit inputs
	
	assign #1 OUT = IN1 & IN2; //bitwise and on inputs with 1 unit delay
endmodule	//AND

//module for OR function
module OR(IN1,IN2,OUT);
	input [7:0] IN1,IN2; //8 bit inputs
	output [7:0] OUT;    //8 bit output
	
	assign #1 OUT = IN1 | IN2;	//bitwise or on inputs with 1 unit delay
endmodule	//OR

//module for ADD function
module ADD(IN1,IN2,OUT);
	input [7:0] IN1;			//8 bit unsigned input 
	input signed [7:0] IN2;		//8 bit signed input 
	output signed [7:0] OUT;	//8 bit signed output 
	
	assign #2 OUT=IN1+IN2;	//addition of inputs with 2 unit delay
endmodule	//ADD
