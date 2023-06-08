module testbed;
	reg [7:0]OPERAND1;
	reg signed[7:0]OPERAND2;
	reg [2:0]ALUOP;
	wire signed [7:0] ALURESULT;
	
	//instantiate alu module
	alu a1(.DATA1(OPERAND1),.DATA2(OPERAND2),.RESULT(ALURESULT),.SELECT(ALUOP));
	
	initial begin
	//assign values to inputs to get the outputs
	OPERAND1 = 8'd100;
	OPERAND2 = 8'd20;
	ALUOP = 3'd0;
	#5
	
	OPERAND1 = 8'd95;
	OPERAND2 = 8'd10;
	ALUOP = 3'd1;
	#5
	
	OPERAND1 = 8'd110;
	OPERAND2 = 8'd128;
	ALUOP = 3'd1;
	#5
	
	OPERAND1 = 8'd4;
	OPERAND2 = 8'd4;
	ALUOP = 3'd2;
	#5;
	
	OPERAND1 = 8'd5;
	OPERAND2 = 8'd6;
	ALUOP = 3'd3;
	#5
	$finish;
	end
	
	initial begin
		//display variable values in every change of variables
		$monitor("Time = %g\tALUOP=%d\t\tOPERRAND1=%d\t\tOPERAND2=%d\t\tALURESULT= %d",$time,ALUOP,OPERAND1,OPERAND2,ALURESULT);
		
		//generate files needed to plot the waveform using GTKWave
		$dumpfile("data1.vcd");
        $dumpvars(0,testbed);	
	end
	
endmodule	

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

