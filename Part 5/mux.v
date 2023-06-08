module MUX8x1(OUT,IN0,IN1,IN2,IN3,IN4,IN5,IN6,IN7,SEL);
	input IN0,IN1,IN2,IN3,IN4,IN5,IN6,IN7;
	input [2:0] SEL;
	output reg [0:0] OUT;
	
	always@(*) begin
		case(SEL)
			3'b000 : OUT = IN0;
			3'b001 : OUT = IN1;
			3'b010 : OUT = IN2;
			3'b011 : OUT = IN3;
			3'b100 : OUT = IN4;
			3'b101 : OUT = IN5;
			3'b110 : OUT = IN6;
			3'b111 : OUT = IN7;
		endcase
	end
endmodule


