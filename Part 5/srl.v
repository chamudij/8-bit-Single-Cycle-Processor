
module SRL(OUTPUT,IN,SHIFT); //module to logical shift right by  given SHIFT value
	input[7:0] IN,SHIFT;
	output[7:0] OUTPUT;
	reg [2:0] SEL;
	wire [7:0] OUT;
	
	always@(SHIFT) begin
	case(SHIFT)
	//here depending on the shifting value it assigns values to select signals
		8'b000: 
			SEL = 3'b000;
		8'b001 : 
			SEL = 3'b001;
		8'b010 : 
			SEL = 3'b010;
		8'b011 : 
			SEL = 3'b011;	
		8'b100 : 
			SEL = 3'b100;
		8'b101 : 
			SEL = 3'b101;
		8'd110 : 
			SEL = 3'b110;
		8'd111 : 
			SEL = 3'b111;
	endcase
	end
	
	//here mux 8 represents least significant bit of the value after right shifted and
	//mux 1 represents most significant bit of the value after right shifted 
	//after shifting to the right it's padding out with 0's (most significant bits)

	MUX8x1 mux1(OUT[7],IN[7],1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,SEL);
	MUX8x1 mux2(OUT[6],IN[6],IN[7],1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,SEL);
	MUX8x1 mux3(OUT[5],IN[5],IN[6],IN[7],1'b0,1'b0,1'b0,1'b0,1'b0,SEL);
	MUX8x1 mux4(OUT[4],IN[4],IN[5],IN[6],IN[7],1'b0,1'b0,1'b0,1'b0,SEL);
	MUX8x1 mux5(OUT[3],IN[3],IN[4],IN[5],IN[6],IN[7],1'b0,1'b0,1'b0,SEL);
	MUX8x1 mux6(OUT[2],IN[2],IN[3],IN[4],IN[5],IN[6],IN[7],1'b0,1'b0,SEL);
	MUX8x1 mux7(OUT[1],IN[1],IN[2],IN[3],IN[4],IN[5],IN[6],IN[7],1'b0,SEL);
	MUX8x1 mux8(OUT[0],IN[0],IN[1],IN[2],IN[3],IN[4],IN[5],IN[6],IN[7],SEL);
	
	assign #2 OUTPUT = OUT;
endmodule

