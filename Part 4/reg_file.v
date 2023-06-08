//Register file
module reg_file(IN,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,WRITE,CLK,RESET);
	//port declaration
	input[7:0] IN;						//8-bit input (the data input)
	input[2:0]INADDRESS;					//3-bit input (reg no to store data IN )
	input [2:0] OUT1ADDRESS,OUT2ADDRESS; 			//2-3-bit input(where data retrieved from)
	input CLK,RESET,WRITE;					//3 inputs for synchronization and enable write
	output [7:0] OUT1,OUT2;		//2 8-bit parallel data outputs
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
