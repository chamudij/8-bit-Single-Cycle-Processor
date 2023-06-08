// Computer Architecture (CO224) - Lab 05 Part2
// Design: Register File of Simple Processor
// Group : Group22

module reg_file_tb;
    
    reg [7:0] WRITEDATA;
    reg [2:0] WRITEREG, READREG1, READREG2;
    reg CLK, RESET, WRITEENABLE; 
    wire [7:0] REGOUT1, REGOUT2;
    
    reg_file myregfile(WRITEDATA, REGOUT1, REGOUT2, WRITEREG, READREG1, READREG2, WRITEENABLE, CLK, RESET);
       
    initial
    begin
        CLK = 1'b1;
        
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("reg_file_wavedata.vcd");
		$dumpvars(0, reg_file_tb);
        
        // assign values with time to input signals to see output 
        RESET = 1'b0;
        WRITEENABLE = 1'b0;
        
        #4
        RESET = 1'b1;
        READREG1 = 3'd0;
        READREG2 = 3'd4;
        
        #6
        RESET = 1'b0;
        
        #2
        WRITEREG = 3'd2;
        WRITEDATA = 8'd95;
        WRITEENABLE = 1'b1;
        
        #7
        WRITEENABLE = 1'b0;
        
        #1
        READREG1 = 3'd2;
        
        #7
        WRITEREG = 3'd1;
        WRITEDATA = 8'd28;
        WRITEENABLE = 1'b1;
        READREG1 = 3'd1;
        
        #8
        WRITEENABLE = 1'b0;
        
        #8
        WRITEREG = 3'd4;
        WRITEDATA = 8'd6;
        WRITEENABLE = 1'b1;
        
        #8
        WRITEDATA = 8'd15;
        WRITEENABLE = 1'b1;
        
        #10
        WRITEENABLE = 1'b0;
        
        #6
        WRITEREG = -3'd1;
        WRITEDATA = 8'd50;
        WRITEENABLE = 1'b1;
        
        #5
        WRITEENABLE = 1'b0;
        
        #10
        $finish;
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;
        

endmodule

module reg_file(IN,OUT1,OUT2,INADDRESS,OUT1ADDRESS,OUT2ADDRESS,WRITE,CLK,RESET);
//port declaration
input[7:0] IN;							//8-bit input 
input[2:0]INADDRESS;					//3-bit input
input [2:0] OUT1ADDRESS,OUT2ADDRESS; 	//2-3-bit input
input CLK,RESET,WRITE;					//3 inputs
output reg [7:0] OUT1,OUT2;				//2 8-bit output registers
reg [7:0] r[0:7]; 						//array of 8 8-bit registers

integer i;

//triggered at the positive edge of the clock
always @(posedge CLK)begin
//if RESET is high
if(RESET) begin
	#1  		//delay of 1 unit for writing	
    //All registered cleared(written 0)
    for(i=0;i<8;i++)
     r[i] = 8'd0;	//has 1 unit delay when writing to the registers
   
end
if(WRITE) 
        //Write to the register given by the address with the value given
        #1 r[INADDRESS] = IN;   //has 1 unit delay when writing to the registers
   


end


//Get executed whenever the register values or addresses changes
always @(OUT1ADDRESS or OUT2ADDRESS or r[OUT1ADDRESS] or r[OUT2ADDRESS]) begin

	//read the value in the register with the given OUT1ADDRESS to OUT1
	#2  
	OUT1 = r[OUT1ADDRESS];	//has 2 unit delay when reading from the registers
	//read the value in the register with the given OUT2ADDRESS to OUT2
	OUT2 = r[OUT2ADDRESS];	//has 2 unit delay when reading from the registers
end
endmodule	
