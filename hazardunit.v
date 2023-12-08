module hazardunit(input  [4:0] Rs1D, Rs2D, Rs1E, Rs2E,
                input  [4:0] RdE, RdM, RdW,
                input  RegWriteM, RegWriteW,
				    input  WBSelE0,WBSelM0, WBSelW0, PCSrcF, PCSrcE, PCSrcM, clk,
				    input [31:0] instF_in,
                output reg [1:0] ForwardAE, ForwardBE,
                output reg StallD, StallF, FlushD, FlushE, Bubble, ForwardAD, ForwardBD);
                
    reg lwStall;					 
    reg Bubble;
    always @(posedge clk)begin
    //instF_out <= instF_in;
    end
    
// RAW					 
// Whenever source register (Rs1E, Rs2E) in execution stage matchces with the destination register (RdM, RdW)
// of a previous instruction's Memory or WriteBack stage forward the ALUResultM or ResultW
// And also only when RegWrite is asserted

    always @ (*) begin
    ForwardAE = 2'b00;
    ForwardBE = 2'b00;
    ForwardAD = 1'b0;
    ForwardBD = 1'b0;
 
    if ((WBSelW0 == 1) & (RdW == Rs1D))
    ForwardAD=1'b1;
    if ((WBSelW0 == 1) & (RdW == Rs2D))
    ForwardBD=1'b1;
    
 
       if ((Rs1E == RdM) & (RegWriteM) & (Rs1E != 0)) // higher priority - most recent
            ForwardAE = 2'b10; // for forwarding ALU Result in Memory Stage
        else if ((Rs1E == RdW) & (RegWriteW) & (Rs1E != 0))
            ForwardAE = 2'b01; // for forwarding WriteBack Stage Result
                    
        if ((Rs2E == RdM) & (RegWriteM) & (Rs2E != 0))
            ForwardBE = 2'b10; // for forwarding ALU Result in Memory Stage

        else if ((Rs2E == RdW) & (RegWriteW) & (Rs2E != 0))
            ForwardBE = 2'b01; 
// for forwarding WriteBack Stage Result
// For Load Word Dependency result does not appear until end of Data Memory Access Stage
// if Destination register in EXE stage is equal to souce register in decode stage
// stall previous instructions until the the load word is avialbe at the writeback stage
// Introduce One cycle latency for subsequent instructions after load word 
// There is two cycle difference between Memory Access and the immediate next instruction
        
        Bubble = (PCSrcF==1)&(PCSrcM==1); 
           
        lwStall = ((WBSelE0 == 1) & ((RdE == Rs1D) | (RdE == Rs2D)) | (WBSelM0 == 1) & ((RdM == Rs1D) | (RdM == Rs2D)));
        StallF = (lwStall|Bubble); 
        StallD = (lwStall|Bubble); 
        
        //instF_out=instF_in;
        //jal_jalr = (PCSrcD==1)&(RdW==Rs1D);
	
// control hazard
// whenever branch has been taken, we flush the following two instructions from decode and execute pip reg        

        FlushE = lwStall | PCSrcE;
        FlushD = PCSrcE;
     	  end
endmodule
