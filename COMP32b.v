module COMP32b(output 	Less,
	       input [31:0] A, B,
	       input 	    uMod      // 1:UnsignedMode, 0:SignedMode
	       );

   wire [31:0] 		    Sum, Bm;
   wire 		    IsDiffSign;

   assign Bm = ~B + 1'b1; 
   assign IsDiffSign = A[31] ^ B[31]; 

   FullAdder32bit add32b(
			 // Outputs
			 .Sum			(Sum),
			 .Cout			(Cout),
			 // Inputs
			 .A			(A),
			 .B			(Bm),
			 .Cin			(1'b0));

   assign Less = IsDiffSign? (uMod? ~A[31] : A[31]) : Sum[31];
   
endmodule


module FullAdder32bit (
		       input wire [31:0]  A,
		       input wire [31:0]  B,
		       input wire 	  Cin,
		       output wire [31:0] Sum,
		       output wire 	  Cout
		       );

   assign {Cout, Sum} = A + B + Cin;

endmodule
