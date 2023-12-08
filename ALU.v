module ALU(
	   output [31:0]       ALU_o,
	   input signed [31:0] A, B,
	   input [3:0] 	       ALUSel  // ALUSel[3] = funct7[5] = instruction[30]
           // ALUSel[2:0] = funct3 = instruction[14:12]
	   );

   wire signed [31:0] 	       ADD, SRA, SLL, SRL, XOR, AND, OR, SR;
   wire signed [31:0] 	       B_compliment = ALUSel[3] ? ~B : B;
   wire signed [4:0] 	       shamt = B[4:0];
   wire 		       SLT;
   wire 		       SLT_cond = (ALUSel == 3'b010);

   assign ADD = A + B_compliment + ALUSel[3];  // ALUSel[3]�씠 adder�쓽 carry in�쑝濡� �뱾�뼱媛�硫� 醫뗭쓣 寃� 媛숈쓬
   assign XOR = A ^ B;
   assign AND = A & B;
   assign OR  = A | B;
   assign SLL = A << (shamt);
   assign SRL = A >> (shamt);
   assign SRA = A >>> (shamt);
   assign SR = ALUSel[3] ? SRA : SRL;

   COMP32b COMPARE(SLT, A, B, ALUSel[0]); // �삉 留뚮뱾 �븘�슂�뾾�씠 ADD �씠�슜�빐�꽌 諛붾줈 援ы쁽 媛��뒫
   mux8to1 MUX0(ALU_o, ADD, SLL, {31'b0, SLT}, {31'b0, SLT}, XOR, SR, OR, AND, ALUSel[2:0]);

endmodule


module mux8to1(
	       output [31:0] mux_o,
	       input [31:0]  mux0, mux1, mux2, mux3, mux4, mux5, mux6, mux7,
	       input [2:0]   sel
	       );

   wire [31:0] 		     b0, b1, b2, b3;
   wire [31:0] 		     c0, c1;

   assign b0 = sel[0] ? mux1 : mux0;
   assign b1 = sel[0] ? mux3 : mux2;
   assign b2 = sel[0] ? mux5 : mux4;
   assign b3 = sel[0] ? mux7 : mux6;

   assign c0 = sel[1] ? b1 : b0;
   assign c1 = sel[1] ? b3 : b2;

   assign mux_o = sel[2] ? c1 : c0;

endmodule
