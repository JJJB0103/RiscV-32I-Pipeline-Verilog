module ImmGen(
	      output [31:0] Imm,
	      input [24:0]  inst_Imm, //25鍮꾪듃 吏쒕━, == instruction[31:7]
	      input [2:0]   ImmSel
	      );

   wire [31:0] 		    I, B, U, J, S;  // type of Immediate 0~4
   // 000, 001, 010, 011, 100
   wire [31:0] 		    IB, JU, IS, IBJU;

   assign I = {{20{inst_Imm[24]}}, inst_Imm[24 -: 12]}; // [24 -: 12] -> part-select addressing -> '24 is offset' and '12 is width' 利� 24遺��꽣 �옉�� �씤�뜳�뒪 諛⑺뼢�쑝濡� 12媛쒕�� �꽑�깮�븳�떎�뒗 �쑜. [24:13]怨� �룞�씪
   assign U = {inst_Imm[24 -: 20], 12'b0};
   assign B = {{19{inst_Imm[24]}}, inst_Imm[24], inst_Imm[0], inst_Imm[23 -: 6], inst_Imm[1 +: 4], 1'b0};
   assign J = {{12{inst_Imm[24]}}, inst_Imm[24], inst_Imm[12:5], inst_Imm[13], inst_Imm[23:14], 1'b0};//J�씪�븣�뒗 [31:12]珥� 20bit媛�  input�쑝濡� �뱾�뼱媛�.  imm[20|10:1|11|19:12]濡� �뱾�뼱媛� 珥� 20bit �븵�뿉 12�뒗 sign extension
   assign S = {{20{inst_Imm[24]}}, inst_Imm[24:18], inst_Imm[4:0]};

   assign IB = ImmSel[0] ? B : I;
   assign JU = ImmSel[0] ? J : U;
   assign IS = (|ImmSel[1:0]) ? I : S;
   assign IBJU = ImmSel[1] ? JU : IB;
   assign Imm = ImmSel[2] ? IS : IBJU;
   // I: 000, 101, 110, 111
   // B: 001
   // U: 010
   // J: 011
   // S: 100

endmodule
