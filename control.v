module control(
	       output 	    PCsel, RegWEn, BrUn,
	       output [2:0] ImmSel, WordSizeSel,
	       output 	    BSel, ASel, MemRW, 
	       output [3:0] ALUSel,
	       output [1:0] WBSel,
	       input [31:0] instruction, 
	       input 	    BrEq, BrLT
	       );
   // 諛묒뿉 二쇱꽍 ?꽕紐낆뿉?꽌 ?굹?삤?뒗 opcode5?뒗 7鍮꾪듃 opcode以묒뿉?꽌 msb遺??꽣 5媛? 鍮꾪듃瑜? ?쓽誘?, 利?  opcode[6:2]瑜? ?쓽誘명븿. 留덉?留? 2鍮꾪듃?뒗 RV32I?뿉?꽌?뒗 ?빆?긽 11?씠湲? ?븣臾몄뿉 臾댁떆?븿.

   // ?떊?샇蹂? ?꽕紐?
   // PCsel : pc+4=0, ALUo=1
   // ImmSel : I=0 or 5 or 6 or 7, B=1, U=2, J=3, S=4
   // ASel : RD1=0, PC=1
   // BSel : Imm=1, RD2=0
   // ALUSel : SUB/SRA=[3], OperationFunctionSelect=[2:0]
   // MemRW : Read=0, Write(only at SW opcode)=1
   // RegWEn : NotEnabled=0, Enabled=1
   // WBSel : ALUo=1, MEM=0, pc+4=2
   // WordSizeSel : [7:5] at SaveLoad==1


   wire 		    funct7_5 = instruction[30];
   wire [2:0] 		    funct3 = instruction[14:12];
   wire [6:0] 		    opcode5 = instruction[6:0];
   //
   wire 		    JALR, LOAD, ALUi, LUI, AUIPC;
   wire 		    I_cond, S_cond, B_cond, U_cond, J_cond;
   wire 		    BEQ, BNE, BLT, BGE;
   wire 		    RegNWRT;
   wire 		    WBp4_cond;
   wire 		    StoreLoad;
   wire 		    SRi; // shift right immediate (srli, srai)


   // Format (I, S, B, U, J)
   assign R_cond = (opcode5 == 7'b0110011);  // R-type : opcode5 = 01100 (ALU ?뿰?궛 紐낅졊?뼱 10媛?吏?)

   assign JALR = (opcode5 == 7'b1100111);      // (jalr 1媛?吏?) => ?븯?뱶?썾?뼱?쟻 援ы쁽 : "JALR = opcode5[4] & opcode5[3] & ~opcode5[2] & ~opcode5[1] & opcode5[0];"
   assign LOAD = (opcode5 == 7'b0000011);      // (Load 紐낅졊?뼱 5媛?吏?)
   assign ALUi = (opcode5 == 7'b0010011);      // (ALUi 紐낅졊?뼱 9媛?吏?)
   assign I_cond = JALR | LOAD | ALUi;     // I-type : opcode5 = 00x00 (x=0->Load紐낅졊, x=1->ALU?뿰?궛) or jalr

   assign S_cond = (opcode5 == 7'b0100011);  // S-type : opcode5 = 01000, (Store 紐낅졊?뼱 3媛?吏?)

   assign B_cond = (opcode5 == 7'b1100011);  // B-type : opcode5 = 11000, (Branch 紐낅졊?뼱 6媛?吏?)

   assign LUI = (opcode5 == 7'b0110111);
   assign AUIPC = (opcode5 == 7'b0010111);
   assign U_cond = LUI | AUIPC;            // U-type : opcode5 = 0x101 (x=0->auipc, x=1->lui), (lui, auipc 2媛?吏?)

   assign J_cond = (opcode5 == 7'b1101111);  // J-type : opcode5 = 11011 (J-type?뿉?뒗 jal 紐낅졊?뼱 ?븯?굹諛뽰뿉 ?뾾?쓬), (jal 1媛?吏?)


   // PCsel
   assign BEQ = B_cond & (funct3 == 3'b000); // funct3 = 000 & opcode5 = 11000 -> beq [opcode5 = 11000 -> B-type]
   assign BNE = B_cond & (funct3 == 3'b001); // funct3 = 001 & opcode5 = 11000 -> bne
   assign BLT = B_cond & ({funct3[2],funct3[0]} == 2'b10); // funct3 = 1x0 & opcode5 = 11000 -> blt or bltu (x -> 0 or 1)
   assign BGE = B_cond & ({funct3[2],funct3[0]} == 2'b11); // funct3 = 1x1 & opcode5 = 11000 -> blt or bltu (x -> 0 or 1)

   assign PCsel = J_cond | JALR | (BEQ & BrEq) | (BNE & (~BrEq)) | (BLT & BrLT) | (BGE & (~BrLT)); // 遺꾧린瑜? ?븯寃? ?릺硫? PCsel = 1 -> PCNext = ALU_o (臾댁“嫄? 遺꾧린, 議곌굔 遺꾧린)
   // 遺꾧린瑜? ?븯吏? ?븡?뒗?떎硫? PCsel = 0 -> PCNext = PCp4

   // ImmSel
   assign ImmSel[0] = J_cond | B_cond | I_cond;
   assign ImmSel[1] = J_cond | U_cond | I_cond;
   assign ImmSel[2] = S_cond | I_cond;
   // I: 111, B: 001, U: 010, J: 011, S: 100


   // BrUn
   assign BrUn = B_cond & (funct3[1] == 1'b1); // B-type ?씠怨? {funct3 = x1x} ?씠硫?, BrUn = 1 -> Unsigned mode瑜? ?쓽誘?


   // ASel
   assign ASel = AUIPC | J_cond | BEQ | BNE | BLT | BGE; // 1 if A is PC


   // BSel
   assign BSel = ~R_cond; // R-type?씤 寃쎌슦留? BSel = 0, ?굹癒몄??쓽 寃쎌슦?뒗 紐⑤몢 imm媛? ?엳?쑝誘?濡? BSel = 1


   // ALUSel
   assign SRi = ALUi & (funct3 == 3'b101);
   assign ALUSel = (R_cond | SRi) ? {funct7_5, funct3} : (ALUi ? {1'b0, funct3} : 4'b0); // srai留? msb媛? 1?씠?뼱?빞 ?븿
   // R-type or SR:            {funct7_5, funct3}
   // I-type 以? ALU?뿰?궛:  {1'b0, funct3}
   // ?굹癒몄?:             {4'b0} -> ALU?뒗 Add濡? ?룞?옉


   // MemRW
   assign MemRW = S_cond; // S-type?씤 寃쎌슦留?, MemRW = 1


   // RegWEn
   assign RegNWRT = S_cond | B_cond;    // S-type, B-type => RegWrite ?븞?븿
   assign RegWEn = ~RegNWRT;            // opcode5 = x1000(S-type,B-type)瑜? ?젣?쇅?븳 紐⑤뱺 instruction(R,I,U,J)?쓽 寃쎌슦, RegWEn = 1


   // WBSel
   // Load 紐낅졊?뼱 -> WBMEM_cond = 1
   assign WBp4_cond  = J_cond | JALR;                              // 臾댁“嫄? 遺꾧린(jump) -> WB = pc+4 -> WBp4_cond = 1
   assign WBSel = (WBp4_cond)? 2'b10 : ((LOAD) ? 2'b01: 2'b00);    // WB=? => '01: ReadData -> Load', '00: ALUo', '10: PCp4 -> 臾댁“嫄? 遺꾧린(Jump)'


   // WordSizeSel : (lb, lh, lw) => (byte, half word, word) & (lbu, lhu) => unsigned mode => wordsize[2]=1
   assign StoreLoad = S_cond | LOAD; // opcode5 = 0x000, 利? store紐낅졊?뼱(S) or load紐낅졊?뼱(I)?씤 寃쎌슦
   assign WordSizeSel = (StoreLoad) ? funct3 : (LUI)? 3'b011 : 3'b010;     // ** 肄붾뱶 蹂?寃쎄??뒫 : store?씪 ?븣?뒗 ?븘?슂?뾾?쓬, load?씪 ?븣留? ?븘?슂!


endmodule
