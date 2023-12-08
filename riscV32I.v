module riscV32I(
		output [31:0] WB_o,
		input [31:0]  inst_data, 
		input [6:0]   inst_addr, 
		input 	      clk, rst, inst_wen, enb
	        );

   reg [8:0] 		      PCF;
   wire [31:0]             instF;
   wire [8:0]             PCD,PCE,PCM,PCW; //각각 IF,ID,EX일때 PC값
   wire [31:0] 		      ImmD, ImmE, ImmM, ImmW, instD, WB, DataA_D, DataA_E, DataA_E_hazardout, DataB_D, DataB_E, DataB_E_hazardout, WB_cut, WD_cut_E,WD_cut_M;
   wire [4:0]             RdD,RdE,RdM,RdW,Rs1E,Rs2E;
   wire [31:0] 		      DMEM_M,DMEM_W, WB_Half, WB_Byte, WD_Half, WD_Byte, DataA_D_out, DataB_D_out;
   wire 		      PCsel_D, PCsel_E, PCsel_M, PCsel_W, RegWEn_D, RegWEn_E, RegWEn_M, RegWEn_W;
   wire                 BrUn, ASel_D, ASel_E, BSel_D, BSel_E, MemRW_D, MemRW_E, MemRW_M, BrEq, BrLT,ForwardAD,ForwardBD;
   wire [1:0] 		      WBSel_D, WBSel_E, WBSel_M, WBSel_W, ForwardAE, ForwardBE;
   wire [2:0] 		      ImmSel, WordSizeSel_D, WordSizeSel_E, WordSizeSel_M, WordSizeSel_W;
   wire [3:0] 		      ALUSel_D, ALUSel_E;
   wire                   StallD, StallF, FlushD, FlushE,Bubble;

   wire [8:0] 		      PC_Next;
   wire [8:0] 		      PCp4_F = PCF + 7'd4;
   wire [8:0]		      PCp4_D = PCD + 7'd4;
   wire [8:0]		      PCp4_E = PCE + 7'd4;
   wire [8:0]		      PCp4_M = PCM + 7'd4;
   wire [8:0]		      PCp4_W = PCW + 7'd4;

   wire [31:0] 		      ALU_o_E,ALU_o_M,ALU_o_W, ALU_A, ALU_B;

   assign PC_Next = PCsel_E ? ALU_o_E : PCp4_F; //mux2to1대신 assign문 사용 여기서 pcp4_E인지 pcpp4_F인지 잘 모르겠음 ㅠ
   //assign ALU_jump = jal_jalr ? PCp4_W : ALU_o_E;
   //assign PC_Next_Real = Bubble ? PC_Next - 8'd4 : PC_Next; //JB : 이 부분이 jal jalr 고친부분
   assign ALU_A = ASel_E ? {23'b0,PCE} : DataA_E_hazardout;//여기서 Data_E 수정해야함 포워딩
   assign ALU_B = BSel_E ? ImmE : DataB_E_hazardout;
   assign RdD = instD[11:7];
   
   wire PCsel_F, J_cond, JALR;
   
   assign JALR = (instF[6:2] == 5'b11001);
   assign J_cond = (instF[6:2] == 5'b11011);
   assign PCsel_F = J_cond | JALR;
//assign PC_F = Bubble ? instF_out : PCF;
   
   mux4to1 mux_WB(
		  // Outputs
		  .R			(WB),
		  // Inputs
		  .sel			(WBSel_W),
		  .a			(ALU_o_W),         //00
		  .b			(DMEM_W),          //01
		  .c			({23'b0,PCp4_W}),  //10
		  .d			(32'b0)
		  );
   mux4to1 mux_WB_cut(
		      // Outputs
		      .R		(WB_cut),
		      // Inputs
		      .sel		(WordSizeSel_W[1:0]),
		      .a		(WB_Byte),
		      .b		(WB_Half),
		      .c		(WB),
		      .d		(ImmW) 
		      );

   // assign WB = (WBSel == 2'd2) ? PCp4 : ((WBSel == 2'd1) ? ALU_o : DMEM);
   assign WB_Half = WordSizeSel_W[2] ? {16'b0, WB[15:0]} : {{16{WB[15]}}, WB[15:0]};
   assign WB_Byte = WordSizeSel_W[2] ? {24'b0, WB[7:0]}  : {{24{WB[7]}},  WB[7:0]};
   // assign WB_cut = (WordSizeSel[1:0] == 2'b0) ? WB_Byte : ((WordSizeSel[1:0] == 2'b1) ? WB_Half : WB);

   mux4to1 mux_WD_cut(
		      // Outputs
		      .R		(WD_cut_E),
		      // Inputs
		      .sel		(WordSizeSel_E[1:0]),
		      .a		(WD_Byte),
		      .b		(WD_Half),
		      .c		(DataB_E),
		      .d		(32'b0)
		      );

   assign WD_Half = {16'b0, DataB_E[15:0]};
   assign WD_Byte = {24'b0, DataB_E[7:0]};

   inst_mem IMEM(.inst(instF),
		 // Inputs
		 .PC(PCF[8 -: 7]),
		 .inst_data		(inst_data[31:0]),
		 .inst_addr		(inst_addr[6:0]),
		 .clk			(clk),
		 .rst			(rst),
		 .inst_wen		(inst_wen));

   pipe_IF_ID IF_ID(.inst_D(instD), .PC_D(PCD), .PCPlusF_D(PCp4_D),
		    // Inputs
		    .inst_F(instF) ,.PC_F(PCF), .PCPlusF_F(PCp4_F),
		    .clk(clk), .rst(rst), .clear(FlushD), .enable(~StallD));

   register_file REGFILE(
			 .RD1(DataA_D), .RD2(DataB_D),   
			 .RR1(instD[19:15]), .RR2(instD[24:20]), .WR(RdW),
			 .WD(WB_cut),         
			 .RegWrite(RegWEn_W),
			 // Inputs
			 .clk			(clk),
			 .rst			(rst));
   pipe_ID_EX ID_EX(
		    // Outputs
		    .Imm_EX		(ImmE[31:0]),
		    .PC_EX		(PCE[8:0]),
		    .PCPlusF_EX		(PCp4_E[8:0]),
		    .RD1_EX		(DataA_E[31:0]),
		    .RD2_EX		(DataB_E[31:0]),
		    .PCsel_EX		(PCsel_E),
		    .RegWEn_EX		(RegWEn_E),
		    .Asel_EX		(ASel_E),
		    .Bsel_EX		(BSel_E),
		    .MemRW_EX		(MemRW_E),
		    .WordSizeSel_EX	(WordSizeSel_E[2:0]),
		    .ALUSel_EX		(ALUSel_E[3:0]),
		    .Rd_EX (RdE),
		    .Rs1_EX(Rs1E),
		    .Rs2_EX(Rs2E),
		    .WBSel_EX(WBSel_E),
		    // Inputs
		    .clk		(clk),
		    .rst		(rst),
		    .clear		(FlushE),
		    .Imm_ID		(ImmD[31:0]),
		    .PC_ID		(PCD[8:0]),
		    .PCPlusF_ID		(PCp4_D[8:0]),
		    .RD1_ID		(DataA_D_out[31:0]),
		    .RD2_ID		(DataB_D_out[31:0]),
		    .PCsel_ID		(PCsel_D),
		    .RegWEn_ID		(RegWEn_D),
		    .Asel_ID		(ASel_D),
		    .Bsel_ID		(BSel_D),
		    .MemRW_ID		(MemRW_D),
		    .WordSizeSel_ID	(WordSizeSel_D[2:0]),
		    .ALUSel_ID		(ALUSel_D[3:0]),
		    .Rd_ID (RdD),
		    .Rs1_ID(instD[19:15]),
		    .Rs2_ID(instD[24:20]),		    
		    .WBSel_ID(WBSel_D));
		    
      hazardunit hazard(
		     // Outputs
		     .ForwardAE		(ForwardAE[1:0]),
		     .ForwardBE		(ForwardBE[1:0]),
		     .ForwardAD		(ForwardAD),
		     .ForwardBD		(ForwardBD),
		     .StallD		(StallD), //아직 정의 X
		     .StallF		(StallF), //아직 정의 X
		     .FlushD		(FlushD), //아직 정의 X
		     .FlushE		(FlushE),
		     .Bubble     (Bubble), //아직 정의 X
		     // Inputs
		     .Rs1D		(instD[19:15]),
		     .Rs2D		(instD[24:20]),
		     .Rs1E		(Rs1E[4:0]),
		     .Rs2E		(Rs2E[4:0]),
		     .RdE		(RdE[4:0]),
		     .RdM		(RdM[4:0]),
		     .RdW		(RdW[4:0]),
		     .RegWriteM		(RegWEn_M), 
		     .RegWriteW		(RegWEn_W), 
		     .WBSelE0	(WBSel_E[0]),
		     .WBSelM0	(WBSel_M[0]),
		     .WBSelW0	(WBSel_W[0]),
		     .PCSrcF		(PCsel_F),
		     .PCSrcE		(PCsel_E),
		     .PCSrcM		(PCsel_M),
		     .instF_in         (instF),
		     .clk           (clk));		    
		     
   mux4to1 ForwardmuxA(                 
		      // Outputs
		      .R		(DataA_E_hazardout),
		      // Inputs
		      .sel		(ForwardAE),
		      .a		(DataA_E),
		      .b		(WB),
		      .c		(ALU_o_M[31:0]),
		      .d		(32'b0)
		      );		    
		    
   mux4to1 ForwardmuxB(
		      // Outputs
		      .R		(DataB_E_hazardout),
		      // Inputs
		      .sel		(ForwardBE),
		      .a		(DataB_E),
		      .b		(WB),
		      .c		(ALU_o_M[31:0]),
		      .d		(32'b0)
		      );				    
		      
   assign DataA_D_out = ForwardAD ? WB : DataA_D;
   assign DataB_D_out = ForwardBD ? WB : DataB_D;
   
   assign PC_Next = PCsel_E ? ALU_o_E : PCp4_F; 
   control CTRL(
		// Outputs
		.PCsel			(PCsel_D),
		.RegWEn			(RegWEn_D),
		.BrUn			(BrUn),
		.ImmSel			(ImmSel[2:0]),
		.WordSizeSel		(WordSizeSel_D[2:0]),
		.BSel			(BSel_D),
		.ASel			(ASel_D),
		.MemRW			(MemRW_D),
		.ALUSel			(ALUSel_D[3:0]),
		.WBSel			(WBSel_D[1:0]),
		// Inputs
		.instruction		(instD[31:0]),
		.BrEq			(BrEq),   //JB이게 왜 control로 들어가지?
		.BrLT			(BrLT));

   ImmGen IMMGEN(
		 .inst_Imm(instD[31:7]),
		 // Outputs
		 .Imm			(ImmD[31:0]),
		 // Inputs
		 .ImmSel		(ImmSel[2:0]));
		 
   BranchComp BrCOMP(
		     .RD1(DataA_D_out), .RD2(DataB_D_out),
		     // Outputs
		     .BrEq		(BrEq), 
		     .BrLT		(BrLT),
		     // Inputs
		     .BrUn		(BrUn));
		     
   ALU ALU_riscV(
		 .A(ALU_A), .B(ALU_B),
		 // Outputs
		 .ALU_o			(ALU_o_E[31:0]),
		 // Inputs
		 .ALUSel		(ALUSel_E[3:0]));
		 		 
   pipe_EX_MEM EX_MEM(
		      // Outputs
		      .PC_MEM		(PCM[8:0]),
		      .PCPlusF_MEM	(PCp4_M[8:0]),
		      .WD_cut_MEM	(WD_cut_M[31:0]),
		      .Rd_MEM		(RdM[4:0]),
		      .ALU_o_MEM	(ALU_o_M[31:0]),
		      .MemRW_MEM	(MemRW_M),
		      .RegWEn_MEM	(RegWEn_M),
		      .WBSel_MEM	(WBSel_M[1:0]),
		      .WordSizeSel_MEM	(WordSizeSel_M[2:0]),
		      .Imm_MEM(ImmM),
		      .PCsel_MEM(PCsel_M),
		      // Inputs
		      .clk		(clk),
		      .rst		(rst),
		      .PC_EX		(PCE[8:0]),
		      .PCPlusF_EX	(PCp4_E[8:0]),
		      .WD_cut_EX	(WD_cut_E[31:0]),
		      .Rd_EX		(RdE[4:0]),
		      .ALU_o_EX		(ALU_o_E[31:0]),
		      .MemRW_EX		(MemRW_E),
		      .RegWEn_EX	(RegWEn_E),
		      .WBSel_EX		(WBSel_E[1:0]),
		      .WordSizeSel_EX	(WordSizeSel_E[2:0]),
		      .Imm_EX(ImmE),
		      .PCsel_EX(PCsel_E));

   data_mem DATAMEM(
		    .ReadData(DMEM_M),
		    .ADDR(ALU_o_M), .WriteData(WD_cut_M),
		    .MemWrite(MemRW_M),
		    // Inputs
		    .clk		(clk),
		    .rst		(rst));
		    
   pipe_MEM_WB MEM_WB(
		      // Outputs
		      .PC_WB		(PCW[8:0]),
		      .PCPlusF_WB	(PCp4_W[8:0]),
		      .RD_WB		(RdW[4:0]),
		      .DMEM_WB		(DMEM_W[31:0]),
		      .ALU_o_WB		(ALU_o_W[31:0]),
		      .RegWEn_WB	(RegWEn_W),
		      .WBSel_WB		(WBSel_W[1:0]),
		      .WordSizeSel_WB	(WordSizeSel_W[2:0]),
		      .Imm_WB       (ImmW),
		      .PCsel_WB(PCsel_W),
		      // Inputs
		      .clk		(clk),
		      .rst		(rst),
		      .PC_MEM		(PCM[8:0]),
		      .PCPlusF_MEM	(PCp4_M[8:0]),
		      .RD_MEM		(RdM[4:0]),
		      .DMEM_MEM		(DMEM_M[31:0]),
		      .ALU_o_MEM	(ALU_o_M[31:0]),
		      .RegWEn_MEM	(RegWEn_M),
		      .WBSel_MEM	(WBSel_M[1:0]),
		      .WordSizeSel_MEM	(WordSizeSel_M[2:0]),
		      .Imm_MEM      (ImmM),
		      .PCsel_MEM(PCsel_M));		    



   always @ (posedge clk) begin
      if (rst) begin
         PCF <= 9'b0;
      end
      else if (~StallF) begin
         PCF <= PC_Next;
      end
   end

endmodule // riscV32I


module mux4to1(
	       // Outputs
	       R,
	       // Inputs
	       sel, a, b, c, d
	       );
   input [1:0] sel;
   input [31:0] a,b,c,d; // 00, 01, 10, 11
   output [31:0] R;

   wire [31:0] 	 ab, cd;

   assign ab = sel[0]? b : a;
   assign cd = sel[0]? d : c;

   assign R = sel[1]? cd : ab;
endmodule // mux4to1
