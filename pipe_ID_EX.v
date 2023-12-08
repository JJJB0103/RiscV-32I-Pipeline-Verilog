module pipe_ID_EX(
		  input		    clk, rst, clear,
		  input [31:0]	    Imm_ID, //Imm에서 나오는 부분
		  input [8:0]	    PC_ID, PCPlusF_ID, //PC 관련된 부분
		  input [31:0]	    RD1_ID, RD2_ID, //Register 관련된 부분
		  input		    PCsel_ID, RegWEn_ID, 
		  input		    Asel_ID, Bsel_ID, MemRW_ID,
		  input [1:0]       WBSel_ID,
		  input [2:0]	    WordSizeSel_ID,
		  input [3:0]	    ALUSel_ID,
		  input [4:0]       Rd_ID, Rs1_ID, Rs2_ID,

		  output reg [31:0] Imm_EX,
		  output reg [8:0]  PC_EX, PCPlusF_EX,
		  output reg [31:0] RD1_EX, RD2_EX,
		  output reg	    PCsel_EX, RegWEn_EX,
		  output reg	    Asel_EX, Bsel_EX, MemRW_EX,
		  output reg [1:0]  WBSel_EX,
		  output reg [2:0]  WordSizeSel_EX,
		  output reg [3:0]  ALUSel_EX,
		  output reg [4:0]  Rd_EX, Rs1_EX, Rs2_EX
		  );
		  
   always @ (posedge clk, posedge rst) begin
      if (rst) begin
	 Imm_EX <= 0;
	 PC_EX <= 0;
	 PCPlusF_EX <= 0;
	 RD1_EX <= 0;
	 RD2_EX <= 0;
	 PCsel_EX <= 0;
	 RegWEn_EX <= 0;

	 Asel_EX <= 0;
	 Bsel_EX <= 0;
	 MemRW_EX <= 0;
	 WordSizeSel_EX <= 0;
	 ALUSel_EX <= 0;
	 Rd_EX <= 0;
	 Rs1_EX <= 0;
	 Rs2_EX <= 0;
	 WBSel_EX <=0;
      end // if (rst)
      
      else if (clear) begin
	 Imm_EX <= 0;
	 PC_EX <= 0;
	 PCPlusF_EX <= 0;
	 RD1_EX <= 0;
	 RD2_EX <= 0;
	 PCsel_EX <= 0;
	 RegWEn_EX <= 0;
	 Asel_EX <= 0;
	 Bsel_EX <= 0;
	 MemRW_EX <= 0;
	 WordSizeSel_EX <= 0;
	 ALUSel_EX <= 0;
	 Rd_EX <= 0;
	 Rs1_EX <= 0;
	 Rs2_EX <= 0;
	 WBSel_EX <=0;
      end // if (clear)
      
      else begin
	 Imm_EX <= Imm_ID;
	 PC_EX <= PC_ID;
	 PCPlusF_EX <= PCPlusF_ID;
	 RD1_EX <= RD1_ID;
	 RD2_EX <= RD2_ID;
	 PCsel_EX <= PCsel_ID;
	 RegWEn_EX <= RegWEn_ID;
	 Asel_EX <= Asel_ID;
	 Bsel_EX <= Bsel_ID;
	 MemRW_EX <= MemRW_ID;
	 WordSizeSel_EX <= WordSizeSel_ID;
	 ALUSel_EX <= ALUSel_ID;
	 Rd_EX <= Rd_ID;
	 Rs1_EX <= Rs1_ID;
	 Rs2_EX <= Rs2_ID;	 
	 WBSel_EX <= WBSel_ID;
      end // else: !if(clear)
   end // always @ (posedge clk)
endmodule // pipe_ID_EX
