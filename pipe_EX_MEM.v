module pipe_EX_MEM (
		    input	      clk, rst,
		    input [8:0]	      PC_EX, PCPlusF_EX,
		    input [31:0]      WD_cut_EX, Imm_EX,
		    input [4:0]	      Rd_EX,
		    input [31:0]      ALU_o_EX,
		    input	      MemRW_EX, RegWEn_EX, PCsel_EX,
		    input [1:0]	      WBSel_EX,
		    input [2:0]	      WordSizeSel_EX,

		    output reg [8:0]  PC_MEM,
		    output reg [8:0]  PCPlusF_MEM,
		    output reg [31:0] WD_cut_MEM, Imm_MEM,
		    output reg [4:0]  Rd_MEM,
		    output reg [31:0] ALU_o_MEM,
		    output reg	      MemRW_MEM,
		    output reg	      RegWEn_MEM, PCsel_MEM,
		    output reg [1:0]  WBSel_MEM,
		    output reg [2:0]  WordSizeSel_MEM
		    );

   always @ (posedge clk, posedge rst) begin
      if (rst) begin
         PC_MEM <= 0;
         PCPlusF_MEM <= 0;
         WD_cut_MEM <= 0;
         Rd_MEM <= 0;
         ALU_o_MEM <= 0;
         MemRW_MEM <= 0;
         RegWEn_MEM <= 0;
         WBSel_MEM <= 0;
         WordSizeSel_MEM <= 0;
         Imm_MEM <= 0;
         PCsel_MEM <= 0;
      end
      else begin
         PC_MEM <= PC_EX;
         PCPlusF_MEM <= PCPlusF_EX;
         WD_cut_MEM <= WD_cut_EX;
         Rd_MEM <= Rd_EX;
         ALU_o_MEM <= ALU_o_EX;
         MemRW_MEM <= MemRW_EX;
         RegWEn_MEM <= RegWEn_EX;
         WBSel_MEM <= WBSel_EX;
         WordSizeSel_MEM <= WordSizeSel_EX;
         Imm_MEM <= Imm_EX;    
         PCsel_MEM <= PCsel_EX;              
      end
   end

endmodule // pipe_EX_MEM
