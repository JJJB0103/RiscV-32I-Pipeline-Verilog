module pipe_MEM_WB(
    input clk, rst,
    input [8:0] PC_MEM, PCPlusF_MEM,
    input [4:0] RD_MEM,
    input [31:0] DMEM_MEM, ALU_o_MEM, Imm_MEM,
    input RegWEn_MEM, PCsel_MEM,
    input [1:0] WBSel_MEM,
    input [2:0]	      WordSizeSel_MEM,

    output reg [8:0] PC_WB,
    output reg [8:0] PCPlusF_WB,
    output reg [4:0] RD_WB,
    output reg [31:0] DMEM_WB,
    output reg [31:0] ALU_o_WB, Imm_WB,
    output reg RegWEn_WB, PCsel_WB,
    output reg [1:0] WBSel_WB,
    output reg [2:0] WordSizeSel_WB
);

always @ (posedge clk, posedge rst) begin
    if (rst) begin
        WordSizeSel_WB <= 0;
        PC_WB <= 0;
        PCPlusF_WB <= 0;
        RD_WB <= 0;
        DMEM_WB <= 0;
        ALU_o_WB <= 0;
        RegWEn_WB <= 0;
        WBSel_WB <= 0;
        Imm_WB <= 0;
        PCsel_WB <= 0;        
    end
    else begin
        PC_WB <= PC_MEM;
        PCPlusF_WB <= PCPlusF_MEM;
        RD_WB <= RD_MEM;
        DMEM_WB <= DMEM_MEM;
        ALU_o_WB <= ALU_o_MEM;
        RegWEn_WB <= RegWEn_MEM;
        WBSel_WB <= WBSel_MEM;
        Imm_WB <= Imm_MEM;
        WordSizeSel_WB <= WordSizeSel_MEM;
        PCsel_WB <= PCsel_MEM;
    end
end

endmodule // pipe_MEM_WB
