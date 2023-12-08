`timescale 1ns/1ns
//`include "ALU.v"
//`include "BranchComp.v"
//`include "COMP32b.v"
//`include "control.v"
//`include "data_mem.v"
//`include "ImmGen.v"
//`include "inst_mem.v"
//`include "register_file.v"
//`include "riscV32I.v"

module tb_riscV();

   reg clk, rst, inst_wen, enb;
   reg [31:0] inst_data;
   reg [6:0]  inst_addr;
   wire [31:0] WB_o; //?
   
   riscV32I TEST(
       WB_o, inst_data, inst_addr, clk, rst, inst_wen, enb
       );
   
   always #5 clk <= ~clk;

   initial begin
      //     $dumpfile("tb_riscV.vcd");
      //     $dumpvars(0, tb_riscV);
      
      rst = 1;
      clk = 1;
      enb = 0;
      
      inst_wen = 0;
      inst_addr = 0;
      inst_data = 0;
      #11
   rst = 0;
     #1
      $readmemh("C:/Users/82102/Desktop/2023-2/graduate/RISCVsinglecycle/RISCVsinglecycle.sim/sim_1/behav/xsim/riscv_hazard.txt.", TEST.IMEM.inst_reg);
      //Enter as an absolute path and below code is force input 
      //Your absolute path, 1. window key-> 2. Magnifier Icon -> riscv.txt path cop
//    TEST.IMEM.inst_reg[0]=32'h00f00713;
//    TEST.IMEM.inst_reg[1]=32'h01000793;
//    TEST.IMEM.inst_reg[2]=32'hfff00813;
//    TEST.IMEM.inst_reg[3]=32'h00400893;
//    TEST.IMEM.inst_reg[4]=32'hfe000913;
//    TEST.IMEM.inst_reg[5]=32'h00f0f637;
//    TEST.IMEM.inst_reg[6]=32'h0f060613;
//    TEST.IMEM.inst_reg[7]=32'h40f706b3;
//    TEST.IMEM.inst_reg[8]=32'h00f786b3;
//    TEST.IMEM.inst_reg[9]=32'h00f716b3;
//    TEST.IMEM.inst_reg[10]=32'h00f726b3;
//    TEST.IMEM.inst_reg[11]=32'h00e726b3;
//    TEST.IMEM.inst_reg[12]=32'h010736b3;
//    TEST.IMEM.inst_reg[13]=32'h010836b3;
//    TEST.IMEM.inst_reg[14]=32'h010746b3;
//    TEST.IMEM.inst_reg[15]=32'h011756b3;
//    TEST.IMEM.inst_reg[16]=32'h411956b3;
//    TEST.IMEM.inst_reg[17]=32'h0117e6b3;
//    TEST.IMEM.inst_reg[18]=32'h0117f6b3;
//    TEST.IMEM.inst_reg[19]=32'h0107f6b3;
//    TEST.IMEM.inst_reg[20]=32'h010002ef;
//    TEST.IMEM.inst_reg[21]=32'h110686b7;
//    TEST.IMEM.inst_reg[22]=32'h11068697;
//    TEST.IMEM.inst_reg[23]=32'h0080006f;
//    TEST.IMEM.inst_reg[24]=32'h00028367;
//    TEST.IMEM.inst_reg[25]=32'h01071693;
//    TEST.IMEM.inst_reg[26]=32'h01072693;
//    TEST.IMEM.inst_reg[27]=32'hfff73693;
//    TEST.IMEM.inst_reg[28]=32'hfff74693;
//    TEST.IMEM.inst_reg[29]=32'h00475693;
//    TEST.IMEM.inst_reg[30]=32'h40495693;
//    TEST.IMEM.inst_reg[31]=32'h0047e693;
//    TEST.IMEM.inst_reg[32]=32'hfff7f693;
//    TEST.IMEM.inst_reg[33]=32'h0047f693;
//    TEST.IMEM.inst_reg[34]=32'h00c88123;
//    TEST.IMEM.inst_reg[35]=32'h00c891a3;
//    TEST.IMEM.inst_reg[36]=32'h00c8a223;
//    TEST.IMEM.inst_reg[37]=32'h0048a583;
//    TEST.IMEM.inst_reg[38]=32'h00489583;
//   TEST.IMEM.inst_reg[39]=32'h00488583;
//    TEST.IMEM.inst_reg[40]=32'h0048d583;
//    TEST.IMEM.inst_reg[41]=32'h0048c583;
//    TEST.IMEM.inst_reg[42]=32'hfcf70ee3;
//    TEST.IMEM.inst_reg[43]=32'h00e70a63;
//    TEST.IMEM.inst_reg[44]=32'h00f71a63;
//    TEST.IMEM.inst_reg[45]=32'hfce718e3;
//    TEST.IMEM.inst_reg[46]=32'hfd0746e3;
//    TEST.IMEM.inst_reg[47]=32'h00e70863;
//    TEST.IMEM.inst_reg[48]=32'hff0758e3;
//    TEST.IMEM.inst_reg[49]=32'hfd0770e3;
//    TEST.IMEM.inst_reg[50]=32'hff0766e3;
//    TEST.IMEM.inst_reg[51]=32'hfae74ce3;
//    TEST.IMEM.inst_reg[52]=32'hf30878e3;  //force input

      #10
   enb = 1;
      //$readmemh("C:Users/82102/Desktop/2023-2/graduate/RISCVsinglecycle/riscv2.txt", TEST.IMEM.inst_reg);
      #1200
   //$display("%32h", {TEST.DATAMEM.MEM_Data[87], TEST.DATAMEM.MEM_Data[86], TEST.DATAMEM.MEM_Data[85], TEST.DATAMEM.MEM_Data[84]});
      $finish;
      // $display("%32h", TEST.DATAMEM.MEM_Data[88]);
      // $display("%32h", TEST.DATAMEM.MEM_Data[56]);
      // $display("%32h", TEST.DATAMEM.MEM_Data[57]);
   end

endmodule
