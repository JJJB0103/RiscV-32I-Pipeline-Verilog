module pipe_IF_ID(
		  output reg [31:0] inst_D, 
		  output reg [8:0]  PC_D, PCPlusF_D,
		  input [31:0]	    inst_F, 
		  input [8:0]	    PC_F, PCPlusF_F,
		  input		    clk, rst, clear, enable);

   always @ (posedge clk, posedge rst) begin
      if (rst) begin
	 inst_D <= 0;
	 PC_D <= 0;
	 PCPlusF_D <= 0;
      end

      else if (enable) begin
                 if (clear) begin
                    inst_D <= 0;
                    PC_D <= 0;
                    PCPlusF_D <= 0;
                 end
            
                 else begin
                    inst_D <= inst_F;
                    PC_D <= PC_F;
                    PCPlusF_D <= PCPlusF_F;
                 end // else: !if(clear)
          end // if (enable)
   end // always @ (posedge clk)
endmodule // pipe_IF_ID
