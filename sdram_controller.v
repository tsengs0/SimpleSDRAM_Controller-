module sdram_controller (
 // For connecting with Avalon Bus
 input         sys_clk, rstn,
 input [21:0]  avl_addr, // {BA[1:0], ROW[11:0], COL[7:0]}
 input [1:0]   avl_byte_en,
 input         avl_WRITEen, avl_READen,
 input  [15:0] avl_WRDATA,
 output [15:0] avl_RDDATA,
 output        avl_req_wait,

 // SDRAM
 output reg        CSn, RASn, CASn, WEn,
 output reg [1:0]  BA,
 output reg [11:0] addr,
 inout      [15:0] DQ,
 output     [1:0]  DQM // for setting byte selection (mask)
);
 
// Declaration of FSM for READ/WRITE transactions
parameter IWAIT=5'd0, IPALL=5'd1, IDELAY1=5'd2, IREF=5'd3, IDELAY2=5'd4, IDELAY3=5'd5, IMODE=5'd6;
parameter RACT=5'd7, RDELAY1=5'd8, RDA=5'd9, RDELAY2=5'd10, RDELAY3=5'd11, HALT=5'd12;
parameter WACT=5'd13, WDELAY1=5'd14, WRA=5'd15, WDELAY2=5'd16;
parameter FREF=5'd17, FDELAY=5'd18;

reg [4:0] cur, next;

// The counter for initialising system 200 us
parameter MAX200=14'd10_000;
reg [13:0] i200cnt;
wire i200cntup;
assign i200cntup = (i200cnt == MAX200 - 1) ? 1'b1 : 1'b0;

always @(posedge sys_clk, negedge rstn) begin
	if(!rstn)
		i200cnt[13:0] <= 14'd0;
	else
		i200cnt[13:0] <= i200cnt[13:0] + 1'b1;
end

// The 8-counter for initially refresh SDRAM eight times
reg [2:0] init_ref_cnt;
wire init_RefMax;
assign init_RefMax = (init_ref_cnt == 3'b111) ? 1'b1 : 1'b0;

always @(posedge sys_clk, negedge rstn) begin
	if(!rstn || cur[4:0] == IWAIT[4:0])
		init_ref_cnt[2:0] <= 3'b000;
	else if(cur[4:0] == IDELAY3[4:0])
		init_ref_cnt[2:0] <= init_ref_cnt[2:0] + 1'b1;
	else // do nothing
		init_ref_cnt[2:0] <= init_ref_cnt[2:0];
end

// Refresh counter
// 64ms / 8192 refresh-count / 20ns = 390 counts
// Because the sys_clk = 50 MHz, so 20ns for each clock cycle
// Therefore, every 390 clock cycles the SDRAM have to be refreshed, i.e., tREFI=7.8125us
reg [8:0] ref_cnt;
parameter RefMax=9'd390;

always @(posedge sys_clk, negedge rstn) begin
	if(!rstn || cur[4:0] == FREF[4:0])
		ref_cnt[8:0] <= 9'd0;
	else 
		ref_cnt[8:0] <= ref_cnt[8:0] + 1'b1;
end 

// FSM for READ/WRITE transactions
always @(posedge sys_clk, negedge rstn) begin
	if(!rstn) 
		cur[4:0] <= IWAIT[4:0];
	else
		cur[4:0] <= next[4:0];
end

always @(*) begin
	case (cur[4:0])
		IWAIT  : 
			if(i200cntup == 1'b1) 
				next[4:0] <= IPALL[4:0];
			else
				next[4:0] <= IWAIT[4:0];
		IPALL  : next[4:0] <= IDELAY1[4:0];
		IDELAY1: next[4:0] <= IREF[4:0];
		IREF   : next[4:0] <= IDELAY2[4:0];
		IDELAY3: 
			if(init_RefMax == 1'b1) // the initial refresh opertion will be run eight times
				next[4:0] <= IMODE[4:0];
			else
				next[4:0] <= IDELAY1[4:0];
		IMODE  : next[4:0] <= HALT[4:0];
		HALT   : 
			if(ref_cnt[8:0] >= RefMax[8:0])
				next[4:0] <= FREF[4:0];
			else if(avl_WRITEen && !avl_READen)
				next[4:0] <= WACT[4:0];
			else if(avl_READen && !avl_WRITEen)
				next[4:0] <= RACT[4:0];
			else 
				next[4:0] <= HALT[4:0];
		// Write operation
		WACT   : next[4:0] <= WDELAY1[4:0];
		WDELAY1: next[4:0] <= WRA[4:0];
		WRA    : next[4:0] <= WDELAY2[4:0];
		WDELAY2: next[4:0] <= HALT[4:0];

		// Read operation
		RACT   : next[4:0] <= RDELAY1[4:0];
		RDELAY1: next[4:0] <= RDA[4:0];
		RDA    : next[4:0] <= RDELAY2[4:0];
		RDELAY2: next[4:0] <= RDELAY3[4:0];
		RDELAY3: next[4:0] <= HALT[4:0];

		// Refresh operation
		FREF   : next[4:0] <= FDELAY[4:0];
		FDELAY : next[4:0] <= HALT[4:0];
		default: next[4:0] <= HALT[4:0];   
	endcase
end

// SDRAM control signals
always @(*) begin
	if(cur[4:0] == IMODE[4:0])
		{CSn, RASn, CASn, WEn} <= 4'b0000; // MRS
	else if(cur[4:0] == RACT[4:0] || cur[4:0] == WACT[4:0])
		{CSn, RASn, CASn, WEn} <= 4'b0011; // ACT
	else if(cur[4:0] == IPALL[4:0])
		{CSn, RASn, CASn, WEn} <= 4'b0010; // Precharge all
	else if(cur[4:0] == RDA[4:0])
		{CSn, RASn, CASn, WEn} <= 4'b0101; // READ with auto-precharge
	else if(cur[4:0] == WRA[4:0])
		{CSn, RASn, CASn, WEn} <= 4'b0100; // WRITE with auto-precharge
	else if(cur[4:0] == IREF[4:0] || cur[4:0] == FREF[4:0])
		{CSn, RASn, CASn, WEn} <= 4'b0001; // Refresh operatoin
	else
		{CSn, RASn, CASn, WEn} <= 4'b1111;
end

// Addressing signals
always @(*) begin
	if(cur[4:0] == IMODE[4:0])
		addr[11:0] <= 12'h020; // MRS
	else if(cur[4:0] == RACT[4:0] || cur[4:0] == WACT[4:0])
		addr[11:0] <= avl_addr[19:8]; // Row Address
	else if(cur[4:0] == IPALL[4:0])
		addr[11:0] <= 12'b0100_0000_0000; // Precharge all
	else if(cur[4:0] == RDA[4:0] || cur[4:0] == WRA[4:0])
		addr[11:0] <= {4'b0100, avl_addr[7:0]}; // Column Address
	else
		addr[11:0] <= 12'h000; 	
end

// Banking address signals
always @(*) begin
	if(cur[4:0] == IMODE[4:0]) 
		BA[1:0] <= 2'b00; // MRS
	else if(cur[4:0] == RACT[4:0] || cur[4:0] == WACT[4:0])
		BA[1:0] <= avl_addr[21:20]; // Bank address
	else if(cur[4:0] == RDA[4:0] || cur[4:0] == WRA[4:0])
		BA[1:0] <= avl_addr[21:20]; // Bank address
	else
		BA[1:0] <= 2'b00;
end

// DQ, DQM, Avalon bus signals
assign DQ[15:0] = (cur[4:0] == WRA[4:0]) ? avl_WRDATA[15:0] : 16'hzzzz;
assign DQM[1:0] = !avl_byte_en[1:0]; 
assign avl_RDDATA[15:0] = DQ[15:0];
assign avl_req_wait = (cur[4:0] == RDELAY3[4:0] || cur[4:0] == WDELAY2[4:0]) ? 1'b0 : 1'b1;

endmodule
