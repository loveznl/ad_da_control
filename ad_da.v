`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/07/23 14:13:18
// Design Name: 
// Module Name: ad_da
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ad_da(
    input clk,
    input rst_n,
    //spi
    input cs,
    input sclk,
    input mosi,
    output reg miso,
    //da
    output da1_clk_p,
    output da1_clk_n,
    output reg [13:0]da1_dout,
    output da2_clk_p,
    output da2_clk_n,
    output reg [13:0]da2_dout,

    //ad
	input	[11:0]	Adc_In_A,	// 通道A输入数据
	output			Adc_Clk_A,	// 通道A时钟
	input	[11:0]	Adc_In_B,	// 通道B输入数据
	output			Adc_Clk_B	// 通道B时钟

    );
//===定义读写寄存器da_cmd======================================================== 
parameter  Write_da1 = 16'h4000;
parameter  Write_da2 = 16'h8000;
parameter  enb_da1   = 16'h8001;
parameter  enb_da2   = 16'h8002;
parameter  enb_da    = 16'h8003;
//===定义读写寄存器ad_cmd======================================================== 
parameter  write_ad1_start  = 16'h4005;
parameter  Write_ad1_stop   = 16'h4001;
parameter  read_ad1_enb     = 16'h4002;
parameter  read_ad1_start   = 16'h4003;
parameter  write_ad2_start  = 16'h4008;
parameter  Write_ad2_stop   = 16'h4009;
parameter  read_ad2_enb     = 16'h400a;
parameter  read_ad2_start   = 16'h400b;
parameter  both_stop        = 16'h400c;
parameter  both_start       = 16'h400d;
parameter  ad_check         = 16'h4006;
parameter  ad_check_finish  = 16'h4007;

reg  [15:0]    cmd_reg;         //命令
reg  [3:0]     data_cnt;
reg  [1:0]     state;
reg  [15:0]    mosi_data ;      //接收数据reg

reg  [11:0]    addr_da1;        //
reg  [11:0]    addr_da2;        //
reg  [31:0]    FWord_da1 ;
reg  [31:0]    FWord_da2 ;
reg  [31:0]    Freq_ACC_da1;
reg  [31:0]    Freq_ACC_da2;
reg  [9:0]     Rom_Addr_da1;
reg  [9:0]     Rom_Addr_da2;
//
reg  [11:0]     Adc_Data_CHA;		
reg  [11:0]     Adc_Data_CHB;
reg  [10:0]      addr_ram1_in;
reg  [10:0]      addr_ram2_in;
reg  [10:0]     read_ad1_cnt;
reg  [10:0]     read_ad2_cnt;
wire [15:0]     r_ad1_data;
wire [15:0]     r_ad2_data;


//da时钟
wire clk200m_da1;
wire clk200m_da2;    
assign da1_clk_p = clk200m_da1;
assign da1_clk_n = ~clk200m_da1;
assign da2_clk_p = clk200m_da2;
assign da2_clk_n = ~clk200m_da2;
//ad时钟
wire Adc_ReadClk_65M;			// 65MHz时钟
wire Adc_Clk_65M;				// 65MHz的ADC时钟	
assign Adc_Clk_A = Adc_Clk_65M;
assign Adc_Clk_B = Adc_Clk_65M;


//------------- clkout ---------------------------
clkout u_clkout
   (
    .clk_out1(clk200m_da1),     
    .clk_out2(clk200m_da2),    
    .clk_out3(Adc_Clk_65M),     
    .clk_out4(Adc_ReadClk_65M),//滞后Adc_Clk_65M 5ns   
    .resetn(rst_n), 
    .locked(),       
    .clk_in1(clk)
    );      



//sclk边沿检测
reg pose_sclk0,pose_sclk1;
wire pedge_sclk_flag;
always@(posedge clk or negedge rst_n) begin
if(!rst_n) begin
    pose_sclk0 <= 0;
    pose_sclk1 <= 0;
end
else begin
    pose_sclk0 <= sclk;
    pose_sclk1 <= pose_sclk0;
end
end
assign pedge_sclk_flag = ((pose_sclk0) && (!pose_sclk1)) ; 

//cs边沿检测
reg edge_cs0,edge_cs1;
wire nedge_cs_flag,pedge_cs_flag;
always@(posedge clk or negedge rst_n) begin
if(!rst_n) begin
    edge_cs0 <= 0;
    edge_cs1 <= 0;
end
else begin
    edge_cs0 <= cs;
    edge_cs1 <= edge_cs0;
end
end
assign nedge_cs_flag = ((!edge_cs0) && (edge_cs1));
assign pedge_cs_flag = ((edge_cs0) && (!edge_cs1));

//cs启动
reg cs_state;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        cs_state <= 0;
    else if(nedge_cs_flag)
        cs_state <= 1;
    else if (pedge_cs_flag) 
        cs_state <= 0;
    else
        cs_state <= cs_state;
end

////////////////////////////  ad read  /////////////////////////////////////////////////////
always @(posedge Adc_ReadClk_65M or negedge rst_n)
if (!rst_n) begin
    addr_ram1_in <= 0;
    Adc_Data_CHA <= 0;
end
else begin
    addr_ram1_in <= addr_ram1_in + 1;
	Adc_Data_CHA <= Adc_In_A^12'hFFF; 
end
always @(posedge Adc_ReadClk_65M or negedge rst_n)
if (!rst_n) begin
    addr_ram2_in <= 0;
    Adc_Data_CHB <= 0;
end
else begin
    addr_ram2_in <= addr_ram2_in + 1;
	Adc_Data_CHB <= Adc_In_B^12'hFFF; 
end 

/*  da_data
0-1023 -> 波形数据，
1024,1025,1026 ->频率信息
1027 ->完成信号
*/

//da
reg ready_da1;
reg ready_da2;
reg receive_da1;
reg receive_da2;

//ad
reg w_ad1_state;
reg r_ad1_state;
reg ad1_enb_state;
reg w_ad2_state;
reg r_ad2_state;
reg ad2_enb_state;
//校验
reg [15:0]check;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state           <= 0;
        cmd_reg         <= 0;
        mosi_data       <= 0;
        data_cnt        <= 0;

        ready_da1       <= 0;
        ready_da2       <= 0;
        addr_da1        <= 0;
        addr_da2        <= 0;
        receive_da1     <= 0;
        receive_da2     <= 0;

        w_ad1_state     <= 1;
        r_ad1_state     <= 0;
        ad1_enb_state   <= 0;
        read_ad1_cnt    <= 0;
        w_ad2_state     <= 1;
        r_ad2_state     <= 0;
        ad2_enb_state   <= 0;
        read_ad2_cnt    <= 0;
        check           <= 0;
    end
    else if((cs_state) && (pedge_sclk_flag))begin
        case (state)
            0: begin
                case(data_cnt)
                    0:begin  cmd_reg[15] <= mosi;  miso <= check[15];  data_cnt <= data_cnt + 1; end     
                    1:begin  cmd_reg[14] <= mosi;  miso <= check[14];  data_cnt <= data_cnt + 1; end
                    2:begin  cmd_reg[13] <= mosi;  miso <= check[13];  data_cnt <= data_cnt + 1; end 
                    3:begin  cmd_reg[12] <= mosi;  miso <= check[12];  data_cnt <= data_cnt + 1; end
                    4:begin  cmd_reg[11] <= mosi;  miso <= check[11];  data_cnt <= data_cnt + 1; end
                    5:begin  cmd_reg[10] <= mosi;  miso <= check[10];  data_cnt <= data_cnt + 1; end
                    6:begin  cmd_reg[9] <= mosi;   miso <= check[9];   data_cnt <= data_cnt + 1; end
                    7:begin  cmd_reg[8] <= mosi;   miso <= check[8];   data_cnt <= data_cnt + 1; end     
                    8:begin  cmd_reg[7] <= mosi;   miso <= check[7];   data_cnt <= data_cnt + 1; end
                    9:begin  cmd_reg[6] <= mosi;   miso <= check[6];   data_cnt <= data_cnt + 1; end
                    10:begin cmd_reg[5] <= mosi;   miso <= check[5];   data_cnt <= data_cnt + 1; end
                    11:begin cmd_reg[4] <= mosi;   miso <= check[4];   data_cnt <= data_cnt + 1; end
                    12:begin cmd_reg[3] <= mosi;   miso <= check[3];   data_cnt <= data_cnt + 1; end
                    13:begin cmd_reg[2] <= mosi;   miso <= check[2];   data_cnt <= data_cnt + 1; end
                    14:begin cmd_reg[1] <= mosi;   miso <= check[1];   data_cnt <= data_cnt + 1; end
                    15:begin cmd_reg[0] <= mosi;   miso <= check[0];   data_cnt <= data_cnt + 1;  
                             state <= 1;
                    end   
                endcase 
            end      
            1: begin
                case (cmd_reg)
                ////////////////////////////////  da set  ////////////////////////////////////////////////////
                    Write_da1: begin
                        case(data_cnt)
                            0:begin  mosi_data[15] <= mosi;   data_cnt <= data_cnt + 1; end     
                            1:begin  mosi_data[14] <= mosi;   data_cnt <= data_cnt + 1; end
                            2:begin  mosi_data[13] <= mosi;   data_cnt <= data_cnt + 1; end 
                            3:begin  mosi_data[12] <= mosi;   data_cnt <= data_cnt + 1; end
                            4:begin  mosi_data[11] <= mosi;   data_cnt <= data_cnt + 1; end
                            5:begin  mosi_data[10] <= mosi;   data_cnt <= data_cnt + 1; end
                            6:begin  mosi_data[9] <= mosi;    data_cnt <= data_cnt + 1; end
                            7:begin  mosi_data[8] <= mosi;    data_cnt <= data_cnt + 1; end     
                            8:begin  mosi_data[7] <= mosi;    data_cnt <= data_cnt + 1; end
                            9:begin  mosi_data[6] <= mosi;    data_cnt <= data_cnt + 1; end
                            10:begin mosi_data[5] <= mosi;    data_cnt <= data_cnt + 1; end
                            11:begin mosi_data[4] <= mosi;    data_cnt <= data_cnt + 1; end
                            12:begin mosi_data[3] <= mosi;    data_cnt <= data_cnt + 1; end
                            13:begin mosi_data[2] <= mosi;    data_cnt <= data_cnt + 1; end
                            14:begin mosi_data[1] <= mosi;    data_cnt <= data_cnt + 1; end
                            15:begin mosi_data[0] <= mosi;    data_cnt <= data_cnt + 1;  

                                    if(addr_da1 == 1027) begin
                                        addr_da1 <= 0;
                                        state <= 0;
                                    end
                                    else begin
                                        addr_da1 <= addr_da1 + 1;
                                        receive_da1 <= 1;
                                    end   

                                end   
                        endcase 
                    end 
                    Write_da2: begin
                        case(data_cnt)
                            0:begin  mosi_data[15] <= mosi;   data_cnt <= data_cnt + 1; end     
                            1:begin  mosi_data[14] <= mosi;   data_cnt <= data_cnt + 1; end
                            2:begin  mosi_data[13] <= mosi;   data_cnt <= data_cnt + 1; end 
                            3:begin  mosi_data[12] <= mosi;   data_cnt <= data_cnt + 1; end
                            4:begin  mosi_data[11] <= mosi;   data_cnt <= data_cnt + 1; end
                            5:begin  mosi_data[10] <= mosi;   data_cnt <= data_cnt + 1; end
                            6:begin  mosi_data[9] <= mosi;    data_cnt <= data_cnt + 1; end
                            7:begin  mosi_data[8] <= mosi;    data_cnt <= data_cnt + 1; end     
                            8:begin  mosi_data[7] <= mosi;    data_cnt <= data_cnt + 1; end
                            9:begin  mosi_data[6] <= mosi;    data_cnt <= data_cnt + 1; end
                            10:begin mosi_data[5] <= mosi;    data_cnt <= data_cnt + 1; end
                            11:begin mosi_data[4] <= mosi;    data_cnt <= data_cnt + 1; end
                            12:begin mosi_data[3] <= mosi;    data_cnt <= data_cnt + 1; end
                            13:begin mosi_data[2] <= mosi;    data_cnt <= data_cnt + 1; end
                            14:begin mosi_data[1] <= mosi;    data_cnt <= data_cnt + 1; end
                            15:begin mosi_data[0] <= mosi;    data_cnt <= data_cnt + 1;  

                                    if(addr_da2 == 1027) begin
                                        addr_da2 <= 0;
                                        state <= 0;
                                    end
                                    else begin
                                        addr_da2 <= addr_da2 + 1;
                                        receive_da2 <= 1;
                                    end   

                                end   
                        endcase 
                    end 
                    enb_da1: begin
                        ready_da1 <= 1;
                        state <= 0;
                    end 
                    enb_da2: begin
                        ready_da2 <= 1;
                        state <= 0;
                    end 
                    enb_da: begin
                        ready_da1 <= 1;
                        ready_da2 <= 1;
                        state <= 0;
                    end 
                ////////////////////////////////////    ad set   /////////////////////////////////////////////// 
                     Write_ad1_stop: begin
                        w_ad1_state <= 0;
                        state <= 0;
                    end 
                    read_ad1_enb: begin
                        ad1_enb_state <= 1;
                        state <= 0;
                    end
                    read_ad1_start: begin
                        case(data_cnt)
                            0:begin   miso <= r_ad1_data[15];  data_cnt <= data_cnt + 1; end     
                            1:begin   miso <= r_ad1_data[14];  data_cnt <= data_cnt + 1; end
                            2:begin   miso <= r_ad1_data[13];  data_cnt <= data_cnt + 1; end 
                            3:begin   miso <= r_ad1_data[12];  data_cnt <= data_cnt + 1; end
                            4:begin   miso <= r_ad1_data[11];  data_cnt <= data_cnt + 1; end
                            5:begin   miso <= r_ad1_data[10];  data_cnt <= data_cnt + 1; end
                            6:begin   miso <= r_ad1_data[9];   data_cnt <= data_cnt + 1; end
                            7:begin   miso <= r_ad1_data[8];   data_cnt <= data_cnt + 1; end     
                            8:begin   miso <= r_ad1_data[7];   data_cnt <= data_cnt + 1; end
                            9:begin   miso <= r_ad1_data[6];   data_cnt <= data_cnt + 1; end
                            10:begin  miso <= r_ad1_data[5];   data_cnt <= data_cnt + 1; end
                            11:begin  miso <= r_ad1_data[4];   data_cnt <= data_cnt + 1; end
                            12:begin  miso <= r_ad1_data[3];   data_cnt <= data_cnt + 1; end
                            13:begin  miso <= r_ad1_data[2];   data_cnt <= data_cnt + 1; end
                            14:begin  miso <= r_ad1_data[1];   data_cnt <= data_cnt + 1; end
                            15:begin  miso <= r_ad1_data[0];   data_cnt <= data_cnt + 1;  
                                    if(read_ad1_cnt == 2047) begin
                                        read_ad1_cnt <= 0;
                                        state <= 0;
                                    end
                                    else begin
                                        read_ad1_cnt <= read_ad1_cnt + 1;
                                    end   

                                end   
                        endcase 
                    end 
                    write_ad1_start: begin
                        w_ad1_state <= 1;
                        state <= 0;
                    end
                    Write_ad2_stop: begin
                        w_ad2_state <= 0;
                        state <= 0;
                    end 
                    read_ad2_enb: begin
                        ad2_enb_state <= 1;
                        state <= 0;
                    end
                    read_ad2_start: begin
                        case(data_cnt)
                            0:begin   miso <= r_ad2_data[15];  data_cnt <= data_cnt + 1; end     
                            1:begin   miso <= r_ad2_data[14];  data_cnt <= data_cnt + 1; end
                            2:begin   miso <= r_ad2_data[13];  data_cnt <= data_cnt + 1; end 
                            3:begin   miso <= r_ad2_data[12];  data_cnt <= data_cnt + 1; end
                            4:begin   miso <= r_ad2_data[11];  data_cnt <= data_cnt + 1; end
                            5:begin   miso <= r_ad2_data[10];  data_cnt <= data_cnt + 1; end
                            6:begin   miso <= r_ad2_data[9];   data_cnt <= data_cnt + 1; end
                            7:begin   miso <= r_ad2_data[8];   data_cnt <= data_cnt + 1; end     
                            8:begin   miso <= r_ad2_data[7];   data_cnt <= data_cnt + 1; end
                            9:begin   miso <= r_ad2_data[6];   data_cnt <= data_cnt + 1; end
                            10:begin  miso <= r_ad2_data[5];   data_cnt <= data_cnt + 1; end
                            11:begin  miso <= r_ad2_data[4];   data_cnt <= data_cnt + 1; end
                            12:begin  miso <= r_ad2_data[3];   data_cnt <= data_cnt + 1; end
                            13:begin  miso <= r_ad2_data[2];   data_cnt <= data_cnt + 1; end
                            14:begin  miso <= r_ad2_data[1];   data_cnt <= data_cnt + 1; end
                            15:begin  miso <= r_ad2_data[0];   data_cnt <= data_cnt + 1;  
                                    if(read_ad2_cnt == 2047) begin
                                        read_ad2_cnt <= 0;
                                        state <= 0;
                                    end
                                    else begin
                                        read_ad2_cnt <= read_ad2_cnt + 1;
                                    end   

                                end   
                        endcase 
                    end 
                    write_ad2_start: begin
                        w_ad2_state <= 1;
                        state <= 0;
                    end
                    ad_check: begin
                        check <= 1500;
                        state <= 0;
                    end  
                    ad_check_finish: begin
                        check <= 0;
                        state <= 0;
                    end 
                    both_start: begin
                        w_ad1_state <= 1;
                        w_ad2_state <= 1;
                        state <= 0;
                    end 
                    both_stop: begin
                        w_ad1_state <= 0;
                        w_ad2_state <= 0;
                        state <= 0;
                    end 
                    default: state <= 0;
                endcase
            end                                                         
        endcase
    end
    else if(receive_da1) begin
        receive_da1 <= 0;
    end
    else if (receive_da2) begin
        receive_da2 <= 0;
    end
    else begin
        receive_da1 <= receive_da1;
        receive_da2 <= receive_da2;
         state <= state;
    end
end

//频率控制
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) 
        FWord_da1 <= 28633;
    else if(addr_da1 == 1025)
        FWord_da1[31:16] <= mosi_data;
    else if(addr_da1 == 1026)
        FWord_da1[15:0] <= mosi_data;   
    else
        FWord_da1 <= FWord_da1;
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) 
        FWord_da2 <= 28633;
    else if(addr_da2 == 1025)
        FWord_da2[31:16] <= mosi_data;
    else if(addr_da2 == 1026)
        FWord_da2[15:0] <= mosi_data;   
    else
        FWord_da2 <= FWord_da2;
end

//相位累加da1
always@(posedge clk200m_da1 or negedge rst_n) begin
    if(!rst_n)
        Freq_ACC_da1 <= 1'd0;
    else if(ready_da1 == 1)
        Freq_ACC_da1 <= Freq_ACC_da1 + FWord_da1;
end
always@(posedge clk200m_da1 or negedge rst_n) begin
    if(!rst_n)
        Rom_Addr_da1 <= 1'd0;
    else if(ready_da1 == 1)
        Rom_Addr_da1 = Freq_ACC_da1[31:22];
end 

//相位累加da2
always@(posedge clk200m_da1 or negedge rst_n) begin
    if(!rst_n)
        Freq_ACC_da2 <= 1'd0;
    else if(ready_da2 == 1)
        Freq_ACC_da2 <= Freq_ACC_da2 + FWord_da2;
end
always@(posedge clk200m_da1 or negedge rst_n) begin
    if(!rst_n)
        Rom_Addr_da2 <= 1'd0;
    else if(ready_da2 == 1)
        Rom_Addr_da2 = Freq_ACC_da2[31:22];
end 
//da1
wire [15:0]r_da1_dout;
da1_bram u_da1_bram (
  .clka(clk),    // input wire clka
  .wea(receive_da1),      // input wire [0 : 0] wea
  .addra((addr_da1-1)),  // input wire [10 : 0] addra
  .dina(mosi_data),    // input wire [15 : 0] dina
  .clkb(clk200m_da1),    // input wire clkb
  .enb(1),      // input wire enb
  .addrb({1'b0,Rom_Addr_da1}),  // input wire [10 : 0] addrb {1'b0,Rom_Addr}
  .doutb(r_da1_dout)  // output wire [15 : 0] doutb
);

always@(posedge clk200m_da1) begin
    if(ready_da1 == 1) begin
        da1_dout <= r_da1_dout[13:0];
    end
end
//da2
wire [15:0]r_da2_dout;
da2_bram u_da2_bram (
  .clka(clk),    // input wire clka
  .wea(receive_da2),      // input wire [0 : 0] wea
  .addra(addr_da2-1),  // input wire [10 : 0] addra
  .dina(mosi_data),    // input wire [15 : 0] dina
  .clkb(clk200m_da2),    // input wire clkb
  .enb(1),      // input wire enb
  .addrb({1'b0,Rom_Addr_da2}),  // input wire [10 : 0] addrb {1'b0,Rom_Addr}
  .doutb(r_da2_dout)  // output wire [15 : 0] doutb
);

always@(posedge clk200m_da1) begin
    if(ready_da1 == 1) begin
        da2_dout <= r_da2_dout[13:0];
    end
end

ad1_ram u_ad1_ram (
  .clka(Adc_ReadClk_65M),    // input wire clka
  .wea(w_ad1_state),      // input wire [0 : 0] wea
  .addra({1'b0,addr_ram1_in}),  // input wire [11 : 0] addra
  .dina(Adc_Data_CHA),    // input wire [15 : 0] dina
  .clkb(clk),    // input wire clkb
  .enb(ad1_enb_state),      // input wire enb
  .addrb(read_ad1_cnt),  // input wire [10 : 0] addrb
  .doutb(r_ad1_data)  // output wire [15 : 0] doutb
);

ad2_ram u_ad2_ram (
  .clka(Adc_ReadClk_65M),    // input wire clka
  .wea(w_ad2_state),      // input wire [0 : 0] wea
  .addra({1'b0,addr_ram2_in}),  // input wire [11 : 0] addra
  .dina(Adc_Data_CHB),    // input wire [15 : 0] dina
  .clkb(clk),    // input wire clkb
  .enb(ad2_enb_state),      // input wire enb
  .addrb(read_ad2_cnt),  // input wire [10 : 0] addrb
  .doutb(r_ad2_data)  // output wire [15 : 0] doutb
);

ila_0 u_ila_0 (
	.clk(da1_clk_p), // input wire clk
	.probe0(r_da1_dout), // input wire [15:0]  probe0  
	.probe1(r_da2_dout), // input wire [15:0]  probe1 
	.probe2(Rom_Addr_da1), // input wire [10:0]  probe2 
	.probe3(Rom_Addr_da2), // input wire [10:0]  probe3 
	.probe4(mosi_data) // input wire [15:0]  probe4 
);
ila_1 u_ila_1 (
	.clk(Adc_Clk_65M), // input wire clk
	.probe0(Adc_Data_CHA), // input wire [11:0]  probe0  
	.probe1(Adc_Data_CHB), // input wire [11:0]  probe1
	.probe2(cmd_reg) // input wire [15:0]  probe2
); 

endmodule
