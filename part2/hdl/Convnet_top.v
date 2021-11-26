module Convnet_top #(
parameter CH_NUM = 4,
parameter ACT_PER_ADDR = 4,
parameter BW_PER_ACT = 12,
parameter WEIGHT_PER_ADDR = 9, 
parameter BIAS_PER_ADDR = 1,
parameter BW_PER_PARAM = 8
)
(
input clk,                          
input rst_n,  // synchronous reset (active low)
input enable, // start sending image from testbanch
output reg busy,  // control signal for stopping loading input image
output reg valid, // output valid for testbench to check answers in corresponding SRAM groups
input [BW_PER_ACT-1:0] input_data, // input image data
// read data from SRAM group A
input [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_a0,
input [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_a1,
input [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_a2,
input [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_a3,
// read data from SRAM group B
input [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_b0,
input [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_b1,
input [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_b2,
input [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_b3,
// read data from parameter SRAM
input [WEIGHT_PER_ADDR*BW_PER_PARAM-1:0] sram_rdata_weight,  
input [BIAS_PER_ADDR*BW_PER_PARAM-1:0] sram_rdata_bias,     
// read address to SRAM group A
output reg [5:0] sram_raddr_a0,
output reg [5:0] sram_raddr_a1,
output reg [5:0] sram_raddr_a2,
output reg [5:0] sram_raddr_a3,
// read address to SRAM group B
output [5:0] sram_raddr_b0,
output [5:0] sram_raddr_b1,
output [5:0] sram_raddr_b2,
output [5:0] sram_raddr_b3,
// read address to parameter SRAM
output reg [9:0] sram_raddr_weight,       
output reg [5:0] sram_raddr_bias,         
// write enable for SRAM groups A & B
output reg sram_wen_a0,
output reg sram_wen_a1,
output reg sram_wen_a2,
output reg sram_wen_a3,
output reg sram_wen_b0,
output reg sram_wen_b1,
output reg sram_wen_b2,
output reg sram_wen_b3,
// word mask for SRAM groups A & B
output reg [CH_NUM*ACT_PER_ADDR-1:0] sram_wordmask_a,
output reg [CH_NUM*ACT_PER_ADDR-1:0] sram_wordmask_b,
// write addrress to SRAM groups A & B
output reg [5:0] sram_waddr_a,
output reg [5:0] sram_waddr_b,
// write data to SRAM groups A & B
output reg [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_wdata_a,
output reg [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_wdata_b
);


localparam [3:0] PART1 = 0;
localparam [3:0] PART2 = 1;

reg [3:0] state, state_n;

reg rst_n_d;
reg enable_d;
reg [BW_PER_ACT-1:0] input_data_d;

wire busy_n;
wire sram_wen_a0_n, sram_wen_a1_n, sram_wen_a2_n, sram_wen_a3_n;
wire [CH_NUM*ACT_PER_ADDR-1:0] sram_wordmask_a_n;
wire [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_wdata_a_n;
wire [5:0] sram_waddr_a_n;
wire valid_n;

wire valid_n_part2;
wire sram_wen_b0_n, sram_wen_b1_n, sram_wen_b2_n, sram_wen_b3_n;
wire [CH_NUM*ACT_PER_ADDR-1:0] sram_wordmask_b_n;
wire [5:0] sram_waddr_b_n;
wire [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_wdata_b_n;


wire [5:0] sram_raddr_a0_n,sram_raddr_a1_n, sram_raddr_a2_n, sram_raddr_a3_n;
wire [9:0] sram_raddr_weight_n;
wire [5:0] sram_raddr_bias_n;


reg [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_a0_d;
reg [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_a1_d;
reg [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_a2_d;
reg [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_a3_d;

reg [WEIGHT_PER_ADDR*BW_PER_PARAM-1:0] sram_rdata_weight_d;  
reg [BIAS_PER_ADDR*BW_PER_PARAM-1:0] sram_rdata_bias_d; 



wire[11:0] test = sram_wdata_a [11:0];

/* ===========output reg ===================*/
always@(posedge clk)begin
    if(~rst_n_d)begin
        busy <= 0;
        valid <= 0;
    end
    else begin
        busy <= busy_n;
        //valid <= valid_n;
        valid <= valid_n_part2;
    end
end

always@(posedge clk)begin
    sram_wen_a0 <= sram_wen_a0_n;
    sram_wen_a1 <= sram_wen_a1_n;
    sram_wen_a2 <= sram_wen_a2_n;
    sram_wen_a3 <= sram_wen_a3_n;

    sram_wen_b0 <= sram_wen_b0_n;
    sram_wen_b1 <= sram_wen_b1_n;
    sram_wen_b2 <= sram_wen_b2_n;
    sram_wen_b3 <= sram_wen_b3_n;
    
    sram_raddr_a0 <= sram_raddr_a0_n;
    sram_raddr_a1 <= sram_raddr_a1_n;
    sram_raddr_a2 <= sram_raddr_a2_n;
    sram_raddr_a3 <= sram_raddr_a3_n;


    sram_wordmask_a <= sram_wordmask_a_n;
    sram_wdata_a <= sram_wdata_a_n;
    sram_waddr_a <= sram_waddr_a_n;
    
    sram_wordmask_b <= sram_wordmask_b_n;
    sram_waddr_b <= sram_waddr_b_n;
    sram_wdata_b <= sram_wdata_b_n;
    
    sram_raddr_weight <= sram_raddr_weight_n;
    sram_raddr_bias <= sram_raddr_bias_n;

end
/*=======================================*/

/*==================input reg =============*/
always@(posedge clk)begin
    rst_n_d <= rst_n;
    enable_d <= enable;
    input_data_d <= input_data;
    sram_rdata_a0_d <= sram_rdata_a0;
    sram_rdata_a1_d <= sram_rdata_a1;
    sram_rdata_a2_d <= sram_rdata_a2;
    sram_rdata_a3_d <= sram_rdata_a3;
    sram_rdata_weight_d <= sram_rdata_weight;
    sram_rdata_bias_d <= sram_rdata_bias;
end
/*========================================*/

always@(posedge clk)begin
    if(~rst_n_d)begin
        state <= PART1;
    end
    else begin
        state <= state_n;
    end
end


always@(*)begin
    case(state)
        PART1:begin
            state_n = (valid_n == 1)? PART2:PART1;
        end
        PART2:begin
            state_n = PART2;
        end
        default:begin
            state_n = 0;
        end
    endcase
end




part1#(
.CH_NUM(CH_NUM),
.ACT_PER_ADDR(ACT_PER_ADDR),
.BW_PER_ACT(BW_PER_ACT)
)
part1_U0(
.clk(clk),
.rst_n(rst_n_d),
.enable(enable_d),
.busy_n(busy_n),
.valid_n(valid_n),

.sram_wen_a0_n(sram_wen_a0_n),
.sram_wen_a1_n(sram_wen_a1_n),
.sram_wen_a2_n(sram_wen_a2_n),
.sram_wen_a3_n(sram_wen_a3_n),

.input_data(input_data_d), 

.sram_wordmask_a_n(sram_wordmask_a_n),

.sram_waddr_a_n(sram_waddr_a_n),

.sram_wdata_a_n(sram_wdata_a_n)

);

part2 #(
.CH_NUM(CH_NUM),
.ACT_PER_ADDR(ACT_PER_ADDR),
.BW_PER_ACT(BW_PER_ACT),
.WEIGHT_PER_ADDR(WEIGHT_PER_ADDR), 
.BIAS_PER_ADDR(BIAS_PER_ADDR),
.BW_PER_PARAM(BW_PER_PARAM)

)
part2_U0(
.clk(clk),
.rst_n(rst_n_d),
.part2_en(valid_n),

.sram_rdata_weight_d(sram_rdata_weight_d),  
.sram_rdata_bias_d(sram_rdata_bias_d),     

.sram_raddr_a0(sram_raddr_a0),
.sram_raddr_a1(sram_raddr_a1),
.sram_raddr_a2(sram_raddr_a2),
.sram_raddr_a3(sram_raddr_a3),



.sram_raddr_a0_n(sram_raddr_a0_n),
.sram_raddr_a1_n(sram_raddr_a1_n),
.sram_raddr_a2_n(sram_raddr_a2_n),
.sram_raddr_a3_n(sram_raddr_a3_n),

.sram_rdata_a0_d(sram_rdata_a0_d),
.sram_rdata_a1_d(sram_rdata_a1_d),
.sram_rdata_a2_d(sram_rdata_a2_d),
.sram_rdata_a3_d(sram_rdata_a3_d),

.valid_n(valid_n_part2),
.sram_wen_b0_n(sram_wen_b0_n),
.sram_wen_b1_n(sram_wen_b1_n),
.sram_wen_b2_n(sram_wen_b2_n),
.sram_wen_b3_n(sram_wen_b3_n),

.sram_wordmask_b_n(sram_wordmask_b_n),
.sram_waddr_b_n(sram_waddr_b_n),
.sram_wdata_b_n(sram_wdata_b_n),
.sram_raddr_weight_n(sram_raddr_weight_n),
.sram_raddr_bias_n(sram_raddr_bias_n)

);


endmodule

module part1 #(
parameter CH_NUM = 4,
parameter ACT_PER_ADDR = 4,
parameter BW_PER_ACT = 12
)
(
input clk,
input rst_n,
input enable,
output reg busy_n,
output reg valid_n,

output sram_wen_a0_n,
output sram_wen_a1_n,
output sram_wen_a2_n,
output sram_wen_a3_n,

input [BW_PER_ACT-1:0] input_data, 

output [CH_NUM*ACT_PER_ADDR-1:0] sram_wordmask_a_n,

output [5:0] sram_waddr_a_n,

output reg [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_wdata_a_n

);

localparam [1:0] IDLE = 0;
localparam [1:0] READ = 1;
localparam [1:0] FINISH = 2;

reg [1:0] state, state_n;
reg[4:0] x, y;
reg[4:0] x_n, y_n;

integer i;

always@(posedge clk)begin
    if(~rst_n)begin
        state <= IDLE;
    end
    else begin
        state <= state_n;
    end
end

always@(posedge clk)begin
    x <= x_n;
    y <= y_n;
end

assign sram_wen_a0_n = (state == READ)? ~(x[2] == 0 & y[2] == 0) : 1;
assign sram_wen_a1_n = (state == READ)? ~(x[2] == 1 & y[2] == 0) : 1;
assign sram_wen_a2_n = (state == READ)? ~(x[2] == 0 & y[2] == 1) : 1;
assign sram_wen_a3_n = (state == READ)? ~(x[2] == 1 & y[2] == 1) : 1;


assign sram_wordmask_a_n[15] = ~(x[1:0]==0 & y[1:0] == 0);
assign sram_wordmask_a_n[14] = ~(x[1:0]==2 & y[1:0] == 0);
assign sram_wordmask_a_n[13] = ~(x[1:0]==0 & y[1:0] == 2);
assign sram_wordmask_a_n[12] = ~(x[1:0]==2 & y[1:0] == 2);

assign sram_wordmask_a_n[11] = ~(x[1:0]==1 & y[1:0] == 0);
assign sram_wordmask_a_n[10] = ~(x[1:0]==3 & y[1:0] == 0);
assign sram_wordmask_a_n[ 9] = ~(x[1:0]==1 & y[1:0] == 2);
assign sram_wordmask_a_n[ 8] = ~(x[1:0]==3 & y[1:0] == 2);

assign sram_wordmask_a_n[ 7] = ~(x[1:0]==0 & y[1:0] == 1);
assign sram_wordmask_a_n[ 6] = ~(x[1:0]==2 & y[1:0] == 1);
assign sram_wordmask_a_n[ 5] = ~(x[1:0]==0 & y[1:0] == 3);
assign sram_wordmask_a_n[ 4] = ~(x[1:0]==2 & y[1:0] == 3);

assign sram_wordmask_a_n[ 3] = ~(x[1:0]==1 & y[1:0] == 1);
assign sram_wordmask_a_n[ 2] = ~(x[1:0]==3 & y[1:0] == 1);
assign sram_wordmask_a_n[ 1] = ~(x[1:0]==1 & y[1:0] == 3);
assign sram_wordmask_a_n[ 0] = ~(x[1:0]==3 & y[1:0] == 3);



always@(*)begin
    for(i = 0;i<16;i=i+1)begin
        sram_wdata_a_n[i*12 +: 12] = input_data;
    end
end

assign sram_waddr_a_n = x[4:3] + y[4:3]*6;

always@(*)begin
    case(state)
        IDLE:begin
            state_n = (enable)? READ:IDLE;
            busy_n = (enable) ? 0:1;
            x_n = 0;
            y_n = 0;
            valid_n = 0;
        end
        READ:begin
            state_n = (x == 27 && y == 27)? FINISH : READ;
            busy_n = 0;
            x_n = (x != 27)? x + 1 : 0;
            y_n = (x == 27)? y + 1 : y;
            valid_n = 0;
        end
        FINISH:begin
            state_n = FINISH;
            busy_n = 1;
            x_n = 0;
            y_n = 0;
            valid_n = 1;
        end
    endcase
end

endmodule


module part2 #(
parameter CH_NUM = 4,
parameter ACT_PER_ADDR = 4,
parameter BW_PER_ACT = 12,
parameter WEIGHT_PER_ADDR = 9, 
parameter BIAS_PER_ADDR = 1,
parameter BW_PER_PARAM = 8

)
(
input clk,
input rst_n,
input part2_en,
input [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_a0_d,
input [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_a1_d,
input [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_a2_d,
input [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_rdata_a3_d,
input [WEIGHT_PER_ADDR*BW_PER_PARAM-1:0] sram_rdata_weight_d,  
input [BIAS_PER_ADDR*BW_PER_PARAM-1:0] sram_rdata_bias_d,     

input [5:0] sram_raddr_a0,
input [5:0] sram_raddr_a1,
input [5:0] sram_raddr_a2,
input [5:0] sram_raddr_a3,





output reg valid_n,

output reg [5:0] sram_raddr_a0_n,
output reg [5:0] sram_raddr_a1_n,
output reg [5:0] sram_raddr_a2_n,
output reg [5:0] sram_raddr_a3_n,


output sram_wen_b0_n,
output sram_wen_b1_n,
output sram_wen_b2_n,
output sram_wen_b3_n,
output [CH_NUM*ACT_PER_ADDR-1:0] sram_wordmask_b_n,
output [5:0] sram_waddr_b_n,
output [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] sram_wdata_b_n,
output [9:0] sram_raddr_weight_n,
output reg [5:0] sram_raddr_bias_n


);
localparam [3:0] IDLE = 0;
localparam [3:0] BLANK_1 = 1;
localparam [3:0] BLANK_2 = 2;
localparam [3:0] BLANK_3 = 3;
localparam [3:0] CAL = 4;

reg [3:0] state, state_n;
reg [3:0] feature_map_cnt, feature_map_cnt_n;
reg [1:0] in_cnt, in_cnt_n;
reg [1:0] ker_cnt, ker_cnt_n;


wire [19:0] buf_0 [0:8];

wire [19:0] buf_1 [0:8];

wire [19:0] buf_2 [0:8];

wire [19:0] buf_3 [0:8];

wire  [11:0] pos [0:15];

reg [11:0] pos2[0:15];
reg [19:0] buf_0_2[0:8];
reg [19:0] buf_1_2[0:8];
reg [19:0] buf_2_2[0:8];
reg [19:0] buf_3_2[0:8];

wire [19:0] sum [0:3];
reg [19:0] sum2 [0:3];


wire [19:0] weight [0:8];

reg [2:0] x, x_n;
reg [2:0] y, y_n;

reg [2:0] x_2, x_2_n;
reg [2:0] y_2, y_2_n;

reg [2:0] x_2_delay[0:1];
reg [2:0] y_2_delay[0:1];

wire [11:0] data [0:3];

reg [CH_NUM*ACT_PER_ADDR*BW_PER_ACT-1:0] left_up, right_up, left_down, right_down;



integer i, j;

assign data[0] = sram_wdata_b_n[3*48 + 36 +: 12];
assign data[1] = sram_wdata_b_n[3*48 + 24 +: 12];
assign data[2] = sram_wdata_b_n[3*48 + 12 +: 12];
assign data[3] = sram_wdata_b_n[3*48 +  0 +: 12];


assign weight[0] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-0) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-0) - 1 -: 8]};
assign weight[1] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-1) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-1) - 1 -: 8]};
assign weight[2] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-2) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-2) - 1 -: 8]};
assign weight[3] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-3) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-3) - 1 -: 8]};
assign weight[4] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-4) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-4) - 1 -: 8]};
assign weight[5] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-5) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-5) - 1 -: 8]};
assign weight[6] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-6) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-6) - 1 -: 8]};
assign weight[7] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-7) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-7) - 1 -: 8]};
assign weight[8] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-8) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-8) - 1 -: 8]};

always@(*)begin
    left_up    = (x_2_delay[1][0] == 1 && y_2_delay[1][0]==0)? sram_rdata_a0_d : (x_2_delay[1][0] == 0 && y_2_delay[1][0]==0) ? sram_rdata_a1_d : (x_2_delay[1][0] == 1 && y_2_delay[1][0] == 1)? sram_rdata_a2_d : sram_rdata_a3_d;
    right_up   = (x_2_delay[1][0] == 1 && y_2_delay[1][0]==0)? sram_rdata_a1_d : (x_2_delay[1][0] == 0 && y_2_delay[1][0]==0) ? sram_rdata_a0_d : (x_2_delay[1][0] == 1 && y_2_delay[1][0] == 1)? sram_rdata_a3_d : sram_rdata_a2_d;
    left_down  = (x_2_delay[1][0] == 1 && y_2_delay[1][0]==0)? sram_rdata_a2_d : (x_2_delay[1][0] == 0 && y_2_delay[1][0]==0) ? sram_rdata_a3_d : (x_2_delay[1][0] == 1 && y_2_delay[1][0] == 1)? sram_rdata_a0_d : sram_rdata_a1_d;
    right_down = (x_2_delay[1][0] == 1 && y_2_delay[1][0]==0)? sram_rdata_a3_d : (x_2_delay[1][0] == 0 && y_2_delay[1][0]==0) ? sram_rdata_a2_d : (x_2_delay[1][0] == 1 && y_2_delay[1][0] == 1)? sram_rdata_a1_d : sram_rdata_a0_d;
end


//assign pos[ 0] = sram_rdata_a0_d[(in_cnt)*48 + 36 +: 12];
//assign pos[ 1] = sram_rdata_a0_d[(in_cnt)*48 + 24 +: 12];
//assign pos[ 2] = sram_rdata_a1_d[(in_cnt)*48 + 36 +: 12];
//assign pos[ 3] = sram_rdata_a1_d[(in_cnt)*48 + 24 +: 12];
//assign pos[ 4] = sram_rdata_a0_d[(in_cnt)*48 + 12 +: 12];
//assign pos[ 5] = sram_rdata_a0_d[(in_cnt)*48 +  0 +: 12];
//assign pos[ 6] = sram_rdata_a1_d[(in_cnt)*48 + 12 +: 12];
//assign pos[ 7] = sram_rdata_a1_d[(in_cnt)*48 +  0 +: 12];
//assign pos[ 8] = sram_rdata_a2_d[(in_cnt)*48 + 36 +: 12];
//assign pos[ 9] = sram_rdata_a2_d[(in_cnt)*48 + 24 +: 12];
//assign pos[10] = sram_rdata_a3_d[(in_cnt)*48 + 36 +: 12];
//assign pos[11] = sram_rdata_a3_d[(in_cnt)*48 + 24 +: 12];
//assign pos[12] = sram_rdata_a2_d[(in_cnt)*48 + 12 +: 12];
//assign pos[13] = sram_rdata_a2_d[(in_cnt)*48 +  0 +: 12];
//assign pos[14] = sram_rdata_a3_d[(in_cnt)*48 + 12 +: 12];
//assign pos[15] = sram_rdata_a3_d[(in_cnt)*48 +  0 +: 12];

assign pos[ 0] = left_up   [(in_cnt)*48 + 36 +: 12];
assign pos[ 1] = left_up   [(in_cnt)*48 + 24 +: 12];
assign pos[ 2] = right_up  [(in_cnt)*48 + 36 +: 12];
assign pos[ 3] = right_up  [(in_cnt)*48 + 24 +: 12];
assign pos[ 4] = left_up   [(in_cnt)*48 + 12 +: 12];
assign pos[ 5] = left_up   [(in_cnt)*48 +  0 +: 12];
assign pos[ 6] = right_up  [(in_cnt)*48 + 12 +: 12];
assign pos[ 7] = right_up  [(in_cnt)*48 +  0 +: 12];
assign pos[ 8] = left_down [(in_cnt)*48 + 36 +: 12];
assign pos[ 9] = left_down [(in_cnt)*48 + 24 +: 12];
assign pos[10] = right_down[(in_cnt)*48 + 36 +: 12];
assign pos[11] = right_down[(in_cnt)*48 + 24 +: 12];
assign pos[12] = left_down [(in_cnt)*48 + 12 +: 12];
assign pos[13] = left_down [(in_cnt)*48 +  0 +: 12];
assign pos[14] = right_down[(in_cnt)*48 + 12 +: 12];
assign pos[15] = right_down[(in_cnt)*48 +  0 +: 12];

always@(posedge clk)begin
    for( i = 0 ; i < 16 ; i = i + 1)begin
        pos2[i] <= pos[i]; 
    end
end



assign buf_0[0] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-0) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-0) - 1 -: 8]} * {{8{pos2[0][BW_PER_ACT-1]}},  pos2[0]};
assign buf_0[1] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-1) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-1) - 1 -: 8]} * {{8{pos2[1][BW_PER_ACT-1]}},  pos2[1]};
assign buf_0[2] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-2) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-2) - 1 -: 8]} * {{8{pos2[2][BW_PER_ACT-1]}},  pos2[2]};
assign buf_0[3] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-3) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-3) - 1 -: 8]} * {{8{pos2[4][BW_PER_ACT-1]}},  pos2[4]};
assign buf_0[4] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-4) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-4) - 1 -: 8]} * {{8{pos2[5][BW_PER_ACT-1]}},  pos2[5]};
assign buf_0[5] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-5) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-5) - 1 -: 8]} * {{8{pos2[6][BW_PER_ACT-1]}},  pos2[6]};
assign buf_0[6] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-6) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-6) - 1 -: 8]} * {{8{pos2[8][BW_PER_ACT-1]}},  pos2[8]};
assign buf_0[7] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-7) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-7) - 1 -: 8]} * {{8{pos2[9][BW_PER_ACT-1]}},  pos2[9]};
assign buf_0[8] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-8) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-8) - 1 -: 8]} * {{8{pos2[10][BW_PER_ACT-1]}}, pos2[10]};

assign buf_1[0] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-0) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-0) - 1 -: 8]} * {{8{pos2[1][BW_PER_ACT-1]}},  pos2[1]};
assign buf_1[1] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-1) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-1) - 1 -: 8]} * {{8{pos2[2][BW_PER_ACT-1]}},  pos2[2]};
assign buf_1[2] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-2) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-2) - 1 -: 8]} * {{8{pos2[3][BW_PER_ACT-1]}},  pos2[3]};
assign buf_1[3] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-3) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-3) - 1 -: 8]} * {{8{pos2[5][BW_PER_ACT-1]}},  pos2[5]};
assign buf_1[4] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-4) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-4) - 1 -: 8]} * {{8{pos2[6][BW_PER_ACT-1]}},  pos2[6]};
assign buf_1[5] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-5) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-5) - 1 -: 8]} * {{8{pos2[7][BW_PER_ACT-1]}},  pos2[7]};
assign buf_1[6] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-6) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-6) - 1 -: 8]} * {{8{pos2[9][BW_PER_ACT-1]}},  pos2[9]};
assign buf_1[7] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-7) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-7) - 1 -: 8]} * {{8{pos2[10][BW_PER_ACT-1]}}, pos2[10]};
assign buf_1[8] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-8) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-8) - 1 -: 8]} * {{8{pos2[11][BW_PER_ACT-1]}}, pos2[11]};

assign buf_2[0] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-0) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-0) - 1 -: 8]} * {{8{pos2[4][BW_PER_ACT-1]}},  pos2[4]};
assign buf_2[1] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-1) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-1) - 1 -: 8]} * {{8{pos2[5][BW_PER_ACT-1]}},  pos2[5]};
assign buf_2[2] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-2) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-2) - 1 -: 8]} * {{8{pos2[6][BW_PER_ACT-1]}},  pos2[6]};
assign buf_2[3] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-3) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-3) - 1 -: 8]} * {{8{pos2[8][BW_PER_ACT-1]}},  pos2[8]};
assign buf_2[4] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-4) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-4) - 1 -: 8]} * {{8{pos2[9][BW_PER_ACT-1]}},  pos2[9]};
assign buf_2[5] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-5) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-5) - 1 -: 8]} * {{8{pos2[10][BW_PER_ACT-1]}}, pos2[10]};
assign buf_2[6] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-6) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-6) - 1 -: 8]} * {{8{pos2[12][BW_PER_ACT-1]}}, pos2[12]};
assign buf_2[7] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-7) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-7) - 1 -: 8]} * {{8{pos2[13][BW_PER_ACT-1]}}, pos2[13]};
assign buf_2[8] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-8) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-8) - 1 -: 8]} * {{8{pos2[14][BW_PER_ACT-1]}}, pos2[14]};

assign buf_3[0] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-0) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-0) - 1 -: 8]} * {{8{pos2[5] [BW_PER_ACT-1]}}, pos2[5] };
assign buf_3[1] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-1) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-1) - 1 -: 8]} * {{8{pos2[6] [BW_PER_ACT-1]}}, pos2[6] };
assign buf_3[2] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-2) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-2) - 1 -: 8]} * {{8{pos2[7] [BW_PER_ACT-1]}}, pos2[7] };
assign buf_3[3] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-3) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-3) - 1 -: 8]} * {{8{pos2[9] [BW_PER_ACT-1]}}, pos2[9] };
assign buf_3[4] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-4) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-4) - 1 -: 8]} * {{8{pos2[10][BW_PER_ACT-1]}}, pos2[10]};
assign buf_3[5] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-5) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-5) - 1 -: 8]} * {{8{pos2[11][BW_PER_ACT-1]}}, pos2[11]};
assign buf_3[6] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-6) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-6) - 1 -: 8]} * {{8{pos2[13][BW_PER_ACT-1]}}, pos2[13]};
assign buf_3[7] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-7) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-7) - 1 -: 8]} * {{8{pos2[14][BW_PER_ACT-1]}}, pos2[14]};
assign buf_3[8] = {{12{sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-8) - 1]}},sram_rdata_weight_d[BW_PER_PARAM * (WEIGHT_PER_ADDR-8) - 1 -: 8]} * {{8{pos2[15][BW_PER_ACT-1]}}, pos2[15]};



always@(posedge clk)begin
    for(j=0;j<9;j=j+1)begin
        buf_0_2[j] <= buf_0[j];
        buf_1_2[j] <= buf_1[j];
        buf_2_2[j] <= buf_2[j];
        buf_3_2[j] <= buf_3[j];
    end
end

assign sum[0] = buf_0_2[0] + buf_0_2[1] + buf_0_2[2] + buf_0_2[3] + buf_0_2[4] + buf_0_2[5] + buf_0_2[6] + buf_0_2[7] + buf_0_2[8]; 
assign sum[1] = buf_1_2[0] + buf_1_2[1] + buf_1_2[2] + buf_1_2[3] + buf_1_2[4] + buf_1_2[5] + buf_1_2[6] + buf_1_2[7] + buf_1_2[8]; 
assign sum[2] = buf_2_2[0] + buf_2_2[1] + buf_2_2[2] + buf_2_2[3] + buf_2_2[4] + buf_2_2[5] + buf_2_2[6] + buf_2_2[7] + buf_2_2[8]; 
assign sum[3] = buf_3_2[0] + buf_3_2[1] + buf_3_2[2] + buf_3_2[3] + buf_3_2[4] + buf_3_2[5] + buf_3_2[6] + buf_3_2[7] + buf_3_2[8]; 

always@(posedge clk)begin
    sum2[0] <= (in_cnt != 1) ? sum[0]+sum2[0]:sum[0] + {sram_rdata_bias_d, 8'd0} + 64;
    sum2[1] <= (in_cnt != 1) ? sum[1]+sum2[1]:sum[1] + {sram_rdata_bias_d, 8'd0} + 64;
    sum2[2] <= (in_cnt != 1) ? sum[2]+sum2[2]:sum[2] + {sram_rdata_bias_d, 8'd0} + 64;
    sum2[3] <= (in_cnt != 1) ? sum[3]+sum2[3]:sum[3] + {sram_rdata_bias_d, 8'd0} + 64;
end

assign sram_wdata_b_n[3*48 + 36 +: 12] = sum2[0][7 +: 12];
assign sram_wdata_b_n[3*48 + 24 +: 12] = sum2[1][7 +: 12];
assign sram_wdata_b_n[3*48 + 12 +: 12] = sum2[2][7 +: 12];
assign sram_wdata_b_n[3*48 +  0 +: 12] = sum2[3][7 +: 12];

assign sram_wdata_b_n[2*48 + 36 +: 12] = sum2[0][7 +: 12];
assign sram_wdata_b_n[2*48 + 24 +: 12] = sum2[1][7 +: 12];
assign sram_wdata_b_n[2*48 + 12 +: 12] = sum2[2][7 +: 12];
assign sram_wdata_b_n[2*48 +  0 +: 12] = sum2[3][7 +: 12];

assign sram_wdata_b_n[1*48 + 36 +: 12] = sum2[0][7 +: 12];
assign sram_wdata_b_n[1*48 + 24 +: 12] = sum2[1][7 +: 12];
assign sram_wdata_b_n[1*48 + 12 +: 12] = sum2[2][7 +: 12];
assign sram_wdata_b_n[1*48 +  0 +: 12] = sum2[3][7 +: 12];

assign sram_wdata_b_n[0*48 + 36 +: 12] = sum2[0][7 +: 12];
assign sram_wdata_b_n[0*48 + 24 +: 12] = sum2[1][7 +: 12];
assign sram_wdata_b_n[0*48 + 12 +: 12] = sum2[2][7 +: 12];
assign sram_wdata_b_n[0*48 +  0 +: 12] = sum2[3][7 +: 12];


assign sram_wen_b0_n = (in_cnt == 1)? ~(x[0]==0 & y[0]==0):1;
assign sram_wen_b1_n = (in_cnt == 1)? ~(x[0]==1 & y[0]==0):1;
assign sram_wen_b2_n = (in_cnt == 1)? ~(x[0]==0 & y[0]==1):1;
assign sram_wen_b3_n = (in_cnt == 1)? ~(x[0]==1 & y[0]==1):1;

assign sram_wordmask_b_n[ 0] = ~(feature_map_cnt[3:2] == 3); 
assign sram_wordmask_b_n[ 1] = ~(feature_map_cnt[3:2] == 3); 
assign sram_wordmask_b_n[ 2] = ~(feature_map_cnt[3:2] == 3); 
assign sram_wordmask_b_n[ 3] = ~(feature_map_cnt[3:2] == 3); 
assign sram_wordmask_b_n[ 4] = ~(feature_map_cnt[3:2] == 2); 
assign sram_wordmask_b_n[ 5] = ~(feature_map_cnt[3:2] == 2); 
assign sram_wordmask_b_n[ 6] = ~(feature_map_cnt[3:2] == 2); 
assign sram_wordmask_b_n[ 7] = ~(feature_map_cnt[3:2] == 2); 
assign sram_wordmask_b_n[ 8] = ~(feature_map_cnt[3:2] == 1); 
assign sram_wordmask_b_n[ 9] = ~(feature_map_cnt[3:2] == 1); 
assign sram_wordmask_b_n[10] = ~(feature_map_cnt[3:2] == 1); 
assign sram_wordmask_b_n[11] = ~(feature_map_cnt[3:2] == 1); 
assign sram_wordmask_b_n[12] = ~(feature_map_cnt[3:2] == 0); 
assign sram_wordmask_b_n[13] = ~(feature_map_cnt[3:2] == 0); 
assign sram_wordmask_b_n[14] = ~(feature_map_cnt[3:2] == 0); 
assign sram_wordmask_b_n[15] = ~(feature_map_cnt[3:2] == 0); 

assign sram_waddr_b_n = y*3 + x;



assign sram_raddr_weight_n = feature_map_cnt + ker_cnt;

always@(posedge clk)begin
    if(~rst_n)begin
        state <= IDLE;
    end
    else begin
        state <= state_n;
    end
end

always@(posedge clk)begin
    feature_map_cnt <= feature_map_cnt_n;
    ker_cnt <= ker_cnt_n;
    in_cnt <= in_cnt_n;
    x <= x_n;
    y <= y_n;
    x_2 <= x_2_n;
    y_2 <= y_2_n;

    x_2_delay[0] <= x_2;
    x_2_delay[1] <= x_2_delay[0];
    
    y_2_delay[0] <= y_2;
    y_2_delay[1] <= y_2_delay[0];
end

always@(*)begin
    case(state)
        IDLE:begin
            state_n = (part2_en)? BLANK_1 : IDLE;
            sram_raddr_a0_n = 0;
            sram_raddr_a1_n = 0;
            sram_raddr_a2_n = 0;
            sram_raddr_a3_n = 0;
            sram_raddr_bias_n = 0;
            feature_map_cnt_n = 0;
            ker_cnt_n = 0;
            in_cnt_n = (part2_en)? in_cnt - 1 : 2;
            x_n = 7;
            y_n = 0;

            x_2_n = (part2_en) ? 1 : 0;
            y_2_n = 0; 
        end
        BLANK_1:begin
            state_n = BLANK_2;
            sram_raddr_a0_n = 0;
            sram_raddr_a1_n = 0;
            sram_raddr_a2_n = 0;
            sram_raddr_a3_n = 0;
            sram_raddr_bias_n = 0;
            feature_map_cnt_n = 0;
            ker_cnt_n = ker_cnt+1;
            in_cnt_n = in_cnt - 1;
            x_n = 7;
            y_n = 0;
            
            x_2_n = 1;
            y_2_n = 0; 
        end
        BLANK_2:begin
            state_n = CAL;
            sram_raddr_a0_n = 0;
            sram_raddr_a1_n = 0;
            sram_raddr_a2_n = 0;
            sram_raddr_a3_n = 0;
            sram_raddr_bias_n = 0;
            feature_map_cnt_n = 0;
            ker_cnt_n = ker_cnt + 1;
            in_cnt_n = in_cnt - 1;
            x_n = 7;
            y_n = 0;
            

            x_2_n = 1;
            y_2_n = 0;
        end
        //BLANK_3:begin
        //    state_n = CAL;
        //    sram_raddr_a0_n = 0;
        //    sram_raddr_a1_n = 0;
        //    sram_raddr_a2_n = 0;
        //    sram_raddr_a3_n = 0;
        //    sram_raddr_bias_n = 0;
        //    feature_map_cnt_n = 0;
        //    ker_cnt_n = ker_cnt + 1;
        //    in_cnt_n = in_cnt - 1;
        //    x_n = -1;
        //    y_n = 0;
        //end
        CAL:begin
            state_n = CAL;
            sram_raddr_a0_n = (in_cnt == 2) ? x_2[2:1] + x_2[0] + (y_2[2:1] + y_2[0]) * 6 : sram_raddr_a0;
            sram_raddr_a1_n = (in_cnt == 2) ? x_2[2:1]          + (y_2[2:1] + y_2[0]) * 6 : sram_raddr_a1;
            sram_raddr_a2_n = (in_cnt == 2) ? x_2[2:1] + x_2[0] + y_2[2:1] * 6 : sram_raddr_a2;
            sram_raddr_a3_n = (in_cnt == 2) ? x_2[2:1]          + y_2[2:1] * 6 : sram_raddr_a3;
            sram_raddr_bias_n = 0;
            feature_map_cnt_n = 0;
            ker_cnt_n = ker_cnt + 1;
            in_cnt_n = in_cnt - 1;
            x_n = (in_cnt == 1) ? (x != 5)? x+1 : 0 :x;
            y_n = (x == 5 & in_cnt == 1) ? y+1: y;
            

            x_2_n = (in_cnt == 2) ? (x_2 != 5)? x_2+1 : 0 : x_2;
            y_2_n = (in_cnt == 2 && x_2 == 5)? y_2+1 : y_2;
        end
        default:begin
        end
    endcase
end


endmodule

















