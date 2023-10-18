`timescale 1ns / 1ps
module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,    
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

parameter RSET = 2'b00;
parameter WAIT = 2'b01;
parameter GETX = 2'b10;
parameter CALC = 2'b11;

reg awready_r, awready_w;
reg wready_r, wready_w;
reg awflag_r, awflag_w;
reg wflag_r, wflag_w;
reg [(pADDR_WIDTH-1):0] axi_lite_waddr_r, axi_lite_waddr_w;
reg [(pDATA_WIDTH-1):0] axi_lite_wdata_r, axi_lite_wdata_w;
reg [9:0] data_length_r, data_length_w;

reg arready_r, arready_w;
reg rvalid_r, rvalid_w;
reg [(pADDR_WIDTH-1):0] axi_lite_raddr_r, axi_lite_raddr_w;

reg ss_tready_r, ss_tready_w;
reg x_buf_emp_r, x_buf_emp_w;
reg [(pDATA_WIDTH-1):0] x_buf_r, x_buf_w;

reg sm_tvalid_r, sm_tvalid_w;
reg sm_tlast_r, sm_tlast_w;
reg y_buf_emp_r, y_buf_emp_w;
reg [(pDATA_WIDTH-1):0] y_buf_r, y_buf_w;

reg       halt_r, halt_w;
reg       done_r, done_w;
reg [1:0] state_r, state_w;
reg [3:0] data_ptr_r, data_ptr_w;
reg [3:0] coef_ptr_r, coef_ptr_w;
reg [(pDATA_WIDTH-1):0] accum_r, accum_w;
reg [(pDATA_WIDTH-1):0] tap_Do_r, tap_Do_w, data_Do_r, data_Do_w;
reg [9:0] data_length_cnt_r, data_length_cnt_w;

reg [2:0] ap_status_r, ap_status_w;
reg [2:0] ap_status_d1_r, ap_status_d1_w;

wire [(pDATA_WIDTH-1):0] product;

wire ap_start, save_coef, save_length, sample_state, send_coef, calc_one;
assign ap_start = (awflag_w & wflag_w) & (axi_lite_waddr_w[7:4] == 0) & ap_status_r == 3'b100;
assign save_length = (awflag_r & wflag_r) & (axi_lite_waddr_r[7:4] == 1);
assign save_coef = (awflag_r & wflag_r) & (axi_lite_waddr_r[7:4] > 1);
assign sample_state = (arready & arvalid) & (axi_lite_raddr_w[7:4] == 0);
assign send_coef = (arready & arvalid) & (axi_lite_raddr_w[7:4] > 1);
assign calc_one = (state_r == CALC) & (coef_ptr_r == 0);

assign awready = awready_r;
assign wready = wready_r;

assign arready = arready_r;
assign rvalid = rvalid_r;
assign rdata = (axi_lite_raddr_r[7:4] == 0) ? ap_status_d1_r : tap_Do;

assign ss_tready = ss_tready_r;

assign sm_tvalid = sm_tvalid_r;
assign sm_tlast = sm_tlast_r;
assign sm_tdata = y_buf_r;

assign tap_WE = (save_coef) ? 4'b1111 : 4'b0000;
assign tap_EN = 1'b1;
assign tap_Di = (save_coef) ? axi_lite_wdata_r : 4'bx;
assign tap_A = (save_coef) ? (axi_lite_waddr_r - 12'h20) : (send_coef) ? (axi_lite_raddr_w - 12'h20) : {coef_ptr_w, 2'b0};

assign data_WE = (state_r == RSET || state_r == GETX) ? 4'b1111 : 4'b0000;
assign data_EN = 1'b1;
assign data_Di = (state_r == RSET) ? 0 : x_buf_r;
assign data_A = (state_r == CALC) ? {data_ptr_w, 2'b0} : {data_ptr_r, 2'b0};

assign product = $signed(tap_Do_r) * $signed(data_Do_r);

always @(*) begin
	awready_w = (ap_status_r[2]) ? !(awready & awvalid) : 1'b0;
	wready_w = (ap_status_r[2]) ? !(wready & wvalid) : 1'b0;
	awflag_w = (awready & awvalid) ? 1'b1 : (wflag_r) ? 1'b0 : awflag_r;
	wflag_w = (wready & wvalid) ? 1'b1 : (awflag_r) ? 1'b0 : wflag_r;
	axi_lite_waddr_w = (awready & awvalid) ? awaddr : axi_lite_waddr_r;
	axi_lite_wdata_w = (wready & wvalid) ? wdata : axi_lite_wdata_r;
	data_length_w = (save_length) ? axi_lite_wdata_r : data_length_r;

	arready_w = (arready & arvalid) ? 1'b0 : (rvalid & rready) ? 1'b1 : arready_r;
	axi_lite_raddr_w = (arvalid & arready) ? araddr : axi_lite_raddr_r;
	rvalid_w = (!arvalid) ? 1'b0 : (arready & arvalid) ? 1'b1 : (rready & rvalid) ? 1'b0 : rvalid_r; 

	ss_tready_w = (ss_tready & ss_tvalid) ? 1'b0 : (x_buf_emp_r & ss_tvalid);
	x_buf_emp_w = (ss_tready & ss_tvalid) ? 1'b0 : (state_r == GETX) ? 1'b1 : x_buf_emp_r;
	x_buf_w = (ss_tready & ss_tvalid) ? ss_tdata : x_buf_r;

	sm_tvalid_w = (sm_tready & sm_tvalid) ? 1'b0 : (!y_buf_emp_r) ? 1'b1 : sm_tvalid_r;
	sm_tlast_w = (sm_tready & sm_tvalid) ? 1'b0 : (calc_one & data_length_cnt_r == data_length_r - 1'b1) ? 1'b1 : sm_tlast_r; 
	y_buf_emp_w = (sm_tready & sm_tvalid) ? 1'b1 : calc_one ? 1'b0 : y_buf_emp_r;
	y_buf_w = (calc_one & y_buf_emp_r) ? accum_w : y_buf_r;
end

always @(*) begin
    // ap_start
    ap_status_w[0] = (ap_start) ? 1'b1 : 1'b0; 
    // ap_done
    ap_status_w[1] = (done_r) ? 1'b1 : (sample_state) ? 1'b0 : ap_status_r[1]; 
    // ap_idle
    ap_status_w[2] = (ap_start) ? 1'b0 : (done_r) ? 1'b1 : ap_status_r[2];
end

always @(*) begin
    state_w = state_r;
    data_ptr_w = data_ptr_r;
    coef_ptr_w = coef_ptr_r;
    tap_Do_w = tap_Do;
    data_Do_w = data_Do;
    accum_w = accum_r;
    done_w = 0;
    data_length_cnt_w = data_length_cnt_r;
    halt_w = 0;
    case (state_r) 
        RSET: begin
            data_ptr_w = (data_ptr_r == Tape_Num - 1'b1) ? 0 : (data_ptr_r + 1'b1);
            if ((data_ptr_r == Tape_Num - 1'b1) & y_buf_emp_r) begin
                state_w = WAIT;
                data_ptr_w = 0;
                if (!ap_status_r[2]) done_w = 1;
            end
        end
        WAIT: begin
            if (ap_status_r[0]) state_w = GETX;
        end
        GETX: begin
            if (!x_buf_emp_r) begin
                state_w = CALC;
                data_ptr_w = (data_ptr_r == Tape_Num - 1'b1) ? 0 : (data_ptr_r + 1'b1);
                coef_ptr_w = (coef_ptr_r == 0) ? (Tape_Num - 1'b1) : (coef_ptr_r - 1'b1);
                accum_w = 0;
            end
        end
        CALC: begin
            data_ptr_w = (data_ptr_r == Tape_Num - 1'b1) ? 0 : (data_ptr_r + 1'b1);
            coef_ptr_w = (coef_ptr_r == 0) ? 0 : (coef_ptr_r - 1'b1);
            if (!halt_r) accum_w = $signed(accum_r) + $signed(product);
            if (coef_ptr_r == 0) begin
                if (y_buf_emp_r) begin
                    state_w = GETX;
                    data_length_cnt_w = data_length_cnt_r + 1'b1;
                    if (data_length_cnt_r == data_length_r - 1'b1) begin
                        state_w = RSET;
                        data_ptr_w = 0;
                        data_length_cnt_w = 0;
                    end
                end
                else begin
                    halt_w = 1;
                    data_ptr_w = data_ptr_r;
                end
            end 
        end
    endcase
end

always @(posedge axis_clk or negedge axis_rst_n) begin
	if (!axis_rst_n) begin
        awready_r <= 0;
        wready_r <= 0; 
        awflag_r <= 0;
        wflag_r <= 0;
        axi_lite_waddr_r <= 0;
        axi_lite_wdata_r <= 0;
        data_length_r <= 0;

        arready_r <= 1;
        axi_lite_raddr_r <= 0;
        rvalid_r <= 0;

        ss_tready_r <= 0;
        x_buf_emp_r <= 1;
        x_buf_r <= 0;

        sm_tvalid_r <= 0;
        sm_tlast_r <= 0;
        y_buf_emp_r <= 1;
        y_buf_r <= 0;

        ap_status_r <= 3'b100;
        ap_status_d1_r <= 0;

        state_r <= 0;
        data_ptr_r <= 0;
        coef_ptr_r <= 0;
        tap_Do_r <= 0;
        data_Do_r <= 0;
        accum_r <= 0;
        done_r <= 0;
        data_length_cnt_r <= 0;
        halt_r <= 0;
	end
	else begin
        awready_r <= awready_w;
        wready_r <= wready_w;
        awflag_r <= awflag_w;
        wflag_r <= wflag_w;
        axi_lite_waddr_r <= axi_lite_waddr_w;
        axi_lite_wdata_r <= axi_lite_wdata_w;
        data_length_r <= data_length_w;

        arready_r <= arready_w;
        axi_lite_raddr_r <= axi_lite_raddr_w;
        rvalid_r <= rvalid_w;

        ss_tready_r <= ss_tready_w;
        x_buf_emp_r <= x_buf_emp_w;
        x_buf_r <= x_buf_w;

        sm_tvalid_r <= sm_tvalid_w;
        sm_tlast_r <= sm_tlast_w;
        y_buf_emp_r <= y_buf_emp_w;
        y_buf_r <= y_buf_w;

        ap_status_r <= ap_status_w;
        ap_status_d1_r <= ap_status_r;

        state_r <= state_w;
        data_ptr_r <= data_ptr_w;
        coef_ptr_r <= coef_ptr_w;
        tap_Do_r <= tap_Do_w;
        data_Do_r <= data_Do_w;
        accum_r <= accum_w;
        done_r <= done_w;
        data_length_cnt_r <= data_length_cnt_w;
        halt_r <= halt_w;
	end
end
endmodule