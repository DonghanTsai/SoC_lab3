`timescale 1ns / 1ps
module fir_tb
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11,
    parameter Data_Num    = 600
)();
    wire                        awready;
    wire                        wready;
    reg                         awvalid;
    reg   [(pADDR_WIDTH-1): 0]  awaddr;
    reg                         wvalid;
    reg signed [(pDATA_WIDTH-1) : 0] wdata;
    wire                        arready;
    reg                         rready;
    reg                         arvalid;
    reg         [(pADDR_WIDTH-1): 0] araddr;
    wire                        rvalid;
    wire signed [(pDATA_WIDTH-1): 0] rdata;
    reg                         ss_tvalid;
    reg signed [(pDATA_WIDTH-1) : 0] ss_tdata;
    reg                         ss_tlast;
    wire                        ss_tready;
    reg                         sm_tready;
    wire                        sm_tvalid;
    wire signed [(pDATA_WIDTH-1) : 0] sm_tdata;
    wire                        sm_tlast;
    reg                         axis_clk;
    reg                         axis_rst_n;

// ram for tap
    wire [3:0]               tap_WE;
    wire                     tap_EN;
    wire [(pDATA_WIDTH-1):0] tap_Di;
    wire [(pADDR_WIDTH-1):0] tap_A;
    wire [(pDATA_WIDTH-1):0] tap_Do;

// ram for data RAM
    wire [3:0]               data_WE;
    wire                     data_EN;
    wire [(pDATA_WIDTH-1):0] data_Di;
    wire [(pADDR_WIDTH-1):0] data_A;
    wire [(pDATA_WIDTH-1):0] data_Do;

    fir fir_DUT(
        .awready(awready),
        .wready(wready),
        .awvalid(awvalid),
        .awaddr(awaddr),
        .wvalid(wvalid),
        .wdata(wdata),
        .arready(arready),
        .rready(rready),
        .arvalid(arvalid),
        .araddr(araddr),
        .rvalid(rvalid),
        .rdata(rdata),
        .ss_tvalid(ss_tvalid),
        .ss_tdata(ss_tdata),
        .ss_tlast(ss_tlast),
        .ss_tready(ss_tready),
        .sm_tready(sm_tready),
        .sm_tvalid(sm_tvalid),
        .sm_tdata(sm_tdata),
        .sm_tlast(sm_tlast),

        // ram for tap
        .tap_WE(tap_WE),
        .tap_EN(tap_EN),
        .tap_Di(tap_Di),
        .tap_A(tap_A),
        .tap_Do(tap_Do),

        // ram for data
        .data_WE(data_WE),
        .data_EN(data_EN),
        .data_Di(data_Di),
        .data_A(data_A),
        .data_Do(data_Do),

        .axis_clk(axis_clk),
        .axis_rst_n(axis_rst_n)

        );
    
    // RAM for tap
    bram11 tap_RAM (
        .CLK(axis_clk),
        .WE(tap_WE),
        .EN(tap_EN),
        .Di(tap_Di),
        .A(tap_A),
        .Do(tap_Do)
    );

    // RAM for data: choose bram11 or bram12
    bram11 data_RAM(
        .CLK(axis_clk),
        .WE(data_WE),
        .EN(data_EN),
        .Di(data_Di),
        .A(data_A),
        .Do(data_Do)
    );

    reg signed [(pDATA_WIDTH-1):0] Din_list[0:(Data_Num-1)];
    reg signed [(pDATA_WIDTH-1):0] golden_list[0:(Data_Num-1)];

    //initial begin
    //    $dumpfile("fir.vcd");
    //    $dumpvars();
    //end
    //    initial begin
    //        $fsdbDumpfile("core.fsdb");
    //        $fsdbDumpvars(3, "+mda");
    //    end
    
    initial begin
        axis_clk = 0;
        forever begin
            #6 axis_clk = (~axis_clk);
        end
    end

    initial begin
        axis_rst_n = 0;
        @(posedge axis_clk); @(posedge axis_clk);
        axis_rst_n = 1;
    end

    reg [31:0]  data_length;
    integer Din, golden, input_data, golden_data, m;
    initial begin
        data_length = 0;
        Din = $fopen("./samples_triangular_wave.dat","r");
        golden = $fopen("./out_gold.dat","r");
        for(m=0;m<Data_Num;m=m+1) begin
            input_data = $fscanf(Din,"%d", Din_list[m]);
            golden_data = $fscanf(golden,"%d", golden_list[m]);
            data_length = data_length + 1;
        end
    end

    // fill in coef 
    reg signed [31:0] coef[0:10];
    initial begin
        coef[0]  =  32'd0;
        coef[1]  = -32'd10;
        coef[2]  = -32'd9;
        coef[3]  =  32'd23;
        coef[4]  =  32'd56;
        coef[5]  =  32'd63;
        coef[6]  =  32'd56;
        coef[7]  =  32'd23;
        coef[8]  = -32'd9;
        coef[9]  = -32'd10;
        coef[10] =  32'd0;
    end

    // Prevent hang
    integer timeout = (1000000);
    initial begin
        while(timeout > 0) begin
            @(posedge axis_clk);
            timeout = timeout - 1;
        end
        $display($time, "Simualtion Hang ....");
        $finish;
    end

    integer i, t;
    reg ap_start;
    initial begin
        // reset all signals
        reset_all;
        // wait for IDLE state
        wait_status(32'h0000_0004, 32'hffff_ffff);

        // send coefficient
        $display("----sending coefficient----");
        config_write(12'h10, data_length);
        for(i=0; i<Tape_Num; i=i+1) begin
            config_write(12'h20+4*i, coef[i]);
        end

        // read coefficient and check
        $display("----checking coefficient----");
        for(i=0; i<Tape_Num; i=i+1) begin
            config_read_check(12'h20+4*i, coef[i], 32'hffffffff);
        end

        for (t=0; t<3; t=t+1) begin
            // wait for IDLE state
            wait_status(32'h0000_0004, 32'hffff_ffff);
            $display(" Start FIR round %d", t);
            $display("Now time %0t ps", $time);
            ap_start <= 1;
            @(posedge axis_clk) config_write(12'h00, 32'h0000_0001);
            ap_start <= 0;
            // wait for DONE state
            wait_status(32'h0000_0002, 32'h0000_0002);
            $display("----get done signal----");
            $display("Now time %0t ps", $time);
            if (t==2) begin
                $display("**************************************");
                $display("*              All pass              *");
                $display("**************************************");
                $finish;
            end
        end
    end

    integer j;
    initial begin
        while (1) begin
            wait (ap_start);
            $display ("----start sending x----");
            ss_tvalid <= 0;
            for(j=0; j<(data_length-1); j=j+1) begin
                ss_tlast <= 0; ss(Din_list[j], j);
            end
            ss_tlast <= 1; ss(Din_list[(Data_Num-1)], data_length-1);
        end
    end

    integer k;
    initial begin
        while (1) begin
            wait (ap_start);
            $display("----start receiving y----");
            for(k=0; k<data_length; k=k+1) begin
                sm(golden_list[k],k);
            end
        end
    end


    task reset_all;
        begin
            awvalid <= 0;
            wvalid <= 0;
            awaddr <= 0;
            wdata <= 0;
            arvalid <= 0;
            araddr <= 0;
            rready <= 0;
            ss_tvalid <= 0;
            sm_tready <= 0;
        end
    endtask

    task wait_status;
        input [31:0] data;
        input [31:0] mask;
        begin
            arvalid <= 0;
            @(posedge axis_clk);
            arvalid <= 1; araddr <= 0; rready <= 1;
            @(posedge axis_clk);
            while ((!rvalid) || ((rdata & mask) !== (data & mask))) @(posedge axis_clk);
            arvalid <= 0; rready <= 0;
        end
    endtask

    task config_write;
        input [11:0]    addr;
        input [31:0]    data;
        begin
            awvalid <= 0; wvalid <= 0;
            @(posedge axis_clk);
            awvalid <= 1; awaddr <= addr;
            wvalid  <= 1; wdata <= data;
            @(posedge axis_clk);
            while (!wready) @(posedge axis_clk);
            awvalid <= 0; wvalid <= 0;
        end
    endtask

    task config_read_check;
        input [11:0]        addr;
        input signed [31:0] exp_data;
        input [31:0]        mask;                                                         
        begin
            arvalid <= 0;
            @(posedge axis_clk);
            arvalid <= 1; araddr <= addr; rready <= 1;
            @(posedge axis_clk);
            while (!rvalid) @(posedge axis_clk);
            if( (rdata & mask) !== (exp_data & mask)) begin
                $display("ERROR: exp = %d, rdata = %d", exp_data, rdata);
                $finish;
            end else begin
                $display("OK: exp = %d, rdata = %d", exp_data, rdata);
            end
            arvalid <= 0;
        end
    endtask

    integer n;
    task ss;
        input  signed [31:0] in1;
        input         [31:0] pcnt;
        begin
            ss_tvalid <= 1;
            ss_tdata  <= in1;
            @(posedge axis_clk);
            while (!ss_tready) @(posedge axis_clk);
            ss_tvalid <= 0;
            if (pcnt != data_length-1) for (n=0; n<16; n=n+1) @(posedge axis_clk);
        end
    endtask

    integer l;
    task sm;
        input  signed [31:0] in2; // golden data
        input         [31:0] pcnt; // pattern count
        begin
            @(posedge axis_clk);
            while(!sm_tvalid) @(posedge axis_clk);
            sm_tready <= 1;
            if (sm_tdata !== in2) begin
                $display("[ERROR] [Pattern %d] Golden answer: %d, Your answer: %d", pcnt, in2, sm_tdata);
                $finish;
            end
            else begin
                if (pcnt % 50 == 0) begin
                    $display("[PASS] [Pattern %d] Golden answer: %d, Your answer: %d", pcnt, in2, sm_tdata);
                end
            end
            @(posedge axis_clk);
            sm_tready <= 0;
            if (pcnt != data_length-1) for (l=0; l<16; l=l+1) @(posedge axis_clk);
        end
    endtask
endmodule

