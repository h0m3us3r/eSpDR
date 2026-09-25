// DDR3 memory: the MIG core configured by ip/mig.prj (DDR3L at 666.7 MT/s,
// 128-bit user interface at 83.3 MHz). Its 166.7 MHz input clock comes from
// a separate MMCM, so recalibrating DDR never disturbs the ESP reference.
module ddr_memory (
    input  wire         oscillator,     // 100 MHz, from an IBUF
    input  wire         reference_200,
    input  wire         reset,
    output wire         ui_clk,
    output wire         ui_reset,
    output wire         calibrated,

    input  wire [27:0]  app_addr,
    input  wire [2:0]   app_cmd,
    input  wire         app_en,
    input  wire [127:0] app_wdf_data,
    input  wire [15:0]  app_wdf_mask,
    input  wire         app_wdf_end,
    input  wire         app_wdf_wren,
    output wire         app_rdy,
    output wire         app_wdf_rdy,
    output wire [127:0] app_rd_data,
    output wire         app_rd_data_valid,

    inout  wire [15:0]  ddr3_dq,
    inout  wire [1:0]   ddr3_dqs_n,
    inout  wire [1:0]   ddr3_dqs_p,
    output wire [13:0]  ddr3_addr,
    output wire [2:0]   ddr3_ba,
    output wire         ddr3_ras_n,
    output wire         ddr3_cas_n,
    output wire         ddr3_we_n,
    output wire         ddr3_reset_n,
    output wire [0:0]   ddr3_ck_p,
    output wire [0:0]   ddr3_ck_n,
    output wire [0:0]   ddr3_cke,
    output wire [0:0]   ddr3_cs_n,
    output wire [0:0]   ddr3_odt,
    output wire [1:0]   ddr3_dm
);
    // A global buffer: the oscillator IBUF cannot also drive two MMCMs directly.
    wire oscillator_global, feedback, feedback_buffered, clock_166, mig_clock, locked;
    BUFG oscillator_buffer (.I(oscillator), .O(oscillator_global));
    MMCME2_BASE #(
        .CLKIN1_PERIOD(10.0), .CLKFBOUT_MULT_F(10.0), .CLKOUT0_DIVIDE_F(6.0), .DIVCLK_DIVIDE(1)
    ) input_clock (
        .CLKIN1(oscillator_global), .CLKFBIN(feedback_buffered), .CLKFBOUT(feedback),
        .CLKOUT0(clock_166), .LOCKED(locked), .PWRDWN(1'b0), .RST(1'b0));
    BUFG feedback_buffer (.I(feedback), .O(feedback_buffered));
    BUFG mig_buffer (.I(clock_166), .O(mig_clock));

    mig_7series_0 mig (
        .ddr3_dq(ddr3_dq), .ddr3_dqs_n(ddr3_dqs_n), .ddr3_dqs_p(ddr3_dqs_p),
        .ddr3_addr(ddr3_addr), .ddr3_ba(ddr3_ba), .ddr3_ras_n(ddr3_ras_n),
        .ddr3_cas_n(ddr3_cas_n), .ddr3_we_n(ddr3_we_n), .ddr3_reset_n(ddr3_reset_n),
        .ddr3_ck_p(ddr3_ck_p), .ddr3_ck_n(ddr3_ck_n), .ddr3_cke(ddr3_cke),
        .ddr3_cs_n(ddr3_cs_n), .ddr3_dm(ddr3_dm), .ddr3_odt(ddr3_odt),
        .sys_clk_i(mig_clock), .clk_ref_i(reference_200), .sys_rst(reset || !locked),
        .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en),
        .app_wdf_data(app_wdf_data), .app_wdf_end(app_wdf_end), .app_wdf_mask(app_wdf_mask),
        .app_wdf_wren(app_wdf_wren), .app_rd_data(app_rd_data),
        .app_rd_data_valid(app_rd_data_valid), .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy),
        .app_sr_req(1'b0), .app_ref_req(1'b0), .app_zq_req(1'b0),
        .ui_clk(ui_clk), .ui_clk_sync_rst(ui_reset), .init_calib_complete(calibrated));
endmodule
