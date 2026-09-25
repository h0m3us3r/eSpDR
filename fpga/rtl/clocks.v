// Receiver clocks from the board's 100 MHz oscillator, all from one MMCM
// (VCO 1200 MHz) so the ESP's reference and the link sampler share a source:
//   sample_clk   240 MHz, fine phase-shifted to `target` steps (1/56 of the
//                VCO period each, 280 steps per sample period)
//   process_clk  120 MHz
//   delay_clk    200 MHz (IDELAYCTRL and MIG reference)
//   esp_clk      40 MHz ESP32-S3 reference
// After lock the phase walks one step at a time to the target, in either
// direction; phase_ready reports that it has arrived.
module clocks (
    input  wire       oscillator,
    input  wire [8:0] target,           // process_clk domain, changed only while idle
    output wire       sample_clk,
    output wire       process_clk,
    output wire       delay_clk,
    output wire       esp_clk,
    output wire       locked,
    output wire       phase_ready,      // process_clk domain
    output wire [8:0] phase             // process_clk domain
);
    wire feedback, feedback_buffered, c240, c120, c200, c40, control_clk, phase_done;
    BUFG feedback_buffer (.I(feedback), .O(feedback_buffered));
    BUFG sample_buffer (.I(c240), .O(sample_clk));
    BUFG process_buffer (.I(c120), .O(process_clk));
    BUFG delay_buffer (.I(c200), .O(delay_clk));
    BUFG esp_buffer (.I(c40), .O(esp_clk));
    BUFG control_buffer (.I(oscillator), .O(control_clk));

    (* ASYNC_REG = "TRUE" *) reg [8:0] target_meta = 0, target_sync = 0;
    reg [8:0] current = 0;
    reg       phase_enable = 0, increment = 0, waiting = 0;
    always @(posedge control_clk) begin
        target_meta <= target;
        target_sync <= target_meta;
        phase_enable <= 0;
        if (!locked) begin
            current <= 0;
            waiting <= 0;
        end else if (waiting) begin
            if (phase_done) begin
                current <= increment ? current + 1'b1 : current - 1'b1;
                waiting <= 0;
            end
        end else if (current != target_sync) begin
            phase_enable <= 1;
            increment <= current < target_sync;
            waiting <= 1;
        end
    end

    MMCME2_ADV #(
        .CLKIN1_PERIOD(10.0), .CLKFBOUT_MULT_F(12.0),
        .CLKOUT0_DIVIDE_F(5.0), .CLKOUT0_USE_FINE_PS("TRUE"),
        .CLKOUT1_DIVIDE(10), .CLKOUT2_DIVIDE(6), .CLKOUT3_DIVIDE(30)
    ) mmcm (
        .CLKIN1(oscillator), .CLKIN2(1'b0), .CLKINSEL(1'b1),
        .CLKFBIN(feedback_buffered), .CLKFBOUT(feedback),
        .CLKOUT0(c240), .CLKOUT1(c120), .CLKOUT2(c200), .CLKOUT3(c40),
        .LOCKED(locked), .RST(1'b0), .PWRDWN(1'b0),
        .PSCLK(control_clk), .PSEN(phase_enable), .PSINCDEC(increment), .PSDONE(phase_done),
        .DADDR(7'b0), .DCLK(1'b0), .DEN(1'b0), .DI(16'b0), .DWE(1'b0));

    // Status back to the processing clock (read only when settled).
    wire settled = locked && !waiting && current == target_sync && target_sync == target_meta;
    (* ASYNC_REG = "TRUE" *) reg [9:0] status_meta = 0, status_sync = 0;
    always @(posedge process_clk) begin
        status_meta <= {settled, current};
        status_sync <= status_meta;
    end
    assign phase_ready = status_sync[9] && status_sync[8:0] == target;
    assign phase = status_sync[8:0];
endmodule
