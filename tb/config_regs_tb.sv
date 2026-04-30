`timescale 1ns/1ps

module config_regs_tb;

    localparam time CLK_PERIOD = 20ns;
    localparam time SPI_HALF_PERIOD = 250ns;

    localparam logic [15:0] DEFAULT_THRESHOLD = 16'd2560;
    localparam logic [15:0] DEFAULT_COEFF0    = 16'sd112;
    localparam logic [15:0] DEFAULT_COEFF1    = 16'sd243;
    localparam logic [15:0] DEFAULT_COEFF2    = 16'sd618;
    localparam logic [15:0] DEFAULT_COEFF3    = 16'sd1293;
    localparam logic [15:0] DEFAULT_COEFF4    = 16'sd2217;
    localparam logic [15:0] DEFAULT_COEFF5    = 16'sd3225;
    localparam logic [15:0] DEFAULT_COEFF6    = 16'sd4089;
    localparam logic [15:0] DEFAULT_COEFF7    = 16'sd4587;

    logic        clk;
    logic        rst_n;
    logic        spi_csn;
    logic        spi_sck;
    logic        spi_mosi;
    logic [15:0] threshold;
    logic [15:0] coeff0;
    logic [15:0] coeff1;
    logic [15:0] coeff2;
    logic [15:0] coeff3;
    logic [15:0] coeff4;
    logic [15:0] coeff5;
    logic [15:0] coeff6;
    logic [15:0] coeff7;

    int unsigned failures;
    int unsigned test_id;
    int unsigned failures_before_test;

    config_regs #(
        .DEFAULT_THRESHOLD(DEFAULT_THRESHOLD),
        .DEFAULT_COEFF0(DEFAULT_COEFF0),
        .DEFAULT_COEFF1(DEFAULT_COEFF1),
        .DEFAULT_COEFF2(DEFAULT_COEFF2),
        .DEFAULT_COEFF3(DEFAULT_COEFF3),
        .DEFAULT_COEFF4(DEFAULT_COEFF4),
        .DEFAULT_COEFF5(DEFAULT_COEFF5),
        .DEFAULT_COEFF6(DEFAULT_COEFF6),
        .DEFAULT_COEFF7(DEFAULT_COEFF7)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .spi_csn(spi_csn),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .threshold(threshold),
        .coeff0(coeff0),
        .coeff1(coeff1),
        .coeff2(coeff2),
        .coeff3(coeff3),
        .coeff4(coeff4),
        .coeff5(coeff5),
        .coeff6(coeff6),
        .coeff7(coeff7)
    );

    always #(CLK_PERIOD / 2) clk = ~clk;

    // ----------------------------------------------------------------
    // Helper tasks
    // ----------------------------------------------------------------
    task automatic wait_clocks(input integer cycles);
        begin
            repeat (cycles) @(posedge clk);
        end
    endtask

    task automatic apply_reset;
        begin
            rst_n    = 1'b0;
            spi_csn  = 1'b1;
            spi_sck  = 1'b0;
            spi_mosi = 1'b0;
            #(CLK_PERIOD + (CLK_PERIOD / 3));
            rst_n = 1'b1;
            wait_clocks(6);
        end
    endtask

    task automatic spi_clock_bit(input logic bit_value);
        begin
            spi_mosi = bit_value;
            #(SPI_HALF_PERIOD);
            spi_sck = 1'b1;
            #(SPI_HALF_PERIOD);
            spi_sck = 1'b0;
            #(SPI_HALF_PERIOD);
        end
    endtask

    task automatic spi_write_frame(input logic [7:0] addr, input logic [15:0] data);
        logic [23:0] frame;
        int bit_idx;
        begin
            frame = {addr, data};
            spi_csn = 1'b0;
            #(SPI_HALF_PERIOD);

            for (bit_idx = 23; bit_idx >= 0; bit_idx--) begin
                spi_clock_bit(frame[bit_idx]);
            end

            spi_csn = 1'b1;
            spi_mosi = 1'b0;
            #(SPI_HALF_PERIOD);
            wait_clocks(4);
        end
    endtask

    task automatic spi_write_partial(input logic [23:0] frame, input integer bits_to_send);
        int bit_idx;
        begin
            spi_csn = 1'b0;
            #(SPI_HALF_PERIOD);

            for (bit_idx = 23; bit_idx >= (24 - bits_to_send); bit_idx--) begin
                spi_clock_bit(frame[bit_idx]);
            end

            spi_csn = 1'b1;
            spi_mosi = 1'b0;
            #(SPI_HALF_PERIOD);
            wait_clocks(4);
        end
    endtask

    // Clock 24 raw bits with CSN held low — used for back-to-back frames
    task automatic spi_clock_raw_frame(input logic [23:0] frame);
        int bit_idx;
        begin
            for (bit_idx = 23; bit_idx >= 0; bit_idx--) begin
                spi_clock_bit(frame[bit_idx]);
            end
        end
    endtask

    task automatic check_reg(input [127:0] name, input logic [15:0] actual, input logic [15:0] expected);
        begin
            if (actual !== expected) begin
                failures = failures + 1'b1;
                $display("[%0t] ERROR: test %0d %0s expected 0x%04h, got 0x%04h",
                         $time, test_id, name, expected, actual);
            end
        end
    endtask

    task automatic check_defaults;
        begin
            check_reg("threshold", threshold, DEFAULT_THRESHOLD);
            check_reg("coeff0",    coeff0,    DEFAULT_COEFF0);
            check_reg("coeff1",    coeff1,    DEFAULT_COEFF1);
            check_reg("coeff2",    coeff2,    DEFAULT_COEFF2);
            check_reg("coeff3",    coeff3,    DEFAULT_COEFF3);
            check_reg("coeff4",    coeff4,    DEFAULT_COEFF4);
            check_reg("coeff5",    coeff5,    DEFAULT_COEFF5);
            check_reg("coeff6",    coeff6,    DEFAULT_COEFF6);
            check_reg("coeff7",    coeff7,    DEFAULT_COEFF7);
        end
    endtask

    task automatic check_all_coeffs(
        input logic [15:0] exp0, input logic [15:0] exp1,
        input logic [15:0] exp2, input logic [15:0] exp3,
        input logic [15:0] exp4, input logic [15:0] exp5,
        input logic [15:0] exp6, input logic [15:0] exp7
    );
        begin
            check_reg("coeff0", coeff0, exp0);
            check_reg("coeff1", coeff1, exp1);
            check_reg("coeff2", coeff2, exp2);
            check_reg("coeff3", coeff3, exp3);
            check_reg("coeff4", coeff4, exp4);
            check_reg("coeff5", coeff5, exp5);
            check_reg("coeff6", coeff6, exp6);
            check_reg("coeff7", coeff7, exp7);
        end
    endtask

    task automatic report_test_result;
        begin
            if (failures == failures_before_test) begin
                $display("[%0t] TESTCASE %0d PASS", $time, test_id);
            end else begin
                $display("[%0t] TESTCASE %0d FAIL", $time, test_id);
            end
        end
    endtask

    // ================================================================
    // MAIN STIMULUS
    // ================================================================
    initial begin
        clk      = 1'b0;
        rst_n    = 1'b1;
        spi_csn  = 1'b1;
        spi_sck  = 1'b0;
        spi_mosi = 1'b0;
        failures = 0;
        test_id  = 0;
        failures_before_test = 0;

        apply_reset();

        // ============================================================
        // TEST 1: Reset defaults — all registers at power-on values
        // ============================================================
        test_id = 1;
        failures_before_test = failures;
        $display("[%0t] TESTCASE %0d START: reset defaults", $time, test_id);
        $display("[%0t] TESTCASE %0d INPUTS: rst_n asserted low, SPI idle with csn=1 sck=0 mosi=0",
                 $time, test_id);
        $display("[%0t] TESTCASE %0d STIMULUS DONE: checking default threshold and coefficients",
                 $time, test_id);
        check_defaults();
        report_test_result();

        // ============================================================
        // TEST 2: Single threshold write — only threshold changes
        // ============================================================
        test_id = 2;
        failures_before_test = failures;
        $display("[%0t] TESTCASE %0d START: threshold write", $time, test_id);
        $display("[%0t] TESTCASE %0d INPUTS: SPI frame addr=0x00 data=0x0A00",
                 $time, test_id);
        spi_write_frame(8'h00, 16'h0A00);
        $display("[%0t] TESTCASE %0d STIMULUS DONE: checking threshold update only",
                 $time, test_id);
        check_reg("threshold", threshold, 16'h0A00);
        check_all_coeffs(
            DEFAULT_COEFF0, DEFAULT_COEFF1, DEFAULT_COEFF2, DEFAULT_COEFF3,
            DEFAULT_COEFF4, DEFAULT_COEFF5, DEFAULT_COEFF6, DEFAULT_COEFF7
        );
        report_test_result();

        // ============================================================
        // TEST 3: Write first and last coefficient
        // ============================================================
        test_id = 3;
        failures_before_test = failures;
        $display("[%0t] TESTCASE %0d START: coefficient register writes", $time, test_id);
        $display("[%0t] TESTCASE %0d INPUTS: write addr=0x01 data=0x1111 and addr=0x08 data=0x8888",
                 $time, test_id);
        spi_write_frame(8'h01, 16'h1111);
        spi_write_frame(8'h08, 16'h8888);
        $display("[%0t] TESTCASE %0d STIMULUS DONE: checking coeff0 and coeff7 updates",
                 $time, test_id);
        check_reg("threshold", threshold, 16'h0A00);
        check_reg("coeff0", coeff0, 16'h1111);
        check_reg("coeff1", coeff1, DEFAULT_COEFF1);
        check_reg("coeff7", coeff7, 16'h8888);
        report_test_result();

        // ============================================================
        // TEST 4: Write all middle coefficients
        // ============================================================
        test_id = 4;
        failures_before_test = failures;
        $display("[%0t] TESTCASE %0d START: all middle coefficient writes", $time, test_id);
        $display("[%0t] TESTCASE %0d INPUTS: write coeff1 through coeff6 using addresses 0x02 through 0x07",
                 $time, test_id);
        spi_write_frame(8'h02, 16'h2222);
        spi_write_frame(8'h03, 16'h3333);
        spi_write_frame(8'h04, 16'h4444);
        spi_write_frame(8'h05, 16'h5555);
        spi_write_frame(8'h06, 16'h6666);
        spi_write_frame(8'h07, 16'h7777);
        $display("[%0t] TESTCASE %0d STIMULUS DONE: checking every coefficient register",
                 $time, test_id);
        check_all_coeffs(
            16'h1111, 16'h2222, 16'h3333, 16'h4444,
            16'h5555, 16'h6666, 16'h7777, 16'h8888
        );
        report_test_result();

        // ============================================================
        // TEST 5: Invalid address — no register changes
        // ============================================================
        test_id = 5;
        failures_before_test = failures;
        $display("[%0t] TESTCASE %0d START: invalid address ignored", $time, test_id);
        $display("[%0t] TESTCASE %0d INPUTS: SPI frame addr=0x09 data=0xDEAD",
                 $time, test_id);
        spi_write_frame(8'h09, 16'hDEAD);
        $display("[%0t] TESTCASE %0d STIMULUS DONE: checking no register changed",
                 $time, test_id);
        check_reg("threshold", threshold, 16'h0A00);
        check_all_coeffs(
            16'h1111, 16'h2222, 16'h3333, 16'h4444,
            16'h5555, 16'h6666, 16'h7777, 16'h8888
        );
        report_test_result();

        // ============================================================
        // TEST 6: Partial frame discarded by CSN going high
        // ============================================================
        test_id = 6;
        failures_before_test = failures;
        $display("[%0t] TESTCASE %0d START: partial frame discarded by csn high",
                 $time, test_id);
        $display("[%0t] TESTCASE %0d INPUTS: send only 12 bits of addr=0x00 data=0x1234, then deassert csn",
                 $time, test_id);
        spi_write_partial(24'h00_1234, 12);
        $display("[%0t] TESTCASE %0d STIMULUS DONE: checking threshold did not update",
                 $time, test_id);
        check_reg("threshold", threshold, 16'h0A00);
        report_test_result();

        // ============================================================
        // TEST 7: Full frame after partial — recovery works
        // ============================================================
        test_id = 7;
        failures_before_test = failures;
        $display("[%0t] TESTCASE %0d START: new full frame after partial frame",
                 $time, test_id);
        $display("[%0t] TESTCASE %0d INPUTS: SPI frame addr=0x00 data=0x3456",
                 $time, test_id);
        spi_write_frame(8'h00, 16'h3456);
        $display("[%0t] TESTCASE %0d STIMULUS DONE: checking receiver recovered and threshold updated",
                 $time, test_id);
        check_reg("threshold", threshold, 16'h3456);
        report_test_result();

        // ============================================================
        // TEST 8: Reset restores defaults after writes
        // ============================================================
        test_id = 8;
        failures_before_test = failures;
        $display("[%0t] TESTCASE %0d START: reset restores defaults after writes",
                 $time, test_id);
        $display("[%0t] TESTCASE %0d INPUTS: assert reset after several successful SPI writes",
                 $time, test_id);
        apply_reset();
        $display("[%0t] TESTCASE %0d STIMULUS DONE: checking all registers returned to defaults",
                 $time, test_id);
        check_defaults();
        report_test_result();

        // ============================================================
        // TEST 9: Back-to-back frames without CSN toggle
        // Two 24-bit frames clocked with CSN held low continuously.
        // bit_cnt wraps after 24 bits, so both writes must land.
        // ============================================================
        test_id = 9;
        failures_before_test = failures;
        $display("[%0t] TESTCASE %0d START: back-to-back frames without CSN toggle",
                 $time, test_id);
        $display("[%0t] TESTCASE %0d INPUTS: CSN held low, 48 bits: addr=0x00 data=0x1234 then addr=0x01 data=0xABCD",
                 $time, test_id);

        spi_csn = 1'b0;
        #(SPI_HALF_PERIOD);

        spi_clock_raw_frame({8'h00, 16'h1234});  // threshold = 0x1234
        spi_clock_raw_frame({8'h01, 16'hABCD});  // coeff0   = 0xABCD

        spi_csn  = 1'b1;
        spi_mosi = 1'b0;
        #(SPI_HALF_PERIOD);
        wait_clocks(4);

        $display("[%0t] TESTCASE %0d STIMULUS DONE: checking both writes landed",
                 $time, test_id);
        check_reg("threshold", threshold, 16'h1234);
        check_reg("coeff0",    coeff0,    16'hABCD);
        report_test_result();

        // ============================================================
        // TEST 10: Overwrite same register twice — last write wins
        // ============================================================
        test_id = 10;
        failures_before_test = failures;
        $display("[%0t] TESTCASE %0d START: double write to same register", $time, test_id);
        $display("[%0t] TESTCASE %0d INPUTS: write threshold=0xAAAA then threshold=0xBBBB",
                 $time, test_id);
        spi_write_frame(8'h00, 16'hAAAA);
        spi_write_frame(8'h00, 16'hBBBB);
        $display("[%0t] TESTCASE %0d STIMULUS DONE: checking last write wins",
                 $time, test_id);
        check_reg("threshold", threshold, 16'hBBBB);
        report_test_result();

        // ============================================================
        // TEST 11: Maximum address 0xFF — must be ignored
        // ============================================================
        test_id = 11;
        failures_before_test = failures;
        $display("[%0t] TESTCASE %0d START: max address 0xFF ignored", $time, test_id);
        $display("[%0t] TESTCASE %0d INPUTS: SPI frame addr=0xFF data=0xFFFF",
                 $time, test_id);
        spi_write_frame(8'hFF, 16'hFFFF);
        $display("[%0t] TESTCASE %0d STIMULUS DONE: checking no register changed",
                 $time, test_id);
        check_reg("threshold", threshold, 16'hBBBB);
        check_reg("coeff0",    coeff0,    16'hABCD);
        report_test_result();

        // ============================================================
        // TEST 12: Write all zeros to all registers
        // ============================================================
        test_id = 12;
        failures_before_test = failures;
        $display("[%0t] TESTCASE %0d START: write all zeros", $time, test_id);
        $display("[%0t] TESTCASE %0d INPUTS: write 0x0000 to threshold and all 8 coefficients",
                 $time, test_id);
        spi_write_frame(8'h00, 16'h0000);
        spi_write_frame(8'h01, 16'h0000);
        spi_write_frame(8'h02, 16'h0000);
        spi_write_frame(8'h03, 16'h0000);
        spi_write_frame(8'h04, 16'h0000);
        spi_write_frame(8'h05, 16'h0000);
        spi_write_frame(8'h06, 16'h0000);
        spi_write_frame(8'h07, 16'h0000);
        spi_write_frame(8'h08, 16'h0000);
        $display("[%0t] TESTCASE %0d STIMULUS DONE: checking all registers are zero",
                 $time, test_id);
        check_reg("threshold", threshold, 16'h0000);
        check_all_coeffs(
            16'h0000, 16'h0000, 16'h0000, 16'h0000,
            16'h0000, 16'h0000, 16'h0000, 16'h0000
        );
        report_test_result();

        // ============================================================
        // TEST 13: Write all ones to all registers
        // ============================================================
        test_id = 13;
        failures_before_test = failures;
        $display("[%0t] TESTCASE %0d START: write all ones", $time, test_id);
        $display("[%0t] TESTCASE %0d INPUTS: write 0xFFFF to threshold and all 8 coefficients",
                 $time, test_id);
        spi_write_frame(8'h00, 16'hFFFF);
        spi_write_frame(8'h01, 16'hFFFF);
        spi_write_frame(8'h02, 16'hFFFF);
        spi_write_frame(8'h03, 16'hFFFF);
        spi_write_frame(8'h04, 16'hFFFF);
        spi_write_frame(8'h05, 16'hFFFF);
        spi_write_frame(8'h06, 16'hFFFF);
        spi_write_frame(8'h07, 16'hFFFF);
        spi_write_frame(8'h08, 16'hFFFF);
        $display("[%0t] TESTCASE %0d STIMULUS DONE: checking all registers are 0xFFFF",
                 $time, test_id);
        check_reg("threshold", threshold, 16'hFFFF);
        check_all_coeffs(
            16'hFFFF, 16'hFFFF, 16'hFFFF, 16'hFFFF,
            16'hFFFF, 16'hFFFF, 16'hFFFF, 16'hFFFF
        );
        report_test_result();

        // ============================================================
        // TEST 14: Reset after all-ones — confirms defaults restored
        // ============================================================
        test_id = 14;
        failures_before_test = failures;
        $display("[%0t] TESTCASE %0d START: final reset restores real filter defaults",
                 $time, test_id);
        $display("[%0t] TESTCASE %0d INPUTS: assert reset after all-ones write",
                 $time, test_id);
        apply_reset();
        $display("[%0t] TESTCASE %0d STIMULUS DONE: checking all registers returned to filter defaults",
                 $time, test_id);
        check_defaults();
        report_test_result();

        // ============================================================
        // FINAL REPORT
        // ============================================================
        $display("\n================================================");
        $display("  proxcore config_regs Verification");
        $display("================================================");
        $display("  Total failures: %0d", failures);
        if (failures == 0)
            $display("  RESULT: PASS — all %0d tests passed", test_id);
        else
            $display("  RESULT: FAIL — %0d error(s)", failures);
        $display("================================================\n");

        if (failures != 0) $fatal(1);
        $finish;
    end

    initial begin
        #(4ms);
        $display("[%0t] ERROR: simulation timeout", $time);
        $fatal(1);
    end

endmodule