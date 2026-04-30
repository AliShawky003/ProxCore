`timescale 1ns/1ps

module tb_threshold_fsm;

    localparam time   CLK_PERIOD = 20;        // 50 MHz
    localparam [15:0] THRESHOLD  = 16'd2560;  // 40m in Q10.6
    localparam [15:0] DIST_100M  = 16'd6400;  // clear track
    localparam [15:0] DIST_30M   = 16'd1920;  // real obstacle
    localparam [15:0] DIST_45M   = 16'd2880;  // just above threshold
    localparam [15:0] DIST_39M   = 16'd2496;  // 39m, just below threshold

    logic        clk;
    logic        rst_n;
    logic [15:0] filtered_in;
    logic        data_valid;
    logic [15:0] threshold;
    logic        brake_irq;

    integer      failures;
    integer      test_id;
    integer      failures_before_test;
    logic        brake_irq_d;

    threshold_fsm dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .filtered_in (filtered_in),
        .data_valid  (data_valid),
        .threshold   (threshold),
        .brake_irq   (brake_irq)
    );

    initial clk = 1'b0;
    always #(CLK_PERIOD / 2) clk = ~clk;

    // Pulse-width monitor — brake_irq must never be high two cycles running
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            brake_irq_d <= 1'b0;
        else begin
            if (brake_irq && brake_irq_d) begin
                failures = failures + 1;
                $display("[%0t] ERROR: brake_irq held high >1 cycle", $time);
            end
            brake_irq_d <= brake_irq;
        end
    end

    // Watchdog
    initial begin
        #(600 * CLK_PERIOD);
        $display("[%0t] FATAL: timeout", $time);
        $fatal(1);
    end

    // ----------------------------------------------------------------
    // Tasks — all stimulus driven on the NEGEDGE to avoid posedge races
    // Outputs checked mid-cycle (after negedge that follows the posedge)
    // ----------------------------------------------------------------

    task automatic apply_reset;
        begin
            rst_n       = 1'b0;
            filtered_in = DIST_100M;
            data_valid  = 1'b0;
            threshold   = THRESHOLD;
            repeat (4) @(posedge clk);
            @(negedge clk); rst_n = 1'b1;
            @(posedge clk); // one settled clock after reset
        end
    endtask

    // Drive one valid sample: set on negedge, captured on next posedge
    task automatic drive_sample;
        input [15:0] distance;
        begin
            @(negedge clk);
            filtered_in = distance;
            data_valid  = 1'b1;
            @(posedge clk); // DUT captures here
            @(negedge clk); // settle
            data_valid  = 1'b0;
            filtered_in = DIST_100M;
        end
    endtask

    // Drive N identical samples
    task automatic drive_n;
        input [15:0] distance;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1)
                drive_sample(distance);
        end
    endtask

    // Idle for N full clock cycles
    task automatic idle_cycles;
        input integer n;
        begin
            data_valid = 1'b0;
            repeat (n) @(posedge clk);
        end
    endtask

    // Check brake_irq mid-cycle (on negedge after a posedge event)
    // Returns the sampled value
    task automatic sample_irq;
        output logic val;
        begin
            @(negedge clk);
            val = brake_irq;
        end
    endtask

    // ================================================================
    // MAIN STIMULUS
    // ================================================================
    initial begin
        failures = 0;
        test_id  = 0;
        apply_reset();

        // ============================================================
        // TEST 1: Idle — no valid samples, brake_irq must stay low
        // ============================================================
        test_id = 1; failures_before_test = failures;
        $display("[%0t] TEST %0d START: idle, no samples", $time, test_id);

        idle_cycles(10);
        @(negedge clk);
        if (brake_irq !== 1'b0) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: brake_irq high during idle", $time, test_id);
        end

        $display("[%0t] TEST %0d %s", $time, test_id,
                 (failures==failures_before_test) ? "PASS" : "FAIL");

        // ============================================================
        // TEST 2: Clear track — 10 x 100m, no IRQ
        // ============================================================
        test_id = 2; failures_before_test = failures;
        apply_reset();
        $display("[%0t] TEST %0d START: clear track 10 x 100m", $time, test_id);

        drive_n(DIST_100M, 10);
        @(negedge clk);
        if (brake_irq !== 1'b0) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: brake_irq high on clear track", $time, test_id);
        end

        $display("[%0t] TEST %0d %s", $time, test_id,
                 (failures==failures_before_test) ? "PASS" : "FAIL");

        // ============================================================
        // TEST 3: One sub-threshold sample — must NOT fire
        // ============================================================
        test_id = 3; failures_before_test = failures;
        apply_reset();
        $display("[%0t] TEST %0d START: single sub-threshold must not fire", $time, test_id);

        drive_sample(DIST_30M);
        @(negedge clk);
        if (brake_irq !== 1'b0) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: fired after 1 sub-threshold", $time, test_id);
        end

        $display("[%0t] TEST %0d %s", $time, test_id,
                 (failures==failures_before_test) ? "PASS" : "FAIL");

        // ============================================================
        // TEST 4: Two sub-threshold samples — must NOT fire
        // ============================================================
        test_id = 4; failures_before_test = failures;
        apply_reset();
        $display("[%0t] TEST %0d START: two sub-threshold must not fire", $time, test_id);

        drive_sample(DIST_30M);
        drive_sample(DIST_30M);
        @(negedge clk);
        if (brake_irq !== 1'b0) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: fired after 2 sub-threshold", $time, test_id);
        end

        $display("[%0t] TEST %0d %s", $time, test_id,
                 (failures==failures_before_test) ? "PASS" : "FAIL");

        // ============================================================
        // TEST 5: Three consecutive sub-threshold — MUST fire as 1-cycle pulse
        // ============================================================
        test_id = 5; failures_before_test = failures;
        apply_reset();
        $display("[%0t] TEST %0d START: three consecutive fires single pulse", $time, test_id);

        drive_sample(DIST_30M);   // WARN1
        drive_sample(DIST_30M);   // WARN2
        drive_sample(DIST_30M);   // BRAKING — IRQ fires

        // Check: brake_irq must be high in the cycle after the 3rd posedge
        // drive_sample ends after @(negedge clk) following the posedge
        // so brake_irq is readable right now (mid-cycle)
        if (brake_irq !== 1'b1) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: brake_irq not high after 3rd sample negedge",
                     $time, test_id);
        end

        // Must deassert by next negedge
        @(posedge clk); @(negedge clk);
        if (brake_irq !== 1'b0) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: brake_irq did not deassert after 1 cycle",
                     $time, test_id);
        end

        $display("[%0t] TEST %0d %s", $time, test_id,
                 (failures==failures_before_test) ? "PASS" : "FAIL");

        // ============================================================
        // TEST 6: Debounce reset at WARN1
        // sub → clear → sub → sub — only 2 consecutive, must NOT fire
        // ============================================================
        test_id = 6; failures_before_test = failures;
        apply_reset();
        $display("[%0t] TEST %0d START: clear at WARN1 resets debounce", $time, test_id);

        drive_sample(DIST_30M);   // WARN1
        drive_sample(DIST_100M);  // CLEAR — resets counter
        drive_sample(DIST_30M);   // WARN1
        drive_sample(DIST_30M);   // WARN2
        @(negedge clk);
        if (brake_irq !== 1'b0) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: fired despite debounce reset at WARN1",
                     $time, test_id);
        end

        $display("[%0t] TEST %0d %s", $time, test_id,
                 (failures==failures_before_test) ? "PASS" : "FAIL");

        // ============================================================
        // TEST 7: Debounce reset at WARN2
        // sub → sub → clear → sub → sub — must NOT fire
        // ============================================================
        test_id = 7; failures_before_test = failures;
        apply_reset();
        $display("[%0t] TEST %0d START: clear at WARN2 resets debounce", $time, test_id);

        drive_sample(DIST_30M);   // WARN1
        drive_sample(DIST_30M);   // WARN2
        drive_sample(DIST_100M);  // CLEAR
        drive_sample(DIST_30M);   // WARN1
        drive_sample(DIST_30M);   // WARN2
        @(negedge clk);
        if (brake_irq !== 1'b0) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: fired despite debounce reset at WARN2",
                     $time, test_id);
        end

        $display("[%0t] TEST %0d %s", $time, test_id,
                 (failures==failures_before_test) ? "PASS" : "FAIL");

        // ============================================================
        // TEST 8: No repeated IRQ in BRAKING state
        // ============================================================
        test_id = 8; failures_before_test = failures;
        apply_reset();
        $display("[%0t] TEST %0d START: no repeated IRQ in BRAKING", $time, test_id);

        drive_sample(DIST_30M);
        drive_sample(DIST_30M);
        drive_sample(DIST_30M);   // enters BRAKING, IRQ fires once

        // 10 more sub-threshold — pulse monitor catches re-assertion
        drive_n(DIST_30M, 10);
        @(negedge clk);
        if (brake_irq !== 1'b0) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: brake_irq still high", $time, test_id);
        end

        $display("[%0t] TEST %0d %s", $time, test_id,
                 (failures==failures_before_test) ? "PASS" : "FAIL");

        // ============================================================
        // TEST 9: Recovery from BRAKING, then second obstacle detected
        // ============================================================
        test_id = 9; failures_before_test = failures;
        apply_reset();
        $display("[%0t] TEST %0d START: recovery and re-detection", $time, test_id);

        drive_sample(DIST_30M);
        drive_sample(DIST_30M);
        drive_sample(DIST_30M);   // BRAKING, IRQ fires

        drive_sample(DIST_100M);  // back to CLEAR

        drive_sample(DIST_30M);   // WARN1
        drive_sample(DIST_30M);   // WARN2
        drive_sample(DIST_30M);   // BRAKING again, IRQ fires

        if (brake_irq !== 1'b1) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: second obstacle not detected", $time, test_id);
        end

        $display("[%0t] TEST %0d %s", $time, test_id,
                 (failures==failures_before_test) ? "PASS" : "FAIL");

        // ============================================================
        // TEST 10: Boundary — exactly at threshold must NOT fire
        // ============================================================
        test_id = 10; failures_before_test = failures;
        apply_reset();
        $display("[%0t] TEST %0d START: value == threshold must not fire", $time, test_id);

        drive_n(THRESHOLD, 5);
        @(negedge clk);
        if (brake_irq !== 1'b0) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: fired at exact threshold", $time, test_id);
        end

        $display("[%0t] TEST %0d %s", $time, test_id,
                 (failures==failures_before_test) ? "PASS" : "FAIL");

        // ============================================================
        // TEST 11: Boundary — one below threshold MUST fire after 3
        // DIST_39M = 2496 (39m) < threshold (2560)
        // ============================================================
        test_id = 11; failures_before_test = failures;
        apply_reset();
        $display("[%0t] TEST %0d START: threshold-1 fires after 3 samples", $time, test_id);

        drive_sample(DIST_39M);
        drive_sample(DIST_39M);
        drive_sample(DIST_39M);   // must fire

        if (brake_irq !== 1'b1) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: did not fire at 39m", $time, test_id);
        end

        $display("[%0t] TEST %0d %s", $time, test_id,
                 (failures==failures_before_test) ? "PASS" : "FAIL");

        // ============================================================
        // TEST 12: data_valid gating — FSM frozen when data_valid=0
        // ============================================================
        test_id = 12; failures_before_test = failures;
        apply_reset();
        $display("[%0t] TEST %0d START: FSM frozen when data_valid low", $time, test_id);

        filtered_in = DIST_30M;
        data_valid  = 1'b0;
        repeat (20) @(posedge clk);
        @(negedge clk);

        if (brake_irq !== 1'b0) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: fired with data_valid low", $time, test_id);
        end

        filtered_in = DIST_100M;

        $display("[%0t] TEST %0d %s", $time, test_id,
                 (failures==failures_before_test) ? "PASS" : "FAIL");

        // ============================================================
        // TEST 13: Runtime threshold change
        // ============================================================
        test_id = 13; failures_before_test = failures;
        apply_reset();
        $display("[%0t] TEST %0d START: runtime threshold change", $time, test_id);

        // 45m is safe at 40m threshold
        drive_n(DIST_45M, 5);
        @(negedge clk);
        if (brake_irq !== 1'b0) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: fired at 45m with 40m threshold", $time, test_id);
        end

        // Raise threshold to 50m — now 45m should trigger
        @(negedge clk); threshold = 16'd3200;
        drive_sample(DIST_45M);  // WARN1 (2880 < 3200)
        drive_sample(DIST_45M);  // WARN2
        drive_sample(DIST_45M);  // BRAKING

        if (brake_irq !== 1'b1) begin
            failures = failures + 1;
            $display("[%0t] ERROR test %0d: did not fire after threshold raised", $time, test_id);
        end

        @(negedge clk); threshold = THRESHOLD;

        $display("[%0t] TEST %0d %s", $time, test_id,
                 (failures==failures_before_test) ? "PASS" : "FAIL");

        // ============================================================
        // FINAL REPORT
        // ============================================================
        $display("\n================================================");
        $display("  proxcore threshold_fsm Verification");
        $display("================================================");
        $display("  Total failures: %0d", failures);
        if (failures == 0)
            $display("  RESULT: PASS — all 13 tests passed");
        else
            $display("  RESULT: FAIL — %0d error(s)", failures);
        $display("================================================\n");

        if (failures != 0) $fatal(1);
        $finish;
    end

endmodule
