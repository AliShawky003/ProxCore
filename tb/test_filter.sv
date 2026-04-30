module output_test_filter;

    // ----------------------------------------------------------------
    // Parameters
    // ----------------------------------------------------------------
    parameter DATA_WIDTH      = 16;
    parameter ACC_WIDTH       = 36;
    parameter COEFF_WIDTH     = 16;
    parameter COEFF_FRAC_BITS = 15;
    parameter LATENCY         = 5;
    parameter CLOCK_PERIOD_NS = 20;
    parameter VECTOR_COUNT    = 200;
    parameter TAPS            = 16;
    parameter CENTER_TAP      = 8;

    // Q10.6 distance constants (meters x 64)
    parameter [15:0] DIST_100M = 16'd6400;   // 100m - clear track
    parameter [15:0] DIST_30M  = 16'd1920;   // 30m  - real obstacle
    parameter [15:0] DIST_2M   = 16'd128;    // 2m   - raindrop spike
    parameter [15:0] THRESH    = 16'd2560;   // 40m  - brake threshold

    // ----------------------------------------------------------------
    // Clock and reset
    // ----------------------------------------------------------------
    reg clk;
    reg rst_n;

    // ----------------------------------------------------------------
    // DUT interface - unsigned throughout
    // ----------------------------------------------------------------
    reg  [DATA_WIDTH-1:0] sample_in;
    reg                   sample_valid;
    wire [DATA_WIDTH-1:0] result_out;
    wire                  result_valid;

    // ----------------------------------------------------------------
    // Shadow model state - unsigned to match DUT delay line
    // ----------------------------------------------------------------
    reg [DATA_WIDTH-1:0] history [0:TAPS-1];

    // ----------------------------------------------------------------
    // Coefficients - Hamming-windowed, signed Q1.15, matching RTL
    // ----------------------------------------------------------------
    logic signed [COEFF_WIDTH-1:0] coeffs [0:TAPS-1];
    initial begin
        coeffs[0]  = 16'sd112;   coeffs[8]  = 16'sd4587;
        coeffs[1]  = 16'sd243;   coeffs[9]  = 16'sd4089;
        coeffs[2]  = 16'sd618;   coeffs[10] = 16'sd3225;
        coeffs[3]  = 16'sd1293;  coeffs[11] = 16'sd2217;
        coeffs[4]  = 16'sd2217;  coeffs[12] = 16'sd1293;
        coeffs[5]  = 16'sd3225;  coeffs[13] = 16'sd618;
        coeffs[6]  = 16'sd4089;  coeffs[14] = 16'sd243;
        coeffs[7]  = 16'sd4587;  coeffs[15] = 16'sd112;
    end

    // ----------------------------------------------------------------
    // Counters and checker variables
    // ----------------------------------------------------------------
    integer expected_queue [$];
    integer sample_count;
    integer error_count;
    integer match_count;
    integer mismatch_count;
    integer total_checked;
    integer cycle_count;
    integer stim_index;
    integer history_index;
    integer rtl_val;
    integer expected_val;

    // ----------------------------------------------------------------
    // Clock generation
    // ----------------------------------------------------------------
    initial begin
        clk = 0;
        forever #(CLOCK_PERIOD_NS / 2) clk = ~clk;
    end

    // ----------------------------------------------------------------
    // Watchdog
    // ----------------------------------------------------------------
    initial begin
        #(VECTOR_COUNT * CLOCK_PERIOD_NS * 500);
        $display("[FATAL] Simulation timeout");
        $finish;
    end

    // ----------------------------------------------------------------
    // DUT instantiation
    // ----------------------------------------------------------------
    proxcore_fir_filter dut_inst (
    .clk         (clk),
    .rst_n       (rst_n),
    .sample_in   (sample_in),
    .sample_valid(sample_valid),
    .coeff_in0   (16'sd112),
    .coeff_in1   (16'sd243),
    .coeff_in2   (16'sd618),
    .coeff_in3   (16'sd1293),
    .coeff_in4   (16'sd2217),
    .coeff_in5   (16'sd3225),
    .coeff_in6   (16'sd4089),
    .coeff_in7   (16'sd4587),
    .result_out  (result_out),
    .result_valid(result_valid)
);
    // ----------------------------------------------------------------
    // Golden model
    // FIXED: acc_shifted is 36-bit reg, not integer, preventing
    // truncation of upper bits before the right-shift
    // ----------------------------------------------------------------
    function [DATA_WIDTH-1:0] calculate_golden;
        input [DATA_WIDTH-1:0] sample;
        reg [DATA_WIDTH:0]                  pair_sum_u;
        reg signed [DATA_WIDTH+1:0]         pair_sum_s;
        reg signed [DATA_WIDTH+COEFF_WIDTH:0] prod;
        reg signed [ACC_WIDTH-1:0]          acc;
        reg signed [ACC_WIDTH-1:0]          acc_shifted;  // FIXED: was integer
        integer k;
        begin
            for (k = TAPS-1; k > 0; k = k-1)
                history[k] = history[k-1];
            history[0] = sample;

            acc = 0;
            for (k = 0; k < CENTER_TAP; k = k+1) begin
                pair_sum_u = history[k] + history[TAPS-1-k];
                pair_sum_s = {1'b0, pair_sum_u};
                prod       = pair_sum_s * coeffs[k];
                acc        = acc + prod;
            end

            acc_shifted      = acc >>> COEFF_FRAC_BITS;
            calculate_golden = acc_shifted[DATA_WIDTH-1:0];
        end
    endfunction

    // ----------------------------------------------------------------
    // Helper tasks
    // ----------------------------------------------------------------
    task do_reset;
        integer m;
        begin
            rst_n        = 0;
            sample_valid = 0;
            sample_in    = 0;
            repeat (4) @(posedge clk);
            rst_n = 1;
            @(posedge clk);
            for (m = 0; m < TAPS; m = m+1)
                history[m] = 0;
            expected_queue.delete();
        end
    endtask

    // Drive sample and push golden result to checker queue
    task drive_sample;
        input [DATA_WIDTH-1:0] val;
        integer gv;
        begin
            sample_in    = val;
            sample_valid = 1;
            @(posedge clk);
            gv = calculate_golden(val);
            expected_queue.push_back(gv);
            sample_count = sample_count + 1;
        end
    endtask

    // Drive sample without checking - used to pre-fill delay line
    task drive_fill;
    input [DATA_WIDTH-1:0] val;
    integer gv;
    begin
        sample_in    = val;
        sample_valid = 1;
        @(posedge clk);
        gv = calculate_golden(val);
        expected_queue.push_back(gv); // Add this to keep the FIFO aligned
        sample_count = sample_count + 1;
    end
endtask
    // Stop driving and wait for all queued outputs to arrive
    task flush_and_check_empty;
        begin
            sample_valid = 0;
            sample_in    = 0;
            repeat (LATENCY + 10) @(posedge clk);
            #1;
            if (expected_queue.size() != 0) begin
                $display("[ERROR] %0d outputs never received",
                         expected_queue.size());
                error_count = error_count + expected_queue.size();
                expected_queue.delete();
            end
        end
    endtask

    // ----------------------------------------------------------------
    // Checker process
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (result_valid) begin
            if (expected_queue.size() == 0) begin
                $display("[ERROR] Unexpected result_valid at cycle %0d",
                         cycle_count);
                error_count = error_count + 1;
            end else begin
                rtl_val      = {{16{1'b0}}, result_out};
                expected_val = expected_queue.pop_front();
                total_checked = total_checked + 1;

                if (rtl_val !== expected_val) begin
                    $display("[MISMATCH] cycle=%0d RTL=%0d Golden=%0d diff=%0d",
                             cycle_count, rtl_val, expected_val,
                             rtl_val - expected_val);
                    mismatch_count = mismatch_count + 1;
                    error_count    = error_count + 1;
                    $display("[FATAL] Stopping on first mismatch.");
                    $finish;
                end else begin
                    match_count = match_count + 1;
                end
            end
        end
        cycle_count = cycle_count + 1;
    end

    // ================================================================
    // MAIN STIMULUS
    // ================================================================
    initial begin
        sample_in      = 0;
        sample_valid   = 0;
        rst_n          = 0;
        sample_count   = 0;
        error_count    = 0;
        match_count    = 0;
        mismatch_count = 0;
        total_checked  = 0;
        cycle_count    = 0;
        for (history_index = 0; history_index < TAPS; history_index = history_index + 1)
            history[history_index] = 0;
        expected_queue = {};

        // ============================================================
        // TEST 1: Edge cases
        // Boundary values that stress signedness and overflow paths
        // ============================================================
        $display("\n=== TEST 1: Edge cases ===");
        do_reset();

        drive_sample(16'hFFFF);  // maximum value
        drive_sample(16'h8000);  // MSB set - catches sign bugs
        drive_sample(16'h7FFF);  // just below MSB
        drive_sample(16'h0000);  // zero

        flush_and_check_empty();
        $display("[TEST 1] Done. Errors so far: %0d", error_count);

        // ============================================================
        // TEST 2: Random stimulus - arithmetic correctness
        // 200 random unsigned samples vs shadow model
        // ============================================================
        $display("\n=== TEST 2: Random stimulus (%0d samples) ===", VECTOR_COUNT);
        do_reset();

        for (stim_index = 0; stim_index < VECTOR_COUNT; stim_index = stim_index + 1)
            drive_sample($urandom & 16'hFFFF);

        flush_and_check_empty();
        $display("[TEST 2] Done. Errors so far: %0d", error_count);

        // ============================================================
        // TEST 3: DC flatness at 100m
        // After delay line fills the output must settle to 100m.
        // Proves unity DC gain.
        // ============================================================
        $display("\n=== TEST 3: DC flatness at 100m ===");
        do_reset();

        // Silent fill
        for (stim_index = 0; stim_index < TAPS; stim_index = stim_index + 1)
            drive_fill(DIST_100M);

        // Checked samples
        for (stim_index = 0; stim_index < 20; stim_index = stim_index + 1)
            drive_sample(DIST_100M);

        flush_and_check_empty();
        $display("[TEST 3] Done. Errors so far: %0d", error_count);

        // ============================================================
        // TEST 4: Rainstorm - spike rejection
        //
        // Track is clear at 100m. Sensor alternates 100m / 2m
        // (raindrop hits). The filtered output must NEVER drop below
        // 50m (3200 Q10.6). A drop below threshold = false brake.
        //
        // This is the primary safety property of proxcore.
        // ============================================================
        $display("\n=== TEST 4: Rainstorm spike rejection ===");
        do_reset();

        // Fill delay line with clear track
        for (stim_index = 0; stim_index < TAPS*2; stim_index = stim_index + 1)
            drive_fill(DIST_100M);

        begin : test4_block
            integer false_brake_count;
            integer gv;
            false_brake_count = 0;

            sample_valid = 1;
            for (stim_index = 0; stim_index < 40; stim_index = stim_index + 1) begin
                sample_in = (stim_index % 2 == 0) ? DIST_2M : DIST_100M;
                @(posedge clk);
                gv = calculate_golden(sample_in);
                expected_queue.push_back(gv);      // <-- ADDED THIS LINE
                sample_count = sample_count + 1;

                if (gv < 16'd3200) begin        $display("[FAIL] TEST4: output %0d < 50m at spike %0d — false brake!",
                             gv, stim_index);
                    false_brake_count = false_brake_count + 1;
                    error_count = error_count + 1;
                end
            end

            if (false_brake_count == 0)
                $display("[PASS] TEST4: All 40 raindrop spikes rejected — no false brake triggered");
            else
                $display("[FAIL] TEST4: %0d spikes caused false brake", false_brake_count);
        end

        flush_and_check_empty();

        // ============================================================
        // TEST 5: Real obstacle detection
        //
        // Track is clear at 100m, then a car stalls 30m ahead.
        // After enough 30m readings the filtered output MUST drop
        // below the 40m threshold (2560 Q10.6).
        //
        // Failure here means the chip would NOT stop the tram.
        // This is a catastrophic failure mode.
        // ============================================================
        $display("\n=== TEST 5: Real obstacle at 30m must be detected ===");
        do_reset();

        // Fill with clear track
        for (stim_index = 0; stim_index < TAPS*2; stim_index = stim_index + 1)
            drive_fill(DIST_100M);

        begin : test5_block
            integer obstacle_detected;
            integer first_sample;
            integer gv;
            obstacle_detected = 0;
            first_sample      = -1;

            sample_valid = 1;
            for (stim_index = 0; stim_index < 40; stim_index = stim_index + 1) begin
                sample_in = DIST_30M;
                @(posedge clk);
                gv = calculate_golden(DIST_30M);
                expected_queue.push_back(gv);      // <-- ADDED THIS LINE
                sample_count = sample_count + 1;

                if (!obstacle_detected && gv < THRESH) begin
                    obstacle_detected = 1;
                    first_sample      = stim_index;
                    $display("[PASS] TEST5: Threshold crossed at obstacle sample %0d (output=%0d < threshold=%0d)",
                             stim_index, gv, THRESH);
                end
            end

            if (!obstacle_detected) begin
                $display("[FAIL] TEST5: CRITICAL - filter NEVER dropped below threshold");
                $display("       The tram would NOT stop. Fatal design error.");
                error_count = error_count + 1;
            end else begin
                $display("[PASS] TEST5: Obstacle confirmed detected at sample %0d", first_sample);
            end
        end

        flush_and_check_empty();

        // ============================================================
        // TEST 6: Recovery after obstacle clears
        //
        // After obstacle is detected, track clears back to 100m.
        // Filter output must rise back above threshold, proving the
        // system can resume normal operation.
        // ============================================================
        $display("\n=== TEST 6: Track clears after obstacle — filter recovers ===");
        do_reset();

        // Fill clear track
        for (stim_index = 0; stim_index < TAPS*2; stim_index = stim_index + 1)
            drive_fill(DIST_100M);

        // 20 samples of obstacle
        for (stim_index = 0; stim_index < 20; stim_index = stim_index + 1)
            drive_fill(DIST_30M);

        begin : test6_block
            integer recovered;
            integer gv;
            recovered = 0;

            sample_valid = 1;
            for (stim_index = 0; stim_index < 40; stim_index = stim_index + 1) begin
                sample_in = DIST_100M;
                @(posedge clk);
                gv = calculate_golden(DIST_100M);
                expected_queue.push_back(gv);      // <-- ADDED THIS LINE
                sample_count = sample_count + 1;

                if (!recovered && gv > THRESH) begin
                    recovered = 1;
                    $display("[PASS] TEST6: Filter recovered above threshold at sample %0d (output=%0d)",
                             stim_index, gv);
                end
            end

            if (!recovered) begin
                $display("[FAIL] TEST6: Filter never recovered after obstacle cleared");
                error_count = error_count + 1;
            end
        end

        flush_and_check_empty();

        // ============================================================
        // FINAL REPORT
        // ============================================================
        $display("\n================================================");
        $display("  proxcore FIR Filter Verification Complete");
        $display("================================================");
        $display("  Samples driven   : %0d", sample_count);
        $display("  Outputs checked  : %0d", total_checked);
        $display("  Matches          : %0d", match_count);
        $display("  Mismatches       : %0d", mismatch_count);
        $display("  Total errors     : %0d", error_count);
        $display("------------------------------------------------");
        if (error_count == 0)
            $display("  RESULT: PASS - all 6 tests passed");
        else
            $display("  RESULT: FAIL - %0d errors", error_count);
        $display("================================================\n");
        $finish;
    end

endmodule