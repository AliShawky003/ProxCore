`timescale 1ns/1ps

module proxcore_top_tb;

    // ----------------------------------------------------------------
    // Parameters
    // ----------------------------------------------------------------
    localparam int unsigned CLK_FREQ_HZ  = 50_000_000;
    localparam int unsigned BAUD_RATE    = 460_800;
    localparam time         CLK_PERIOD   = 20ns;

    localparam int unsigned CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;  // 108
    localparam time         BIT_PERIOD   = CLKS_PER_BIT * CLK_PERIOD; // 2160ns
    
    // Additional baud rate timing for TEST 7
    localparam int unsigned CLKS_PER_BIT_115200 = 434;
    localparam time         BIT_PERIOD_115200   = CLKS_PER_BIT_115200 * CLK_PERIOD; // 8680ns

    localparam int unsigned TAPS       = 16;
    localparam int unsigned LATENCY    = 5;
    localparam int unsigned FILL_COUNT = TAPS + LATENCY + 4; // 25 — fully primes pipeline

    // Distance constants (Q10.6)
    localparam [15:0] DIST_100M  = 16'd6400;  // 100m — clear track
    localparam [15:0] DIST_30M   = 16'd1920;  // 30m  — real obstacle
    localparam [15:0] DIST_2M    = 16'd128;   // 2m   — raindrop spike
    localparam [15:0] THRESH_20M = 16'd1280;  // 20m  — reconfigured threshold

    // SPI timing
    localparam time SPI_HALF_PERIOD = 250ns;

    // ----------------------------------------------------------------
    // DUT signals
    // ----------------------------------------------------------------
    logic        clk;
    logic        rst_n;
    logic        uart_rx;
    logic        spi_csn;
    logic        spi_sck;
    logic        spi_mosi;
    logic        brake_irq;
    logic [15:0] dbg_filtered_out;
    logic        dbg_filtered_valid;
    logic [15:0] dbg_raw_distance;
    logic        dbg_raw_valid;

    // ----------------------------------------------------------------
    // Test tracking
    // ----------------------------------------------------------------
    int unsigned failures;
    int unsigned test_id;
    int unsigned failures_before;
    int unsigned brake_count;

    // ----------------------------------------------------------------
    // DUT instantiation
    // ----------------------------------------------------------------
    proxcore_top #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .DEFAULT_THRESHOLD (16'd2560),
        .DEFAULT_BAUD_DIV (16'd108)
    ) dut (
        .clk                (clk),
        .rst_n              (rst_n),
        .uart_rx            (uart_rx),
        .spi_csn            (spi_csn),
        .spi_sck            (spi_sck),
        .spi_mosi           (spi_mosi),
        .brake_irq          (brake_irq),
        .dbg_filtered_out   (dbg_filtered_out),
        .dbg_filtered_valid (dbg_filtered_valid),
        .dbg_raw_distance   (dbg_raw_distance),
        .dbg_raw_valid      (dbg_raw_valid)
    );

    // ----------------------------------------------------------------
    // Clock generation
    // ----------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD / 2) clk = ~clk;

    // ----------------------------------------------------------------
    // brake_irq monitor — counts pulses on system clock
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (brake_irq && rst_n)
            brake_count = brake_count + 1;
    end

    // ----------------------------------------------------------------
    // Watchdog
    // ----------------------------------------------------------------
    initial begin
        #(50ms);
        $display("[FATAL] Simulation timeout");
        $fatal(1);
    end

    // ----------------------------------------------------------------
    // UART Tasks
    // ----------------------------------------------------------------
    task automatic uart_send_byte(input logic [7:0] data);
        integer i;
        begin
            // Start bit
            uart_rx = 1'b0;
            #(BIT_PERIOD);
            // Data bits (LSB first)
            for (i = 0; i < 8; i++) begin
                uart_rx = data[i];
                #(BIT_PERIOD);
            end
            // Stop bit
            uart_rx = 1'b1;
            #(BIT_PERIOD);
        end
    endtask

    // UART task variant with configurable bit period for TEST 7
    task automatic uart_send_byte_bp(input logic [7:0] data, input time bit_period);
        integer i;
        begin
            // Start bit
            uart_rx = 1'b0;
            #(bit_period);
            // Data bits (LSB first)
            for (i = 0; i < 8; i++) begin
                uart_rx = data[i];
                #(bit_period);
            end
            // Stop bit
            uart_rx = 1'b1;
            #(bit_period);
        end
    endtask

    task automatic uart_send_word(input logic [15:0] data);
        begin
            uart_send_byte(data[7:0]);   // low byte first
            uart_send_byte(data[15:8]);
        end
    endtask

    task automatic uart_send_word_bp(input logic [15:0] data, input time bit_period);
        begin
            uart_send_byte_bp(data[7:0], bit_period);   // low byte first
            uart_send_byte_bp(data[15:8], bit_period);
        end
    endtask

    task automatic send_n_samples(input logic [15:0] distance, input integer n);
        integer i;
        begin
            for (i = 0; i < n; i++)
                uart_send_word(distance);
        end
    endtask

    task automatic send_n_samples_bp(input logic [15:0] distance, input integer n, input time bit_period);
        integer i;
        begin
            for (i = 0; i < n; i++)
                uart_send_word_bp(distance, bit_period);
        end
    endtask

    task automatic send_alternating(
        input logic [15:0] dist_a,
        input logic [15:0] dist_b,
        input integer n
    );
        integer i;
        begin
            for (i = 0; i < n; i++) begin
                if (i % 2 == 0)
                    uart_send_word(dist_a);
                else
                    uart_send_word(dist_b);
            end
        end
    endtask

    // Wait for all pipeline stages to drain after last UART word
    task automatic wait_pipeline_drain;
        begin
            repeat (CLKS_PER_BIT * 20 + LATENCY + 10) @(posedge clk);
        end
    endtask

    // ----------------------------------------------------------------
    // SPI Tasks
    // ----------------------------------------------------------------
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

    task automatic spi_write(input logic [7:0] addr, input logic [15:0] data);
        logic [23:0] frame;
        integer i;
        begin
            frame = {addr, data};
            spi_csn = 1'b0;
            #(SPI_HALF_PERIOD);
            for (i = 23; i >= 0; i--)
                spi_clock_bit(frame[i]);
            spi_csn  = 1'b1;
            spi_mosi = 1'b0;
            #(SPI_HALF_PERIOD);
            repeat (4) @(posedge clk);
        end
    endtask

    // ----------------------------------------------------------------
    // Reset Task
    // ----------------------------------------------------------------
    task automatic apply_reset;
        begin
            rst_n    = 1'b0;
            uart_rx  = 1'b1;  // UART idle
            spi_csn  = 1'b1;
            spi_sck  = 1'b0;
            spi_mosi = 1'b0;
            repeat (4) @(posedge clk);
            rst_n = 1'b1;
            @(posedge clk);
        end
    endtask

    // Fill pipeline and wait for settling
    // NOTE: During fill, the FIR output starts small (delay line filling)
    // so the FSM may fire brake_irq during this transient. Callers must
    // reset brake_count AFTER fill_pipeline returns.
    task automatic fill_pipeline(input logic [15:0] distance);
        begin
            send_n_samples(distance, FILL_COUNT);
            wait_pipeline_drain();
        end
    endtask

    // Fill pipeline with custom bit period (for TEST 7)
    task automatic fill_pipeline_bp(input logic [15:0] distance, input time bit_period);
        begin
            send_n_samples_bp(distance, FILL_COUNT, bit_period);
            wait_pipeline_drain();
        end
    endtask

    // ----------------------------------------------------------------
    // Result reporting
    // ----------------------------------------------------------------
    task automatic report;
        begin
            if (failures == failures_before)
                $display("[%0t] TEST %0d PASS", $time, test_id);
            else
                $display("[%0t] TEST %0d FAIL", $time, test_id);
        end
    endtask

    // ================================================================
    // MAIN STIMULUS
    // ================================================================
    initial begin
        clk         = 1'b0;
        rst_n       = 1'b1;
        uart_rx     = 1'b1;
        spi_csn     = 1'b1;
        spi_sck     = 1'b0;
        spi_mosi    = 1'b0;
        failures    = 0;
        brake_count = 0;

        apply_reset();

        // ============================================================
        // TEST 1: Clear track — 100m, no false brakes
        // ============================================================
        test_id = 1;
        failures_before = failures;
        $display("[%0t] TEST %0d START: Clear track — no false brakes", $time, test_id);

        apply_reset();
        brake_count = 0;

        fill_pipeline(DIST_100M);
        brake_count = 0;

        send_n_samples(DIST_100M, 10);
        wait_pipeline_drain();

        if (brake_count != 0) begin
            failures++;
            $display("[%0t] ERROR test %0d: brake fired %0d time(s) on clear track",
                     $time, test_id, brake_count);
        end

        report();

        // ============================================================
        // TEST 2: Rainstorm — alternating 100m / 2m, no brake
        // ============================================================
        test_id = 2;
        failures_before = failures;
        $display("[%0t] TEST %0d START: Rainstorm spike rejection", $time, test_id);

        apply_reset();
        brake_count = 0;

        fill_pipeline(DIST_100M);
        brake_count = 0;

        send_alternating(DIST_2M, DIST_100M, 20);
        wait_pipeline_drain();

        if (brake_count != 0) begin
            failures++;
            $display("[%0t] ERROR test %0d: brake fired %0d time(s) during rainstorm",
                     $time, test_id, brake_count);
        end

        report();

        // ============================================================
        // TEST 3: Real obstacle at 30m — brake MUST fire
        // ============================================================
        test_id = 3;
        failures_before = failures;
        $display("[%0t] TEST %0d START: 30m obstacle detection", $time, test_id);

        apply_reset();
        brake_count = 0;

        fill_pipeline(DIST_100M);
        brake_count = 0;

        send_n_samples(DIST_30M, 25);
        wait_pipeline_drain();

        if (brake_count == 0) begin
            failures++;
            $display("[%0t] ERROR test %0d: brake NEVER fired — obstacle missed!", $time, test_id);
        end else begin
            $display("[%0t] INFO  test %0d: brake fired %0d time(s)", $time, test_id, brake_count);
        end

        report();

        // ============================================================
        // TEST 4: Recovery + re-detection
        // ============================================================
        test_id = 4;
        failures_before = failures;
        $display("[%0t] TEST %0d START: Recovery and second obstacle", $time, test_id);

        apply_reset();
        brake_count = 0;

        fill_pipeline(DIST_100M);
        brake_count = 0;

        send_n_samples(DIST_30M, 25);
        wait_pipeline_drain();

        if (brake_count == 0) begin
            failures++;
            $display("[%0t] ERROR test %0d: first obstacle not detected", $time, test_id);
        end else begin
            $display("[%0t] INFO  test %0d: first brake fired (%0d pulse(s))",
                     $time, test_id, brake_count);
        end

        send_n_samples(DIST_100M, FILL_COUNT);
        wait_pipeline_drain();

        brake_count = 0;

        send_n_samples(DIST_30M, 25);
        wait_pipeline_drain();

        if (brake_count == 0) begin
            failures++;
            $display("[%0t] ERROR test %0d: second obstacle not detected", $time, test_id);
        end else begin
            $display("[%0t] INFO  test %0d: second brake fired (%0d pulse(s))",
                     $time, test_id, brake_count);
        end

        report();

        // ============================================================
        // TEST 5: SPI threshold reconfiguration
        // ============================================================
        test_id = 5;
        failures_before = failures;
        $display("[%0t] TEST %0d START: SPI threshold reconfiguration", $time, test_id);

        apply_reset();
        brake_count = 0;

        spi_write(8'h00, THRESH_20M);

        fill_pipeline(DIST_100M);
        brake_count = 0;

        send_n_samples(DIST_30M, 25);
        wait_pipeline_drain();

        if (brake_count != 0) begin
            failures++;
            $display("[%0t] ERROR test %0d: brake fired %0d time(s) — SPI threshold not respected",
                     $time, test_id, brake_count);
        end else begin
            $display("[%0t] INFO  test %0d: 30m above 20m threshold — no brake (correct)",
                     $time, test_id);
        end

        report();

        // ============================================================
        // TEST 6: Reset recovery
        // ============================================================
        test_id = 6;
        failures_before = failures;
        $display("[%0t] TEST %0d START: Reset recovery", $time, test_id);

        apply_reset();
        brake_count = 0;

        fill_pipeline(DIST_100M);
        send_n_samples(DIST_30M, 25);
        wait_pipeline_drain();

        uart_rx = 1'b1;
        repeat (CLKS_PER_BIT * 20) @(posedge clk);

        apply_reset();
        brake_count = 0;

        fill_pipeline(DIST_100M);
        brake_count = 0;

        send_n_samples(DIST_100M, 10);
        wait_pipeline_drain();

        if (brake_count != 0) begin
            failures++;
            $display("[%0t] ERROR test %0d: brake fired %0d time(s) after reset on clear track",
                     $time, test_id, brake_count);
        end else begin
            $display("[%0t] INFO  test %0d: clean recovery after reset", $time, test_id);
        end

        report();

        // ============================================================
        // TEST 7: End-to-end baud rate change via SPI
        // ============================================================
        test_id = 7;
        failures_before = failures;
        $display("[%0t] TEST %0d START: End-to-end baud rate change via SPI", $time, test_id);

        apply_reset();
        brake_count = 0;

        fill_pipeline(DIST_100M);
        brake_count = 0;

        send_n_samples(DIST_100M, 10);
        wait_pipeline_drain();

        if (brake_count != 0) begin
            failures++;
            $display("[%0t] ERROR test %0d phase 1: unexpected brake at 460800", $time, test_id);
        end else begin
            $display("[%0t] INFO  test %0d phase 1: 460800 baud OK", $time, test_id);
        end

        spi_write(8'h09, 16'd434);
        repeat (8) @(posedge clk);
        repeat (8) @(posedge clk);

        fill_pipeline_bp(DIST_100M, BIT_PERIOD_115200);
        brake_count = 0;

        send_n_samples_bp(DIST_30M, 40, BIT_PERIOD_115200);
        wait_pipeline_drain();

        if (brake_count == 0) begin
            failures++;
            $display("[%0t] ERROR test %0d phase 4: brake NOT fired at 115200", $time, test_id);
        end else begin
            $display("[%0t] INFO  test %0d phase 4: 115200 baud OK — brake fired (%0d pulse(s))",
                     $time, test_id, brake_count);
        end

        report();

        // ============================================================
        // FINAL REPORT
        // ============================================================
        $display("\n================================================");
        $display("  ProxCore Integration Verification");
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

endmodule