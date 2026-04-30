`timescale 1ns/1ps

module tb_deserializer_gc;

    // ================================================================
    // Parameters
    // ================================================================
    localparam int unsigned CLK_FREQ_HZ = 50_000_000;
    localparam time         CLK_PERIOD  = 20ns;

    // Test baud rate constants (for comments and reference)
    localparam int unsigned BAUD_9600   = 5208;
    localparam int unsigned BAUD_115200 = 434;
    localparam int unsigned BAUD_230400 = 217;
    localparam int unsigned BAUD_460800 = 108;
    localparam int unsigned BAUD_921600 = 54;

    // ================================================================
    // DUT signals
    // ================================================================
    logic        clk;
    logic        rst_n;
    logic        rx;
    logic [15:0] baud_div_tb;
    logic [15:0] data_out;
    logic        data_valid;

    // ================================================================
    // Test signals
    // ================================================================
    int unsigned failures;
    int unsigned test_id;
    int unsigned failures_before;
    int unsigned pulse_count;
    time         BIT_TIME;

    // ================================================================
    // DUT instantiation
    // ================================================================
    deserializer_gc dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .rx         (rx),
        .baud_div   (baud_div_tb),
        .data_out   (data_out),
        .data_valid (data_valid)
    );

    // ================================================================
    // Clock generation
    // ================================================================
    initial clk = 1'b0;
    always #(CLK_PERIOD / 2) clk = ~clk;

    // ================================================================
    // data_valid pulse monitor
    // ================================================================
    always @(posedge clk) begin
        if (data_valid && rst_n)
            pulse_count = pulse_count + 1;
    end

    // ================================================================
    // Watchdog timer
    // ================================================================
    initial begin
        #(50ms);
        $display("[FATAL] Simulation timeout");
        $fatal(1);
    end

    // ================================================================
    // UART transmission task — accepts bit period as parameter
    // ================================================================
    task automatic uart_send_byte(input logic [7:0] data, input time bit_period);
        integer i;
        begin
            // Start bit
            rx = 1'b0;
            #(bit_period);
            // Data bits (LSB first)
            for (i = 0; i < 8; i++) begin
                rx = data[i];
                #(bit_period);
            end
            // Stop bit
            rx = 1'b1;
            #(bit_period);
        end
    endtask

    task automatic uart_send_word(input logic [15:0] data, input time bit_period);
        begin
            uart_send_byte(data[7:0], bit_period);    // low byte first
            uart_send_byte(data[15:8], bit_period);
        end
    endtask

    // ================================================================
    // Reset task
    // ================================================================
    task automatic apply_reset;
        begin
            rst_n = 1'b0;
            rx    = 1'b1;  // UART idle
            repeat (4) @(posedge clk);
            rst_n = 1'b1;
            @(posedge clk);
        end
    endtask

    // ================================================================
    // Helper tasks
    // ================================================================
    task automatic wait_cycles(input integer n);
        begin
            repeat (n) @(posedge clk);
        end
    endtask

    // ================================================================
    // Result reporting
    // ================================================================
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
        clk             = 1'b0;
        rst_n           = 1'b1;
        rx              = 1'b1;
        baud_div_tb     = BAUD_460800;
        failures        = 0;
        pulse_count     = 0;
        BIT_TIME        = baud_div_tb * CLK_PERIOD;

        apply_reset();

        // ============================================================
        // TEST 1: Single 16-bit word at 460800 baud
        //
        // Verify the basic deserializer operation with baud_div driven
        // as a signal instead of parameter.
        // ============================================================
        test_id = 1;
        failures_before = failures;
        $display("[%0t] TEST %0d START: Single word at 460800 baud", $time, test_id);

        apply_reset();
        baud_div_tb = BAUD_460800;
        BIT_TIME = baud_div_tb * CLK_PERIOD;
        pulse_count = 0;

        uart_send_word(16'hDEAD, BIT_TIME);
        wait_cycles(500);  // Allow time for processing

        if (pulse_count != 1) begin
            failures++;
            $display("[%0t] ERROR test %0d: expected 1 pulse, got %0d", $time, test_id, pulse_count);
        end
        if (data_out != 16'hDEAD) begin
            failures++;
            $display("[%0t] ERROR test %0d: expected 0xDEAD, got 0x%04X", $time, test_id, data_out);
        end

        report();

        // ============================================================
        // TEST 2: Multiple words at 460800 baud
        //
        // Verify back-to-back word reception.
        // ============================================================
        test_id = 2;
        failures_before = failures;
        $display("[%0t] TEST %0d START: Multiple words at 460800 baud", $time, test_id);

        apply_reset();
        baud_div_tb = BAUD_460800;
        BIT_TIME = baud_div_tb * CLK_PERIOD;
        pulse_count = 0;

        uart_send_word(16'h1234, BIT_TIME);
        uart_send_word(16'h5678, BIT_TIME);
        uart_send_word(16'hABCD, BIT_TIME);
        wait_cycles(1000);

        if (pulse_count != 3) begin
            failures++;
            $display("[%0t] ERROR test %0d: expected 3 pulses, got %0d", $time, test_id, pulse_count);
        end

        report();

        // ============================================================
        // TEST 3: Glitch rejection on start bit
        //
        // Send a short pulse on rx (glitch), then a valid word.
        // Deserializer should reject the glitch and correctly receive
        // the subsequent word.
        // ============================================================
        test_id = 3;
        failures_before = failures;
        $display("[%0t] TEST %0d START: Glitch rejection on start bit", $time, test_id);

        apply_reset();
        baud_div_tb = BAUD_460800;
        BIT_TIME = baud_div_tb * CLK_PERIOD;
        pulse_count = 0;

        // Send a glitch: short pulse on rx
        rx = 1'b0;
        #(BIT_TIME / 4);  // Only 1/4 bit period
        rx = 1'b1;
        wait_cycles(10);

        // Now send valid word
        uart_send_word(16'hBEEF, BIT_TIME);
        wait_cycles(500);

        if (pulse_count != 1) begin
            failures++;
            $display("[%0t] ERROR test %0d: expected 1 pulse (glitch rejected), got %0d",
                     $time, test_id, pulse_count);
        end
        if (data_out != 16'hBEEF) begin
            failures++;
            $display("[%0t] ERROR test %0d: expected 0xBEEF after glitch, got 0x%04X",
                     $time, test_id, data_out);
        end

        report();

        // ============================================================
        // TEST 4: Framing error handling
        //
        // Send an incomplete word (no stop bit), then a valid word.
        // Deserializer should drop the malformed word and correctly
        // receive the next one.
        // ============================================================
        test_id = 4;
        failures_before = failures;
        $display("[%0t] TEST %0d START: Framing error handling", $time, test_id);

        apply_reset();
        baud_div_tb = BAUD_460800;
        BIT_TIME = baud_div_tb * CLK_PERIOD;
        pulse_count = 0;

        // Send partial word: start bit, low byte, low byte (no stop, low stop bit)
        rx = 1'b0;
        #(BIT_TIME);  // Start bit
        for (int i = 0; i < 8; i++) begin
            rx = i[0];
            #(BIT_TIME);
        end
        rx = 1'b0;  // Stop bit is low (framing error)
        #(BIT_TIME);
        rx = 1'b1;
        wait_cycles(100);

        // Now send a valid word
        uart_send_word(16'h7777, BIT_TIME);
        wait_cycles(500);

        if (pulse_count != 1) begin
            failures++;
            $display("[%0t] ERROR test %0d: expected 1 pulse (error dropped), got %0d",
                     $time, test_id, pulse_count);
        end
        if (data_out != 16'h7777) begin
            failures++;
            $display("[%0t] ERROR test %0d: expected 0x7777 after error, got 0x%04X",
                     $time, test_id, data_out);
        end

        report();

        // ============================================================
        // TEST 5: Byte order verification (LOW_BYTE_FIRST)
        //
        // Verify that the low byte is received first and assembled
        // into the output as {high_byte, low_byte}.
        // ============================================================
        test_id = 5;
        failures_before = failures;
        $display("[%0t] TEST %0d START: Byte order (LOW_BYTE_FIRST)", $time, test_id);

        apply_reset();
        baud_div_tb = BAUD_460800;
        BIT_TIME = baud_div_tb * CLK_PERIOD;
        pulse_count = 0;

        // Send low byte 0x34, then high byte 0x12 → should output 0x1234
        uart_send_byte(8'h34, BIT_TIME);
        uart_send_byte(8'h12, BIT_TIME);
        wait_cycles(500);

        if (pulse_count != 1) begin
            failures++;
            $display("[%0t] ERROR test %0d: expected 1 pulse, got %0d", $time, test_id, pulse_count);
        end
        if (data_out != 16'h1234) begin
            failures++;
            $display("[%0t] ERROR test %0d: byte order wrong — expected 0x1234, got 0x%04X",
                     $time, test_id, data_out);
        end

        report();

        // ============================================================
        // TEST 6: Runtime baud rate change
        //
        // Start at 460800 baud (baud_div=108).
        // Send one valid word at 460800 baud. Verify received correctly.
        // Change baud_div to 115200 baud (baud_div=434) — drive port directly.
        // Wait 4 clock cycles for the change to take effect.
        // Send one valid word at 115200 baud timing. Verify received correctly.
        // ============================================================
        test_id = 6;
        failures_before = failures;
        $display("[%0t] TEST %0d START: Runtime baud rate change", $time, test_id);

        apply_reset();
        baud_div_tb = BAUD_460800;
        BIT_TIME = baud_div_tb * CLK_PERIOD;
        pulse_count = 0;

        // Send at 460800 baud
        uart_send_word(16'hC0FF, BIT_TIME);
        wait_cycles(100);

        if (pulse_count != 1) begin
            failures++;
            $display("[%0t] ERROR test %0d phase A: expected 1 pulse at 460800, got %0d",
                     $time, test_id, pulse_count);
        end
        if (data_out != 16'hC0FF) begin
            failures++;
            $display("[%0t] ERROR test %0d phase A: expected 0xC0FF, got 0x%04X",
                     $time, test_id, data_out);
        end

        // Change baud rate to 115200
        baud_div_tb = BAUD_115200;
        BIT_TIME = baud_div_tb * CLK_PERIOD;
        wait_cycles(4);  // Allow 4 cycles for change to propagate

        pulse_count = 0;

        // Send at 115200 baud
        uart_send_word(16'hEEEE, BIT_TIME);
        wait_cycles(500);  // Longer wait for slower baud rate

        if (pulse_count != 1) begin
            failures++;
            $display("[%0t] ERROR test %0d phase B: expected 1 pulse at 115200, got %0d",
                     $time, test_id, pulse_count);
        end
        if (data_out != 16'hEEEE) begin
            failures++;
            $display("[%0t] ERROR test %0d phase B: expected 0xEEEE, got 0x%04X",
                     $time, test_id, data_out);
        end

        report();

        // ============================================================
        // TEST 7: 9600 baud — slowest supported rate
        //
        // Set baud_div=5208 (9600 baud).
        // Send word 0xABCD at 9600 baud timing.
        // Verify received correctly.
        // This proves the 16-bit baud_cnt_q is wide enough.
        // ============================================================
        test_id = 7;
        failures_before = failures;
        $display("[%0t] TEST %0d START: 9600 baud (slowest)", $time, test_id);

        apply_reset();
        baud_div_tb = BAUD_9600;  // 5208
        BIT_TIME = baud_div_tb * CLK_PERIOD;
        pulse_count = 0;

        uart_send_word(16'hABCD, BIT_TIME);
        wait_cycles(600000);  // Much longer wait for very slow baud rate

        if (pulse_count != 1) begin
            failures++;
            $display("[%0t] ERROR test %0d: expected 1 pulse at 9600, got %0d",
                     $time, test_id, pulse_count);
        end
        if (data_out != 16'hABCD) begin
            failures++;
            $display("[%0t] ERROR test %0d: expected 0xABCD at 9600, got 0x%04X",
                     $time, test_id, data_out);
        end

        report();

        // ============================================================
        // TEST 8: 921600 baud — fastest supported rate
        //
        // Set baud_div=54 (921600 baud).
        // Send word 0x1234 at 921600 baud timing.
        // Verify received correctly.
        // This proves fast baud rates work with smaller counters.
        // ============================================================
        test_id = 8;
        failures_before = failures;
        $display("[%0t] TEST %0d START: 921600 baud (fastest)", $time, test_id);

        apply_reset();
        baud_div_tb = BAUD_921600;  // 54
        BIT_TIME = baud_div_tb * CLK_PERIOD;
        pulse_count = 0;

        uart_send_word(16'h1234, BIT_TIME);
        wait_cycles(1500);  // Shorter wait for very fast baud rate

        if (pulse_count != 1) begin
            failures++;
            $display("[%0t] ERROR test %0d: expected 1 pulse at 921600, got %0d",
                     $time, test_id, pulse_count);
        end
        if (data_out != 16'h1234) begin
            failures++;
            $display("[%0t] ERROR test %0d: expected 0x1234 at 921600, got 0x%04X",
                     $time, test_id, data_out);
        end

        report();

        // ============================================================
        // FINAL REPORT
        // ============================================================
        $display("\n================================================");
        $display("  Deserializer_GC Verification");
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
