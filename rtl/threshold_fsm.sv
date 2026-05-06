module threshold_fsm (
    input  logic        clk,
    input  logic        rst_n,
    input  logic [15:0] filtered_in,
    input  logic        data_valid,
    input  logic [15:0] threshold,
    output logic        brake_irq
);

    typedef enum logic [1:0] {
        CLEAR   = 2'b00,
        WARN1   = 2'b01,
        WARN2   = 2'b10,
        BRAKING = 2'b11
    } state_t;

    state_t state_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q   <= CLEAR;
            brake_irq <= 1'b0;
        end else begin
            brake_irq <= 1'b0; // default: deassert every cycle

            if (data_valid) begin
                unique case (state_q)

                    CLEAR: begin
                        if (filtered_in < threshold)
                            state_q <= WARN1;
                        // else stay CLEAR
                    end

                    WARN1: begin
                        if (filtered_in < threshold)
                            state_q <= WARN2;
                        else
                            state_q <= CLEAR; // not consecutive — reset
                    end

                    WARN2: begin
                        if (filtered_in < threshold) begin
                            state_q   <= BRAKING;
                            brake_irq <= 1'b1; // one-cycle pulse
                        end else
                            state_q <= CLEAR; // not consecutive — reset
                    end

                    BRAKING: begin
                        if (filtered_in >= threshold)
                            state_q <= CLEAR; // obstacle cleared
                        // else stay BRAKING, no repeated IRQ
                    end

                endcase
            end
        end
    end

endmodule
