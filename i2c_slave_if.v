module i2c_slave_if (
    input  wire       clk,          // System clock (100 MHz)
    input  wire       rst_n,        // Active-low reset

    // Physical bus (after IOBUF, in top.v)
    input  wire       scl_m,        // SCL from master — read only
    input  wire       sda_m_in,     // SDA read value (IOBUF output pin)
    output reg        sda_m_oe,     // SDA output enable: 1=Hi-Z, 0=drive low

    // Handshake to FSM
    output reg        start_det,    // Pulse when START detected
    output reg        stop_det,     // Pulse when STOP detected
    output reg [7:0]  rx_byte,      // Captured byte (addr[6:0] + rw)
    output reg        rx_done,      // Pulse when rx_byte is valid
    input  wire       send_ack,     // FSM: drive ACK this cycle
    input  wire       send_nack     // FSM: drive NACK (release line)
);

    // ─── Edge detection registers ─────────────────────────────────────
    reg scl_prev, sda_prev;

    wire scl_rise =  scl_m & ~scl_prev;
    wire scl_fall = ~scl_m &  scl_prev;
    wire start_cond = sda_prev & ~sda_m_in & scl_m;  // SDA fell, SCL high
    wire stop_cond  = ~sda_prev & sda_m_in & scl_m;  // SDA rose, SCL high

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_prev <= 1'b1;
            sda_prev <= 1'b1;
        end else begin
            scl_prev <= scl_m;
            sda_prev <= sda_m_in;
        end
    end

    // ─── START / STOP detection ───────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_det <= 1'b0;
            stop_det  <= 1'b0;
        end else begin
            start_det <= start_cond;   // 1-cycle pulse
            stop_det  <= stop_cond;
        end
    end

    // ─── Bit shift register ───────────────────────────────────────────
    // Shifts in MSB-first on every SCL rising edge
    // I2C always sends MSB first: bit7=addr[6], bit0=R/W

    reg [2:0] bit_cnt;      // counts 0–7
    reg       recv_active;  // we're mid-frame (after START, before STOP)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_byte     <= 8'h00;
            rx_done     <= 1'b0;
            bit_cnt     <= 3'd0;
            recv_active <= 1'b0;
        end else begin
            rx_done <= 1'b0;   // default: pulse low

            if (start_det) begin
                recv_active <= 1'b1;
                bit_cnt     <= 3'd0;
            end

            if (stop_det) begin
                recv_active <= 1'b0;
            end

            if (recv_active && scl_rise) begin
                if (bit_cnt < 3'd8) begin
                    // Shift in one bit MSB-first
                    rx_byte <= {rx_byte[6:0], sda_m_in};
                    bit_cnt <= bit_cnt + 1;

                    if (bit_cnt == 3'd7) begin
                        rx_done <= 1'b1;    // All 8 bits received
                        // bit_cnt will be 8 next, that's the ACK cycle
                    end
                end
                // bit_cnt == 8: ACK/NACK cycle — driven by sda_m_oe below
                // Don't increment past 8; FSM resets recv_active on STOP
            end
        end
    end

    // ─── ACK/NACK driving ─────────────────────────────────────────────
    // On the 9th SCL cycle (bit_cnt == 8), FSM decides ACK or NACK
    // ACK  = pull SDA low  → sda_m_oe = 0 (drive)
    // NACK = release SDA   → sda_m_oe = 1 (Hi-Z)
    // Default: always release (Hi-Z) unless FSM explicitly says ACK

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sda_m_oe <= 1'b1;   // default Hi-Z
        end else begin
            if (send_ack)
                sda_m_oe <= 1'b0;   // pull low = ACK
            else
                sda_m_oe <= 1'b1;   // release = NACK or idle
        end
    end

endmodule
