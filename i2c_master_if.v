`timescale 1ns/1ps

module i2c_master_if (
    input  wire       clk,
    input  wire       rst_n,

    // ── Sync from bridge_fsm SCL generator ───────────────────────────────────
    input  wire       scl_rise_in,      // 1-cycle pulse: SCL rising edge
    input  wire       scl_fall_in,      // 1-cycle pulse: SCL falling edge

    // ── Master-side SDA (raw from IOBUF.O — already synchronized) ───────────
    input  wire       sda_m_in,         // What the master is currently driving
    output reg        sda_m_oe,         // Tristate: 1=Hi-Z, 0=drive low (master side)

    // ── Slave-side SDA ────────────────────────────────────────────────────────
    input  wire       sda_s_in,         // What the slave is driving (or sampled data)
    output reg        sda_s_oe,         // Tristate: 1=Hi-Z, 0=drive low (slave side)

    // ── Handshake with bridge_fsm ─────────────────────────────────────────────
    input  wire       fwd_en,           // 1 = this module is active
    input  wire       fwd_rw,           // 0 = write (M→S), 1 = read (S→M)
    output reg        fwd_done,         // Pulse: transaction data phase complete
                                        // (NACK from master = end of read burst)

    // ── Stop/Start abort from master (monitored here too) ────────────────────
    input  wire       stop_det,         // Master sent STOP — abort immediately
    input  wire       start_det         // Repeated START — abort
);

// ─── Sub-state encoding ──────────────────────────────────────────────────────
// Only 3 sub-states — the FSM envelope is handled by bridge_fsm.
// This module handles: bit shifting, byte boundaries, ACK slots.

localparam [1:0]
    FW_IDLE    = 2'd0,
    FW_DATA    = 2'd1,    // Relaying 8 data bits
    FW_ACK     = 2'd2;    // 9th clock: ACK/NACK slot

reg [1:0] fw_state;

// ─── Internal registers ───────────────────────────────────────────────────────
reg [7:0] shift_reg;    // Incoming bit accumulator (write) or outgoing bits (read)
reg [2:0] bit_cnt;      // 7 downto 0, counts bits within a byte
reg       ack_sample;   // Latched ACK value on SCL rising during ACK slot
reg       first_fall;   // Flag: first scl_fall_in after fwd_en — skip (FSM used it)

// ─── Main logic ───────────────────────────────────────────────────────────────
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fw_state   <= FW_IDLE;
        sda_m_oe   <= 1'b1;
        sda_s_oe   <= 1'b1;
        fwd_done   <= 1'b0;
        shift_reg  <= 8'h00;
        bit_cnt    <= 3'd7;
        ack_sample <= 1'b0;
        first_fall <= 1'b0;
    end else begin

        fwd_done <= 1'b0;   // Default: pulse low

        // ── Abort conditions (higher priority than state logic) ───────────────
        // bridge_fsm also catches these, but we deassert our tristate controls
        // immediately to avoid bus conflicts.
        if (!fwd_en || stop_det || start_det) begin
            fw_state  <= FW_IDLE;
            sda_m_oe  <= 1'b1;   // Release both sides
            sda_s_oe  <= 1'b1;
        end else begin

            case (fw_state)

            // ─────────────────────────────────────────────────────────────────
            // FW_IDLE
            // Waiting for fwd_en to go high. Once asserted, arm the first byte.
            // The SCL generator is already running. The first scl_fall_in after
            // fwd_en is the fallthrough from RECV_ACK_S sub_cnt=3 — skip it.
            // ─────────────────────────────────────────────────────────────────
            FW_IDLE: begin
                sda_m_oe  <= 1'b1;
                sda_s_oe  <= 1'b1;
                bit_cnt   <= 3'd7;
                first_fall <= 1'b1;   // Next fall is the hand-off fall — skip

                if (fwd_en)
                    fw_state <= FW_DATA;
            end

            // ─────────────────────────────────────────────────────────────────
            // FW_DATA — Relay bits
            //
            // WRITE (fwd_rw=0): master drives SDA_M, we sample on scl_rise_in
            //   and re-drive SDA_S on scl_fall_in (setup before next rise).
            //   Our SDA_M side stays Hi-Z (master owns it).
            //   We drive SDA_S: 1→Hi-Z, 0→pull low.
            //
            // READ (fwd_rw=1): slave drives SDA_S, we sample on scl_rise_in
            //   and re-drive SDA_M on scl_fall_in.
            //   Our SDA_S side stays Hi-Z (slave owns it).
            //   We drive SDA_M: 1→Hi-Z, 0→pull low.
            //
            // bit_cnt counts from 7 downto 0. When it hits 0: last bit placed.
            // Move to FW_ACK after handling last bit on scl_fall_in.
            // ─────────────────────────────────────────────────────────────────
            FW_DATA: begin
                if (scl_fall_in) begin
                    if (first_fall) begin
                        first_fall <= 1'b0;  // Discard hand-off fall
                    end else if (!fwd_rw) begin
                        // WRITE: drive the bit we sampled from master on SDA_S
                        sda_s_oe <= shift_reg[7] ? 1'b1 : 1'b0;
                        shift_reg <= {shift_reg[6:0], 1'b0};

                        if (bit_cnt == 3'd0) begin
                            bit_cnt  <= 3'd7;
                            fw_state <= FW_ACK;
                        end else
                            bit_cnt <= bit_cnt - 1;

                    end else begin
                        // READ: drive the bit we sampled from slave on SDA_M
                        sda_m_oe <= shift_reg[7] ? 1'b1 : 1'b0;
                        shift_reg <= {shift_reg[6:0], 1'b0};

                        if (bit_cnt == 3'd0) begin
                            bit_cnt  <= 3'd7;
                            fw_state <= FW_ACK;
                        end else
                            bit_cnt <= bit_cnt - 1;
                    end
                end

                if (scl_rise_in) begin
                    // Sample incoming bit into shift register
                    if (!fwd_rw)
                        shift_reg <= {shift_reg[6:0], sda_m_in};  // WRITE: sample master
                    else
                        shift_reg <= {shift_reg[6:0], sda_s_in};  // READ: sample slave
                end
            end

            // ─────────────────────────────────────────────────────────────────
            // FW_ACK — 9th clock slot after each byte
            //
            // WRITE: we release SDA_S (slave ACKs by pulling low).
            //   Sample sda_s_in on scl_rise_in.
            //   If slave ACKs (low): loop back for next byte.
            //   If slave NACKs (high): assert fwd_done → FSM generates STOP.
            //
            // READ: master drives ACK/NACK on SDA_M to continue/stop burst.
            //   We release SDA_M (master owns this slot).
            //   Sample sda_m_in on scl_rise_in.
            //   NACK from master (high) = end of read burst → fwd_done.
            //   ACK from master (low)  = more bytes → loop back.
            //
            // In both cases: move to next byte or assert fwd_done on scl_fall_in
            // AFTER sampling (ack_sample is registered, stable for evaluation).
            // ─────────────────────────────────────────────────────────────────
            FW_ACK: begin
                // Release the driven side — the other party owns ACK slot
                if (!fwd_rw) begin
                    sda_s_oe <= 1'b1;   // Slave ACKs on slave bus
                    sda_m_oe <= 1'b1;   // Master side: not our problem this slot
                end else begin
                    sda_m_oe <= 1'b1;   // Master drives NACK/ACK for read burst
                    sda_s_oe <= 1'b1;   // Release slave side
                end

                if (scl_rise_in) begin
                    // Sample the ACK/NACK bit
                    ack_sample <= !fwd_rw ? sda_s_in : sda_m_in;
                    //   Write → sample slave ACK on slave bus
                    //   Read  → sample master ACK on master bus
                end

                if (scl_fall_in) begin
                    // Evaluate after sampling (ack_sample stable from prior rise)
                    if (ack_sample == 1'b0) begin
                        // ACK received → continue with next byte
                        fw_state <= FW_DATA;
                    end else begin
                        // NACK → end of transaction
                        fwd_done <= 1'b1;
                        fw_state <= FW_IDLE;
                    end
                end
            end

            default: fw_state <= FW_IDLE;
            endcase
        end
    end
end

endmodule
