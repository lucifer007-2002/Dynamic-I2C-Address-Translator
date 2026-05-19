`timescale 1ns/1ps

module bridge_fsm (
    input  wire        clk,           // 100 MHz system clock
    input  wire        rst_n,         // Active-low synchronous reset

    // ── Master-facing (from i2c_slave_if) ─────────────────────────────────
    input  wire        start_det,     // START condition on master bus
    input  wire        stop_det,      // STOP condition on master bus
    input  wire [7:0]  rx_byte,       // [7:1] = addr, [0] = R/W
    input  wire        rx_done,       // Pulse: rx_byte valid this cycle
    output reg         send_ack,      // Hold high: slave_if pulls SDA_M low (ACK)
    output reg         send_nack,     // 1-cycle pulse: slave_if releases SDA_M (NACK)

    // ── Address translator (combinational) ────────────────────────────────
    output reg  [7:0]  xlat_in,       // Registered input to translator
    input  wire [6:0]  xlat_addr,     // Translated 7-bit address (comb.)
    input  wire        xlat_bus_sel,  // Which slave bus (comb.)

    // ── Slave bus SDA ──────────────────────────────────────────────────────
    input  wire        sda_s_in,      // SDA sampled from slave bus
    output reg         sda_s_oe,      // 1 = Hi-Z (release), 0 = drive low

    // ── Slave bus SCL ──────────────────────────────────────────────────────
    output reg         scl_s,
    output wire        scl_rise_out,  // 1-cycle pulse: SCL rising (for master_if)
    output wire        scl_fall_out,  // 1-cycle pulse: SCL falling (for master_if)

    // ── Bus select for top-level routing ──────────────────────────────────
    output reg         bus_sel_out,

    // ── Data forwarding handshake ──────────────────────────────────────────
    output reg         fwd_en,        // 1 = master_if handles data bytes
    output reg         fwd_rw,        // R/W direction for master_if
    input  wire        fwd_done,      // master_if: data phase complete

    // ── Debug ──────────────────────────────────────────────────────────────
    output wire [3:0]  dbg_state,
    output reg         error_flag
);

// ─── State encoding ──────────────────────────────────────────────────────────
localparam [3:0]
    S_IDLE        = 4'd0,
    S_RECV_ADDR   = 4'd1,
    S_ACK_MASTER  = 4'd2,
    S_TRANSLATE   = 4'd3,
    S_START_S     = 4'd4,
    S_SEND_ADDR_S = 4'd5,
    S_RECV_ACK_S  = 4'd6,
    S_FORWARD     = 4'd7,
    S_NACK_MASTER = 4'd8,
    S_STOP_S      = 4'd9,
    S_WAIT_STOP   = 4'd10;

reg [3:0] state;
assign dbg_state = state;

// ─── SCL Generator ───────────────────────────────────────────────────────────
// Target: 400 kHz I2C Fast Mode
// System clock: 100 MHz → period = 10 ns
// I2C period: 2500 ns → half period = 1250 ns = 125 system clocks
// Counter runs 0 → 124 (125 ticks) then toggles SCL

localparam SCL_HALF = 7'd124;   // 0..124 inclusive

reg       scl_en;    // 1 = run generator,  0 = hold SCL high (idle)
reg [6:0] scl_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        scl_s   <= 1'b1;
        scl_cnt <= 7'd0;
    end else if (scl_en) begin
        if (scl_cnt == SCL_HALF) begin
            scl_cnt <= 7'd0;
            scl_s   <= ~scl_s;   // Toggle: generates symmetric 400 kHz square wave
        end else
            scl_cnt <= scl_cnt + 1;
    end else begin
        scl_s   <= 1'b1;    // Idle: I2C bus rests with SCL high
        scl_cnt <= 7'd0;    // Reset counter so first enabled period is full-width
    end
end

// ─── SCL edge indicators ─────────────────────────────────────────────────────
// These are COMBINATIONAL wires — true for exactly ONE system clock cycle.
// "scl_falling" = the cycle in which scl_cnt hits terminal count AND SCL is 1.
// After this cycle: SCL register toggles to 0 (via non-blocking assign above).
// The FSM reads pre-toggle values — correct by Verilog simulation semantics.
//
// In hardware: scl_s and scl_cnt are flip-flop outputs; these are just
// combinational logic gates from those outputs. Fully synthesizable.

wire scl_falling = (scl_cnt == SCL_HALF) &&  scl_s;  // SCL: 1→0 this cycle
wire scl_rising  = (scl_cnt == SCL_HALF) && ~scl_s;  // SCL: 0→1 this cycle

assign scl_fall_out = scl_falling;
assign scl_rise_out = scl_rising;

// ─── Internal data registers ──────────────────────────────────────────────────
reg [6:0] xlat_addr_r;   // Latched translated address
reg       rw_latch;       // Latched R/W bit (0=write, 1=read)
reg       bus_sel_r;      // Latched bus select

reg [7:0] shift_reg;      // Outgoing shift register for slave-bus address
reg [2:0] bit_cnt;        // Bit counter: 7 downto 0 (8 bits per frame)
reg [1:0] sub_cnt;        // Sub-step counter (multi-phase states: START_S, RECV_ACK_S, STOP_S)
reg [6:0] delay_cnt;      // General delay counter (counts up to SCL_HALF)
reg       ack_ok;         // 1 = slave ACK received on slave bus

// ─── Main FSM ─────────────────────────────────────────────────────────────────
// Single sequential always block — no combinational always block for outputs.
// This style (registered Moore outputs) avoids glitches on output signals.

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= S_IDLE;
        send_ack    <= 1'b0;
        send_nack   <= 1'b0;
        sda_s_oe    <= 1'b1;
        scl_en      <= 1'b0;
        bus_sel_out <= 1'b0;
        fwd_en      <= 1'b0;
        fwd_rw      <= 1'b0;
        error_flag  <= 1'b0;
        xlat_in     <= 8'h00;
        xlat_addr_r <= 7'h00;
        rw_latch    <= 1'b0;
        bus_sel_r   <= 1'b0;
        shift_reg   <= 8'h00;
        bit_cnt     <= 3'd7;
        sub_cnt     <= 2'd0;
        delay_cnt   <= 7'd0;
        ack_ok      <= 1'b0;
    end else begin

        // Pulse signals: deassert by default. Asserted below in specific states.
        send_nack <= 1'b0;

        case (state)

        // ─────────────────────────────────────────────────────────────────────
        // S_IDLE
        // Bus quiescent. All slave-bus signals released. Wait for START.
        // ─────────────────────────────────────────────────────────────────────
        S_IDLE: begin
            scl_en     <= 1'b0;
            sda_s_oe   <= 1'b1;    // Hi-Z on slave SDA
            send_ack   <= 1'b0;
            fwd_en     <= 1'b0;
            error_flag <= 1'b0;
            sub_cnt    <= 2'd0;
            delay_cnt  <= 7'd0;

            if (start_det)
                state <= S_RECV_ADDR;
        end

        // ─────────────────────────────────────────────────────────────────────
        // S_RECV_ADDR
        // i2c_slave_if is sampling SCL_M and shifting bits. We wait here
        // until it asserts rx_done with the full 8-bit frame (7-bit addr + R/W).
        //
        // stop_det here means master aborted — bail cleanly.
        // ─────────────────────────────────────────────────────────────────────
        S_RECV_ADDR: begin
            if (stop_det) begin
                state <= S_IDLE;
            end else if (rx_done) begin
                xlat_in  <= rx_byte;      // Feed translator (comb. output ready next cycle)
                rw_latch <= rx_byte[0];   // Latch R/W bit before rx_byte changes
                state    <= S_ACK_MASTER;
            end
        end

        // ─────────────────────────────────────────────────────────────────────
        // S_ACK_MASTER
        // Assert send_ack HIGH. This state AND the following TRANSLATE state
        // both hold send_ack=1 (2 system cycles). slave_if latches it and
        // holds SDA_M low through the entire 9th SCL_M period.
        // (See Part 2A fix: slave_if needs a latch, not a direct wire.)
        // ─────────────────────────────────────────────────────────────────────
        S_ACK_MASTER: begin
            send_ack <= 1'b1;
            state    <= S_TRANSLATE;
        end

        // ─────────────────────────────────────────────────────────────────────
        // S_TRANSLATE
        // xlat_in was set in RECV_ADDR. The combinational translator has had
        // at least 2 cycles to settle. Latch its outputs now.
        // Deassert send_ack — slave_if's latch holds SDA_M low from here.
        // ─────────────────────────────────────────────────────────────────────
        S_TRANSLATE: begin
            send_ack    <= 1'b0;           // slave_if's ack_latch holds from here
            xlat_addr_r <= xlat_addr;      // Latch translated 7-bit address
            bus_sel_r   <= xlat_bus_sel;
            bus_sel_out <= xlat_bus_sel;
            sub_cnt     <= 2'd0;
            delay_cnt   <= 7'd0;
            state       <= S_START_S;
        end

        // ─────────────────────────────────────────────────────────────────────
        // S_START_S
        // Generate I2C START condition on slave bus.
        //
        // I2C Fast Mode spec (tHD;STA >= 600 ns):
        //   SDA_S must fall while SCL_S is high, then be held low for
        //   at least 600 ns before SCL_S falls.
        //
        // We use one SCL half-period (1250 ns @ 100 MHz) as the hold delay —
        // safely exceeds the 600 ns minimum.
        //
        // SCL_S is high on entry (held high by scl_en=0 in IDLE/TRANSLATE).
        //
        // sub_cnt=0: Pull SDA_S low. Count delay (tHD;STA via delay_cnt).
        // sub_cnt=1: Enable SCL generator. Wait for first scl_falling edge.
        //            On that edge: load shift_reg, transition to SEND_ADDR_S.
        // ─────────────────────────────────────────────────────────────────────
        S_START_S: begin
            case (sub_cnt)
                2'd0: begin
                    sda_s_oe <= 1'b0;   // Pull SDA low → START condition
                    if (delay_cnt == SCL_HALF) begin
                        delay_cnt <= 7'd0;
                        sub_cnt   <= 2'd1;
                    end else
                        delay_cnt <= delay_cnt + 1;
                end

                2'd1: begin
                    scl_en <= 1'b1;     // SCL generator runs. Starts at 1, falls first.
                    if (scl_falling) begin
                        // SCL just fell → first bit slot open
                        // Load shift register: MSB of addr first, R/W last
                        shift_reg <= {xlat_addr_r, rw_latch};
                        bit_cnt   <= 3'd7;
                        sub_cnt   <= 2'd0;   // Reset for RECV_ACK_S
                        state     <= S_SEND_ADDR_S;
                    end
                end

                default: sub_cnt <= 2'd0;
            endcase
        end

        // ─────────────────────────────────────────────────────────────────────
        // S_SEND_ADDR_S
        // Shift out 8 bits of {xlat_addr_r[6:0], rw_latch} on SDA_S.
        //
        // I2C protocol:
        //   - Change SDA on SCL FALLING edge (low phase = setup time)
        //   - Hold SDA stable while SCL is HIGH (slave samples on SCL rising)
        //
        // On each scl_falling:
        //   → Output shift_reg[7] (MSB) on SDA_S (open-drain: release for 1, pull low for 0)
        //   → Shift register left by 1
        //   → Decrement bit_cnt
        //   → If bit_cnt was 0: last bit placed → go to S_RECV_ACK_S
        //
        // On scl_rising: slave samples SDA. We hold it stable. Nothing to do.
        //
        // NOTE: The first scl_falling in this state is actually the SECOND
        // falling edge overall — the first was consumed by S_START_S to
        // trigger the transition here. State transitions on posedge clk via
        // non-blocking assign, so the transition cycle's scl_falling is
        // processed by S_START_S, not this state.
        // ─────────────────────────────────────────────────────────────────────
        S_SEND_ADDR_S: begin
            if (scl_falling) begin
                // Open-drain SDA drive:
                //   bit=1 → Hi-Z (pull-up resistor pulls line high)
                //   bit=0 → drive low (MOSFET/FPGA output pulls low)
                sda_s_oe  <= shift_reg[7] ? 1'b1 : 1'b0;
                shift_reg <= {shift_reg[6:0], 1'b0};  // Shift MSB out

                if (bit_cnt == 3'd0) begin
                    // Last bit (R/W) just placed on SDA.
                    // Do NOT release SDA here — must hold through SCL rising
                    // so slave can sample it. Release happens in RECV_ACK_S.
                    sub_cnt <= 2'd0;   // Reset for RECV_ACK_S sub-steps
                    state   <= S_RECV_ACK_S;
                end else begin
                    bit_cnt <= bit_cnt - 1;
                end
            end
            // scl_rising: slave samples — hold SDA stable, do nothing
        end

        // ─────────────────────────────────────────────────────────────────────
        // S_RECV_ACK_S
        // 9th SCL period: slave drives ACK (low) or NACK (high) on SDA_S.
        //
        // ── CRITICAL TIMING: The Spurious STOP Problem ──────────────────────
        // When we enter this state, SDA_S still shows the last data bit (R/W).
        // If we release SDA (sda_s_oe=1) immediately and that last bit was 0
        // (we were pulling SDA low), then SDA rises from 0→1 while SCL is
        // still high → this is a valid STOP condition by I2C spec → slave
        // aborts. This is a real bug, not a theoretical one.
        //
        // FIX: Do NOT release SDA until SCL FALLS for the 9th time.
        // Only after SCL goes low (ACK slot low phase) can we safely release.
        //
        // ── Sub-state sequence: ──────────────────────────────────────────────
        // sub_cnt=0: SCL low (just fell from 8th bit). Wait for scl_rising #8.
        //            (Slave samples last data bit on this rising edge.)
        //            DON'T touch sda_s_oe.
        //
        // sub_cnt=1: SCL high. Wait for scl_falling #9 (ACK slot begins).
        //            Still DON'T release SDA while SCL is high (spurious STOP).
        //
        // sub_cnt=2: SCL just fell (low phase of ACK slot).
        //            NOW release SDA (sda_s_oe=1). Slave pulls low for ACK.
        //            Wait for scl_rising #9 (sample point).
        //
        // sub_cnt=3: SCL high. Sample SDA: ack_ok = ~sda_s_in.
        //            Wait for scl_falling #10. Then evaluate and transition.
        //            (ack_ok is registered — stable before scl_falling hits.)
        // ─────────────────────────────────────────────────────────────────────
        S_RECV_ACK_S: begin
            case (sub_cnt)

                // Wait for 8th rising edge (slave samples last data bit)
                2'd0: begin
                    // HOLD sda_s_oe at whatever it was — don't change it
                    if (scl_rising)
                        sub_cnt <= 2'd1;
                end

                // SCL high after 8th rising. Wait for 9th falling (ACK slot).
                // Still holding SDA at last data bit. Do NOT release yet.
                2'd1: begin
                    if (scl_falling)
                        sub_cnt <= 2'd2;
                end

                // SCL just fell. Safe to release SDA now (SCL is low — no STOP risk).
                // This entire sub-state holds sda_s_oe=1 until scl_rising.
                2'd2: begin
                    sda_s_oe <= 1'b1;   // Release — slave takes over SDA
                    if (scl_rising) begin
                        ack_ok  <= ~sda_s_in;  // Sample: 0 on line = ACK = good
                        sub_cnt <= 2'd3;
                    end
                end

                // SCL high. ACK sampled in ack_ok register.
                // Wait for next falling edge to evaluate.
                // (125 cycles between rising and falling — ack_ok is rock solid)
                2'd3: begin
                    if (scl_falling) begin
                        sub_cnt <= 2'd0;
                        if (ack_ok) begin
                            // Slave acknowledged — proceed to data phase
                            fwd_en  <= 1'b1;
                            fwd_rw  <= rw_latch;
                            state   <= S_FORWARD;
                        end else begin
                            // Slave refused — propagate failure to master
                            state   <= S_NACK_MASTER;
                        end
                    end
                end

                default: sub_cnt <= 2'd0;
            endcase
        end

        // ─────────────────────────────────────────────────────────────────────
        // S_FORWARD
        // Data forwarding: entirely handled by i2c_master_if (Part 2C).
        //
        // This FSM:
        //   - Holds fwd_en=1 to enable master_if
        //   - Keeps scl_en=1 (SCL generator continues running)
        //   - Releases sda_s_oe to top.v mux (master_if drives SDA_S directly)
        //   - Exports scl_rise_out/scl_fall_out so master_if can synchronize
        //
        // master_if handles: byte-level shifting, per-byte ACK/NACK, direction
        // flip for READ transactions (R/W=1), and driving/sampling SDA_S.
        //
        // Exit when:
        //   - Master sends STOP (stop_det): abort, go generate STOP on slave
        //   - master_if asserts fwd_done: normal completion, generate STOP
        // ─────────────────────────────────────────────────────────────────────
        S_FORWARD: begin
            fwd_en <= 1'b1;

            if (stop_det || fwd_done) begin
                fwd_en    <= 1'b0;
                sub_cnt   <= 2'd0;
                delay_cnt <= 7'd0;
                state     <= S_STOP_S;
            end
        end

        // ─────────────────────────────────────────────────────────────────────
        // S_NACK_MASTER
        // Slave NACKed the address. Propagate failure to master:
        //   - Pulse send_nack → slave_if releases SDA_M (master sees NACK)
        //   - Disable slave bus
        //   - Set error_flag
        //   - Wait for master to send STOP → then clean IDLE
        // ─────────────────────────────────────────────────────────────────────
        S_NACK_MASTER: begin
            send_nack  <= 1'b1;   // 1-cycle pulse
            scl_en     <= 1'b0;   // Stop slave bus SCL
            sda_s_oe   <= 1'b1;   // Release slave SDA
            error_flag <= 1'b1;
            state      <= S_WAIT_STOP;
        end

        // ─────────────────────────────────────────────────────────────────────
        // S_STOP_S
        // Generate I2C STOP condition on slave bus.
        //
        // I2C STOP: SDA_S rises while SCL_S is HIGH.
        // I2C Fast Mode spec: tSU;STO >= 600 ns (setup time before SDA rises).
        //
        // Entry assumption: SCL_S is currently LOW (between data bytes, after
        // last ACK slot). master_if guaranteed to leave SCL low when fwd_done.
        //
        // sub_cnt=0: Pull SDA_S low (if not already). Wait for scl_rising.
        //            (SCL rises on its own — generator still running.)
        //
        // sub_cnt=1: SCL is high, SDA is low. Count tSU;STO delay (1 half-period).
        //            Then: release SDA → SDA rises while SCL high = STOP.
        //            Disable SCL generator → SCL freezes high (idle state). → IDLE.
        //
        // NOTE: sda_s_oe transitions from 0 → 1 while SCL is high.
        // SDA rises from 0 → 1 while SCL = 1 — this IS the STOP condition. ✓
        // ─────────────────────────────────────────────────────────────────────
        S_STOP_S: begin
            case (sub_cnt)
                2'd0: begin
                    sda_s_oe <= 1'b0;   // Hold SDA low before SCL rises
                    if (scl_rising) begin
                        delay_cnt <= 7'd0;
                        sub_cnt   <= 2'd1;
                    end
                end

                2'd1: begin
                    // SCL is high, SDA is low. Count tSU;STO (1250 ns > 600 ns min).
                    if (delay_cnt == SCL_HALF) begin
                        sda_s_oe <= 1'b1;   // Release SDA → rises = STOP ✓
                        scl_en   <= 1'b0;   // Freeze SCL high (I2C idle)
                        state    <= S_IDLE;
                    end else
                        delay_cnt <= delay_cnt + 1;
                end

                default: sub_cnt <= 2'd0;
            endcase
        end

        // ─────────────────────────────────────────────────────────────────────
        // S_WAIT_STOP
        // Error recovery. Slave bus released. Waiting for master STOP.
        // Master will see NACK and generate a STOP condition.
        // Once stop_det fires: return cleanly to IDLE for next transaction.
        // ─────────────────────────────────────────────────────────────────────
        S_WAIT_STOP: begin
            scl_en   <= 1'b0;
            sda_s_oe <= 1'b1;
            if (stop_det)
                state <= S_IDLE;
        end

        default: state <= S_IDLE;
        endcase
    end
end

endmodule
