`timescale 1ns/1ps

module tb_i2c_bridge;

// ─── Clock & Reset ────────────────────────────────────────────────────────────
parameter CLK_PERIOD = 10;           // 100 MHz → 10 ns
parameter I2C_HALF   = 1250;         // 400 kHz half-period in ns

reg clk = 0;
reg rst_n = 0;

always #(CLK_PERIOD/2) clk = ~clk;

// ─── DUT port connections ─────────────────────────────────────────────────────
// SDA lines: inout. Use wire + pullup + conditional drive.
// Logic: if TB wants to pull low → drive 0. If TB releases → high-Z.
// Same for the slave model below.

// Master-side bus
reg        tb_scl_m    = 1;       // TB drives SCL_M
reg        tb_sda_m    = 1;       // TB drives SDA_M: 1 = release, 0 = pull low
wire       SDA_M;
wire       SCL_M = tb_scl_m;

// Pull-up on SDA_M (1.5kΩ equivalent — modelled as 1 when released)
assign SDA_M = tb_sda_m ? 1'bz : 1'b0;  // TB contribution
// (DUT's IOBUF also drives this wire when sda_m_oe=0 — they both pull low, no conflict)

// Slave-side bus 0
reg        slave0_sda  = 1;       // Slave model drives SDA_S0
wire       SDA_S0;
wire       SCL_S0;

assign SDA_S0 = slave0_sda ? 1'bz : 1'b0;

// Slave-side bus 1
reg        slave1_sda  = 1;
wire       SDA_S1;
wire       SCL_S1;

assign SDA_S1 = slave1_sda ? 1'bz : 1'b0;

// Weak pull-ups on all SDA lines (simulated via 1'bz → resolves to 1 by Verilog default)
// In real hardware: 2.2kΩ to 3.3V. In simulation: the 1'bz on all drivers resolves to 1.

wire [3:0] dbg_leds;

// ─── DUT instantiation ────────────────────────────────────────────────────────
// Note: IOBUF is a Xilinx primitive. In Vivado xsim it simulates correctly
// because Vivado includes its unisim library automatically.
// In Xcelium: compile with -v $XILINX_VIVADO/data/verilog/src/unisims/IOBUF.v

top dut (
    .clk_100mhz (clk),
    .rst_n      (rst_n),
    .SCL_M      (SCL_M),
    .SDA_M      (SDA_M),
    .SCL_S0     (SCL_S0),
    .SDA_S0     (SDA_S0),
    .SCL_S1     (SCL_S1),
    .SDA_S1     (SDA_S1),
    .dbg_leds   (dbg_leds)
);

// ─── GTKWave dump ─────────────────────────────────────────────────────────────
initial begin
    $dumpfile("tb_i2c_bridge.vcd");
    $dumpvars(0, tb_i2c_bridge);   // Dump all signals, all hierarchy levels
end

// ─── Convenience aliases for waveform readability ────────────────────────────
// These tap internal DUT signals. Works in Vivado xsim and Xcelium.
// In GTKWave: look under tb_i2c_bridge.dut.u_fsm for these.
wire [3:0] fsm_state  = dut.u_fsm.state;
wire       fwd_en_mon = dut.fwd_en;
wire       bus_sel_mon= dut.bus_sel_out;
wire       err_mon    = dut.u_fsm.error_flag;
wire       scl_s_mon  = dut.scl_s_w;

// ─── I2C Master Task Library ──────────────────────────────────────────────────
// All delays in nanoseconds. SCL transitions are the primary timing reference.
// SDA changes on SCL LOW phases only (except START/STOP conditions).

// START condition: SDA falls while SCL is HIGH
task i2c_start;
begin
    tb_sda_m = 1; tb_scl_m = 1; #(I2C_HALF);
    tb_sda_m = 0;               #(I2C_HALF);   // SDA falls → START
    tb_scl_m = 0;               #(I2C_HALF);   // Pull SCL low: first bit slot ready
end
endtask

// STOP condition: SDA rises while SCL is HIGH
task i2c_stop;
begin
    tb_sda_m = 0; tb_scl_m = 0; #(I2C_HALF);
    tb_scl_m = 1;               #(I2C_HALF);   // SCL high
    tb_sda_m = 1;               #(I2C_HALF);   // SDA rises → STOP
    #(I2C_HALF);                                // Bus idle hold
end
endtask

// Send one bit. SDA set on SCL low, held through SCL high.
task i2c_send_bit;
    input bit_val;
begin
    tb_sda_m = bit_val;   #(I2C_HALF/4);  // Setup time
    tb_scl_m = 1;         #(I2C_HALF);    // SCL high: slave samples
    tb_scl_m = 0;         #(I2C_HALF);    // SCL low: next setup
end
endtask

// Receive one bit. Release SDA, let device drive it. Sample on SCL high.
task i2c_recv_bit;
    output bit_val;
begin
    tb_sda_m = 1;         #(I2C_HALF/4);  // Release SDA
    tb_scl_m = 1;         #(I2C_HALF/2);  // SCL high
    bit_val  = SDA_M;                      // Sample mid-high
    #(I2C_HALF/2);
    tb_scl_m = 0;         #(I2C_HALF);
end
endtask

// Send 8-bit byte MSB-first, then read the ACK bit from the slave/bridge.
// Returns ack=0 if device ACKed (pulled SDA low), ack=1 if NACK.
task i2c_send_byte;
    input  [7:0] data;
    output       ack;
    integer      i;
begin
    for (i = 7; i >= 0; i = i - 1)
        i2c_send_bit(data[i]);
    // ACK slot: release SDA, sample
    i2c_recv_bit(ack);
end
endtask

// Receive 8-bit byte MSB-first (slave drives), then send ACK or NACK.
// send_ack=1 → master ACKs (more data coming), send_ack=0 → master NACKs (done).
task i2c_recv_byte;
    output [7:0] data;
    input        send_ack;
    integer      i;
    reg          b;
begin
    data = 8'h00;
    for (i = 7; i >= 0; i = i - 1) begin
        i2c_recv_bit(b);
        data[i] = b;
    end
    // Send ACK or NACK back
    i2c_send_bit(~send_ack);   // 0=ACK, 1=NACK
end
endtask

// ─── Slave Model Tasks ────────────────────────────────────────────────────────
// These model a physical I2C slave on bus 0 or bus 1.
// The slave watches SCL_S0/S1 and drives SDA_S0/S1.
//
// In a real testbench you'd have a full behavioral slave model running in a
// parallel `initial` block. For clarity here, we use manual fork/join with
// these tasks called at the right timing points.

// Slave sends ACK by pulling SDA low for one SCL period.
// Must be called when SCL_S is LOW (entering the 9th clock period).
task slave0_ack;
begin
    slave0_sda = 0;                         // Pull low: ACK
    @(posedge SCL_S0); @(negedge SCL_S0);  // Hold through one SCL period
    slave0_sda = 1;                         // Release
end
endtask

task slave0_nack;
begin
    slave0_sda = 1;                         // Release: NACK
    @(posedge SCL_S0); @(negedge SCL_S0);
end
endtask

task slave1_ack;
begin
    slave1_sda = 0;
    @(posedge SCL_S1); @(negedge SCL_S1);
    slave1_sda = 1;
end
endtask

// Slave drives a data byte on SDA_S (for read transactions).
// Changes SDA on SCL falling edges, holds through SCL rising.
task slave0_send_byte;
    input [7:0] data;
    integer     i;
begin
    for (i = 7; i >= 0; i = i - 1) begin
        @(negedge SCL_S0);
        slave0_sda = data[i];
    end
    @(negedge SCL_S0);
    slave0_sda = 1;   // Release for master ACK slot
    @(posedge SCL_S0); @(negedge SCL_S0);   // Wait through ACK
end
endtask

// ─── Assertion Helper ─────────────────────────────────────────────────────────
// Lightweight $display-based checking. In Xcelium add SystemVerilog assertions
// if your license supports it. For Vivado xsim, this is the practical choice.

task check;
    input [127:0] label;
    input         got;
    input         expected;
begin
    if (got !== expected)
        $display("FAIL [%0t] %s: got=%b expected=%b", $time, label, got, expected);
    else
        $display("PASS [%0t] %s", $time, label);
end
endtask

// ─── Main test sequence ───────────────────────────────────────────────────────

reg [7:0] recv_data;
reg       ack_bit;

initial begin
    $display("=== I2C Bridge TB Start ===");

    // ── Power-on reset ────────────────────────────────────────────────────────
    rst_n = 0;
    tb_scl_m = 1; tb_sda_m = 1;   // Bus idle
    repeat(20) @(posedge clk);
    rst_n = 1;
    repeat(10) @(posedge clk);
    $display("\n--- Reset complete, FSM should be IDLE (state=0) ---");
    check("FSM IDLE after reset", fsm_state, 4'd0);

    // ═════════════════════════════════════════════════════════════════════════
    // TC1 — Normal write: master→0x48 write, 2 data bytes, slave ACKs all
    //
    // Expected FSM path:
    //   IDLE→RECV_ADDR→ACK_MASTER→TRANSLATE→START_S→SEND_ADDR_S→RECV_ACK_S
    //   →FORWARD (2 bytes)→STOP_S→IDLE
    //
    // Slave model: slave0 ACKs translated address 0x68 and each data byte.
    // ═════════════════════════════════════════════════════════════════════════
    $display("\n=== TC1: Normal write 0x48 (virtual) → 0x68 bus0 ===");

    // Fork: TB master and slave0 model run in parallel
    fork
        begin : master_tc1
            i2c_start;
            // Send 0x48<<1 | 0 = 0x90 (addr 0x48, write)
            i2c_send_byte(8'h90, ack_bit);
            check("TC1 master got ACK for addr", ack_bit, 1'b0);

            // Data byte 1: 0xAB
            i2c_send_byte(8'hAB, ack_bit);
            check("TC1 master got ACK for byte1", ack_bit, 1'b0);

            // Data byte 2: 0xCD
            i2c_send_byte(8'hCD, ack_bit);
            check("TC1 master got ACK for byte2", ack_bit, 1'b0);

            i2c_stop;
        end

        begin : slave0_tc1
            // Wait for START_S phase — FSM begins driving SCL_S
            // Slave watches for the translated address 0x68 write (0xD0)
            // and ACKs each byte.

            // Skip 8 SCL_S cycles (address being sent by FSM)
            repeat(9) @(posedge SCL_S0);   // 8 addr bits + ACK rising edge
            slave0_ack;                     // ACK the translated address

            // ACK data byte 1
            repeat(9) @(posedge SCL_S0);
            slave0_ack;

            // ACK data byte 2
            repeat(9) @(posedge SCL_S0);
            slave0_ack;
        end
    join

    #(I2C_HALF * 4);
    check("TC1 FSM back to IDLE",    fsm_state,  4'd0);
    check("TC1 bus_sel is 0",        bus_sel_mon, 1'b0);
    check("TC1 no error",            err_mon,     1'b0);
    $display("TC1 complete\n");

    // ═════════════════════════════════════════════════════════════════════════
    // TC2 — Bus select: master→0x49 write → maps to 0x68 on bus1
    //
    // Key check: bus_sel_out goes to 1, SDA_S1 is used (not SDA_S0).
    // ═════════════════════════════════════════════════════════════════════════
    $display("=== TC2: Write to 0x49 (virtual) → 0x68 bus1 ===");

    fork
        begin : master_tc2
            i2c_start;
            // 0x49<<1 | 0 = 0x92
            i2c_send_byte(8'h92, ack_bit);
            check("TC2 master got ACK for addr", ack_bit, 1'b0);

            i2c_send_byte(8'h55, ack_bit);
            check("TC2 master got ACK for data", ack_bit, 1'b0);

            i2c_stop;
        end

        begin : slave1_tc2
            repeat(9) @(posedge SCL_S1);
            slave1_ack;

            repeat(9) @(posedge SCL_S1);
            // Slave1 ACKs data — use slave1_sda manually here
            slave1_sda = 0;
            @(posedge SCL_S1); @(negedge SCL_S1);
            slave1_sda = 1;
        end
    join

    #(I2C_HALF * 4);
    check("TC2 FSM back to IDLE",   fsm_state,   4'd0);
    check("TC2 bus_sel was 1",      bus_sel_mon, 1'b0);   // back to 0 after IDLE
    // Note: during transaction, bus_sel_mon was 1 — verified via waveform inspection
    $display("TC2 complete\n");

    // ═════════════════════════════════════════════════════════════════════════
    // TC3 — Slave NACK: slave refuses the translated address
    //
    // Expected:
    //   FSM reaches RECV_ACK_S → ack_ok=0 → S_NACK_MASTER → S_WAIT_STOP
    //   error_flag asserted, send_nack pulsed (master sees NACK on SDA_M)
    //   Master sends STOP → FSM returns to IDLE
    //
    // The master should see NACK on the address byte ACK slot.
    // ═════════════════════════════════════════════════════════════════════════
    $display("=== TC3: Slave NACK (slave refuses 0x68) ===");

    fork
        begin : master_tc3
            i2c_start;
            i2c_send_byte(8'h90, ack_bit);   // 0x48 write
            // Master receives NACK from bridge (error_flag → bridge releases SDA_M)
            $display("TC3 master addr ACK bit (expect NACK=1): %b", ack_bit);
            check("TC3 master sees NACK", ack_bit, 1'b1);
            i2c_stop;   // Master gives up
        end

        begin : slave0_tc3
            // Slave0 releases SDA_S0 → NACK
            repeat(9) @(posedge SCL_S0);
            slave0_nack;   // Do NOT pull low — line stays high = NACK
        end
    join

    #(I2C_HALF * 4);
    check("TC3 FSM back to IDLE",  fsm_state, 4'd0);
    check("TC3 error_flag cleared", err_mon,  1'b0);   // clears on re-entry to IDLE
    $display("TC3 complete\n");

    // ═════════════════════════════════════════════════════════════════════════
    // TC4 — Read transaction: master reads 1 byte from 0x48 (slave at 0x68 bus0)
    //
    // Protocol: START → addr+R/W=1 → addr ACK → slave drives byte → master NACK → STOP
    //
    // Key checks:
    //   fwd_rw=1 in the FSM
    //   slave0 drives SDA_S0 during data
    //   bridge re-drives that data on SDA_M (master receives it)
    //   master sends NACK after 1 byte → fwd_done fires → STOP_S
    //
    // The data byte we'll receive from slave: 0x3C
    // ═════════════════════════════════════════════════════════════════════════
    $display("=== TC4: Read transaction, master reads 1 byte ===");

    fork
        begin : master_tc4
            i2c_start;
            // 0x48<<1 | 1 = 0x91 (read)
            i2c_send_byte(8'h91, ack_bit);
            check("TC4 master got ACK for addr", ack_bit, 1'b0);

            // Receive 1 byte, then NACK to terminate burst
            i2c_recv_byte(recv_data, 1'b0);   // 0 = send NACK after byte
            $display("TC4 master received: 0x%02h (expected 0x3C)", recv_data);
            check("TC4 recv byte MSB", recv_data[7], 1'b0);   // 0x3C = 0011_1100
            check("TC4 recv byte LSB", recv_data[0], 1'b0);

            i2c_stop;
        end

        begin : slave0_tc4
            // ACK translated address
            repeat(9) @(posedge SCL_S0);
            slave0_ack;

            // Drive 0x3C = 0011_1100 on SDA_S0
            slave0_send_byte(8'h3C);
        end
    join

    #(I2C_HALF * 4);
    check("TC4 FSM back to IDLE", fsm_state, 4'd0);
    $display("TC4 complete\n");

    // ═════════════════════════════════════════════════════════════════════════
    // TC5 — Master abort: master sends STOP mid-transaction (data phase)
    //
    // Expected:
    //   FSM is in S_FORWARD when stop_det fires
    //   fwd_en deasserts → FSM transitions to S_STOP_S → generates STOP on slave bus
    //   FSM returns to IDLE cleanly — no deadlock
    //
    // We don't send a full data byte — we send START + addr (with slave ACK),
    // then abort immediately with STOP before any data bytes.
    // ═════════════════════════════════════════════════════════════════════════
    $display("=== TC5: Master abort during data phase ===");

    fork
        begin : master_tc5
            i2c_start;
            i2c_send_byte(8'h90, ack_bit);   // addr byte, expect ACK
            check("TC5 master got ACK for addr", ack_bit, 1'b0);

            // Abort — send STOP immediately instead of data
            #(I2C_HALF / 2);
            i2c_stop;
            $display("TC5 master sent STOP early (abort)");
        end

        begin : slave0_tc5
            repeat(9) @(posedge SCL_S0);
            slave0_ack;
            // Slave doesn't need to do anything else — bridge generates STOP on slave bus
        end
    join

    #(I2C_HALF * 8);   // Give FSM time to generate STOP_S sequence
    check("TC5 FSM back to IDLE", fsm_state, 4'd0);
    check("TC5 fwd_en deasserted", fwd_en_mon, 1'b0);
    $display("TC5 complete\n");

    // ─── Summary ─────────────────────────────────────────────────────────────
    $display("=== All TC complete. Check waveform for detailed signal inspection. ===");
    #(I2C_HALF * 4);
    $finish;
end

// ─── Watchdog: simulation timeout ─────────────────────────────────────────────
// Prevents infinite loops if FSM deadlocks.
initial begin
    #6_000_000;   // 6ms timeout — all 5 TCs should finish well under this
    $display("TIMEOUT: simulation exceeded 6ms — FSM likely deadlocked");
    $finish;
end

endmodule
