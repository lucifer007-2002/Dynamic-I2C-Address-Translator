`timescale 1ns/1ps

module top (
    input  wire       clk_100mhz,   // Basys-3 on-board 100 MHz oscillator
    input  wire       rst_n,        // Active-low reset — wire to a pushbutton

    // ── Master-side I2C bus ────────────────────────────────────────────────────
    input  wire       SCL_M,        // Master's clock — FPGA reads only
    inout  wire       SDA_M,        // Bidirectional — IOBUF instantiated below

    // ── Slave-side I2C bus (two buses, one per physical device) ───────────────
    output wire       SCL_S0,       // Slave bus 0 clock (driven by FSM)
    inout  wire       SDA_S0,       // Slave bus 0 SDA — IOBUF
    output wire       SCL_S1,       // Slave bus 1 clock (same SCL from FSM)
    inout  wire       SDA_S1,       // Slave bus 1 SDA — IOBUF

    // ── Status LEDs (optional, useful for debug) ───────────────────────────────
    output wire [3:0] dbg_leds      // {error_flag, bus_sel, fwd_en, fwd_rw}
);

// ─── Synchronizer chain for async inputs ─────────────────────────────────────
// SCL_M and SDA_M are asynchronous to clk_100mhz.
// Double-flip-flop synchronizer: eliminates metastability.
// 2-cycle latency is acceptable — at 400kHz, one SCL period = 250ns = 25 ticks.

reg scl_m_s0, scl_m_s1;
reg sda_m_raw;           // Direct read from SDA_M IOBUF
reg sda_m_s0, sda_m_s1;

always @(posedge clk_100mhz) begin
    scl_m_s0 <= SCL_M;
    scl_m_s1 <= scl_m_s0;
    sda_m_s0 <= sda_m_raw;
    sda_m_s1 <= sda_m_s0;
end

// These are the signals the rest of the design uses — not SCL_M/SDA_M directly.
wire scl_m_sync = scl_m_s1;
wire sda_m_sync = sda_m_s1;

// ─── IOBUF: SDA_M (master-side) ──────────────────────────────────────────────
// I = 1'b0    : we only ever drive LOW (open-drain)
// T = sda_m_oe: 1 = Hi-Z (release), 0 = drive low
// O = sda_m_raw: what's on the bus right now (always valid, feeds synchronizer)

wire sda_m_oe_w;   // From i2c_slave_if

IOBUF #(
    .DRIVE      (4),        // 4 mA — adequate for 400kHz on a short PCB trace
    .SLEW       ("SLOW"),   // SLOW: reduces EMI, fine for I2C
    .IBUF_LOW_PWR("TRUE"),  // Power optimization for input path
    .IOSTANDARD ("LVCMOS33") // Basys-3 I/O bank is 3.3V
) u_sda_m_iobuf (
    .IO (SDA_M),
    .O  (sda_m_raw),        // → synchronizer chain
    .I  (1'b0),             // Open-drain: we NEVER drive high
    .T  (sda_m_oe_w)        // 1 = release, 0 = pull low
);

// ─── IOBUF: SDA_S0 and SDA_S1 (slave-side) ───────────────────────────────────
// Same open-drain instantiation pattern.
// Two separate inout wires — one per physical I2C bus.
// bus_sel_out from FSM determines which bus is active.

wire sda_s_oe_final;   // Muxed output enable — see SDA mux below
wire sda_s0_raw, sda_s1_raw;

// Slave bus 0
IOBUF #(
    .DRIVE       (4),
    .SLEW        ("SLOW"),
    .IBUF_LOW_PWR("TRUE"),
    .IOSTANDARD  ("LVCMOS33")
) u_sda_s0_iobuf (
    .IO (SDA_S0),
    .O  (sda_s0_raw),       // Read from slave bus 0
    .I  (1'b0),
    .T  (sda_s0_oe_mux)     // See mux below
);

// Slave bus 1
IOBUF #(
    .DRIVE       (4),
    .SLEW        ("SLOW"),
    .IBUF_LOW_PWR("TRUE"),
    .IOSTANDARD  ("LVCMOS33")
) u_sda_s1_iobuf (
    .IO (SDA_S1),
    .O  (sda_s1_raw),       // Read from slave bus 1
    .I  (1'b0),
    .T  (sda_s1_oe_mux)     // See mux below
);

// ─── Slave SDA mux: bus_sel routes oe to correct IOBUF ───────────────────────
// When bus_sel_out=0: SDA_S0 is active, SDA_S1 stays Hi-Z (released)
// When bus_sel_out=1: SDA_S1 is active, SDA_S0 stays Hi-Z
// sda_s_oe_combined = muxed output from either FSM or master_if (see below)

wire bus_sel_out;
wire sda_s_oe_combined;   // Driven low = pull line low; Hi-Z = release

assign sda_s0_oe_mux = (bus_sel_out == 1'b0) ? sda_s_oe_combined : 1'b1;
assign sda_s1_oe_mux = (bus_sel_out == 1'b1) ? sda_s_oe_combined : 1'b1;

// The inactive bus is always released (1'b1 = Hi-Z).
// This is critical — you must not leave the inactive IOBUF driving low
// while the active bus is switching, or you'll create a spurious
// stop condition on the inactive bus.

// ─── SDA_S oe mux: FSM vs master_if ──────────────────────────────────────────
// During address phase (S_START_S, S_SEND_ADDR_S, S_RECV_ACK_S):
//   FSM drives sda_s_oe directly.
// During data phase (S_FORWARD):
//   master_if drives sda_s_oe.
// fwd_en is the selector:

wire fsm_sda_s_oe;
wire mif_sda_s_oe;
wire fwd_en;

assign sda_s_oe_combined = fwd_en ? mif_sda_s_oe : fsm_sda_s_oe;

// ─── Slave SDA input mux ─────────────────────────────────────────────────────
// bridge_fsm and i2c_master_if both need to READ from the slave bus.
// Feed both from the selected bus:

wire sda_s_in_active = (bus_sel_out == 1'b0) ? sda_s0_raw : sda_s1_raw;

// ─── SCL_S ────────────────────────────────────────────────────────────────────
// The FSM's scl_s output drives both slave bus clocks.
// Both slaves share one SCL — that is architecturally correct because only
// one slave is active at a time (bus_sel routes SDA; SCL going to both is safe
// because the inactive slave ignores traffic when its SDA is not being asserted).
//
// If you want strict isolation, use two separate SCL outputs with a bus_sel mux.
// For this project, shared SCL is fine and is the simpler choice.

wire scl_s_w;
assign SCL_S0 = scl_s_w;
assign SCL_S1 = scl_s_w;

// ─── Interconnect wires ───────────────────────────────────────────────────────

// slave_if → FSM
wire        start_det, stop_det;
wire [7:0]  rx_byte;
wire        rx_done;
wire        send_ack, send_nack;

// FSM → translator
wire [7:0]  xlat_in;
wire [6:0]  xlat_addr;
wire        xlat_bus_sel;

// FSM → master_if
wire        scl_rise_out, scl_fall_out;
wire        fwd_rw;
wire        fwd_done;

// master_if SDA
wire        mif_sda_m_oe;   // master_if drives master-side SDA during reads

// For reads (fwd_rw=1): master_if needs to drive SDA_M.
// Override sda_m_oe_w during read data phase:
assign sda_m_oe_w = fwd_en ? mif_sda_m_oe : slave_if_sda_m_oe;
wire slave_if_sda_m_oe;   // Renamed from i2c_slave_if output below

// ─── Module instantiations ────────────────────────────────────────────────────

i2c_slave_if u_slave_if (
    .clk        (clk_100mhz),
    .rst_n      (rst_n),
    .scl_m      (scl_m_sync),
    .sda_m_in   (sda_m_sync),
    .sda_m_oe   (slave_if_sda_m_oe),
    .start_det  (start_det),
    .stop_det   (stop_det),
    .rx_byte    (rx_byte),
    .rx_done    (rx_done),
    .send_ack   (send_ack),
    .send_nack  (send_nack)
);

addr_translator u_translator (
    .addr_rw_in (xlat_in),
    .addr_out   (xlat_addr),
    .bus_sel    (xlat_bus_sel),
    .valid      ()              // Unused — default passthrough in translator handles it
);

bridge_fsm u_fsm (
    .clk            (clk_100mhz),
    .rst_n          (rst_n),
    .start_det      (start_det),
    .stop_det       (stop_det),
    .rx_byte        (rx_byte),
    .rx_done        (rx_done),
    .send_ack       (send_ack),
    .send_nack      (send_nack),
    .xlat_in        (xlat_in),
    .xlat_addr      (xlat_addr),
    .xlat_bus_sel   (xlat_bus_sel),
    .sda_s_in       (sda_s_in_active),
    .sda_s_oe       (fsm_sda_s_oe),
    .scl_s          (scl_s_w),
    .scl_rise_out   (scl_rise_out),
    .scl_fall_out   (scl_fall_out),
    .bus_sel_out    (bus_sel_out),
    .fwd_en         (fwd_en),
    .fwd_rw         (fwd_rw),
    .fwd_done       (fwd_done),
    .dbg_state      (dbg_leds[3:0]),
    .error_flag     ()
);

i2c_master_if u_master_if (
    .clk        (clk_100mhz),
    .rst_n      (rst_n),
    .scl_rise_in (scl_rise_out),
    .scl_fall_in (scl_fall_out),
    .sda_m_in   (sda_m_sync),
    .sda_m_oe   (mif_sda_m_oe),
    .sda_s_in   (sda_s_in_active),
    .sda_s_oe   (mif_sda_s_oe),
    .fwd_en     (fwd_en),
    .fwd_rw     (fwd_rw),
    .fwd_done   (fwd_done),
    .stop_det   (stop_det),
    .start_det  (start_det)
);

// ─── Debug LEDs ───────────────────────────────────────────────────────────────
// Wire 4 LSBs of dbg_state to LEDs.
// In Vivado's logic analyser, also probe: start_det, stop_det, rx_byte,
// scl_rise_out, sda_s_oe_combined, bus_sel_out.

endmodule
