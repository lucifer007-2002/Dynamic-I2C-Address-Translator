

# Dynamic I2C Address Translator — FPGA (Basys-3, Artix-7)

## What This Is

I2C uses a 7-bit address space. If two identical sensors share the same hardwired address (e.g., two MPU-6050s both fixed at 0x68), you cannot put them on the same bus — both respond simultaneously, SDA lines collide, and communication fails. Hardware muxes exist but waste GPIO and don't scale.

This project solves it at the protocol level. The FPGA sits between the master and two physical slaves, operating three separate I2C buses simultaneously. The master talks to virtual addresses (0x48 and 0x49). The FPGA intercepts each transaction, rewrites the address, and re-drives it on the correct slave bus. The master never sees the real addresses. The slaves never know they've been remapped. This is live transaction interception — not a register write, not a mux.

```
[Master] ── SCL_M / SDA_M ──► [FPGA Bridge] ── SCL_S0 / SDA_S0 ──► [Slave A: 0x68]
                                             └── SCL_S1 / SDA_S1 ──► [Slave B: 0x68]
```

---

## What Was Built

### RTL — 5 Modules

**`i2c_slave_if.v`** — Master-facing decoder. Watches SCL_M and SDA_M, detects START/STOP conditions by monitoring SDA transitions relative to SCL level (not on clock edges — that's the wrong approach and misses them). Shifts in the 8-bit address frame MSB-first on SCL rising edges. Drives ACK back to the master via a latching mechanism that holds SDA_M low for the full 9th SCL period, not just a single system clock pulse.

**`addr_translator.v`** — Combinational lookup table. Takes the 7-bit address from the master, outputs the translated physical address and which slave bus to use. Currently hardcoded (`0x48 → 0x68 bus0`, `0x49 → 0x68 bus1`). The case statement is the intentional extension point — replacing it with a BRAM register file would make the table runtime-writable.

**`bridge_fsm.v`** — The core of the project. An 11-state FSM that owns the entire transaction lifecycle. It also contains the slave-bus SCL generator — a counter-based clock divider running at 400 kHz (125 ticks at 100 MHz system clock). States: IDLE → RECV_ADDR → ACK_MASTER → TRANSLATE → START_S → SEND_ADDR_S → RECV_ACK_S → FORWARD → STOP_S → NACK_MASTER → WAIT_STOP. The FSM generates timing-compliant START and STOP conditions on the slave bus (tHD;STA ≥ 600 ns, tSU;STO ≥ 600 ns enforced in RTL via counter-based delay, not just hoping synthesis gets it right). One non-obvious bug fixed here: releasing SDA while SCL is high after transmitting a zero bit creates a valid STOP condition by I2C spec — slave aborts. The RECV_ACK_S state uses a 4-sub-state sequence to delay SDA release until SCL falls, eliminating this spurious STOP.

**`i2c_master_if.v`** — Data-phase relay. Takes over SDA once the address translation is complete. Handles bidirectional forwarding — write transactions relay master SDA to slave, read transactions relay slave SDA back to master. Does not generate its own SCL. It consumes `scl_rise_out` / `scl_fall_out` exported by the FSM — one clock source, zero contention. One edge case handled: the first SCL falling edge after handoff belongs to the FSM's ACK cleanup cycle, not the first data bit. A `first_fall` flag discards exactly one falling edge on entry to prevent bit-timing shift.

**`top.v`** — Integration shell. Explicitly instantiates Xilinx `IOBUF` primitives for every SDA line (4 mA drive, SLOW slew, LVCMOS33). I2C SDA is open-drain bidirectional — you cannot use a plain inout and trust synthesis to infer the right primitive. The module also contains a double-FF synchronizer for SCL_M and SDA_M (external async signals), an SDA mux that hands off slave-bus drive control from the FSM (address phase) to master_if (data phase) based on `fwd_en`, and a bus-select mux that routes SDA activity to the correct slave bus while keeping the inactive bus released.

---

### Testbench — `tb_i2c_bridge.v`

Five directed test cases, each targeting a distinct failure mode:

- **TC1** — Normal write: master writes to 0x48, slave on bus 0 ACKs the translated address 0x68, two data bytes relayed and ACKed.
- **TC2** — Bus select: master writes to 0x49, verifies `bus_sel_out` switches to 1 and SDA_S1 is used instead of SDA_S0.
- **TC3** — Slave NACK: slave refuses the translated address, FSM propagates NACK back to master, `error_flag` asserts, FSM recovers to IDLE on master STOP.
- **TC4** — Read transaction: master requests data, slave drives a byte on SDA_S0, bridge re-drives it on SDA_M, master NACKs to terminate the burst.
- **TC5** — Master abort: master sends STOP mid-transaction during the data phase, FSM detects `stop_det`, deasserts `fwd_en`, generates a clean STOP on the slave bus, returns to IDLE without deadlock.

All I2C operations (START, STOP, send byte, receive byte, ACK, NACK) are implemented as reusable Verilog tasks. Simulation dumps a VCD file for GTKWave inspection. A watchdog timer kills the simulation at 6 ms if the FSM deadlocks.

---

### Constraints — `constraints.xdc`

System clock declared at 100 MHz on pin W5. I2C ports assigned to Pmod headers JA, JB, JC (LVCMOS33, 4 mA). Two key non-obvious constraints:

A virtual 400 kHz clock (`i2c_clk_virtual`) is declared for STA to analyze paths touching I2C signals — this does **not** generate a 400 kHz clock; that comes from the RTL counter. The actual 400 kHz generation is entirely in `bridge_fsm.v`.

A `set_multicycle_path 125` is applied to the SCL_S output register. Without it, STA tries to fit the SCL_S launch-to-output path into a 10 ns budget (one system clock cycle), which is technically wrong — the SCL_S register changes every 1250 ns by design. The multicycle constraint corrects STA's requirement to match the actual RTL behavior.

False paths declared on synchronizer first-stage FFs (SCL_M, SDA_M inputs) and on the reset port. Inter-clock paths between `i2c_scl_m` and `clk_100mhz` domains marked false — the double-FF synchronizer handles metastability, not STA.

**Timing result:** WNS ≈ +8.4 ns. Zero failing paths. Design utilizes approximately 80–150 LUTs and 100–180 FFs on Artix-7-35T.

---

## Known Limitations

**Clock stretching not implemented.** If a slow slave holds SCL_S low, the FSM's counter continues regardless and assumes the clock transitioned on schedule. Fix: feed SCL_S back as an input, stall `scl_cnt` when the line is held low unexpectedly.

**Repeated START not handled.** If the master issues a START without a preceding STOP (direction change within a transaction), the FSM does not distinguish it from a new transaction START. Explicit repeated-START detection would be needed for full I2C compliance.

**Address table is static.** The translation mappings are hardcoded in a `case` statement. Runtime reconfigurability would require replacing it with a small dual-port BRAM and a configuration write interface.

---

## File Structure

```
i2c_address_translator/
├── rtl/
│   ├── top.v
│   ├── bridge_fsm.v
│   ├── i2c_slave_if.v
│   ├── i2c_master_if.v
│   └── addr_translator.v
├── tb/
│   └── tb_i2c_bridge.v
├── constraints/
│   └── constraints.xdc
├── sim/
│   └── tb_i2c_bridge.vcd
└── reports/
    ├── timing_summary.rpt
    └── utilization.rpt
```
