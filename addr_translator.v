module addr_translator (
    input  wire [7:0] addr_rw_in,    // Raw byte from master: [7:1]=addr, [0]=R/W
    output reg  [6:0] addr_out,      // Translated 7-bit address for slave bus
    output reg        bus_sel,       // 0 = Slave bus 0, 1 = Slave bus 1
    output reg        valid          // 0 = no mapping found (passthrough/unknown)
);

    wire [6:0] addr_in = addr_rw_in[7:1];   // strip R/W bit

    always @(*) begin
        // Defaults — safe state if no match
        addr_out = addr_in;     // passthrough
        bus_sel  = 1'b0;
        valid    = 1'b1;

        case (addr_in)
            7'h48: begin
                addr_out = 7'h68;   // Map virtual 0x48 → physical 0x68
                bus_sel  = 1'b0;    // On slave bus 0
                valid    = 1'b1;
            end
            7'h49: begin
                addr_out = 7'h68;   // Map virtual 0x49 → also physical 0x68
                bus_sel  = 1'b1;    // But on slave bus 1 (different device)
                valid    = 1'b1;
            end
            default: begin
                addr_out = addr_in; // No remapping — forward as-is
                bus_sel  = 1'b0;
                valid    = 1'b1;
            end
        endcase
    end

endmodule
