// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is an example of a (trivially simple) user project,
 * showing how the user project can connect to the logic
 * analyzer, the wishbone bus, and the I/O pads.
 *
 * This project generates an integer count, which is output
 * on the user area GPIO pads (digital output only).  The
 * wishbone connection allows the project to be controlled
 * (start and stop) from the management SoC program.
 *
 * See the testbenches in directory "mprj_counter" for the
 * example programs that drive this user project.  The three
 * testbenches are "io_ports", "la_test1", and "la_test2".
 *
 *-------------------------------------------------------------
 */

module user_proj_example #(
    parameter BITS = 16
)(
`ifdef USE_POWER_PINS
    inout vdd,	// User area 1 1.8V supply
    inout vss,	// User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [63:0] la_data_in,
    output [63:0] la_data_out,
    input  [63:0] la_oenb,

    // IOs
    input  [BITS-1:0] io_in,
    output [BITS-1:0] io_out,
    output [BITS-1:0] io_oeb,

    // IRQ
    output [2:0] irq
);
    wire clk;
    wire rst;

    wire [BITS-1:0] rdata; 
    wire [BITS-1:0] wdata;
    wire [BITS-1:0] count;

    wire valid;
    wire [3:0] wstrb;
    wire [BITS-1:0] la_write;

    // WB MI A
    assign valid = wbs_cyc_i && wbs_stb_i; 
    assign wstrb = wbs_sel_i & {4{wbs_we_i}};
    assign wbs_dat_o = {{(32-BITS){1'b0}}, rdata};
    assign wdata = wbs_dat_i[BITS-1:0];

    // IO
    assign io_out = count;
    assign io_oeb = {(BITS){rst}};

    // IRQ
    assign irq = 3'b000;	// Unused

    // LA
    assign la_data_out = {{(64-BITS){1'b0}}, count};
    // Assuming LA probes [61:46] are for controlling the count register  
    assign la_write = ~la_oenb[61:62-BITS] & ~{BITS{valid}};
    // Assuming LA probes [63:62] are for controlling the count clk & reset  
    assign clk = (~la_oenb[62]) ? la_data_in[62]: wb_clk_i;
    assign rst = (~la_oenb[63]) ? la_data_in[63]: wb_rst_i;

    counter #(
        .BITS(BITS)
    ) counter(
        .clk(clk),
        .reset(rst),
        .ready(wbs_ack_o),
        .valid(valid),
        .rdata(rdata),
        .wdata(wbs_dat_i[BITS-1:0]),
        .wstrb(wstrb),
        .la_write(la_write),
        .la_input(la_data_in[61:62-BITS]),
        .count(count)
    );

    airi5c_top_asic DUT(
        .clk(CLK),
        .nreset(nRESET),
        .ext_interrupt(EXT_INT),
    
        .tdi(tdi),
        .tdo(tdo),
        .tms(tms),
        .tck(tck),
    
        .testmode(testmode),
        .sdi(sdi),
        .sdo(sdo),
        .sen(sen),

        .imem_haddr(imem_haddr),
        .imem_hwrite(imem_hwrite),
        .imem_hsize(imem_hsize),
        .imem_hburst(imem_hburst),
        .imem_hmastlock(imem_hmastlock),
        .imem_hprot(imem_hprot),
        .imem_htrans(imem_htrans),
        .imem_hwdata(imem_hwdata),
        .imem_hrdata(imem_hrdata),
        .imem_hready(imem_hready),
        .imem_hresp(imem_hresp),

        .dmem_haddr(dmem_haddr),
        .dmem_hwrite(dmem_hwrite),
        .dmem_hsize(dmem_hsize),
        .dmem_hburst(dmem_hburst),
        .dmem_hmastlock(dmem_hmastlock),
        .dmem_hprot(dmem_hprot),
        .dmem_htrans(dmem_htrans),
        .dmem_hwdata(dmem_hwdata),
        .dmem_hrdata(dmem_hrdata),
        .dmem_hready(dmem_hready),
        .dmem_hresp(dmem_hresp),

      // GPIO 0
        .gpio0_out(gpio0_out),
        .gpio0_in(gpio0_in),
        .gpio0_oe(gpio0_oe),

      // UART 0
        .uart0_tx(uart0_tx),
        .uart0_rx(uart0_rx),

      // SPI 0
        .spi0_mosi_out(spi0_mosi_out),
        .spi0_mosi_in(spi0_mosi_in),
        .spi0_mosi_oe(spi0_mosi_oe),

        .spi0_miso_out(spi0_miso_out),
        .spi0_miso_in(spi0_miso_in),
        .spi0_miso_oe(spi0_miso_oe),

        .spi0_sclk_out(spi0_sclk_out),
        .spi0_sclk_in(spi0_sclk_in),
        .spi0_sclk_oe(spi0_sclk_oe),

        .spi0_ss_out(spi0_ss_out),
        .spi0_ss_in(spi0_ss_in),
        .spi0_ss_oe(spi0_ss_oe),

        .debug_out(debug_out)
    );

endmodule

module counter #(
    parameter BITS = 16
)(
    input clk,
    input reset,
    input valid,
    input [3:0] wstrb,
    input [BITS-1:0] wdata,
    input [BITS-1:0] la_write,
    input [BITS-1:0] la_input,
    output reg ready,
    output reg [BITS-1:0] rdata,
    output reg [BITS-1:0] count
);

    always @(posedge clk) begin
        if (reset) begin
            count <= 0;
            ready <= 0;
        end else begin
            ready <= 1'b0;
            if (~|la_write) begin
                count <= count + 1;
            end
            if (valid && !ready) begin
                ready <= 1'b1;
                rdata <= count;
                if (wstrb[0]) count[7:0]   <= wdata[7:0];
                if (wstrb[1]) count[15:8]  <= wdata[15:8];
            end else if (|la_write) begin
                count <= la_write & la_input;
            end
        end
    end

endmodule
`default_nettype wire
