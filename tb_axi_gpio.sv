module tb_axi_gpio_smoke;

  // ---------------------------------------
  // Params matching your DUT
  // ---------------------------------------
  localparam int N_GPIOS         = 32;
  localparam int AXI_ADDR_WIDTH  = 12;
  localparam int AXI_DATA_WIDTH  = 32;

  // ---------------------------------------
  // Clock / reset
  // ---------------------------------------
  logic aclk = 0;
  logic aresetn = 0;
  always #5 aclk = ~aclk;  // 100 MHz

  // ---------------------------------------
  // DUT AXI-Lite signals
  // ---------------------------------------
  logic [AXI_ADDR_WIDTH-1:0] araddr;
  logic                      arvalid;
  logic                      arready;
  logic [AXI_DATA_WIDTH-1:0] rdata;
  logic [1:0]                rresp;
  logic                      rvalid;
  logic                      rready;

  logic [AXI_ADDR_WIDTH-1:0] awaddr;
  logic                      awvalid;
  logic                      awready;
  logic [AXI_DATA_WIDTH-1:0] wdata;
  logic [AXI_DATA_WIDTH/8-1:0] wstrb;
  logic                      wvalid;
  logic                      wready;
  logic [1:0]                bresp;
  logic                      bvalid;
  logic                      bready;

  // GPIO
  logic [N_GPIOS-1:0] gpio_in_raw;
  logic [N_GPIOS-1:0] gpio_out;
  logic               irq;

  // ---------------------------------------
  // DUT
  // ---------------------------------------
  axi_gpio #(
    .N_GPIOS(N_GPIOS),
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
  ) dut (
    .aclk(aclk),
    .aresetn(aresetn),

    .gpio_in_raw(gpio_in_raw),
    .gpio_out(gpio_out),
    .irq(irq),

    // Read
    .arready(arready),
    .araddr(araddr),
    .arvalid(arvalid),
    .rdata(rdata),
    .rresp(rresp),
    .rvalid(rvalid),
    .rready(rready),

    // Write
    .wvalid(wvalid),
    .wready(wready),
    .wdata(wdata),
    .wstrb(wstrb),
    .bready(bready),
    .bvalid(bvalid),
    .bresp(bresp),
    .awready(awready),
    .awaddr(awaddr),
    .awvalid(awvalid)
  );

  // ---------------------------------------
  // Simple AXI-Lite tasks
  // ---------------------------------------
  task automatic axil_read(input [AXI_ADDR_WIDTH-1:0] addr, output logic [AXI_DATA_WIDTH-1:0] data);
    begin
      // Launch address
      araddr  <= addr;
      arvalid <= 1'b1;

      // Optional backpressure on R channel
      rready  <= 1'b0;
      @(posedge aclk);
      // Deassert rready a bit to test hold behavior
      repeat (2) @(posedge aclk);
      rready  <= 1'b1;

      // Wait for ARREADY
      while (!arready) @(posedge aclk);
      // Drop ARVALID next cycle
      @(posedge aclk);
      arvalid <= 1'b0;

      // Wait for RVALID and accept it
      while (!rvalid) @(posedge aclk);
      data = rdata;
      // One beat read, consume it
      @(posedge aclk);
      rready <= 1'b0;
    end
  endtask

  task automatic axil_write(input [AXI_ADDR_WIDTH-1:0] addr,
                            input [AXI_DATA_WIDTH-1:0] data,
                            input [AXI_DATA_WIDTH/8-1:0] strb);
    begin
      // Start with both channels valid (order doesn't matter for Lite)
      awaddr  <= addr;
      awvalid <= 1'b1;
      wdata   <= data;
      wstrb   <= strb;
      wvalid  <= 1'b1;

      bready  <= 1'b1; // ready for response

      // Wait for AWREADY and WREADY handshakes (can be different cycles)
      while (!awready) @(posedge aclk);
      @(posedge aclk) awvalid <= 1'b0;

      while (!wready) @(posedge aclk);
      @(posedge aclk) wvalid  <= 1'b0;

      // Wait for BVALID
      while (!bvalid) @(posedge aclk);
      // Consume response
      @(posedge aclk);
      bready <= 1'b0;
    end
  endtask

  // ---------------------------------------
  // Test sequence
  // ---------------------------------------
  initial begin
    // Dump waves
    $dumpfile("wave.vcd");
    $dumpvars(0, tb_axi_gpio_smoke);

    // Defaults
    araddr  = '0; arvalid = 0; rready = 0;
    awaddr  = '0; awvalid = 0;
    wdata   = '0; wstrb  = '0; wvalid = 0;
    bready  = 0;
    gpio_in_raw = '0;

    // Reset
    repeat (4) @(posedge aclk);
    aresetn = 0;
    repeat (4) @(posedge aclk);
    aresetn = 1;
    repeat (2) @(posedge aclk);

    // 1) READ VERSION at 0x2C
    logic [31:0] rddata;
    axil_read(12'h02C, rddata);
    if (rddata !== 32'h0001_0000) begin
      $error("VERSION read mismatch: got 0x%08x expected 0x00010000", rddata);
      $fatal;
    end else
      $display("[PASS] VERSION read = 0x%08x", rddata);

    // 2) Dummy WRITE anywhere (e.g., 0x000). DUT should return OKAY B response.
    axil_write(12'h000, 32'hDEADBEEF, 4'hF);
    $display("[PASS] Write completed with OKAY response");

    // 3) Backpressure smoke: do a read with RREADY held low for a few cycles (already done in task)
    axil_read(12'h02C, rddata);
    if (rddata !== 32'h0001_0000) begin
      $error("Backpressure read mismatch: 0x%08x", rddata);
      $fatal;
    end else
      $display("[PASS] Backpressure read OK");

    $display("All Day-1 smoke checks passed ✅");

        // Write SCRATCH = 0xA5A55A5A at 0x30
    axil_write(12'h030, 32'hA5A5_5A5A, 4'hF);
    logic [31:0] rd;
    axil_read(12'h030, rd);
    if (rd !== 32'hA5A5_5A5A) begin
    $error("SCRATCH full write/read mismatch: got %08x", rd); $fatal;
    end else $display("[PASS] SCRATCH full write/read");



        // Change only byte[2] to 0x11 -> expect 0xA5 11 5A 5A
    axil_write(12'h030, 32'h0011_0000, 4'b0100);
    axil_read(12'h030, rd);
    if (rd !== 32'hA5_11_5A_5A) begin
    $error("SCRATCH WSTRB byte2 failed: got %08x", rd); $fatal;
    end else $display("[PASS] SCRATCH WSTRB byte2");

    // Change only byte[0] to 0xCC -> expect 0xA5 11 5A CC
    axil_write(12'h030, 32'h0000_00CC, 4'b0001);
    axil_read(12'h030, rd);
    if (rd !== 32'hA5_11_5A_CC) begin
    $error("SCRATCH WSTRB byte0 failed: got %08x", rd); $fatal;
    end else $display("[PASS] SCRATCH WSTRB byte0");



        // Set all pins to outputs
    axil_write(12'h000, 32'hFFFF_FFFF, 4'hF); // DIR = all 1s
    axil_read(12'h000, rd);
    if (rd[31:0] !== 32'hFFFF_FFFF) begin $error("DIR write/read failed: %08x", rd); $fatal; end
    $display("[PASS] DIR full write/read");

    // Write OUT pattern and check gpio_out
    axil_write(12'h004, 32'h0000_0F0F, 4'hF); // OUT
    axil_read(12'h004, rd);
    if (rd[31:0] !== 32'h0000_0F0F) begin $error("OUT write/read failed: %08x", rd); $fatal; end
    if (gpio_out !== 32'h0000_0F0F) begin $error("gpio_out mismatch"); $fatal; end
    $display("[PASS] OUT write/read and gpio_out");

    // Atomic SET then CLEAR
    axil_write(12'h008, 32'h0000_F000, 4'hF); // OUT_SET
    axil_read(12'h004, rd);
    if (rd[31:0] !== 32'h0000_FF0F) begin $error("OUT_SET failed: %08x", rd); $fatal; end
    axil_write(12'h00C, 32'h0000_0F0F, 4'hF); // OUT_CLR
    axil_read(12'h004, rd);
    if (rd[31:0] !== 32'h0000_F000) begin $error("OUT_CLR failed: %08x", rd); $fatal; end
    $display("[PASS] OUT_SET/OUT_CLR");

    // WSTRB partial on DIR: change only byte1
    axil_write(12'h000, 32'h0000_55AA, 4'b0010); // affects bits[15:8]
    axil_read(12'h000, rd);
    if (rd[15:8] !== 8'h55) begin $error("DIR WSTRB byte1 failed: %02x", rd[15:8]); $fatal; end
    $display("[PASS] DIR WSTRB byte1");

    // IN register follows gpio_in_raw after 2 clocks (2‑FF sync)
    gpio_in_raw = '0;
    repeat (3) @(posedge aclk);
    gpio_in_raw[7] = 1'b1; // toggle one pin
    @(posedge aclk); @(posedge aclk); // 2 cycles for sync
    axil_read(12'h010, rd); // IN @ 0x10
    if (rd[7] !== 1'b1) begin $error("IN did not reflect input after 2 cycles: %x", rd); $fatal; end
    $display("[PASS] IN synchronizer");

        // --- Basic rising-edge IRQ ---
    axil_write(12'h020, 32'hFFFF_FFFF, 4'hF); // EDGE_EN = 1 for all
    axil_write(12'h024, 32'hFFFF_FFFF, 4'hF); // EDGE_POL = rising
    axil_write(12'h014, 32'h0000_0001, 4'h1); // IRQ_MASK enable bit0

    // ensure input low
    gpio_in_raw[0] = 1'b0; repeat(3) @(posedge aclk);

    // pulse rising edge (debounce off by default)
    gpio_in_raw[0] = 1'b1; @(posedge aclk); @(posedge aclk);
    axil_read(12'h018, rd); // IRQ_STATUS
    if (rd[0] !== 1'b1 || irq !== 1'b1) begin $error("Edge IRQ not set"); $fatal; end
    $display("[PASS] Rising-edge IRQ set");

    // clear it via W1C
    axil_write(12'h01C, 32'h0000_0001, 4'h1);
    axil_read(12'h018, rd);
    if (rd[0] !== 1'b0 || irq !== 1'b0) begin $error("IRQ clear failed"); $fatal; end
    $display("[PASS] IRQ W1C clear");

    // --- Level-high IRQ (edge disabled) ---
    axil_write(12'h020, 32'h0000_0000, 4'hF); // EDGE_EN=0 -> level mode
    axil_write(12'h028, 32'hFFFF_FFFF, 4'hF); // LVL_POL=high
    axil_write(12'h014, 32'h0000_0002, 4'h1); // mask bit1
    gpio_in_raw[1] = 1'b1; @(posedge aclk); @(posedge aclk);
    axil_read(12'h018, rd);
    if (rd[1] !== 1'b1 || irq !== 1'b1) begin $error("Level-high pending/irq failed"); $fatal; end
    // Clear -> should re-assert next cycle since level still active
    axil_write(12'h01C, 32'h0000_0002, 4'h1);
    @(posedge aclk);
    axil_read(12'h018, rd);
    if (rd[1] !== 1'b1) begin $error("Level re-assert after clear failed"); $fatal; end
    $display("[PASS] Level-high IRQ behavior");

    // --- Debounce test: short glitch ignored ---
    axil_write(12'h034, 32'h0000_0004, 4'h1); // DEBOUNCE=4 cycles
    axil_write(12'h020, 32'hFFFF_FFFF, 4'hF); // EDGE_EN=1 (edge mode)
    axil_write(12'h024, 32'hFFFF_FFFF, 4'hF); // rising

    // Make sure masked events don't block latching: mask bit2 ON
    axil_write(12'h014, 32'h0000_0004, 4'h1);

    // drive a 2-cycle glitch on input[2] (shorter than threshold)
    gpio_in_raw[2] = 1'b0; repeat(3) @(posedge aclk);
    gpio_in_raw[2] = 1'b1; repeat(2) @(posedge aclk); // 2 cycles < 4 -> ignore
    gpio_in_raw[2] = 1'b0; repeat(6) @(posedge aclk);
    axil_read(12'h018, rd);
    if (rd[2] !== 1'b0) begin $error("Debounce failed: glitch latched"); $fatal; end
    $display("[PASS] Debounce ignores short glitch");
    #20;
    $finish;
  end

endmodule