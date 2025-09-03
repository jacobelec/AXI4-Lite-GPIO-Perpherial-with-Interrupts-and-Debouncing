module axi_gpio #(
    parameter int N_GPIOS = 32,
    parameter int AXI_ADDR_WIDTH = 12,
    parameter int AXI_DATA_WIDTH = 32
)(
   input aclk,
   input aresetn, 
   input  logic [N_GPIOS-1:0] gpio_in_raw ,
   output logic [N_GPIOS-1:0] gpio_out,
   output logic irq, // interrupt

   //// Read Address Channel

   output logic arready,
   input logic [AXI_ADDR_WIDTH-1:0] araddr,
   input logic arvalid,

   //// Read Data Channel
   output logic [AXI_DATA_WIDTH-1:0] rdata,
   output logic [1:0] rresp,
   output logic rvalid,
   input logic rready,

   /// write data channel
   input logic wvalid,
   output logic wready,
   input logic [AXI_DATA_WIDTH-1:0] wdata,
   input logic [AXI_DATA_WIDTH/8-1:0] wstrb,
  

   /// Write Response Channel
   input logic bready,
   output logic bvalid,
   output logic [1:0] bresp,

   /// Write Address Channel
   output logic awready,
   input logic [AXI_ADDR_WIDTH-1:0] awaddr,
   input logic awvalid
);


  localparam logic [31:0] VERSION = 32'h0001_0000;
  localparam int WORD_IDX_W = (AXI_ADDR_WIDTH >= 2) ? (AXI_ADDR_WIDTH-2) : 1;
  localparam logic [WORD_IDX_W-1:0] VERSION_WORD = 'h0B; // 0x2C word index
  localparam logic [WORD_IDX_W-1:0] SCRATCH_WORD = 'h0C; // 0x30 word index
  localparam logic [WORD_IDX_W-1:0] DIR_WORD       = 'h000; // 0x00
  localparam logic [WORD_IDX_W-1:0] OUT_WORD       = 'h001; // 0x04
  localparam logic [WORD_IDX_W-1:0] OUT_SET_WORD   = 'h002; // 0x08 (W1S)
  localparam logic [WORD_IDX_W-1:0] OUT_CLR_WORD   = 'h003; // 0x0C (W1C)
  localparam logic [WORD_IDX_W-1:0] IN_WORD        = 'h004; // 0x10

  localparam logic [WORD_IDX_W-1:0] IRQ_MASK_WORD  = 'h005; // 0x14, enables or disables interrupt
  localparam logic [WORD_IDX_W-1:0] IRQ_STATUS_WORD= 'h006; // 0x18 (RO, sticky) //
  localparam logic [WORD_IDX_W-1:0] IRQ_CLEAR_WORD = 'h007; // 0x1C (W1C)
  localparam logic [WORD_IDX_W-1:0] EDGE_EN_WORD   = 'h008; // 0x20 (1=edge, 0=level)
  localparam logic [WORD_IDX_W-1:0] EDGE_POL_WORD  = 'h009; // 0x24 (1=rising, 0=falling)
  localparam logic [WORD_IDX_W-1:0] LVL_POL_WORD   = 'h00A; // 0x28 (1=high, 0=low)
  localparam logic [WORD_IDX_W-1:0] DEBOUNCE_WORD  = 'h00D; // 0x34 (global cycles)

  logic [AXI_DATA_WIDTH-1:0] scratch_q; //SCRATCH @0x30
  logic [N_GPIOS-1:0] dir_q, out_q;
  logic [N_GPIOS-1:0] in_meta, in_sync; // 2‑FF input synchronizer

  // one-cycle clear mask for IRQ_STATUS
  logic [N_GPIOS-1:0] clr_mask, clr_mask_next;

  // IRQ controls
  logic [N_GPIOS-1:0] irq_mask_q, irq_status_q, edge_en_q, edge_pol_q, lvl_pol_q;
  // Debounce: global threshold and per-bit debounced value
  logic [7:0]         debounce_cfg_q;            // 0 = off
  logic [N_GPIOS-1:0] in_db_q, in_db_d;         // debounced and delayed (for edge detect)

  logic [AXI_ADDR_WIDTH-1:0] araddr_q;
  logic [AXI_ADDR_WIDTH-1:0] awaddr_q;
  logic [AXI_DATA_WIDTH-1:0] wdata_q;
  logic [AXI_DATA_WIDTH/8-1:0] wstrb_q;
  logic                  ar_hs; // address handshake
  assign ar_hs         = arvalid & arready;
  

  function automatic logic [AXI_DATA_WIDTH-1:0] apply_wstrb(
    input logic [AXI_DATA_WIDTH-1:0] cur,
    input logic [AXI_DATA_WIDTH-1:0] wdata_i,
    input logic [AXI_DATA_WIDTH/8-1:0] wstrb_i
);
logic [AXI_DATA_WIDTH-1:0] mask;
mask = { {8{wstrb_i[3]}}, {8{wstrb_i[2]}}, {8{wstrb_i[1]}}, {8{wstrb_i[0]}} };
return (cur & ~mask) | (wdata_i & mask);
/// bytes with wstrb[i] = 1, wdata. 
/// bytes with wstrb[i] = 0, stay as cur
  endfunction


//// Helper that applies WSTRB to a GPIO-width register

function automatic [N_GPIOS-1:0] apply_wstrb_to_gpio(
  input [N_GPIOS-1:0]            prev,
  input [AXI_DATA_WIDTH-1:0]     wdata_i,
  input [AXI_DATA_WIDTH/8-1:0]   wstrb_i
);

  automatic logic [AXI_DATA_WIDTH-1:0] prev_ext, nxt_ext;
  prev_ext                 = '0;
  prev_ext[N_GPIOS-1:0]    = prev;
  nxt_ext                  = apply_wstrb(prev_ext, wdata_i, wstrb_i);
  return nxt_ext[N_GPIOS-1:0];

endfunction

function automatic [N_GPIOS-1:0] w1_mask_gpio(
  input [AXI_DATA_WIDTH-1:0]     wdata_i,
  input [AXI_DATA_WIDTH/8-1:0]   wstrb_i
);
  automatic logic [AXI_DATA_WIDTH-1:0] strobe_mask = '0;
  automatic logic [AXI_DATA_WIDTH-1:0] masked;
  for (int b = 0; b < AXI_DATA_WIDTH/8; b++)
    strobe_mask[8*b +: 8] = {8{wstrb_i[b]}};
    masked = wdata_i & strobe_mask;
  return masked[N_GPIOS-1:0];
endfunction


///Synchronizer
always_ff @(posedge aclk or negedge aresetn) begin
  if (!aresetn) begin
    in_meta <= '0;
    in_sync <= '0;
  end else begin
    in_meta <= gpio_in_raw; // async inputs
    in_sync <= in_meta;     // use in_sync for IN register & edge logic (later)
  end
end

// Simple per-bit debounce using a global cycle threshold.
// Rule: if debounce_cfg_q == 0 -> pass-through (no extra delay beyond 2FF).
logic [7:0] db_cnt   [N_GPIOS]; // small per-bit counters

always_ff @(posedge aclk or negedge aresetn) begin
  if (!aresetn) begin
    for (int i=0;i<N_GPIOS;i++) begin
      db_cnt[i] <= '0;
    end
    in_db_q <= '0;
  end else begin
    if (debounce_cfg_q == 8'd0) begin
      // no debounce: just follow synchronized input
      in_db_q <= in_sync;
      for (int i=0;i<N_GPIOS;i++) db_cnt[i] <= '0;
    end else begin
      for (int i=0;i<N_GPIOS;i++) begin
        if (in_sync[i] == in_db_q[i]) begin
          db_cnt[i] <= '0; // stable matches current debounced -> reset counter
        end else begin
          // candidate change: count stable time
          db_cnt[i] <= db_cnt[i] + 8'd1;
          if (db_cnt[i] >= debounce_cfg_q) begin
            in_db_q[i] <= in_sync[i]; // accept new level
            db_cnt[i]  <= '0;
          end
        end
      end
    end
  end
end

// one-cycle delayed for edge detect
always_ff @(posedge aclk or negedge aresetn) begin
  if (!aresetn) in_db_d <= '0;
  else          in_db_d <= in_db_q;
end


// Edge detect (from debounced signals)
wire [N_GPIOS-1:0] rise =  in_db_q & ~in_db_d;
wire [N_GPIOS-1:0] fall = ~in_db_q &  in_db_d;

// Qualified event candidates per mode/polarity
wire [N_GPIOS-1:0] edge_evt  = (edge_pol_q ? rise : fall) & edge_en_q;
wire [N_GPIOS-1:0] level_evt = (in_db_q    ^ ~lvl_pol_q) & ~edge_en_q;
// level_evt is 1 when input == lvl_pol (HIGH if 1, LOW if 0)

wire [N_GPIOS-1:0] new_events = edge_evt | level_evt;

// Sticky pending with W1C clear
// Race policy: event wins if simultaneous with clear (i.e., set has priority).
always_ff @(posedge aclk or negedge aresetn) begin
  if (!aresetn) begin
    irq_status_q <= '0;
  end else begin
    // W1C mask from a write to IRQ_CLEAR (produced in write decode below)
    // We’ll build 'clr_mask' in the write decode and use it here:
    irq_status_q <= (irq_status_q | new_events) & ~clr_mask;
  end
end

// Combined level-IRQ output
assign irq = |(irq_status_q & irq_mask_q);




// simple write channel
    logic aw_hs, w_hs;
    assign aw_hs = awvalid && awready;
    assign w_hs = wvalid && wready;

    always_ff @(posedge aclk or negedge aresetn)begin
        if(!aresetn)begin
            awready <= 1'b1;
            //wvalid <= 1'b0;
            bvalid <= 1'b0;
            //wdata <= '0;
            bresp <= 2'b00;
            wready <= 1'b1;
            awaddr_q <= '0;
            wdata_q <= '0;
            wstrb_q <= '0;
            scratch_q <= '0;
            irq_mask_q     <= '0;
            edge_en_q      <= '0;            // default to LEVEL mode
            edge_pol_q     <= {N_GPIOS{1'b1}};  // rising by default (for edge mode)
            lvl_pol_q      <= {N_GPIOS{1'b1}};  // HIGH by default (for level mode)
            debounce_cfg_q <= 8'd0;          // debounce off (pass-through after 2FF)
            clr_mask <= '0;
            
        end else begin
            // accept address/data in any order
            if(awready && awvalid)begin 
                awready <= 1'b0;    // after the clk edge, awready becomes 0, result of the handshake
                awaddr_q <= awaddr; // this is the result of handshake (gets the address value)
            end
            
            if(wready && wvalid) begin
                wready <= 1'b0;
                wdata_q <= wdata;
                wstrb_q <= wstrb;
            end
            clr_mask_next <= '0;
        
            if (!bvalid && !awready && !wready)begin // !bvalid -> we're not already holding a write response
            //!awready -> address has been accepted //!wready -> data has been accepted 

                unique case (awaddr_q[AXI_ADDR_WIDTH-1:2])
                    DIR_WORD: dir_q <= apply_wstrb_to_gpio(dir_q,wdata_q,wstrb_q);
                    OUT_WORD: out_q <= apply_wstrb_to_gpio(out_q,wdata_q,wstrb_q);

                    OUT_SET_WORD: begin // write 1 to set
                        automatic logic [N_GPIOS-1:0] m = w1_mask_gpio(wdata_q, wstrb_q);
                        out_q <= out_q | m;  
                    end

                    OUT_CLR_WORD: begin // write 1 to clear
                        automatic logic [N_GPIOS-1:0] m = w1_mask_gpio(wdata_q, wstrb_q);
                        out_q <= out_q & ~m;
                    end

                    SCRATCH_WORD: scratch_q <= apply_wstrb(scratch_q, wdata_q, wstrb_q);

                    IRQ_MASK_WORD:  irq_mask_q  <= apply_wstrb_to_gpio(irq_mask_q,  wdata_q, wstrb_q);
                    EDGE_EN_WORD:   edge_en_q   <= apply_wstrb_to_gpio(edge_en_q,   wdata_q, wstrb_q);
                    EDGE_POL_WORD:  edge_pol_q  <= apply_wstrb_to_gpio(edge_pol_q,  wdata_q, wstrb_q);
                    LVL_POL_WORD:   lvl_pol_q   <= apply_wstrb_to_gpio(lvl_pol_q,   wdata_q, wstrb_q);

                    IRQ_CLEAR_WORD: begin
                      // W1C clear of pending bits; honor WSTRB by byte
                      automatic logic [N_GPIOS-1:0] m = w1_mask_gpio(wdata_q, wstrb_q);
                      // create a one-cycle pulse 'clr_mask' for the status update block
                      clr_mask_next <= m;
                    end

                     DEBOUNCE_WORD: begin
                        // Use upper byte (bits[15:8]) or just low 8 bits; here we use low 8 bits
                        // Honor WSTRB; only byte0 matters.
                        if (wstrb_q[0]) debounce_cfg_q <= wdata_q[7:0];
                      end

                    default: ;
                endcase       
                bvalid <= 1'b1;
                bresp <= 2'b00; // okay
        end
            if(bvalid && bready)begin
                bvalid <= 1'b0;
                awready <= 1'b1;
                wready <= 1'b1;
        end

        end

    end

    


always_ff @(posedge aclk or negedge aresetn) begin
  if (!aresetn) clr_mask <= '0;
  else          clr_mask <= clr_mask_next;
end



   
  



  
  // ready when not holding a pending response
  always_ff @(posedge aclk or negedge aresetn)begin
    if(!aresetn) begin
        arready <= 1'b1;
        rvalid <= 1'b0;
        rdata <= '0;
        rresp <= 2'b00; //OKAY
        dir_q <= '0;
        out_q <= '0;
    end
    else begin
        //Accept AR when we are ready and no pending R
        if(arready && arvalid) begin
            araddr_q <= araddr;
            arready <= 1'b0;
            rvalid <= 1'b1;
            //Decode
            unique case (araddr[AXI_ADDR_WIDTH-1:2])
                DIR_WORD:      rdata <= {{(AXI_DATA_WIDTH-N_GPIOS){1'b0}}, dir_q}; // zero extended it
                OUT_WORD:      rdata <= {{(AXI_DATA_WIDTH-N_GPIOS){1'b0}}, out_q};
                OUT_SET_WORD:  rdata <= {{(AXI_DATA_WIDTH-N_GPIOS){1'b0}}, out_q}; // readback = current OUT
                OUT_CLR_WORD:  rdata <= {{(AXI_DATA_WIDTH-N_GPIOS){1'b0}}, out_q}; // readback = current OUT
                IN_WORD:         rdata <= {{(AXI_DATA_WIDTH-N_GPIOS){1'b0}}, in_db_q}; // debounced input
                IRQ_MASK_WORD:   rdata <= {{(AXI_DATA_WIDTH-N_GPIOS){1'b0}}, irq_mask_q};
                IRQ_STATUS_WORD: rdata <= {{(AXI_DATA_WIDTH-N_GPIOS){1'b0}}, irq_status_q};
                EDGE_EN_WORD:    rdata <= {{(AXI_DATA_WIDTH-N_GPIOS){1'b0}}, edge_en_q};
                EDGE_POL_WORD:   rdata <= {{(AXI_DATA_WIDTH-N_GPIOS){1'b0}}, edge_pol_q};
                LVL_POL_WORD:    rdata <= {{(AXI_DATA_WIDTH-N_GPIOS){1'b0}}, lvl_pol_q};
                DEBOUNCE_WORD:   rdata <= {{(AXI_DATA_WIDTH-8){1'b0}}, debounce_cfg_q};
                VERSION_WORD: rdata <= VERSION; //0x2C/44d
                SCRATCH_WORD: rdata <= scratch_q;
                default: rdata <='0;
            endcase
            rresp <= 2'b00;
        end
        //complete read when master takes it
        if(rvalid && rready)begin
            rvalid <= 1'b0;
            arready <= 1'b1;
        end

        end
    
    end

    

    
    assign gpio_out = out_q & dir_q;
    
    





endmodule