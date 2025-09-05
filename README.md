# AXI4-Lite-GPIO Peripheral with Interrupts and Debounce
## Overview
This project implements an AXI4-Lite GPIO peripheral in Systemverilog. The design exposes a parameterizable number of GPIO pins through an AXI4-Lite slave interface, along with programmable control/status registers (CSRs) for software interaction.

This block supports the following...

- **AXI4-Lite protocol compliance (single-beat read/write)**
- **Programmable GPIO direction (input/output)**
- **Data output with atomic SET/CLEAR registers**
- **Input path with 2-flip-flop synchronizer (safe async to sync crossing)**
- **Optional input debounce filter (glitch/bounce protection)**
- **Edge and level-sensitive interrupts with mask/polarity control**
- **Sticky IRQ status with write-1-to-clear (W1C) semantics**
## Features by Development Phase
 I developed the code sequentially
#### Phase 1: AXI4-Lite Skeleton
  - Implemented AXI4-Lite slave interface with AW/W/B and AR/R channels.
  - Supported a simple VERSION register (RO @ 0x2C).
  - Verified basic reads/writes with protocol SVAs (stable payload under VALID && !READY).

#### Phase 2: Scratch Register + WSTRB
  - Added SCRATCH register (RW @ 0x30) for software testing.
  - Implemented full WSTRB (byte-enable) support, allowing partial byte writes.
  - Smoke tested partial updates (e.g., modify only byte[2]).

#### Phase 3: Core GPIO CSRs
- Added registers:
  - DIR @ 0x00: direction (1=output, 0=input)
  - OUT @ 0x04: output values
  - OUT_SET @ 0x08: write-1-to-set
  - OUT_CLR @ 0x0C: write-1-to-clear
  - IN @ 0x10: synchronized input values
- Implemented 2-FF synchronizer for gpio_in_raw.
- Drove gpio_out = out_q & dir_q (only pins configured as outputs drive).
- Verified atomic SET/CLR behaviour and WSTRB on DIR/OUT.

#### Phase 4: Interrupts + Debounce 
- Added interrupt control/status registers:
  - IRQ_MASK @ 0x14: mask which bits assert the irq line
  - IRQ_STATUS @ 0x18: sticky pending (RO)
  - IRQ_CLEAR @ 0x1C: W1C clear of status bits
  - EDGE_EN @ 0x20: edge (1) or level (0) mode
  - EDGE_POL @ 0x24: rising (1) / falling (0) edge polarity
  - LVL_POL @ 0x28: high (1) / low (0) level polarity
  - DEBOUNCE @ 0x34: global debounce threshold (in cycles)
- Implemented:
  - Edge detect from debounced input (rise/fall)
  - Level detect with polarity
  - Sticky IRQ_STATUS with W1C (event wins if simultaneous with clear)
  - IRQ line equation: irq = OR(IRQ_STATUS & IRQ_MASK)
  - Per-bit debounce counters: ignore glitches shorter than threshold
 - Verified:
   - Rising-edge events latch once
   - Level events re-assert if condition persists
   - W1C clears pending bits correctly
   - Short glitches ignored when debounce > 0
  
 ## Verification
 Mostly used a directed testbench method in SystemVerilog to ensure the intended design is implemented properly. Working on using UVM now...

    
