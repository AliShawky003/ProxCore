# ProxCore

**A Runtime-Configurable Proximity Safety Co-Processor in SkyWater Sky130B**

Silicon Sprint SP26 — American University in Cairo (AUC), Egypt

---

## Overview

ProxCore is a dedicated ASIC safety co-processor that receives distance
measurements from a UART-based LiDAR sensor, filters noise with a
16-tap symmetric FIR lowpass filter, and asserts a single-cycle
hardware brake interrupt when an obstacle is detected within a
configurable threshold distance.

All safety parameters — baud rate, braking threshold, and FIR filter
coefficients — are runtime-programmable via SPI, enabling the same
silicon to serve trams, automobiles, forklifts, collaborative robots,
and autonomous guided vehicles.

## Architecture
LiDAR Sensor ─► [deserializer_gc] ─► [proxcore_fir_filter] ─► [threshold_fsm] ─► brake_irq
▲ ▲ ▲
└──────── [config_regs_gc] ◄──── SPI bus
baud_div coeff0-7 threshold


| Module | Function |
|---|---|
| `deserializer_gc` | UART 8N1 receiver with runtime-configurable baud rate |
| `proxcore_fir_filter` | 16-tap symmetric FIR filter, 8 multipliers, 5-cycle pipeline |
| `threshold_fsm` | 3-consecutive-sample debounced threshold comparator |
| `config_regs_gc` | SPI write-only register file (10 registers, 24-bit frames) |
| `proxcore_top` | Top-level integration of all four processing blocks |
| `project_macro` | SP26 shuttle GPIO wrapper with pad configuration |

## Key Specifications

| Parameter | Value |
|---|---|
| Technology | SkyWater Sky130B (130nm) |
| Clock frequency | 25 MHz |
| Supply voltage | 1.8V |
| Active pins | 4 inputs, 2 outputs |
| Baud rate range | 9,600 – 921,600 (runtime-configurable via SPI) |
| Distance range | 0 – 1024m (Q10.6 unsigned fixed-point) |
| FIR filter | 16-tap symmetric Hamming lowpass, Q1.15 signed coefficients |
| Filter latency | 5 clock cycles (200ns at 25 MHz) |
| Braking decision | 3 consecutive sub-threshold filtered samples |
| Configuration | 10 × 16-bit registers via SPI (24-bit frames, write-only) |

## SPI Register Map

| Address | Name | Default | Encoding | Description |
|---|---|---|---|---|
| 0x00 | threshold | 2560 | Q10.6 unsigned | Braking distance threshold (40m) |
| 0x01 | coeff0 | 112 | Q1.15 signed | FIR tap h[0], h[15] |
| 0x02 | coeff1 | 243 | Q1.15 signed | FIR tap h[1], h[14] |
| 0x03 | coeff2 | 618 | Q1.15 signed | FIR tap h[2], h[13] |
| 0x04 | coeff3 | 1293 | Q1.15 signed | FIR tap h[3], h[12] |
| 0x05 | coeff4 | 2217 | Q1.15 signed | FIR tap h[4], h[11] |
| 0x06 | coeff5 | 3225 | Q1.15 signed | FIR tap h[5], h[10] |
| 0x07 | coeff6 | 4089 | Q1.15 signed | FIR tap h[6], h[9] |
| 0x08 | coeff7 | 4587 | Q1.15 signed | FIR tap h[7], h[8] |
| 0x09 | baud_div | 108 | Unsigned int | UART clock divider (CLK_FREQ / baud_rate) |

## Baud Rate Configuration

| Baud Rate | baud_div @25MHz | baud_div @50MHz |
|---|---|---|
| 9,600 | 2,604 | 5,208 |
| 115,200 | 217 | 434 |
| 230,400 | 108 | 217 |
| 460,800 | 54 | 108 |
| 921,600 | 27 | 54 |

## Application Profiles

The same silicon serves multiple markets via SPI configuration at power-on:

| Application | Threshold | Baud Rate | baud_div @25MHz |
|---|---|---|---|
| Urban tram | 40m (2560) | 115,200 | 217 |
| Car — city parking | 8m (512) | 115,200 | 217 |
| Car — highway FCW | 80m (5120) | 460,800 | 54 |
| Warehouse forklift | 3m (192) | 115,200 | 217 |
| Collaborative robot | 0.5m (32) | 460,800 | 54 |
| Industrial robot cell | 1.5m (96) | 230,400 | 108 |

## Repository Structure
proxcore/├── rtl/
         │ ├── project_macro.sv # SP26 shuttle GPIO wrapper
         │ ├── proxcore_top.sv # Top-level integration
         │ ├── deserializer_gc.sv # UART deserializer (runtime baud rate)
         │ ├── proxcore_fir_filter.sv # 16-tap symmetric FIR lowpass filter
         │ ├── threshold_fsm.sv # 3-sample debounced threshold FSM
         │ └── config_regs_gc.sv # SPI configuration registers (+baud_div)
         ├── tb/
         │ ├── tb_proxcore_top.sv # Integration testbench (7 tests)
         │ ├── output_test_filter.sv # FIR filter testbench (6 tests) 
         │ ├── config_regs_tb.sv # SPI config registers testbench (14 tests)
         │ ├── tb_threshold_fsm.sv # Threshold FSM testbench (13 tests)
         │ └── deserializer_tb.sv # UART deserializer testbench (5 tests)
         ├── docs/
         │ └── proxcore_report.pdf # Full design report
         ├── constraints/
         │ └── (OpenLane configuration files)
         └── README.md



## Verification Summary

**45 tests** across 5 testbenches covering every module individually
and the full system end-to-end.

### FIR Filter — `output_test_filter` (6 tests)
| # | Test | What It Proves |
|---|---|---|
| 1 | Edge cases (0x0000, 0x7FFF, 0x8000, 0xFFFF) | Boundary and signedness correctness |
| 2 | 200 random samples vs golden model | Arithmetic correctness across full input range |
| 3 | DC flatness at 100m | Unity DC gain — constant input passes through unchanged |
| 4 | Rainstorm spike rejection (alternating 2m/100m) | Single-sample noise never drops output below safety threshold |
| 5 | Real obstacle at 30m detection | Output crosses below threshold — obstacle is detected |
| 6 | Recovery after obstacle clears | Output rises back above threshold — system resumes |

### SPI Config Registers — `config_regs_tb` (14 tests)
| # | Test | What It Proves |
|---|---|---|
| 1 | Reset defaults | All registers initialize to correct power-on values |
| 2 | Single threshold write | Only target register changes, others untouched |
| 3 | First and last coefficient write | Address decode works at register boundaries |
| 4 | All middle coefficient writes | Full register file writeable |
| 5 | Invalid address (0x09) ignored | Out-of-range addresses silently dropped |
| 6 | Partial frame discarded by CSN | Incomplete SPI transfer does not corrupt registers |
| 7 | Full frame after partial | Receiver recovers cleanly after aborted transfer |
| 8 | Reset restores defaults after writes | Async reset overrides all SPI-written values |
| 9 | Back-to-back frames without CSN toggle | Bit counter wraps correctly at frame boundary |
| 10 | Overwrite same register twice | Last write wins — no write-once behavior |
| 11 | Maximum address 0xFF ignored | Upper address space safely rejected |
| 12 | All zeros to all registers | Zero pattern writes correctly |
| 13 | All ones to all registers | 0xFFFF pattern writes correctly |
| 14 | Final reset restores defaults | Confirms reset from all-ones state |

### Threshold FSM — `tb_threshold_fsm` (13 tests)
| # | Test | What It Proves |
|---|---|---|
| 1 | Idle — no samples | No false IRQ when data_valid is low |
| 2 | Clear track (10 × 100m) | No false brake on safe distances |
| 3 | Single sub-threshold sample | 1 reading alone does not trigger |
| 4 | Two sub-threshold samples | 2 readings alone do not trigger |
| 5 | Three consecutive — fires pulse | 3 consecutive readings trigger exactly 1-cycle IRQ |
| 6 | Debounce reset at WARN1 | One safe reading resets the warning chain |
| 7 | Debounce reset at WARN2 | Two-deep warning chain still resets on safe reading |
| 8 | No repeated IRQ in BRAKING | Sustained obstacle does not flood interrupts |
| 9 | Recovery and re-detection | System unsticks after obstacle clears, detects again |
| 10 | Exact threshold — no fire | Value == threshold is NOT sub-threshold (< only) |
| 11 | One below threshold — fires | Boundary value just below threshold triggers correctly |
| 12 | data_valid gating | FSM frozen when data_valid is deasserted |
| 13 | Runtime threshold change | Raising threshold mid-operation takes effect immediately |

### UART Deserializer — `deserializer_tb` (5 tests)
| # | Test | What It Proves |
|---|---|---|
| 1 | Idle after reset | No spurious output with rx held high |
| 2 | Single 16-bit word | Two bytes assembled correctly (low byte first) |
| 3 | Back-to-back words | Continuous reception without dropping words |
| 4 | Glitch rejection | Short rx pulse rejected by start-bit midpoint check |
| 5 | Reset during partial word | Partial byte discarded, clean recovery |

### System Integration — `tb_proxcore_top` (7 tests)
| # | Test | What It Proves |
|---|---|---|
| 1 | Clear track — 100m | No false brakes through full pipeline |
| 2 | Rainstorm spike rejection | FIR + FSM together reject alternating noise |
| 3 | 30m obstacle detection | Real obstacle triggers brake through full pipeline |
| 4 | Recovery + re-detection | System recovers from BRAKING, detects second obstacle |
| 5 | SPI threshold reconfiguration | Runtime threshold change works end-to-end |
| 6 | Reset recovery | Full pipeline clears cleanly after async reset |
| 7 | End-to-end baud rate change via SPI | Baud rate switch from 460800→115200 through full pipeline |

## Compatible Sensors

Any UART-based LiDAR sensor with 16-bit distance output:

| Sensor | Range | Default Baud | Notes |
|---|---|---|---|
| Benewake TFmini-S | 0.1 – 12m | 115,200 | Compact, ideal for forklift/cobot |
| Benewake TF02-Pro | 0.1 – 40m | 115,200 | Medium range, industrial |
| Benewake TF03 | 0.1 – 180m | 115,200 | Long range, automotive/tram |

**Note:** Sky130B I/O is 1.8V. A 3.3V → 1.8V level shifter is required
between the sensor UART TX and the chip's `uart_rx` pad.

## Tools

| Purpose | Tool |
|---|---|
| HDL | SystemVerilog (IEEE 1800-2017) |
| Synthesis | Yosys + OpenLane 2 |
| Simulation | Icarus Verilog / Verilator / ModelSim |
| PDK | SkyWater Sky130B (open-source) |
| Waveforms | GTKWave |

## Authors

Ali Shawky, Karim Khaled, Farah Moataz — Faculty of Engineering - Ain Shams University

## License

Apache 2.0
