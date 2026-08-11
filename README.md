
# AquaSense SoC

### Clock-Gated RTL Architecture for Automated Water Contamination Detection

AquaSense SoC is an RTL-level hardware architecture developed for automated water-quality monitoring using multiple sensor parameters and low-power digital processing.

The current design explores sensor acquisition, multi-parameter severity assessment, risk classification, power-management control, clock gating, packet generation, CRC generation, and SPI-based communication.

> **Project Status: Work in Progress**
>
> This repository currently contains the initial RTL baseline. Functional verification, RTL refinement, communication validation, and implementation analysis are ongoing.

---

## 1. Project Overview

Water-quality monitoring systems typically rely on multiple parameters such as pH, TDS, turbidity, temperature, and conductivity.

AquaSense explores how this monitoring pipeline can be implemented as a dedicated digital hardware architecture rather than relying entirely on software-based processing.

The current RTL architecture consists of:

```text
Sensor Inputs
     │
     ▼
ADC Sequencing
     │
     ▼
Sensor Register Bank
     │
     ▼
Sensor Fusion Engine
     │
     ▼
Severity Score
     │
     ▼
Risk Classifier
     │
     ▼
Power Management
     │
     ├──────────────► Sleep / Control
     │
     ▼
Packet Generator
     │
     ▼
CRC-8 Generator
     │
     ▼
Packet Serializer
     │
     ▼
SPI Master
     │
     ▼
Communication Output



## 2. Design Objectives

The project investigates the following hardware-design objectives:

Multi-parameter water-quality assessment
Hardware-based severity and risk classification
RTL implementation of system control
Low-power operation using clock-gating concepts
Packet formation and integrity checking
Hardware-based serial communication
Modular SoC-style architecture
3. Current RTL Architecture

The current baseline contains the following modules:

Module	Purpose
adc_controller	Digital sequencing model for sensor-data acquisition
sensor_register_bank	Captures sensor values synchronously
sensor_fusion_engine	Combines sensor parameters into a severity score
risk_classifier	Maps severity into a compact risk level
pmu	Generates domain-level clock-enable requests
clock_gate_and	Simple AND-based clock-gating implementation
clock_gate_icg	RTL concept of a latch-based ICG-style gate
system_fsm	Controls the acquisition, processing, classification, transmission and sleep sequence
packet_generator	Builds the monitoring data packet
crc8_generator	Generates CRC-8 for the packet
packet_serializer	Converts the frame into sequential bytes
spi_master	Serializes bytes through an SPI-style interface
communication_controller	Coordinates serialization and SPI transmission
lora_interface_controller	Preliminary communication-controller implementation
sx1276_controller	Preliminary SX1276 abstraction
aquasense_top	Top-level SoC integration
4. Sensor Processing

The current baseline accepts five sensor-related inputs:

pH
TDS
Turbidity
Temperature
Conductivity

The sensor-fusion stage assigns weighted contributions to parameter violations and produces a compact severity_score.

The risk classifier then maps the resulting severity into four risk levels:

00 → SAFE
01 → LOW
10 → MEDIUM
11 → HIGH
5. Low-Power Architecture

A key research direction of AquaSense is reduction of unnecessary switching activity through clock-gating concepts.

The current RTL includes two gating approaches:

AND-Based Clock Gating
gated_clk = clk & enable
ICG-Style Clock Gating

A latch-based enable is used so that the clock is not intentionally gated in the middle of an active clock pulse.

The top-level parameter allows the communication-domain clock to be selected between:

0 → No clock gating
1 → AND-based gating
2 → ICG-style gating

This architecture is intended to support later comparison of different clock-gating approaches.

6. Communication Pipeline

The current communication path consists of:

Packet Generator
      ↓
CRC-8
      ↓
Packet Serializer
      ↓
Communication Controller
      ↓
SPI Master

The packet currently contains:

Node ID
pH
TDS
Turbidity
Risk Level
Reserved Fields

followed by an 8-bit CRC.

The serializer and communication controller use a byte-level handshake so that the next byte is only released after completion of the current SPI transfer.

7. Verification

A dedicated testbench is included in the current baseline to exercise the top-level design.

The current test scenario applies a representative contaminated-water condition and checks:

Severity score
Risk classification
Number of transmitted bytes
Transmitted byte sequence
Communication completion

Functional verification is currently under refinement.

8. Current Repository

The repository currently contains the initial combined RTL baseline.

AquaSense-SoC/
│
├── README.md
├── LICENSE
└── C_AQUA.v

The RTL will later be reorganized into separate source, testbench, simulation, documentation, and implementation-result directories.

9. Development Roadmap
Phase 1 — RTL Refinement
Separate modules into individual source files
Clean interfaces
Resolve synthesis and simulation issues
Simplify control paths where required
Phase 2 — Verification
Develop structured testbenches
Verify sensor-processing behavior
Verify FSM transitions
Verify packet generation
Verify CRC
Verify SPI transmission
Phase 3 — Low-Power Evaluation
Compare baseline and clock-gated architectures
Analyze switching activity
Evaluate power implications
Compare AND-based and ICG-style approaches
Phase 4 — Communication Refinement
Improve SPI verification
Refine LoRa communication architecture
Evaluate integration with an external communication interface
Phase 5 — Implementation Analysis
Synthesis
Timing analysis
Resource utilization
Power estimation
10. Repository Status

Current status: Work in Progress

The repository is being developed incrementally, with the initial architecture preserved as a baseline for subsequent RTL refinement and verification.

Author

Shivam Vyas

Electronics & Communication Engineering
Interests: RTL Design · VLSI · Low-Power Hardware · Digital Systems · SoC Architecture

