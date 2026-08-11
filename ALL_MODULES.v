C_AQUA

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// AQUASENSE SoC - MASTER RTL FILE
//
// Contains every module in the current architecture (Section 27/28 of the
// project notes) plus the aquasense_top integration module.
//
// Module list in this file, in dataflow order:
//   1.  adc_controller
//   2.  sensor_register_bank
//   3.  sensor_fusion_engine
//   4.  risk_classifier
//   5.  pmu
//   6.  clock_gate_and        (NEW - simple AND-based gate, Section 7)
//   7.  clock_gate_icg        (as given)
//   8.  system_fsm
//   9.  packet_generator      (NEW - was referenced but not yet defined)
//   10. crc8_generator        (NEW - poly 0x07, CRC-8, was only a stub before)
//   11. packet_serializer     (REVISED - byte_ack handshake added, see notes)
//   12. spi_master            (cleaned up duplicate bit_count assignment)
//   13. communication_controller (REVISED - multi-byte handshake loop added)
//   14. lora_interface_controller (as given - preliminary, not wired into top)
//   15. sx1276_controller     (as given - preliminary, not wired into top)
//   16. aquasense_top         (NEW - full SoC integration)
//
// INTERFACE CHANGES vs. the previous version (documented per Section 41 rule
// "before changing an interface, explain why it must change"):
//
//   packet_serializer:
//     + input  byte_ack        (NEW)
//     - tx_byte is now a continuous `assign` output instead of a `reg`,
//       so it always reflects the current top byte of frame_data with zero
//       extra register latency. This removes a class of off-by-one timing
//       bug (the same family of bug that hit spi_master's bit_count).
//
//   communication_controller:
//     + input  byte_valid      (NEW, from packet_serializer)
//     + output byte_ack        (NEW, to packet_serializer)
//     - the FSM now loops through SEND_BYTE/WAIT_SPI/ACK_BYTE once per byte
//       instead of firing spi_start a single time. This is the fix for the
//       core unresolved issue in Section 24/26 of the project notes: the
//       7-byte frame was never actually being clocked out one byte at a
//       time because there was no producer/consumer handshake.
//
// Reason these two interfaces had to change together: the old design had
// no signal telling the serializer "the SPI master actually finished this
// byte, give me the next one." byte_valid/byte_ack close that loop.
//////////////////////////////////////////////////////////////////////////////////
 
 
//==================================================================================
// 1. ADC CONTROLLER
//    Purpose: models the digital handshake side of sensor acquisition
//    (Section 10). Does NOT implement an analog ADC - it is the digital
//    sequencing block that says "start a conversion, wait, tell me when a
//    fresh sample is ready."
//==================================================================================
module adc_controller(
 
           input clk,
           input reset,
           input start_conversion,
 
  output    reg adc_busy,
  output    reg adc_done,
  output    reg sample_valid
 
    );
 
    reg [2:0] state;
 
    parameter       IDLE = 3'b000;
    parameter       START = 3'b001;
    parameter       SAMPLE = 3'b010;
    parameter       CONVERT = 3'b011;
    parameter       STORE = 3'b100;
    parameter       DONE  = 3'b101;
 
 
    always@(posedge clk or posedge reset)
    begin
 
         if(reset)
         begin
 
              state <= IDLE;
 
              adc_busy <= 0;
              adc_done <= 0;
              sample_valid <= 0;
         end
 
         else
         begin
 
              case(state)
 
 
                  IDLE:
                  begin
                       adc_busy <= 0;
                       adc_done <= 0;
                       sample_valid <= 0;
 
                       if(start_conversion)
                        state <= START;
 
                    else
                        state <= IDLE;
 
                end
 
                START:
                begin
 
                    adc_busy <= 1;
                    state <= SAMPLE;
 
                end
 
                SAMPLE:
                begin
 
                    adc_busy <= 1;
                    state <= CONVERT;
 
                end
 
                CONVERT:
                begin
 
                    adc_busy <= 1;
                    state <= STORE;
 
                end
 
                STORE:
                begin
 
                    adc_busy <= 1;
                    sample_valid <= 1;
 
                    state <= DONE;
 
                end
 
                DONE:
                begin
 
                    adc_busy <= 0;
                    adc_done <= 1;
 
                    state <= IDLE;
 
                end
 
                default:
                begin
 
                    state <= IDLE;
 
                end
 
            endcase
 
        end
 
    end
 
endmodule
 
 
//==================================================================================
// 2. SENSOR REGISTER BANK
//    Purpose: synchronous boundary between raw sensor inputs and the
//    processing chain (Section 9). Loads on load_en (driven by adc_done
//    at the top level) so nothing downstream ever sees a raw asynchronous
//    sensor value.
//==================================================================================
module sensor_register_bank(
 
    input clk,
    input reset,
 
    input load_en,
 
    input [7:0] ph_in,
    input [7:0] tds_in,
    input [7:0] turbidity_in,
    input [7:0] temp_in,
    input [7:0] cond_in,
 
    output reg [7:0] ph_reg,
    output reg [7:0] tds_reg,
    output reg [7:0] turbidity_reg,
    output reg [7:0] temp_reg,
    output reg [7:0] cond_reg
 
 
    );
 
    always @ (posedge clk or posedge reset)
    begin
 
 
    if(reset)
    begin
 
        ph_reg <= 0;
        tds_reg <= 0;
        turbidity_reg <= 0;
        temp_reg <= 0;
        cond_reg <= 0;
 
    end
 
    else if(load_en)
    begin
 
        ph_reg <= ph_in;
        tds_reg <= tds_in;
        turbidity_reg <= turbidity_in;
        temp_reg <= temp_in;
        cond_reg <= cond_in;
 
    end
 
end
 
 
endmodule
 
 
//==================================================================================
// 3. SENSOR FUSION ENGINE
//    Purpose: combine multiple parameters into a single severity_score
//    instead of independent OR'd threshold checks (Section 11).
//    Purely combinational - no clock needed for the math itself; the top
//    level adds a pipeline register around this (see aquasense_top).
//==================================================================================
module sensor_fusion_engine(
 
 
    input [7:0] ph_reg,
    input [7:0] tds_reg,
    input [7:0] turbidity_reg,
    input [7:0] temp_reg,
    input [7:0] cond_reg,
 
    output reg [3:0] severity_score
 
 
    );
 
    always @(*)
    begin
 
        severity_score =0;
 
         // pH contribution
 
    if((ph_reg < 65) || (ph_reg > 85))
        severity_score = severity_score + 4;
 
    // TDS contribution
 
    if(tds_reg > 100)
        severity_score = severity_score + 3;
 
    // Turbidity contribution
 
    if(turbidity_reg > 5)
        severity_score = severity_score + 3;
 
    // Temperature contribution
 
    if(temp_reg > 40)
        severity_score = severity_score + 1;
 
    // Conductivity contribution
 
    if(cond_reg > 80)
        severity_score = severity_score + 1;
 
end
 
 
 
endmodule
 
 
//==================================================================================
// 4. RISK CLASSIFIER
//    Purpose: compress severity_score into a compact 2-bit risk_level
//    (Section 12): 00=SAFE, 01=LOW, 10=MED, 11=HIGH.
//==================================================================================
module risk_classifier(
 
   input [3:0] severity_score,
 
   output reg [1:0] risk_level
 
 
 
    );
 
    always @(*)
begin
 
    if(severity_score <= 2)
        risk_level = 2'b00;
 
    else if(severity_score <= 5)
        risk_level = 2'b01;
 
    else if(severity_score <= 8)
        risk_level = 2'b10;
 
    else
        risk_level = 2'b11;
 
end
 
endmodule
 
 
//==================================================================================
// 5. PMU (Power Management Unit)
//    Purpose: translate risk_level into domain-level clock-enable requests
//    (Section 36). Combinational by design - it is a fast, glitch-tolerant
//    policy block; the actual gating (and any latch-safety) lives in the
//    clock_gate_* cells that consume its outputs.
//==================================================================================
module pmu(
 
    input clk,
    input reset,
 
    input [1:0] risk_level,
 
    output reg sensor_clk_enable,
    output reg processing_clk_enable,
    output reg comm_clk_enable
 
 
 
    );
 
    always @(*)
    begin
 
         case(risk_level)
 
         2'b00:
         begin
             sensor_clk_enable =0;
             processing_clk_enable =0;
             comm_clk_enable =0;
         end
 
         2'b01:
         begin
             sensor_clk_enable =1;
             processing_clk_enable =1;
             comm_clk_enable =0;
         end
 
         2'b10:
         begin
             sensor_clk_enable =1;
             processing_clk_enable =1;
             comm_clk_enable =1;
         end
 
         2'b11:
         begin
             sensor_clk_enable =1;
             processing_clk_enable =1;
             comm_clk_enable =1;
         end
 
    default:
    begin
        sensor_clk_enable =0;
        processing_clk_enable =0;
        comm_clk_enable =0;
    end
 
    endcase
 
end
 
 
endmodule
 
 
//==================================================================================
// 6. CLOCK GATE - SIMPLE AND-BASED (NEW MODULE)
//    Purpose: Section 7's "gated_clk = clk & enable" experiment, given its
//    own reusable module so it can be swapped against clock_gate_icg for
//    the Stage-7 AND-vs-ICG comparison without touching any other RTL.
//==================================================================================
module clock_gate_and(
 
    input clk,
    input clk_enable,
 
    output gated_clk
 
    );
 
    assign gated_clk = clk & clk_enable;
 
endmodule
 
 
//==================================================================================
// 7. CLOCK GATE - ICG STYLE (as given)
//    Purpose: Section 8's latch-based enable, so the clock is never gated
//    mid-pulse. NOTE (kept from original discussion): an FPGA LUT-based
//    implementation of this is NOT equivalent to an ASIC standard-cell ICG
//    cell - this module is the RTL *concept* only.
//==================================================================================
module clock_gate_icg(
    input clk,
    input clk_enable,
 
    output gated_clk
 
 
 
    );
 
    reg enable_latched;
 
    always @(clk, clk_enable)
    begin
 
        if(~clk)
              enable_latched = clk_enable;
    end
 
    assign gated_clk = clk & enable_latched;
 
 
endmodule
 
 
//==================================================================================
// 8. SYSTEM FSM (as given)
//    Purpose: top-level sequencer for the Active -> Process -> Transmit ->
//    Sleep cycle (Section 36). This is the "control domain" and stays on
//    the ungated main clock at the top level, since it has to keep running
//    in order to ever wake anything else back up.
//==================================================================================
module system_fsm(
 
    input clk,
    input reset,
 
    input adc_done,
 
    input [1:0] risk_level,
 
    output reg adc_start,
    output reg process_enable,
    output reg classify_enable,
    output reg transmit_enable,
    output reg sleep_enable
 
    );
 
    reg [2:0] state;
 
parameter IDLE      = 3'b000;
parameter ACQUIRE   = 3'b001;
parameter PROCESS   = 3'b010;
parameter CLASSIFY  = 3'b011;
parameter DECIDE    = 3'b100;
parameter TRANSMIT  = 3'b101;
parameter SLEEP     = 3'b110;
 
always @(posedge clk or posedge reset)
begin
       if(reset)
       begin
 
        state <= IDLE;
 
        adc_start <= 0;
        process_enable <= 0;
        classify_enable <= 0;
        transmit_enable <= 0;
        sleep_enable <= 0;
 
    end
 
    else
    begin
 
        case(state)
 
            IDLE:
            begin
 
                adc_start <= 1;
 
                process_enable <= 0;
                classify_enable <= 0;
                transmit_enable <= 0;
                sleep_enable <= 0;
 
                state <= ACQUIRE;
 
            end
 
            ACQUIRE:
            begin
 
                adc_start <= 0;
 
                if(adc_done)
                    state <= PROCESS;
 
            end
 
            PROCESS:
            begin
 
                process_enable <= 1;
                state <= CLASSIFY;
 
            end
 
            CLASSIFY:
            begin
 
                process_enable <= 0;
                classify_enable <= 1;
 
                state <= DECIDE;
 
            end
 
            DECIDE:
            begin
 
                classify_enable <= 0;
 
                if(risk_level == 2'b00)
                    state <= SLEEP;
 
                else
                    state <= TRANSMIT;
 
            end
 
            TRANSMIT:
            begin
 
                transmit_enable <= 1;
 
                state <= SLEEP;
 
            end
 
            SLEEP:
            begin
 
                transmit_enable <= 0;
 
                sleep_enable <= 1;
 
                state <= IDLE;
 
            end
 
            default:
            begin
 
                state <= IDLE;
 
            end
 
        endcase
 
    end
 
end
 
 
 
endmodule
 
 
//==================================================================================
// 9. PACKET GENERATOR (NEW MODULE)
//    Purpose: Section 13. Combinational field mapping into a 48-bit packet.
//    No clock is needed - it just concatenates whatever is currently on
//    its inputs into a packet bus.
//
//    Field layout (MSB first), 48 bits total:
//      node_id[7:0]  | ph_reg[7:0] | tds_reg[7:0] | turbidity_reg[7:0]
//      | risk_level[1:0] | reserved[13:0]
//==================================================================================
module packet_generator(
 
    input [7:0] node_id,
    input [7:0] ph_reg,
    input [7:0] tds_reg,
    input [7:0] turbidity_reg,
    input [1:0] risk_level,
 
    output [47:0] packet_data
 
    );
 
    assign packet_data = {node_id, ph_reg, tds_reg, turbidity_reg, risk_level, 14'b0};
 
endmodule
 
 
//==================================================================================
// 10. CRC8 GENERATOR (NEW MODULE - was a stub before)
//     Purpose: Section 14. CRC-8 with polynomial 0x07, computed over the
//     48-bit packet. Bit-serial algorithm, fully unrolled combinationally
//     via a `for` loop (synthesizes to a flat gate tree, no clock needed).
//==================================================================================
module crc8_generator(
 
    input [47:0] packet_data,
 
    output reg [7:0] crc
 
    );
 
    integer i;
    reg [7:0] crc_temp;
 
    always @(*)
    begin
 
        crc_temp = 8'h00;
 
        for(i = 47; i >= 0; i = i - 1)
        begin
            if((crc_temp[7] ^ packet_data[i]) == 1'b1)
                crc_temp = (crc_temp << 1) ^ 8'h07;
            else
                crc_temp = crc_temp << 1;
        end
 
        crc = crc_temp;
 
    end
 
endmodule
 
 
//==================================================================================
// 11. PACKET SERIALIZER (REVISED - byte_ack handshake)
//     Purpose: Section 16. Converts the 56-bit frame (48-bit packet + 8-bit
//     CRC) into 7 sequential bytes, one at a time, only advancing when the
//     communication_controller explicitly acknowledges (byte_ack) that the
//     previous byte was consumed by the SPI master. This is the fix for
//     the Section 24/26 problem: the old version free-ran through all 7
//     bytes in ~7 clocks regardless of whether SPI had actually sent them.
//==================================================================================
module packet_serializer(
 
    input clk,
    input reset,
 
    input start_serialization,
 
    input [47:0] packet_data,
    input [7:0] crc,
 
    input byte_ack,              // NEW: pulsed by communication_controller
                                  // once spi_done confirms the current byte
                                  // was actually transmitted.
 
    output [7:0] tx_byte,        // NOW combinational (was a reg) - always
                                  // mirrors the top byte of frame_data with
                                  // no extra register delay.
 
    output reg byte_valid,
    output reg serialization_done
 
);
 
reg [55:0] frame_data;
reg [2:0] byte_count;
 
reg [1:0] state;
 
parameter IDLE = 2'b00;
parameter LOAD = 2'b01;
parameter SEND = 2'b10;
parameter DONE = 2'b11;
 
// tx_byte is a live combinational window onto the top byte of frame_data.
assign tx_byte = frame_data[55:48];
 
always @(posedge clk or posedge reset)
begin
 
    if(reset)
    begin
 
        state <= IDLE;
 
        frame_data <= 0;
        byte_count <= 0;
 
        byte_valid <= 0;
        serialization_done <= 0;
 
    end
 
    else
    begin
 
        case(state)
 
            IDLE:
            begin
 
                byte_valid <= 0;
                serialization_done <= 0;
 
                if(start_serialization)
                    state <= LOAD;
 
            end
 
            LOAD:
            begin
 
                frame_data <= {packet_data, crc};
 
                byte_count <= 0;
                byte_valid <= 1;   // byte 0 is ready as soon as SEND begins
 
                state <= SEND;
 
            end
 
            SEND:
            begin
 
                // Hold tx_byte/byte_valid steady until the controller acks
                // that SPI has actually finished sending this byte.
                if(byte_ack)
                begin
 
                    if(byte_count == 6)
                    begin
 
                        byte_valid <= 0;
                        state <= DONE;
 
                    end
                    else
                    begin
 
                        frame_data <= frame_data << 8;
                        byte_count <= byte_count + 1;
                        // byte_valid stays high - next byte is ready
                        // as soon as frame_data has shifted.
 
                    end
 
                end
 
            end
 
            DONE:
            begin
 
                serialization_done <= 1;
 
                state <= IDLE;
 
            end
 
            default:
            begin
 
                state <= IDLE;
 
            end
 
        endcase
 
    end
 
end
 
endmodule
 
 
//==================================================================================
// 12. SPI MASTER (cleaned up - duplicate bit_count assignment removed)
//     Purpose: Section 17. Shifts one 8-bit byte per start_tx pulse.
//     bit_count==7 correctly checks the OLD (pre-edge) counter value for
//     the 8th bit of the transfer, since it's compared against a
//     non-blocking-assigned register - this was already correct in the
//     original; only the accidental duplicate increment line was removed.
//==================================================================================
module spi_master(
 
 
     input clk,
     input reset,
 
     input start_tx,
     input [7:0] tx_data,
 
     output reg sclk,
     output reg mosi,
     output reg cs,
 
     output reg tx_done
 
 
 
 
    );
 
reg [7:0] shift_reg;
reg [3:0] bit_count;
 
reg [1:0] state;
 
parameter IDLE  = 2'b00;
parameter LOAD  = 2'b01;
parameter SHIFT = 2'b10;
parameter DONE  = 2'b11;
 
 
 
always @(posedge clk or posedge reset)
begin
    if(reset)
    begin
 
        state <= IDLE;
 
        sclk <= 0;
        mosi <= 0;
        cs <= 1;
        tx_done <= 0;
 
        shift_reg <= 0;
        bit_count <= 0;
 
    end
 
    else
    begin
        case(state)
 
            IDLE:
            begin
 
                tx_done <= 0;
                cs <= 1;
                sclk <= 0;
 
                if(start_tx)
                    state <= LOAD;
 
            end
 
            LOAD:
            begin
 
                shift_reg <= tx_data;
                bit_count <= 0;
 
                cs <= 0;
 
                state <= SHIFT;
 
            end
 
 
            SHIFT:
            begin
 
                sclk <= ~sclk;
 
                if(sclk == 0)
                begin
 
                    mosi <= shift_reg[7];
 
                    shift_reg <= shift_reg << 1;
 
                    if(bit_count == 7)
                        state <= DONE;
 
                    bit_count <= bit_count + 1;
 
                end
 
            end
 
 
            DONE:
            begin
 
                cs <= 1;
                sclk <= 0;
                tx_done <= 1;
 
                state <= IDLE;
 
                end
 
                default:
                begin
 
                    state <= IDLE;
 
            end
 
        endcase
 
    end
 
end
 
 
endmodule
 
 
//==================================================================================
// 13. COMMUNICATION CONTROLLER (REVISED - multi-byte handshake loop)
//     Purpose: Section 20/26. Sequences the serializer and SPI master
//     through all 7 bytes of a frame, one at a time:
//       wait for a byte to be valid -> start SPI -> wait for SPI done ->
//       ack the byte -> loop back, until the serializer reports
//       serialization_done.
//     This is the direct fix for the Section 24 problem (frame was never
//     cleanly demonstrated as 7 independent SPI transactions).
//==================================================================================
module communication_controller(
 
    input clk,
    input reset,
 
    input transmit_enable,
 
    input byte_valid,            // NEW: from packet_serializer
    input spi_done,
    input serialization_done,
 
    output reg serialize_start,
    output reg spi_start,
    output reg byte_ack,         // NEW: to packet_serializer
 
    output reg comm_done
 
);
 
reg [2:0] state;
 
parameter IDLE      = 3'b000;
parameter WAIT_BYTE = 3'b001;   // waiting for serializer to present a byte
parameter SEND_BYTE = 3'b010;   // pulse spi_start
parameter WAIT_SPI  = 3'b011;   // waiting for spi_done
parameter ACK_BYTE  = 3'b100;   // pulse byte_ack
parameter DONE      = 3'b101;
 
always @(posedge clk or posedge reset)
begin
 
    if(reset)
    begin
 
        state <= IDLE;
 
        serialize_start <= 0;
        spi_start <= 0;
        byte_ack <= 0;
        comm_done <= 0;
 
    end
 
    else
    begin
 
        // Default pulse outputs - one-cycle pulses unless a state below
        // re-asserts them.
        serialize_start <= 0;
        spi_start <= 0;
        byte_ack <= 0;
        comm_done <= 0;
 
        case(state)
 
            IDLE:
            begin
 
                if(transmit_enable)
                begin
                    serialize_start <= 1;
                    state <= WAIT_BYTE;
                end
 
            end
 
            WAIT_BYTE:
            begin
 
                // serialization_done is checked first: it catches the
                // terminal case where the 7th byte has already been
                // acked and the serializer has moved on to DONE, which
                // happens one cycle after byte_valid drops for the last
                // time.
                if(serialization_done)
                    state <= DONE;
                else if(byte_valid)
                    state <= SEND_BYTE;
 
            end
 
            SEND_BYTE:
            begin
 
                spi_start <= 1;
                state <= WAIT_SPI;
 
            end
 
            WAIT_SPI:
            begin
 
                if(spi_done)
                    state <= ACK_BYTE;
 
            end
 
            ACK_BYTE:
            begin
 
                byte_ack <= 1;
                state <= WAIT_BYTE;   // loop back for the next byte
 
            end
 
            DONE:
            begin
 
                comm_done <= 1;
 
                if(!transmit_enable)
                    state <= IDLE;
 
            end
 
            default:
            begin
 
                state <= IDLE;
 
            end
 
        endcase
 
    end
 
end
 
endmodule
 
 
//==================================================================================
// 14. LORA INTERFACE CONTROLLER (as given)
//     Status: preliminary alternate/earlier-draft controller (Section 18).
//     Kept in this file for completeness since it already exists in the
//     project, but it is NOT instantiated in aquasense_top below, because
//     its single-shot spi_start/spi_done handshake predates (and conflicts
//     with) the verified multi-byte communication_controller above. Two
//     independent SPI sequencers cannot safely drive the same spi_master.
//==================================================================================
module lora_interface_controller(
 
    input clk,
    input reset,
 
    input transmit_enable,
    input spi_done,
 
    output reg spi_start,
    output reg tx_done
 
    );
 
reg [2:0] state;
 
parameter IDLE        = 3'b000;
parameter LOAD_PACKET = 3'b001;
parameter SEND_PACKET = 3'b010;
parameter WAIT_SPI    = 3'b011;
parameter DONE        = 3'b100;
 
always @(posedge clk or posedge reset)
begin
 
    if(reset)
    begin
 
         state <= IDLE;
 
         spi_start <=0;
         tx_done <=0;
     end
 
 
     else
     begin
 
          case(state)
 
            IDLE:
            begin
 
                spi_start <= 0;
                tx_done <= 0;
 
                if(transmit_enable)
                    state <= LOAD_PACKET;
 
            end
 
            LOAD_PACKET:
            begin
 
                state <= SEND_PACKET;
 
            end
 
             SEND_PACKET:
            begin
 
                spi_start <= 1;
 
                state <= WAIT_SPI;
 
            end
 
            WAIT_SPI:
            begin
 
                spi_start <= 0;
 
                if(spi_done)
                    state <= DONE;
 
            end
 
           DONE:
            begin
 
                tx_done <= 1;
 
                state <= IDLE;
 
            end
 
            default:
            begin
 
                state <= IDLE;
 
            end
 
        endcase
 
    end
 
end
 
 
endmodule
 
 
//==================================================================================
// 15. SX1276 CONTROLLER (as given)
//     Status: preliminary abstraction (Section 19). Does not yet implement
//     real SX1276 register/FIFO transactions. Kept in this file for
//     completeness but NOT instantiated in aquasense_top below, for the
//     same reason as lora_interface_controller above - it would be a
//     second, competing SPI sequencer. A future revision should replace
//     the current communication_controller's direct SPI drive with a real
//     SX1276 register-level controller once the byte-level SPI path
//     (verified in this file) is confirmed on hardware.
//==================================================================================
module sx1276_controller(
 
    input clk,
    input reset,
 
    input transmit_enable,
    input serialization_done,
    input spi_done,
 
    output reg spi_start,
    output reg tx_done
 
 
    );
 
reg [2:0] state;
 
parameter IDLE        = 3'b000;
parameter LOAD_FIFO   = 3'b001;
parameter WAIT_FIFO   = 3'b010;
parameter START_TX    = 3'b011;
parameter WAIT_TX     = 3'b100;
parameter DONE        = 3'b101;
 
always @(posedge clk or posedge reset)
begin
 
    if(reset)
    begin
 
        state <= IDLE;
 
        spi_start <= 0;
        tx_done <= 0;
 
    end
 
    else
    begin
 
        case(state)
 
            IDLE:
            begin
 
                spi_start <= 0;
                tx_done <= 0;
 
                if(transmit_enable)
                    state <= LOAD_FIFO;
 
            end
 
            LOAD_FIFO:
            begin
 
                if(serialization_done)
                begin
 
                    spi_start <= 1;
                    state <= WAIT_FIFO;
                end
 
            end
 
            WAIT_FIFO:
            begin
 
                spi_start <= 0;
 
                if(spi_done)
                    state <= START_TX;
 
            end
 
            START_TX:
            begin
 
                state <= WAIT_TX;
 
            end
 
            WAIT_TX:
            begin
 
                state <= DONE;
 
            end
 
            DONE:
            begin
 
                tx_done <= 1;
                state <= IDLE;
 
            end
 
            default:
            begin
 
                state <= IDLE;
 
            end
 
        endcase
 
    end
 
end
 
 
 
endmodule
 
 
//==================================================================================
// 16. AQUASENSE TOP - full SoC integration (NEW)
//
//     Dataflow:
//       ph_in/tds_in/turbidity_in/temp_in/cond_in
//         -> adc_controller (sequencing) + sensor_register_bank (capture)
//         -> sensor_fusion_engine -> severity_score_reg (pipeline reg)
//         -> risk_classifier -> risk_level_comb / risk_level_reg
//         -> pmu -> comm_clk_enable
//         -> packet_generator -> crc8_generator
//         -> communication_controller + packet_serializer + spi_master
//         -> sclk / mosi / cs / tx_done
//
//     Timing note on risk_level (important - avoids a real race bug):
//     system_fsm's DECIDE state reads risk_level in the SAME cycle that
//     the pipeline register below would otherwise still be latching last
//     iteration's value. Feeding system_fsm the COMBINATIONAL risk_level
//     (risk_level_comb, valid immediately from the already-updated
//     severity_score_reg) instead of the registered version avoids a
//     one-iteration-stale TRANSMIT/SLEEP decision. The registered version
//     (risk_level_reg) is still used for the PMU and for the transmitted
//     packet content, since those benefit from a stable, glitch-free value
//     and have the rest of the TRANSMIT/SLEEP cycle to settle.
//
//     Power management note (avoids a real deadlock):
//     The PMU's sensor_clk_enable / processing_clk_enable are exposed as
//     status outputs ONLY. They are deliberately NOT used to gate the
//     sensor_register_bank or fusion/classifier logic in this baseline,
//     because that would gate the very clock that produces the sensor
//     data risk_level is computed from - a combinational self-throttling
//     loop that (once risk_level settles to SAFE) would gate the sensor
//     clock off forever with no periodic wake-up, permanently freezing
//     the system. Only comm_clk_enable is used to gate a real domain here
//     (the communication subsystem), which has no such feedback path:
//     system_fsm only ever pulses transmit_enable when risk_level_comb is
//     non-zero, which is exactly when comm_clk_enable is high, so the
//     gated clock is never needed while it is gated off. Extending safe
//     gating to the sensor/processing domains (with a watchdog/periodic
//     wake timer) is future work per Section 36/40.
//
//     CLK_GATE_MODE parameter supports the Stage-7 AND-vs-ICG comparison
//     (Section 35) from a single netlist: 0 = no gating (baseline),
//     1 = simple AND gate, 2 = ICG-style gate.
//==================================================================================
module aquasense_top #(
    parameter CLK_GATE_MODE = 1   // 0 = baseline, 1 = AND gating, 2 = ICG gating
)(
    input clk,
    input reset,
 
    input [7:0] node_id,
    input [7:0] ph_in,
    input [7:0] tds_in,
    input [7:0] turbidity_in,
    input [7:0] temp_in,
    input [7:0] cond_in,
 
    output sclk,
    output mosi,
    output cs,
    output tx_done,
 
    output sleep_enable,
    output [1:0] risk_level_status,
    output [3:0] severity_score_status
);
 
//----------------------------------------------------------------------------
// Control domain (system_fsm + adc_controller) - always on, main clk.
//----------------------------------------------------------------------------
wire adc_start;
wire process_enable;
wire classify_enable;
wire transmit_enable;
wire adc_done;
wire adc_busy;
wire sample_valid;
 
wire [1:0] risk_level_comb;   // combinational classifier output (see note above)
reg  [1:0] risk_level_reg;    // registered / stable snapshot for PMU + packet
 
system_fsm FSM (
    .clk(clk),
    .reset(reset),
    .adc_done(adc_done),
    .risk_level(risk_level_comb),
    .adc_start(adc_start),
    .process_enable(process_enable),
    .classify_enable(classify_enable),
    .transmit_enable(transmit_enable),
    .sleep_enable(sleep_enable)
);
 
adc_controller ADC (
    .clk(clk),
    .reset(reset),
    .start_conversion(adc_start),
    .adc_busy(adc_busy),
    .adc_done(adc_done),
    .sample_valid(sample_valid)
);
 
//----------------------------------------------------------------------------
// Sensor acquisition domain - always on, main clk.
//----------------------------------------------------------------------------
wire [7:0] ph_reg, tds_reg, turbidity_reg, temp_reg, cond_reg;
 
sensor_register_bank SRB (
    .clk(clk),
    .reset(reset),
    .load_en(adc_done),
    .ph_in(ph_in),
    .tds_in(tds_in),
    .turbidity_in(turbidity_in),
    .temp_in(temp_in),
    .cond_in(cond_in),
    .ph_reg(ph_reg),
    .tds_reg(tds_reg),
    .turbidity_reg(turbidity_reg),
    .temp_reg(temp_reg),
    .cond_reg(cond_reg)
);
 
//----------------------------------------------------------------------------
// Processing domain - combinational engine + pipeline registers.
//----------------------------------------------------------------------------
wire [3:0] severity_score_comb;
 
sensor_fusion_engine SFE (
    .ph_reg(ph_reg),
    .tds_reg(tds_reg),
    .turbidity_reg(turbidity_reg),
    .temp_reg(temp_reg),
    .cond_reg(cond_reg),
    .severity_score(severity_score_comb)
);
 
// Processing pipeline register: captures the fused severity score when
// system_fsm is in its PROCESS state (process_enable pulse, visible one
// cycle later during CLASSIFY).
reg [3:0] severity_score_reg;
always @(posedge clk or posedge reset) begin
    if(reset)
        severity_score_reg <= 4'b0;
    else if(process_enable)
        severity_score_reg <= severity_score_comb;
end
 
risk_classifier RC (
    .severity_score(severity_score_reg),
    .risk_level(risk_level_comb)
);
 
// Classification pipeline register: stable snapshot of risk_level_comb,
// captured when system_fsm is in CLASSIFY (classify_enable pulse, visible
// during DECIDE - the same cycle system_fsm reads risk_level_comb to
// branch). Used downstream by PMU and the packet generator.
always @(posedge clk or posedge reset) begin
    if(reset)
        risk_level_reg <= 2'b00;
    else if(classify_enable)
        risk_level_reg <= risk_level_comb;
end
 
assign risk_level_status = risk_level_reg;
assign severity_score_status = severity_score_reg;
 
//----------------------------------------------------------------------------
// Power Management Unit
//----------------------------------------------------------------------------
wire sensor_clk_enable_stat, processing_clk_enable_stat, comm_clk_enable;
 
pmu PMU (
    .clk(clk),
    .reset(reset),
    .risk_level(risk_level_reg),
    .sensor_clk_enable(sensor_clk_enable_stat),
    .processing_clk_enable(processing_clk_enable_stat),
    .comm_clk_enable(comm_clk_enable)
);
// sensor_clk_enable_stat / processing_clk_enable_stat: status-only in this
// baseline top, intentionally unused for gating - see module header note.
 
//----------------------------------------------------------------------------
// Communication clock gating - selectable AND / ICG / none.
// Both gate cells are instantiated so AND-vs-ICG resynthesis comparisons
// (Stage 7) only require changing the CLK_GATE_MODE parameter, not the RTL.
//----------------------------------------------------------------------------
wire comm_clk_and;
wire comm_clk_icg;
wire comm_gated_clk;
 
clock_gate_and CG_AND (
    .clk(clk),
    .clk_enable(comm_clk_enable),
    .gated_clk(comm_clk_and)
);
 
clock_gate_icg CG_ICG (
    .clk(clk),
    .clk_enable(comm_clk_enable),
    .gated_clk(comm_clk_icg)
);
 
assign comm_gated_clk = (CLK_GATE_MODE == 1) ? comm_clk_and :
                         (CLK_GATE_MODE == 2) ? comm_clk_icg :
                                                 clk;
 
//----------------------------------------------------------------------------
// Communication domain - runs on comm_gated_clk. reset stays on the
// ungated global reset so the domain can always be brought out of reset
// regardless of current gating state.
//----------------------------------------------------------------------------
wire [47:0] packet_data;
wire [7:0] crc;
 
packet_generator PG (
    .node_id(node_id),
    .ph_reg(ph_reg),
    .tds_reg(tds_reg),
    .turbidity_reg(turbidity_reg),
    .risk_level(risk_level_reg),
    .packet_data(packet_data)
);
 
crc8_generator CRC8 (
    .packet_data(packet_data),
    .crc(crc)
);
 
wire serialize_start;
wire [7:0] tx_byte;
wire byte_valid;
wire byte_ack;
wire serialization_done;
wire spi_start;
wire spi_done;
 
communication_controller COMM (
    .clk(comm_gated_clk),
    .reset(reset),
    .transmit_enable(transmit_enable),
    .byte_valid(byte_valid),
    .spi_done(spi_done),
    .serialization_done(serialization_done),
    .serialize_start(serialize_start),
    .spi_start(spi_start),
    .byte_ack(byte_ack),
    .comm_done(tx_done)
);
 
packet_serializer PS (
    .clk(comm_gated_clk),
    .reset(reset),
    .start_serialization(serialize_start),
    .packet_data(packet_data),
    .crc(crc),
    .byte_ack(byte_ack),
    .tx_byte(tx_byte),
    .byte_valid(byte_valid),
    .serialization_done(serialization_done)
);
 
spi_master SPI (
    .clk(comm_gated_clk),
    .reset(reset),
    .start_tx(spi_start),
    .tx_data(tx_byte),
    .sclk(sclk),
    .mosi(mosi),
    .cs(cs),
    .tx_done(spi_done)
);
 
endmodule
 


aquasense_top_tb;-
`timescale 1ns/1ps

module aquasense_top_tb;

    reg clk;
    reg reset;

    reg [7:0] node_id;
    reg [7:0] ph_in;
    reg [7:0] tds_in;
    reg [7:0] turbidity_in;
    reg [7:0] temp_in;
    reg [7:0] cond_in;

    wire sclk;
    wire mosi;
    wire cs;
    wire tx_done;
    wire sleep_enable;
    wire [1:0] risk_level_status;
    wire [3:0] severity_score_status;

    reg [7:0] captured_byte [0:6];
    integer bit_index;
    integer byte_index;
    integer tx_done_count;
    integer i;

    aquasense_top #(.CLK_GATE_MODE(2)) DUT (
        .clk(clk),
        .reset(reset),
        .node_id(node_id),
        .ph_in(ph_in),
        .tds_in(tds_in),
        .turbidity_in(turbidity_in),
        .temp_in(temp_in),
        .cond_in(cond_in),
        .sclk(sclk),
        .mosi(mosi),
        .cs(cs),
        .tx_done(tx_done),
        .sleep_enable(sleep_enable),
        .risk_level_status(risk_level_status),
        .severity_score_status(severity_score_status)
    );

    // 100 MHz main clock.
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // Sample MOSI on the SPI rising edge (Mode-0 behavior).
    always @(posedge sclk) begin
        if (!cs) begin
            captured_byte[byte_index][7-bit_index] = mosi;

            if (bit_index == 7) begin
                bit_index = 0;
                byte_index = byte_index + 1;
            end
            else begin
                bit_index = bit_index + 1;
            end
        end
    end

    always @(posedge tx_done)
        tx_done_count = tx_done_count + 1;

    initial begin
        bit_index = 0;
        byte_index = 0;
        tx_done_count = 0;

        for (i = 0; i < 7; i = i + 1)
            captured_byte[i] = 8'h00;

        reset = 1'b1;

        // Representative contaminated sample:
        // pH contribution       = 4
        // TDS contribution      = 3
        // Turbidity contribution= 3
        // Total severity        = 10 -> HIGH (2'b11)
        node_id    = 8'h01;
        ph_in      = 8'd50;
        tds_in     = 8'd150;
        turbidity_in = 8'd10;
        temp_in    = 8'd25;
        cond_in    = 8'd20;

        #20;
        reset = 1'b0;

        #5000;

        $display("---------------------------------------------");
        $display("AquaSense SoC Communication Verification");
        $display("severity_score = %0d", severity_score_status);
        $display("risk_level     = %0d", risk_level_status);
        $display("tx_done_count  = %0d", tx_done_count);
        $display("captured bytes:");
        for (i = 0; i < 7; i = i + 1)
            $display("  byte[%0d] = %02h", i, captured_byte[i]);
        $display("---------------------------------------------");

        if (severity_score_status != 4'd10)
            $display("FAIL: unexpected severity score.");
        else if (risk_level_status != 2'b11)
            $display("FAIL: unexpected risk level.");
        else if (byte_index != 7)
            $display("FAIL: expected 7 transmitted bytes, observed %0d.", byte_index);
        else if (captured_byte[0] !== 8'h01 ||
                 captured_byte[1] !== 8'h32 ||
                 captured_byte[2] !== 8'h96 ||
                 captured_byte[3] !== 8'h0A ||
                 captured_byte[4] !== 8'hC0 ||
                 captured_byte[5] !== 8'h00 ||
                 captured_byte[6] !== 8'hF3)
            $display("FAIL: transmitted byte stream does not match expected frame.");
        else if (tx_done_count != 1)
            $display("FAIL: expected exactly one communication completion pulse.");
        else
            $display("PASS: complete 56-bit frame (7 bytes) transmitted correctly.");

        $finish;
    end

endmodule
