//AAKHRI_AQUA
//adc_controller
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.08.2026 23:34:49
// Design Name: 
// Module Name: adc_controller
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
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

//system_fsm
`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// AQUASENSE SYSTEM FSM
//
// Operation:
//
//   SAFE:
//       Periodically acquires and evaluates sensor data.
//       No transmission.
//
//   CONTAMINATED:
//       First detection -> one transmission.
//       Further measurements may occur, but no additional transmission
//       is allowed until a SAFE measurement is observed.
//
//   RE-ARM:
//       A fresh SAFE measurement clears tx_latched.
//       The next contamination event can therefore transmit again.
//
// Important:
//   tx_latched prevents repeated transmissions while contamination remains.
//   WAIT_CLEAR does NOT blindly transmit again; it forces a fresh acquisition.
//////////////////////////////////////////////////////////////////////////////////

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

/*
 * Transmission lock.
 *
 * 0 = transmission is armed
 * 1 = transmission already sent for the current contamination episode
 */
reg tx_latched;


//==============================================================
// STATE ENCODING
//==============================================================

parameter IDLE       = 3'b000;
parameter ACQUIRE    = 3'b001;
parameter PROCESS    = 3'b010;
parameter CLASSIFY   = 3'b011;
parameter DECIDE     = 3'b100;
parameter TRANSMIT   = 3'b101;
parameter SLEEP      = 3'b110;
parameter WAIT_CLEAR = 3'b111;


//==============================================================
// FSM
//==============================================================

always @(posedge clk or posedge reset)
begin

    if(reset)
    begin

        state <= IDLE;

        tx_latched <= 1'b0;

        adc_start       <= 1'b0;
        process_enable  <= 1'b0;
        classify_enable <= 1'b0;
        transmit_enable <= 1'b0;
        sleep_enable    <= 1'b0;

    end

    else
    begin

        //======================================================
        // Default pulse outputs
        //======================================================

        adc_start       <= 1'b0;
        process_enable  <= 1'b0;
        classify_enable <= 1'b0;
        transmit_enable <= 1'b0;
        sleep_enable    <= 1'b0;


        //======================================================
        // FSM
        //======================================================

        case(state)


            //==================================================
            // IDLE
            //==================================================

            IDLE:
            begin

                state <= ACQUIRE;

            end


            //==================================================
            // ADC ACQUISITION
            //==================================================

            ACQUIRE:
            begin

                adc_start <= 1'b1;

                /*
                 * ADC controller receives start_conversion.
                 * Remain here until conversion completes.
                 */

                if(adc_done)
                    state <= PROCESS;

            end


            //==================================================
            // SENSOR PROCESSING
            //==================================================

            PROCESS:
            begin

                process_enable <= 1'b1;

                state <= CLASSIFY;

            end


            //==================================================
            // RISK CLASSIFICATION
            //==================================================

            CLASSIFY:
            begin

                classify_enable <= 1'b1;

                state <= DECIDE;

            end


            //==================================================
            // DECISION
            //==================================================

            DECIDE:
            begin

                //------------------------------------------------
                // SAFE
                //------------------------------------------------

                if(risk_level == 2'b00)
                begin

                    /*
                     * A fresh SAFE measurement means that the
                     * contamination episode has ended.
                     *
                     * Therefore re-arm the transmitter.
                     */

                    tx_latched <= 1'b0;

                    state <= SLEEP;

                end


                //------------------------------------------------
                // CONTAMINATED
                //------------------------------------------------

                else
                begin

                    /*
                     * If no transmission has occurred for this
                     * contamination episode, transmit once.
                     */

                    if(tx_latched == 1'b0)
                    begin

                        state <= TRANSMIT;

                    end

                    /*
                     * A transmission has already occurred.
                     *
                     * Do NOT transmit again.
                     *
                     * Instead perform another measurement.
                     */

                    else
                    begin

                        state <= WAIT_CLEAR;

                    end

                end

            end


            //==================================================
            // TRANSMIT
            //==================================================

            TRANSMIT:
            begin

                /*
                 * One-cycle transmission request.
                 */

                transmit_enable <= 1'b1;

                /*
                 * Lock transmission for the current
                 * contamination episode.
                 */

                tx_latched <= 1'b1;

                /*
                 * After transmission, continue monitoring
                 * rather than transmitting repeatedly.
                 */

                state <= WAIT_CLEAR;

            end


            //==================================================
            // WAIT / MONITOR CONTAMINATION
            //==================================================

            WAIT_CLEAR:
            begin

                /*
                 * System is waiting for a NEW measurement.
                 *
                 * We deliberately go back through ADC acquisition.
                 *
                 * The next DECIDE state will determine whether
                 * contamination is still present or whether the
                 * system has become SAFE and can re-arm.
                 */

                sleep_enable <= 1'b1;

                state <= ACQUIRE;

            end


            //==================================================
            // SLEEP
            //==================================================

            SLEEP:
            begin

                sleep_enable <= 1'b1;

                /*
                 * Start another monitoring cycle.
                 *
                 * tx_latched has already been cleared in DECIDE
                 * when SAFE was detected.
                 */

                state <= IDLE;

            end


            //==================================================
            // DEFAULT
            //==================================================

            default:
            begin

                state <= IDLE;

                tx_latched <= 1'b0;

            end

        endcase

    end

end

endmodule

//
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.08.2026 23:35:56
// Design Name: 
// Module Name: sensor_register_bank
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

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
 
//

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.08.2026 23:36:49
// Design Name: 
// Module Name: sensor_fusion_engine
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


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
 
//

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.08.2026 23:37:46
// Design Name: 
// Module Name: risk_classifier
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


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

//
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.08.2026 23:38:34
// Design Name: 
// Module Name: clock_gate_and
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module clock_gate_and(
 
    input clk,
    input clk_enable,
 
    output gated_clk
 
    );
 
    assign gated_clk = clk & clk_enable;
 
endmodule

///
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.08.2026 23:39:23
// Design Name: 
// Module Name: clock_gate_icg
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


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
 
//

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.08.2026 23:40:46
// Design Name: 
// Module Name: packet_generator
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


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


//

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.08.2026 23:43:00
// Design Name: 
// Module Name: crc8_generator
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


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


//

`timescale 1ns / 1ps

module sx1276_controller (

    input clk,
    input reset,

    input transmit_enable,

    input  [7:0] tx_byte,
    input        byte_valid,
    input        last_byte,

    output reg   serialize_start,
    output reg   byte_ack,

    output reg        spi_start,
    output reg [7:0]  spi_tx_data,
    output reg        hold_cs,

    input spi_done,
    input sx1276_dio0,

    output reg tx_done
);

    //============================================================
    // SX1276 WRITE ADDRESSES
    //============================================================

    localparam [7:0] A_OPMODE  = 8'h81;
    localparam [7:0] A_DIO     = 8'hC0;
    localparam [7:0] A_TXBASE  = 8'h8E;
    localparam [7:0] A_PTR     = 8'h8D;
    localparam [7:0] A_FIFO    = 8'h80;
    localparam [7:0] A_LENGTH  = 8'hA2;
    localparam [7:0] A_IRQ     = 8'h92;

    //============================================================
    // SX1276 VALUES
    //============================================================

    localparam [7:0] V_STANDBY = 8'h81;
    localparam [7:0] V_DIO     = 8'h00;
    localparam [7:0] V_TXBASE  = 8'h80;
    localparam [7:0] V_PTR     = 8'h80;
    localparam [7:0] V_LENGTH  = 8'h07;
    localparam [7:0] V_TX      = 8'h83;
    localparam [7:0] V_IRQ     = 8'h08;

    //============================================================
    // FSM
    //============================================================

    localparam [7:0]

    IDLE       = 8'd0,

    // OPMODE
    OP_ADDR_L  = 8'd1,
    OP_ADDR_S  = 8'd2,
    OP_ADDR_W  = 8'd3,
    OP_DATA_L  = 8'd4,
    OP_DATA_S  = 8'd5,
    OP_DATA_W  = 8'd6,
    OP_GAP     = 8'd7,

    // DIO
    DIO_ADDR_L = 8'd8,
    DIO_ADDR_S = 8'd9,
    DIO_ADDR_W = 8'd10,
    DIO_DATA_L = 8'd11,
    DIO_DATA_S = 8'd12,
    DIO_DATA_W = 8'd13,
    DIO_GAP    = 8'd14,

    // TX BASE
    TB_ADDR_L  = 8'd15,
    TB_ADDR_S  = 8'd16,
    TB_ADDR_W  = 8'd17,
    TB_DATA_L  = 8'd18,
    TB_DATA_S  = 8'd19,
    TB_DATA_W  = 8'd20,
    TB_GAP     = 8'd21,

    // FIFO POINTER
    PT_ADDR_L  = 8'd22,
    PT_ADDR_S  = 8'd23,
    PT_ADDR_W  = 8'd24,
    PT_DATA_L  = 8'd25,
    PT_DATA_S  = 8'd26,
    PT_DATA_W  = 8'd27,
    PT_GAP     = 8'd28,

    // FIFO
    FIFO_WAIT  = 8'd29,
    FIFO_A_L   = 8'd30,
    FIFO_A_S   = 8'd31,
    FIFO_A_W   = 8'd32,
    FIFO_D_L   = 8'd33,
    FIFO_D_S   = 8'd34,
    FIFO_D_W   = 8'd35,

    // IMPORTANT:
    // One-cycle delay after byte_ack before accepting
    // the next serializer byte.
    FIFO_NEXT  = 8'd59,

    // PAYLOAD LENGTH
    LEN_ADDR_L = 8'd36,
    LEN_ADDR_S = 8'd37,
    LEN_ADDR_W = 8'd38,
    LEN_DATA_L = 8'd39,
    LEN_DATA_S = 8'd40,
    LEN_DATA_W = 8'd41,
    LEN_GAP    = 8'd42,

    // TX MODE
    TX_ADDR_L  = 8'd43,
    TX_ADDR_S  = 8'd44,
    TX_ADDR_W  = 8'd45,
    TX_DATA_L  = 8'd46,
    TX_DATA_S  = 8'd47,
    TX_DATA_W  = 8'd48,
    TX_GAP     = 8'd49,

    // WAIT FOR SX1276
    WAIT_DIO   = 8'd50,

    // CLEAR IRQ
    IRQ_ADDR_L = 8'd51,
    IRQ_ADDR_S = 8'd52,
    IRQ_ADDR_W = 8'd53,
    IRQ_DATA_L = 8'd54,
    IRQ_DATA_S = 8'd55,
    IRQ_DATA_W = 8'd56,
    IRQ_GAP    = 8'd57,

    COMPLETE   = 8'd58;

    reg [7:0] state;

    reg [7:0] fifo_byte;
    reg       fifo_last;


    //============================================================
    // MAIN FSM
    //============================================================

    always @(posedge clk or posedge reset)
    begin

        if(reset)
        begin
            state <= IDLE;

            serialize_start <= 1'b0;
            byte_ack        <= 1'b0;

            spi_start       <= 1'b0;
            spi_tx_data     <= 8'h00;
            hold_cs         <= 1'b0;

            tx_done         <= 1'b0;

            fifo_byte       <= 8'h00;
            fifo_last       <= 1'b0;
        end

        else
        begin

            //====================================================
            // One-cycle pulse outputs
            //====================================================

            serialize_start <= 1'b0;
            byte_ack        <= 1'b0;
            spi_start       <= 1'b0;
            tx_done         <= 1'b0;


            case(state)

            //====================================================
            // IDLE
            //====================================================

            IDLE:
            begin
                hold_cs <= 1'b0;

                if(transmit_enable)
                begin
                    serialize_start <= 1'b1;
                    state <= OP_ADDR_L;
                end
            end


            //====================================================
            // OPMODE = 0x81
            //====================================================

            OP_ADDR_L:
            begin
                spi_tx_data <= A_OPMODE;
                hold_cs <= 1'b1;
                state <= OP_ADDR_S;
            end

            OP_ADDR_S:
            begin
                spi_start <= 1'b1;
                state <= OP_ADDR_W;
            end

            OP_ADDR_W:
            begin
                if(spi_done)
                    state <= OP_DATA_L;
            end

            OP_DATA_L:
            begin
                spi_tx_data <= V_STANDBY;
                hold_cs <= 1'b0;
                state <= OP_DATA_S;
            end

            OP_DATA_S:
            begin
                spi_start <= 1'b1;
                state <= OP_DATA_W;
            end

            OP_DATA_W:
            begin
                if(spi_done)
                    state <= OP_GAP;
            end

            OP_GAP:
            begin
                hold_cs <= 1'b0;
                state <= DIO_ADDR_L;
            end


            //====================================================
            // DIO MAPPING
            //====================================================

            DIO_ADDR_L:
            begin
                spi_tx_data <= A_DIO;
                hold_cs <= 1'b1;
                state <= DIO_ADDR_S;
            end

            DIO_ADDR_S:
            begin
                spi_start <= 1'b1;
                state <= DIO_ADDR_W;
            end

            DIO_ADDR_W:
            begin
                if(spi_done)
                    state <= DIO_DATA_L;
            end

            DIO_DATA_L:
            begin
                spi_tx_data <= V_DIO;
                hold_cs <= 1'b0;
                state <= DIO_DATA_S;
            end

            DIO_DATA_S:
            begin
                spi_start <= 1'b1;
                state <= DIO_DATA_W;
            end

            DIO_DATA_W:
            begin
                if(spi_done)
                    state <= DIO_GAP;
            end

            DIO_GAP:
            begin
                hold_cs <= 1'b0;
                state <= TB_ADDR_L;
            end


            //====================================================
            // TX BASE
            //====================================================

            TB_ADDR_L:
            begin
                spi_tx_data <= A_TXBASE;
                hold_cs <= 1'b1;
                state <= TB_ADDR_S;
            end

            TB_ADDR_S:
            begin
                spi_start <= 1'b1;
                state <= TB_ADDR_W;
            end

            TB_ADDR_W:
            begin
                if(spi_done)
                    state <= TB_DATA_L;
            end

            TB_DATA_L:
            begin
                spi_tx_data <= V_TXBASE;
                hold_cs <= 1'b0;
                state <= TB_DATA_S;
            end

            TB_DATA_S:
            begin
                spi_start <= 1'b1;
                state <= TB_DATA_W;
            end

            TB_DATA_W:
            begin
                if(spi_done)
                    state <= TB_GAP;
            end

            TB_GAP:
            begin
                hold_cs <= 1'b0;
                state <= PT_ADDR_L;
            end


            //====================================================
            // FIFO POINTER
            //====================================================

            PT_ADDR_L:
            begin
                spi_tx_data <= A_PTR;
                hold_cs <= 1'b1;
                state <= PT_ADDR_S;
            end

            PT_ADDR_S:
            begin
                spi_start <= 1'b1;
                state <= PT_ADDR_W;
            end

            PT_ADDR_W:
            begin
                if(spi_done)
                    state <= PT_DATA_L;
            end

            PT_DATA_L:
            begin
                spi_tx_data <= V_PTR;
                hold_cs <= 1'b0;
                state <= PT_DATA_S;
            end

            PT_DATA_S:
            begin
                spi_start <= 1'b1;
                state <= PT_DATA_W;
            end

            PT_DATA_W:
            begin
                if(spi_done)
                    state <= PT_GAP;
            end

            PT_GAP:
            begin
                hold_cs <= 1'b0;
                state <= FIFO_WAIT;
            end


            //====================================================
            // FIFO WAIT
            //====================================================

            FIFO_WAIT:
            begin
                hold_cs <= 1'b0;

                /*
                 * Wait for serializer to present a valid byte.
                 */
                if(byte_valid)
                begin
                    fifo_byte <= tx_byte;
                    fifo_last <= last_byte;

                    state <= FIFO_A_L;
                end
            end


            //====================================================
            // FIFO ADDRESS
            //====================================================

            FIFO_A_L:
            begin
                spi_tx_data <= A_FIFO;
                hold_cs <= 1'b1;
                state <= FIFO_A_S;
            end

            FIFO_A_S:
            begin
                spi_start <= 1'b1;
                state <= FIFO_A_W;
            end

            FIFO_A_W:
            begin
                if(spi_done)
                    state <= FIFO_D_L;
            end


            //====================================================
            // FIFO DATA
            //====================================================

            FIFO_D_L:
            begin
                spi_tx_data <= fifo_byte;
                hold_cs <= 1'b0;
                state <= FIFO_D_S;
            end

            FIFO_D_S:
            begin
                spi_start <= 1'b1;
                state <= FIFO_D_W;
            end

            FIFO_D_W:
            begin
                if(spi_done)
                begin
                    /*
                     * Tell serializer that the current byte
                     * has been successfully transmitted.
                     */
                    byte_ack <= 1'b1;

                    if(fifo_last)
                    begin
                        /*
                         * Last byte complete.
                         * Move directly to payload length.
                         */
                        state <= LEN_ADDR_L;
                    end
                    else
                    begin
                        /*
                         * DO NOT immediately return to FIFO_WAIT.
                         *
                         * Give packet_serializer one complete
                         * clock cycle to consume byte_ack and
                         * advance tx_byte.
                         */
                        state <= FIFO_NEXT;
                    end
                end
            end


            //====================================================
            // FIFO NEXT
            //
            // One-cycle handshake delay.
            //
            // This fixes the duplicated first-byte problem:
            //
            // Before:
            //   A5 -> ACK -> FIFO_WAIT -> A5 again
            //
            // Now:
            //   A5 -> ACK -> FIFO_NEXT -> FIFO_WAIT -> 78
            //====================================================

            FIFO_NEXT:
            begin
                hold_cs <= 1'b0;

                /*
                 * Do not sample byte_valid here.
                 * Just give the serializer one clock to
                 * update its output.
                 */
                state <= FIFO_WAIT;
            end


            //====================================================
            // PAYLOAD LENGTH
            //====================================================

            LEN_ADDR_L:
            begin
                spi_tx_data <= A_LENGTH;
                hold_cs <= 1'b1;
                state <= LEN_ADDR_S;
            end

            LEN_ADDR_S:
            begin
                spi_start <= 1'b1;
                state <= LEN_ADDR_W;
            end

            LEN_ADDR_W:
            begin
                if(spi_done)
                    state <= LEN_DATA_L;
            end

            LEN_DATA_L:
            begin
                spi_tx_data <= V_LENGTH;
                hold_cs <= 1'b0;
                state <= LEN_DATA_S;
            end

            LEN_DATA_S:
            begin
                spi_start <= 1'b1;
                state <= LEN_DATA_W;
            end

            LEN_DATA_W:
            begin
                if(spi_done)
                    state <= LEN_GAP;
            end

            LEN_GAP:
            begin
                hold_cs <= 1'b0;
                state <= TX_ADDR_L;
            end


            //====================================================
            // START TX
            //====================================================

            TX_ADDR_L:
            begin
                spi_tx_data <= A_OPMODE;
                hold_cs <= 1'b1;
                state <= TX_ADDR_S;
            end

            TX_ADDR_S:
            begin
                spi_start <= 1'b1;
                state <= TX_ADDR_W;
            end

            TX_ADDR_W:
            begin
                if(spi_done)
                    state <= TX_DATA_L;
            end

            TX_DATA_L:
            begin
                spi_tx_data <= V_TX;
                hold_cs <= 1'b0;
                state <= TX_DATA_S;
            end

            TX_DATA_S:
            begin
                spi_start <= 1'b1;
                state <= TX_DATA_W;
            end

            TX_DATA_W:
            begin
                if(spi_done)
                    state <= TX_GAP;
            end

            TX_GAP:
            begin
                hold_cs <= 1'b0;
                state <= WAIT_DIO;
            end


            //====================================================
            // WAIT FOR SX1276 DIO0
            //====================================================

            WAIT_DIO:
            begin
                hold_cs <= 1'b0;

                if(sx1276_dio0)
                    state <= IRQ_ADDR_L;
            end


            //====================================================
            // CLEAR IRQ
            //====================================================

            IRQ_ADDR_L:
            begin
                spi_tx_data <= A_IRQ;
                hold_cs <= 1'b1;
                state <= IRQ_ADDR_S;
            end

            IRQ_ADDR_S:
            begin
                spi_start <= 1'b1;
                state <= IRQ_ADDR_W;
            end

            IRQ_ADDR_W:
            begin
                if(spi_done)
                    state <= IRQ_DATA_L;
            end

            IRQ_DATA_L:
            begin
                spi_tx_data <= V_IRQ;
                hold_cs <= 1'b0;
                state <= IRQ_DATA_S;
            end

            IRQ_DATA_S:
            begin
                spi_start <= 1'b1;
                state <= IRQ_DATA_W;
            end

            IRQ_DATA_W:
            begin
                if(spi_done)
                    state <= IRQ_GAP;
            end

            IRQ_GAP:
            begin
                hold_cs <= 1'b0;
                state <= COMPLETE;
            end


            //====================================================
            // COMPLETE
            //====================================================

            COMPLETE:
            begin
                tx_done <= 1'b1;
                hold_cs <= 1'b0;
                state <= IDLE;
            end


            //====================================================
            // DEFAULT
            //====================================================

            default:
            begin
                state <= IDLE;
                hold_cs <= 1'b0;
            end

            endcase
        end
    end

endmodule


//

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.08.2026 23:43:34
// Design Name: 
// Module Name: packet_serializer
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


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
    output reg serialization_done,
    output last_byte
 
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
assign last_byte = byte_valid && (byte_count == 3'd6);
 
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
 
//

`timescale 1ns / 1ps

module spi_master(

    input clk,
    input reset,

    input start_tx,
    input [7:0] tx_data,

    input hold_cs,

    output reg sclk,
    output reg mosi,
    output reg cs,

    output reg tx_done
);

    reg [7:0] shift_reg;
    reg [3:0] bit_count;

    reg hold_cs_latched;

    reg [1:0] state;

    localparam IDLE  = 2'b00;
    localparam LOAD  = 2'b01;
    localparam SHIFT = 2'b10;
    localparam DONE  = 2'b11;


    always @(posedge clk or posedge reset)
    begin

        if(reset)
        begin

            state <= IDLE;

            sclk <= 1'b0;
            mosi <= 1'b0;
            cs <= 1'b1;

            tx_done <= 1'b0;

            shift_reg <= 8'h00;
            bit_count <= 4'd0;

            hold_cs_latched <= 1'b0;
        end

        else
        begin

            case(state)

            //======================================================
            // IDLE
            //======================================================

            IDLE:
            begin

                tx_done <= 1'b0;
                sclk <= 1'b0;

                /*
                 * CS is HIGH only when the previous transaction
                 * requested release.
                 */
                if(!hold_cs_latched)
                    cs <= 1'b1;

                if(start_tx)
                begin

                    /*
                     * IMPORTANT:
                     * Capture hold_cs at the exact moment the
                     * transaction begins.
                     */
                    hold_cs_latched <= hold_cs;

                    state <= LOAD;

                end

            end


            //======================================================
            // LOAD
            //======================================================

            LOAD:
            begin

                shift_reg <= tx_data;

                bit_count <= 4'd0;

                /*
                 * Every transaction begins with CS LOW.
                 */
                cs <= 1'b0;

                /*
                 * Put first MOSI bit on the bus before first
                 * rising edge.
                 */
                mosi <= tx_data[7];

                state <= SHIFT;

            end


            //======================================================
            // SHIFT
            //======================================================

            SHIFT:
            begin

                /*
                 * Toggle SPI clock.
                 */
                sclk <= ~sclk;

                /*
                 * Falling edge:
                 * prepare next MOSI bit.
                 */
                if(sclk == 1'b1)
                begin

                    if(bit_count < 7)
                    begin

                        shift_reg <= {shift_reg[6:0],1'b0};

                        bit_count <= bit_count + 1'b1;

                        mosi <= shift_reg[6];

                    end

                    else
                    begin

                        /*
                         * All 8 bits have been transmitted.
                         */
                        state <= DONE;

                    end

                end

            end


            //======================================================
            // DONE
            //======================================================

            DONE:
            begin

                sclk <= 1'b0;

                tx_done <= 1'b1;

                /*
                 * Use the LATCHED value, NOT live hold_cs.
                 *
                 * This is the critical fix.
                 */
                if(!hold_cs_latched)
                    cs <= 1'b1;

                state <= IDLE;

            end


            //======================================================
            // DEFAULT
            //======================================================

            default:
            begin

                state <= IDLE;

                sclk <= 1'b0;
                mosi <= 1'b0;
                cs <= 1'b1;

                tx_done <= 1'b0;

                hold_cs_latched <= 1'b0;

            end

            endcase

        end

    end

endmodule
//
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// AQUASENSE_TOP_TB
//
// Full-SoC testbench for aquasense_top.
//
// What it does:
//   PHASE 1 - holds sensor inputs at SAFE values. Expect system_fsm to take
//             the SLEEP path every loop (risk_level_comb == 00), so tx_done
//             should never pulse.
//   PHASE 2 - drives sensor inputs to contaminated values (severity_score
//             = 12 -> risk_level = 2'b11, HIGH). Expect system_fsm to take
//             the TRANSMIT path, and the communication subsystem to run a
//             full 7-byte SPI frame per transmission.
//   PHASE 3 - returns to SAFE values, confirms the system goes quiet again.
//
// Self-checking:
//   - byte_count increments on every internal byte_ack pulse (hierarchical
//     reference into the communication_controller instance) and is
//     reported/reset on every tx_done (comm_done) pulse. Each reported
//     frame MUST show exactly 7 bytes for the frame to be considered a
//     correctly verified transmission (Section 40, priority item 2).
//   - frame_count / mismatch flag summarise pass/fail at the end.
//////////////////////////////////////////////////////////////////////////////////

module aquasense_top_tb;

reg clk;
reg reset;

reg [7:0] node_id;
reg [7:0] ph_in, tds_in, turbidity_in, temp_in, cond_in;

wire sclk;
wire mosi;
wire cs;
wire tx_done;

wire sx1276_dio0;
wire sleep_enable;
wire [1:0] risk_level_status;
wire [3:0] severity_score_status;

integer byte_count;
integer frame_count;
integer mismatch_count;

//----------------------------------------------------------------------------
// DUT
//----------------------------------------------------------------------------
aquasense_top #(
    .CLK_GATE_MODE(0)   // 0 = baseline, 1 = AND gating, 2 = ICG gating
) DUT (
    .clk(clk),
    .reset(reset),

    .node_id(node_id),
    .ph_in(ph_in),
    .tds_in(tds_in),
    .turbidity_in(turbidity_in),
    .temp_in(temp_in),
    .cond_in(cond_in),

    .sx1276_dio0(sx1276_dio0),
    .sclk(sclk),
    .mosi(mosi),
    .cs(cs),
    .tx_done(tx_done),

    .sleep_enable(sleep_enable),
    .risk_level_status(risk_level_status),
    .severity_score_status(severity_score_status)
);

//----------------------------------------------------------------------------
// 100 MHz system clock
//----------------------------------------------------------------------------
initial clk = 0;
always #5 clk = ~clk;

//----------------------------------------------------------------------------
// Byte-level monitor: counts every acked SPI byte inside the current frame.
// Hierarchical reference into the communication_controller instance -
// simulation-only visibility, not part of the synthesizable design.
//----------------------------------------------------------------------------
always @(posedge clk) begin
    if(DUT.byte_ack)
        byte_count = byte_count + 1;
end

//----------------------------------------------------------------------------
// Frame-level monitor: fires once per completed transmission.
//----------------------------------------------------------------------------
always @(posedge tx_done) begin
    frame_count = frame_count + 1;

    if(byte_count == 7)
        $display("[%0t ns] FRAME %0d COMPLETE  - PASS  (7/7 bytes)  risk_level=%b severity=%0d",
                  $time, frame_count, risk_level_status, severity_score_status);
    else begin
        $display("[%0t ns] FRAME %0d COMPLETE  - FAIL  (%0d/7 bytes) risk_level=%b severity=%0d",
                  $time, frame_count, byte_count, risk_level_status, severity_score_status);
        mismatch_count = mismatch_count + 1;
    end

    byte_count = 0;
end

//----------------------------------------------------------------------------
// SPI byte content monitor: prints tx_byte every time it is latched into
// spi_master, so the byte stream can be visually confirmed against the
// packet content (node_id, ph, tds, turbidity, risk_level, reserved, crc).
//----------------------------------------------------------------------------
always @(posedge clk)
begin
    if(DUT.spi_start)
        $display("[%0t ns] SPI START: tx_byte = 0x%02h",
                 $time, DUT.tx_byte);
end
//----------------------------------------------------------------------------
// Stimulus
//----------------------------------------------------------------------------
initial begin

    //============================================================
    // INITIALIZATION
    //============================================================

    reset   = 1;
    node_id = 8'hA5;

    // SAFE values
    ph_in        = 8'd70;
    tds_in       = 8'd50;
    turbidity_in = 8'd2;
    temp_in      = 8'd25;
    cond_in      = 8'd40;

    byte_count     = 0;
    frame_count    = 0;
    mismatch_count = 0;

    repeat(4) @(posedge clk);
    reset = 0;


    //============================================================
    // TEST 1
    // SAFE -> CONTAMINATED
    // Expect exactly ONE transmission
    //============================================================

    $display("=================================================================");
    $display("TEST 1: SAFE -> CONTAMINATED");
    $display("Expect exactly ONE SX1276 transmission");
    $display("=================================================================");

    repeat(300) @(posedge clk);

    if(frame_count != 0)
        $display("TEST 1 FAIL: transmission occurred during SAFE");
    else
        $display("SAFE condition confirmed");


    // Contaminated values
    ph_in        = 8'd120;
    tds_in       = 8'd180;
    turbidity_in = 8'd9;
    temp_in      = 8'd45;
    cond_in      = 8'd90;

    // severity = 12
    // risk = HIGH


    // Give the complete TX sequence enough time
    repeat(1500) @(posedge clk);


    if(frame_count == 1)
        $display("TEST 1 PASS: exactly one transmission occurred");
    else
        $display("TEST 1 FAIL: expected 1 transmission, got %0d",
                 frame_count);


    //============================================================
    // TEST 2
    // CONTAMINATION REMAINS HIGH
    // Expect NO SECOND TRANSMISSION
    //============================================================

    $display("=================================================================");
    $display("TEST 2: CONTAMINATION REMAINS HIGH");
    $display("Expect NO second transmission");
    $display("=================================================================");

    repeat(1500) @(posedge clk);

    if(frame_count == 1)
        $display("TEST 2 PASS: no repeated transmission");
    else
        $display("TEST 2 FAIL: unexpected repeated transmission");



    //============================================================
    // TEST 3
    // RETURN TO SAFE
    // Expect NO transmission
    //============================================================

    $display("=================================================================");
    $display("TEST 3: CONTAMINATED -> SAFE");
    $display("Expect no transmission");
    $display("=================================================================");

    ph_in        = 8'd70;
    tds_in       = 8'd50;
    turbidity_in = 8'd2;
    temp_in      = 8'd25;
    cond_in      = 8'd40;

    repeat(500) @(posedge clk);

    if(frame_count == 1)
        $display("TEST 3 PASS: system returned to SAFE without TX");
    else
        $display("TEST 3 FAIL: unexpected transmission after SAFE");



    //============================================================
    // TEST 4
    // SAFE -> CONTAMINATED AGAIN
    // Expect exactly ONE NEW transmission
    //============================================================

    $display("=================================================================");
    $display("TEST 4: SAFE -> CONTAMINATED AGAIN");
    $display("Expect exactly ONE NEW transmission");
    $display("=================================================================");

    ph_in        = 8'd120;
    tds_in       = 8'd180;
    turbidity_in = 8'd9;
    temp_in      = 8'd45;
    cond_in      = 8'd90;

    repeat(1500) @(posedge clk);

    if(frame_count == 2)
        $display("TEST 4 PASS: exactly one new transmission occurred");
    else
        $display("TEST 4 FAIL: expected 2 total transmissions, got %0d",
                 frame_count);



    //============================================================
    // FINAL SUMMARY
    //============================================================

    $display("=================================================================");
    $display("AQUASENSE SX1276 VERIFICATION SUMMARY");
    $display("=================================================================");

    $display("Total frames transmitted : %0d", frame_count);
    $display("Frame byte mismatches    : %0d", mismatch_count);

    if((frame_count == 2) && (mismatch_count == 0))
    begin
        $display("RESULT: PASS");
        $display("All four communication/re-arm tests passed.");
    end
    else
    begin
        $display("RESULT: FAIL");
        $display("Communication/re-arm verification incomplete.");
    end

    $display("=================================================================");

    $finish;

end
//----------------------------------------------------------------------------
// Safety timeout, in case a handshake ever stalls
//----------------------------------------------------------------------------
initial begin
    #100000;
    $display("[%0t ns] TIMEOUT - forcing simulation end", $time);
    $finish;
end
sx1276_behavioral_model SX1276_MODEL (

    .sclk(sclk),
    .mosi(mosi),
    .cs(cs),

    .dio0(sx1276_dio0)

);

//============================================================
// RE-ARM DEBUG MONITOR
//============================================================


endmodule

//

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.08.2026 23:48:57
// Design Name: 
// Module Name: aquasense_top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module aquasense_top #(
    parameter CLK_GATE_MODE = 0   // 0 = baseline, 1 = AND gating, 2 = ICG gating
)(
    input clk,
    input reset,
 
    input [7:0] node_id,
    input [7:0] ph_in,
    input [7:0] tds_in,
    input [7:0] turbidity_in,
    input [7:0] temp_in,
    input [7:0] cond_in,
    input sx1276_dio0,
 
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
wire last_byte;


wire sx_spi_start;
wire [7:0] sx_spi_tx_data;
wire sx_hold_cs;
wire sx_tx_done;



sx1276_controller SX1276 (

    .clk(clk),
    .reset(reset),

    .transmit_enable(transmit_enable),

    .tx_byte(tx_byte),
    .byte_valid(byte_valid),
    .last_byte(last_byte),

    .serialize_start(serialize_start),
    .byte_ack(byte_ack),

    .spi_start(sx_spi_start),
    .spi_tx_data(sx_spi_tx_data),
    .hold_cs(sx_hold_cs),

    .spi_done(spi_done),

    .sx1276_dio0(sx1276_dio0),

    .tx_done(tx_done)
);
 
packet_serializer PS (

    .clk(clk),
    .reset(reset),

    .start_serialization(serialize_start),

    .packet_data(packet_data),
    .crc(crc),

    .byte_ack(byte_ack),

    .tx_byte(tx_byte),
    .byte_valid(byte_valid),
    .serialization_done(serialization_done),
    .last_byte(last_byte)
);
 
spi_master SPI (

    .clk(clk),
    .reset(reset),

    .start_tx(sx_spi_start),
    .tx_data(sx_spi_tx_data),

    .hold_cs(sx_hold_cs),

    .sclk(sclk),
    .mosi(mosi),
    .cs(cs),

    .tx_done(spi_done)
);
 
endmodule
 

//


`timescale 1ns / 1ps

module sx1276_behavioral_model (

    input  wire sclk,
    input  wire mosi,
    input  wire cs,

    output reg dio0
);

    reg [7:0] registers [0:127];

    reg [7:0] shift_reg;
    reg [7:0] rx_byte;
    reg [7:0] received_byte;

    integer bit_count;

    reg address_phase;
    reg [6:0] current_addr;

    reg [7:0] fifo [0:255];
    integer fifo_count;

    initial begin

        dio0          = 1'b0;
        shift_reg     = 8'h00;
        rx_byte       = 8'h00;
        received_byte = 8'h00;

        bit_count     = 0;

        address_phase = 1'b1;
        current_addr = 7'h00;

        fifo_count    = 0;

    end


    //==============================================================
    // SPI RECEIVE
    // SX1276 receives MSB first.
    //==============================================================

    always @(posedge sclk) begin

        if (!cs) begin

            received_byte = {shift_reg[6:0], mosi};

            shift_reg = received_byte;

            bit_count = bit_count + 1;

            if (bit_count == 8) begin

                rx_byte = received_byte;

                bit_count = 0;


                //==================================================
                // FIRST BYTE = ADDRESS
                //==================================================

                if (address_phase) begin

                    current_addr = received_byte[6:0];

                    address_phase = 1'b0;

                    $display(
                        "[%0t ns] SX1276 MODEL: ADDRESS = 0x%02h",
                        $time,
                        received_byte
                    );

                end


                //==================================================
                // SECOND BYTE = DATA
                //==================================================

                else begin

                    registers[current_addr] = received_byte;

                    $display(
                        "[%0t ns] SX1276 MODEL: WRITE addr=0x%02h data=0x%02h",
                        $time,
                        current_addr,
                        received_byte
                    );


                    //================================================
                    // FIFO WRITE
                    //================================================

                    if (current_addr == 7'h00) begin

                        fifo[fifo_count] = received_byte;

                        $display(
                            "[%0t ns] SX1276 MODEL: FIFO[%0d] = 0x%02h",
                            $time,
                            fifo_count,
                            received_byte
                        );

                        fifo_count = fifo_count + 1;

                    end


                    //================================================
                    // START TX
                    //
                    // RegOpMode address = 0x01
                    // TX mode          = 0x83
                    //================================================

                    if ((current_addr == 7'h01) &&
                        (received_byte == 8'h83)) begin

                        $display(
                            "[%0t ns] SX1276 MODEL: TX START",
                            $time
                        );

                        /*
                         * Normal Verilog fork/join.
                         * Do NOT use join_none because this file
                         * is compiled as Verilog.
                         */

                        fork

                            begin

                                #1000;

                                dio0 = 1'b1;

                                $display(
                                    "[%0t ns] SX1276 MODEL: DIO0 TX_DONE",
                                    $time
                                );

                                #100;

                                dio0 = 1'b0;

                            end

                        join

                    end


                    //================================================
                    // CLEAR TxDone
                    //
                    // RegIrqFlags = 0x12
                    // Clear TxDone = 0x08
                    //================================================

                    if ((current_addr == 7'h12) &&
                        (received_byte == 8'h08)) begin

                        $display(
                            "[%0t ns] SX1276 MODEL: IRQ CLEARED",
                            $time
                        );

                    end


                    //================================================
                    // NEXT TRANSACTION STARTS WITH ADDRESS
                    //================================================

                    address_phase = 1'b1;

                end

            end

        end

    end


    //==============================================================
    // CS GOES HIGH
    //
    // One complete SPI register transaction has finished.
    //==============================================================

    always @(posedge cs) begin

        bit_count     = 0;
        shift_reg     = 8'h00;
        address_phase = 1'b1;

    end

endmodule