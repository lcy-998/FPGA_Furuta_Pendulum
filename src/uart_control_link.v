// PC telemetry and runtime LQI gain update link.
// Frame: A5 type seq length payload CRC16-CCITT-FALSE (little-endian CRC).
module uart_control_link #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD = 115200
)(
    input wire clk, input wire rst_n,
    input wire rx, output wire tx,
    input wire sample_done,
    input wire signed [31:0] theta_q16, dtheta_q16,
    input wire signed [31:0] alpha_q16, dalpha_q16,
    input wire signed [31:0] theta_ref_q16, dtheta_ref_q16,
    input wire signed [31:0] alpha_ref_q16, dalpha_ref_q16,
    output wire signed [17:0] gain_k1, gain_k2, gain_k3, gain_k4, gain_k5,
    output reg [7:0] last_command_seq,
    output reg [7:0] last_command_status
);
    localparam integer BIT_CLKS = CLK_HZ / BAUD;
    localparam [7:0] TYPE_TELEMETRY = 8'h01, TYPE_SET_GAINS = 8'h10, TYPE_ACK = 8'h81;

    function [15:0] crc_byte;
        input [15:0] crc_in; input [7:0] data;
        integer n; reg [15:0] c;
        begin
            c = crc_in ^ {data, 8'h00};
            for (n=0; n<8; n=n+1) c = c[15] ? ((c << 1) ^ 16'h1021) : (c << 1);
            crc_byte = c;
        end
    endfunction

    // UART RX, 8N1, center sampling.
    reg rx_meta, rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rx_meta <= 1'b1; rx_sync <= 1'b1; end
        else begin rx_meta <= rx; rx_sync <= rx_meta; end
    end
    localparam [1:0] R_IDLE=0, R_START=1, R_DATA=2, R_STOP=3;
    reg [1:0] r_state;
    reg [15:0] r_cnt;
    reg [2:0] r_bit;
    reg [7:0] r_shift, rx_byte;
    reg rx_valid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state<=R_IDLE; r_cnt<=0; r_bit<=0; r_shift<=0; rx_byte<=0; rx_valid<=0;
        end else begin
            rx_valid <= 1'b0;
            case (r_state)
                R_IDLE: if (!rx_sync) begin r_state<=R_START; r_cnt<=0; end
                R_START: if (r_cnt == (BIT_CLKS/2)-1) begin
                    r_cnt<=0; r_bit<=0; r_state<=rx_sync ? R_IDLE : R_DATA;
                end else r_cnt<=r_cnt+1'b1;
                R_DATA: if (r_cnt == BIT_CLKS-1) begin
                    r_cnt<=0; r_shift<={rx_sync,r_shift[7:1]};
                    if (r_bit==7) r_state<=R_STOP; else r_bit<=r_bit+1'b1;
                end else r_cnt<=r_cnt+1'b1;
                R_STOP: if (r_cnt == BIT_CLKS-1) begin
                    r_state<=R_IDLE;
                    if (rx_sync) begin rx_byte<=r_shift; rx_valid<=1'b1; end
                end else r_cnt<=r_cnt+1'b1;
            endcase
        end
    end

    // Parser buffers a complete 20-byte gain packet before validating or staging it.
    localparam [2:0] P_SYNC=0, P_TYPE=1, P_SEQ=2, P_LEN=3, P_PAYLOAD=4, P_CRC0=5, P_CRC1=6;
    reg [2:0] p_state;
    reg [7:0] p_type, p_seq, p_len, p_index;
    reg [7:0] p_data [0:19];
    reg [15:0] p_crc, p_crc_rx;
    reg [31:0] pending_word [0:4];
    reg pending_valid;
    reg pending_ok;
    reg [7:0] pending_seq;
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_state<=P_SYNC; p_type<=0; p_seq<=0; p_len<=0; p_index<=0;
            p_crc<=16'hffff; p_crc_rx<=0; pending_valid<=0; pending_ok<=0; pending_seq<=0;
            for (i=0;i<20;i=i+1) p_data[i]<=0;
            for (i=0;i<5;i=i+1) pending_word[i]<=0;
        end else begin
            if (sample_done && pending_valid) pending_valid<=1'b0;
            if (rx_valid) begin
                case (p_state)
                    P_SYNC: if (rx_byte==8'ha5) begin p_state<=P_TYPE; p_crc<=16'hffff; end
                    P_TYPE: begin p_type<=rx_byte; p_crc<=crc_byte(p_crc,rx_byte); p_state<=P_SEQ; end
                    P_SEQ: begin p_seq<=rx_byte; p_crc<=crc_byte(p_crc,rx_byte); p_state<=P_LEN; end
                    P_LEN: begin
                        p_len<=rx_byte; p_crc<=crc_byte(p_crc,rx_byte); p_index<=0;
                        if (p_type==TYPE_SET_GAINS && rx_byte==20) p_state<=P_PAYLOAD;
                        else begin
                            p_state<=P_SYNC;
                            if (p_type==TYPE_SET_GAINS) begin
                                pending_seq<=p_seq; pending_valid<=1'b1; pending_ok<=1'b0;
                            end
                        end
                    end
                    P_PAYLOAD: begin
                        p_data[p_index]<=rx_byte; p_crc<=crc_byte(p_crc,rx_byte);
                        if (p_index==p_len-1) p_state<=P_CRC0; else p_index<=p_index+1'b1;
                    end
                    P_CRC0: begin p_crc_rx[7:0]<=rx_byte; p_state<=P_CRC1; end
                    P_CRC1: begin
                        p_state<=P_SYNC;
                        if ({rx_byte,p_crc_rx[7:0]}==p_crc) begin
                            for (i=0;i<5;i=i+1)
                                pending_word[i] <= {p_data[i*4+3],p_data[i*4+2],p_data[i*4+1],p_data[i*4]};
                            pending_seq<=p_seq;
                            // Range check signed 18-bit Q10 sign-extension; invalid packet gets NAK.
                            if ((p_data[2][7:2]=={6{p_data[2][1]}}) && (p_data[3]=={8{p_data[2][1]}}) &&
                                (p_data[6][7:2]=={6{p_data[6][1]}}) && (p_data[7]=={8{p_data[6][1]}}) &&
                                (p_data[10][7:2]=={6{p_data[10][1]}}) && (p_data[11]=={8{p_data[10][1]}}) &&
                                (p_data[14][7:2]=={6{p_data[14][1]}}) && (p_data[15]=={8{p_data[14][1]}}) &&
                                (p_data[18][7:2]=={6{p_data[18][1]}}) && (p_data[19]=={8{p_data[18][1]}})) begin
                                pending_valid<=1'b1; pending_ok<=1'b1;
                            end else begin pending_valid<=1'b1; pending_ok<=1'b0; end
                        end else if (p_type==TYPE_SET_GAINS && p_len==20) begin
                            pending_seq<=p_seq; pending_valid<=1'b1; pending_ok<=1'b0;
                        end
                    end
                    default: p_state<=P_SYNC;
                endcase
            end
        end
    end

    // Active gains only change together, at the 1 kHz control sample boundary.
    reg ack_pending;
    reg [7:0] ack_seq, ack_status;
    reg signed [17:0] active_k1, active_k2, active_k3, active_k4, active_k5;
    assign gain_k1 = (sample_done && pending_valid && pending_ok) ? pending_word[0][17:0] : active_k1;
    assign gain_k2 = (sample_done && pending_valid && pending_ok) ? pending_word[1][17:0] : active_k2;
    assign gain_k3 = (sample_done && pending_valid && pending_ok) ? pending_word[2][17:0] : active_k3;
    assign gain_k4 = (sample_done && pending_valid && pending_ok) ? pending_word[3][17:0] : active_k4;
    assign gain_k5 = (sample_done && pending_valid && pending_ok) ? pending_word[4][17:0] : active_k5;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_k1<=-18'sd49277; active_k2<=-18'sd103; active_k3<=-18'sd600;
            active_k4<=18'sd0; active_k5<=-18'sd100;
            ack_pending<=0; ack_seq<=0; ack_status<=0;
            last_command_seq<=0; last_command_status<=0;
        end else if (sample_done && pending_valid) begin
            if (pending_ok) begin
                active_k1<=pending_word[0][17:0]; active_k2<=pending_word[1][17:0];
                active_k3<=pending_word[2][17:0]; active_k4<=pending_word[3][17:0]; active_k5<=pending_word[4][17:0];
                ack_status<=0; last_command_status<=0;
            end else begin
                ack_status<=1; last_command_status<=1;
            end
            ack_pending<=1; ack_seq<=pending_seq; last_command_seq<=pending_seq;
        end else if (ack_take) begin
            ack_pending<=0;
        end
    end

    // TX packet builder/serializer. Telemetry payload is eight signed Q16.16 words.
    reg [7:0] payload [0:31];
    reg [7:0] tx_type, tx_seq, tx_len, tx_idx;
    reg tx_packet_pending;
    wire ack_take = ack_pending && !tx_packet_pending && (tx_idx==0);
    reg [7:0] telemetry_div;
    reg [7:0] seq_counter;
    reg [15:0] tx_crc;
    reg [7:0] tx_byte_data;
    wire tx_byte_valid = tx_packet_pending;
    wire tx_byte_ready;
    uart_byte_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_tx(
        .clk(clk),.rst_n(rst_n),.data(tx_byte_data),.valid(tx_byte_valid),.ready(tx_byte_ready),.tx(tx)
    );
    always @(*) begin
        case (tx_idx)
            0: tx_byte_data=8'ha5;
            1: tx_byte_data=tx_type;
            2: tx_byte_data=tx_seq;
            3: tx_byte_data=tx_len;
            default: if (tx_idx < tx_len+4) tx_byte_data=payload[tx_idx-4];
                     else if (tx_idx==tx_len+4) tx_byte_data=tx_crc[7:0];
                     else tx_byte_data=tx_crc[15:8];
        endcase
    end
    integer j;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_type<=0; tx_seq<=0; tx_len<=0; tx_idx<=0; tx_packet_pending<=0;
            telemetry_div<=0; seq_counter<=0; tx_crc<=16'hffff;
            for (j=0;j<32;j=j+1) payload[j]<=0;
        end else begin
            if (sample_done) begin
                if (telemetry_div==9) begin
                    telemetry_div<=0;
                    if (!tx_packet_pending && !ack_pending && tx_idx==0) begin
                        tx_type<=TYPE_TELEMETRY; tx_seq<=seq_counter; seq_counter<=seq_counter+1'b1;
                        tx_len<=32; tx_idx<=0; tx_packet_pending<=1;
                        payload[0]<=theta_q16[7:0]; payload[1]<=theta_q16[15:8]; payload[2]<=theta_q16[23:16]; payload[3]<=theta_q16[31:24];
                        payload[4]<=dtheta_q16[7:0]; payload[5]<=dtheta_q16[15:8]; payload[6]<=dtheta_q16[23:16]; payload[7]<=dtheta_q16[31:24];
                        payload[8]<=alpha_q16[7:0]; payload[9]<=alpha_q16[15:8]; payload[10]<=alpha_q16[23:16]; payload[11]<=alpha_q16[31:24];
                        payload[12]<=dalpha_q16[7:0]; payload[13]<=dalpha_q16[15:8]; payload[14]<=dalpha_q16[23:16]; payload[15]<=dalpha_q16[31:24];
                        payload[16]<=theta_ref_q16[7:0]; payload[17]<=theta_ref_q16[15:8]; payload[18]<=theta_ref_q16[23:16]; payload[19]<=theta_ref_q16[31:24];
                        payload[20]<=dtheta_ref_q16[7:0]; payload[21]<=dtheta_ref_q16[15:8]; payload[22]<=dtheta_ref_q16[23:16]; payload[23]<=dtheta_ref_q16[31:24];
                        payload[24]<=alpha_ref_q16[7:0]; payload[25]<=alpha_ref_q16[15:8]; payload[26]<=alpha_ref_q16[23:16]; payload[27]<=alpha_ref_q16[31:24];
                        payload[28]<=dalpha_ref_q16[7:0]; payload[29]<=dalpha_ref_q16[15:8]; payload[30]<=dalpha_ref_q16[23:16]; payload[31]<=dalpha_ref_q16[31:24];
                    end
                end else telemetry_div<=telemetry_div+1'b1;
            end
            if (ack_take) begin
                tx_type<=TYPE_ACK; tx_seq<=ack_seq; tx_len<=1; tx_idx<=0; tx_packet_pending<=1;
                payload[0]<=ack_status;
            end
            if (tx_packet_pending && tx_byte_ready) begin
                case (tx_idx)
                    1: tx_crc<=crc_byte(16'hffff,tx_type);
                    2: tx_crc<=crc_byte(tx_crc,tx_seq);
                    3: tx_crc<=crc_byte(tx_crc,tx_len);
                    default: if (tx_idx < tx_len+4) begin
                        tx_crc<=crc_byte(tx_crc,payload[tx_idx-4]);
                    end
                endcase
                if (tx_idx==tx_len+5) begin tx_idx<=0; tx_packet_pending<=0; end
                else tx_idx<=tx_idx+1'b1;
            end
        end
    end
endmodule

module uart_byte_tx #(parameter integer CLK_HZ=50_000_000, BAUD=115200)(
    input wire clk, rst_n, input wire [7:0] data, input wire valid,
    output wire ready, output reg tx
);
    localparam integer BIT_CLKS=CLK_HZ/BAUD;
    reg busy; reg [15:0] cnt; reg [3:0] bit_idx; reg [9:0] frame;
    assign ready=~busy;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin busy<=0; cnt<=0; bit_idx<=0; frame<=10'h3ff; tx<=1; end
        else if (!busy) begin
            tx<=1;
            if (valid) begin frame<={1'b1,data,1'b0}; busy<=1; cnt<=0; bit_idx<=0; tx<=0; end
        end else if (cnt==BIT_CLKS-1) begin
            cnt<=0;
            if (bit_idx==9) begin busy<=0; tx<=1; end
            else begin bit_idx<=bit_idx+1'b1; tx<=frame[bit_idx+1]; end
        end else cnt<=cnt+1'b1;
    end
endmodule
