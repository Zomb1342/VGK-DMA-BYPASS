`timescale 1ns / 1ps

module wifi_event_simulator (
    input wire clk,
    input wire rst,
    input wire [31:0] control_register,
    
    output reg packet_received,
    output reg tx_completed,
    output reg beacon_received,
    output reg [7:0] signal_strength,
    output reg link_status_changed,
    output reg [31:0] event_status
);
    // Counters for periodic events
    reg [23:0] beacon_counter = 0;
    reg [19:0] traffic_counter = 0;
    reg link_status = 0;
    
    // Control register bit definitions
    wire sim_enable = control_register[0];
    wire force_rx_packet = control_register[8];
    wire force_tx_complete = control_register[9];
    wire force_link_change = control_register[10];
    wire [7:0] set_signal_strength = control_register[23:16];
    
    always @(posedge clk) begin
        if (rst) begin
            packet_received <= 0;
            tx_completed <= 0;
            beacon_received <= 0;
            signal_strength <= 8'h50;  // Default signal strength
            link_status_changed <= 0;
            event_status <= 0;
            beacon_counter <= 0;
            traffic_counter <= 0;
            link_status <= 0;
        end else if (sim_enable) begin
            // Clear one-shot events
            packet_received <= 0;
            tx_completed <= 0;
            beacon_received <= 0;
            link_status_changed <= 0;
            
            // Handle manual triggers from control register
            if (force_rx_packet)
                packet_received <= 1;
                
            if (force_tx_complete)
                tx_completed <= 1;
                
            if (force_link_change) begin
                link_status <= ~link_status;
                link_status_changed <= 1;
            end
            
            // Update signal strength from control register
            if (|set_signal_strength)
                signal_strength <= set_signal_strength;
                
            // Periodic beacon simulation (roughly every 100ms @ 62.5MHz clock)
            if (beacon_counter >= 24'd6_250_000) begin
                beacon_counter <= 0;
                beacon_received <= 1;
            end else
                beacon_counter <= beacon_counter + 1;
                
            // Simulate random background traffic
            if (traffic_counter == 20'hFFFFF) begin
                traffic_counter <= 0;
                if (link_status) begin  // Only generate traffic if link is up
                    packet_received <= 1;
                    // Randomly generate TX completions
                    tx_completed <= traffic_counter[5];
                end
            end else
                traffic_counter <= traffic_counter + 1;
                
            // Update composite status register
            event_status <= {
                16'h0000,                // Reserved
                link_status,             // Current link status
                3'b000,                 // Reserved
                beacon_received,         // Beacon received
                tx_completed,           // TX completed
                packet_received         // Packet received
            };
        end
    end
endmodule
