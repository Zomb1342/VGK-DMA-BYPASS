//
// PCILeech FPGA.
//
// PCIe configuration module - CFG handling for Artix-7.
//
// (c) Ulf Frisk, 2018-2024
// Author: Ulf Frisk, pcileech@frizk.net
//
`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_pcie_cfg_a7 #(
    parameter CLOCK_FREQ_MHZ = 62,                    // Clock frequency in MHz
    parameter STATIC_INT_PERIOD_US = 4000,            // Static interrupt period in microseconds
    parameter INT_MIN_INTERVAL_US = 100               // Minimum interval between dynamic interrupts
) (
    input                   rst,
    input                   clk_sys,
    input                   clk_pcie,
    IfPCIeFifoCfg.mp_pcie   dfifo,
    IfPCIeSignals.mpm       ctx,
    IfAXIS128.source        tlps_static,
    input                   i_valid,
    input [31:0]            i_addr,
    input [31:0]            i_data,
    input                   int_enable,
    input                   dynamic_int_mode,         // 1 = dynamic interrupts, 0 = fixed interval
    input                   event_trigger,            // Signal to trigger dynamic interrupt
    output [15:0]           pcie_id,
    output wire [31:0]      base_address_register
);

    // Parameters for interrupt timing
    localparam INT_STATIC_COUNT = (CLOCK_FREQ_MHZ * STATIC_INT_PERIOD_US);
    localparam INT_MIN_COUNT = (CLOCK_FREQ_MHZ * INT_MIN_INTERVAL_US);

    // Optimize counter for timing checks
    reg [1:0] tick_count = 0;
    always @ (posedge clk_pcie)
        tick_count <= tick_count + 1;
    
     // ----------------------------------------------------------------------------
    // Convert received CFG data from FT601 to PCIe clock domain
    // FIFO depth: 512 / 64-bits
    // ----------------------------------------------------------------------------
    reg             in_rden;
    wire [63:0]     in_dout;
    wire            in_empty;
    wire            in_valid;
    
    reg [63:0]      in_data64;
    wire [31:0]     in_data32   = in_data64[63:32];
    wire [15:0]     in_data16   = in_data64[31:16];
    wire [3:0]      in_type     = in_data64[15:12];
    
    // Interrupt control registers
    reg [7:0]  cfg_int_di;          // Interrupt data
    reg [4:0]  cfg_msg_num;         // MSI vector number
    reg        cfg_int_assert;      // Interrupt assert signal
    reg        cfg_int_valid;       // Interrupt valid signal
    reg        cfg_int_stat;        // Interrupt status
    wire       cfg_int_ready = ctx.cfg_interrupt_rdy;

        // BAR access detection signals
    reg bar_read_detected;
    reg bar_write_detected;
    wire bar_activity = bar_read_detected || bar_write_detected || event_trigger;  // Include external events
    

    
    // Interrupt counters
    reg [17:0] static_int_cnt = 0;  // Counter for fixed-interval "heartbeat" interrupts
    reg [17:0] dynamic_int_cnt = 0; // Counter for minimum spacing between dynamic interrupts
    reg        dynamic_int_pending = 0; // Flag for pending dynamic interrupt
    
    // Combined interrupt trigger
    wire static_int_trigger = (static_int_cnt == INT_STATIC_COUNT-1);
    wire dynamic_int_allowed = (dynamic_int_cnt >= INT_MIN_COUNT-1);
    wire o_int;

    fifo_64_64 i_fifo_pcie_cfg_tx(
        .rst            ( rst                   ),
        .wr_clk         ( clk_sys               ),
        .rd_clk         ( clk_pcie              ),
        .din            ( dfifo.tx_data         ),
        .wr_en          ( dfifo.tx_valid        ),
        .rd_en          ( in_rden               ),
        .dout           ( in_dout               ),
        .full           (                       ),
        .empty          ( in_empty              ),
        .valid          ( in_valid              )
    );

    
   // ------------------------------------------------------------------------
    // Convert received CFG from PCIe core and transmit onwards to FT601
    // FIFO depth: 512 / 64-bits.
    // ------------------------------------------------------------------------
    reg             out_wren;
    reg [31:0]      out_data;
    wire            pcie_cfg_rx_almost_full;
    
    fifo_32_32_clk2 i_fifo_pcie_cfg_rx(
        .rst            ( rst                   ),
        .wr_clk         ( clk_pcie              ),
        .rd_clk         ( clk_sys               ),
        .din            ( out_data              ),
        .wr_en          ( out_wren              ),
        .rd_en          ( dfifo.rx_rd_en        ),
        .dout           ( dfifo.rx_data         ),
        .full           (                       ),
        .almost_full    ( pcie_cfg_rx_almost_full ),
        .empty          (                       ),
        .valid          ( dfifo.rx_valid        )
    );
    
    // ------------------------------------------------------------------------
    // REGISTER FILE: COMMON
    // ------------------------------------------------------------------------
    
    wire    [383:0]     ro;
    reg     [703:0]     rw;
    
    // special non-user accessible registers 
    reg                 rwi_cfg_mgmt_rd_en;
    reg                 rwi_cfg_mgmt_wr_en;
    reg                 rwi_cfgrd_valid;
    reg     [9:0]       rwi_cfgrd_addr;
    reg     [3:0]       rwi_cfgrd_byte_en;
    reg     [31:0]      rwi_cfgrd_data;
    reg     [31:0]      rwi_count_cfgspace_status_cl;
    bit     [31:0]      base_address_register_reg;
    
    // Pipeline register for improved timing
    reg [31:0] base_address_register_pipe;
    always @(posedge clk_pcie)
        base_address_register_pipe <= base_address_register_reg;
    assign base_address_register = base_address_register_pipe;

    // BAR access detection
    always @(posedge clk_pcie) begin
        if (rst) begin
            bar_read_detected <= 1'b0;
            bar_write_detected <= 1'b0;
        end else begin
            // Detect BAR reads
            if (in_cmd_read && (base_address_register_reg == i_addr)) begin
                bar_read_detected <= 1'b1;
                cfg_int_di <= 8'h02;  // Code for BAR read
            end else begin
                bar_read_detected <= 1'b0;
            end
    
            // Detect BAR writes
            if (in_cmd_write && (base_address_register_reg == i_addr)) begin
                bar_write_detected <= 1'b1;
                cfg_int_di <= 8'h03;  // Code for BAR write
            end else begin
                bar_write_detected <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // Interrupt Control Logic
    // ------------------------------------------------------------------------
    
    // Static "heartbeat" interrupt counter
    always @(posedge clk_pcie) begin
        if (rst || !int_enable) begin
            static_int_cnt <= 0;
        end else if (static_int_cnt >= INT_STATIC_COUNT-1) begin
            static_int_cnt <= 0;
        end else begin
            static_int_cnt <= static_int_cnt + 1;
        end
    end

    // Dynamic interrupt spacing counter
    always @(posedge clk_pcie) begin
        if (rst || !int_enable) begin
            dynamic_int_cnt <= 0;
            dynamic_int_pending <= 0;
        end else begin
            if (dynamic_int_cnt < INT_MIN_COUNT-1) begin
                dynamic_int_cnt <= dynamic_int_cnt + 1;
            end
            
            if (event_trigger && dynamic_int_allowed) begin
                dynamic_int_pending <= 1'b1;
                dynamic_int_cnt <= 0;
            end else if (o_int) begin
                dynamic_int_pending <= 1'b0;
            end
        end
    end

    // Combined interrupt generation
    assign o_int = int_enable && ctx.cfg_interrupt_msienable && 
                  (static_int_trigger || (dynamic_int_pending && dynamic_int_allowed));

    // MSI interrupt control
always @(posedge clk_pcie) begin
    if (rst) begin
        cfg_int_valid <= 1'b0;
        cfg_msg_num <= 5'b0;
        cfg_int_assert <= 1'b0;
        cfg_int_di <= 8'b0;
        cfg_int_stat <= 1'b0;
    end else if (cfg_int_ready && cfg_int_valid) begin
        // Clear interrupt when acknowledged
        cfg_int_valid <= 1'b0;
        cfg_int_assert <= 1'b0;
        cfg_int_stat <= 1'b0;
    end else if (o_int) begin
        // Generate interrupt
        cfg_int_valid <= 1'b1;
        cfg_int_assert <= 1'b1;
        cfg_msg_num <= static_int_trigger ? 5'h0 : 5'h1; // Different vectors for static/dynamic
        
        // Only set interrupt data if it's not a BAR access
        if (!bar_activity) begin
            cfg_int_di <= static_int_trigger ? 8'h00 : 8'h01; // Different data for static/dynamic
        end
        // Note: BAR access interrupt data (8'h02 or 8'h03) is set in the BAR detection block
        
        cfg_int_stat <= 1'b1;
    end
end

   
     // ------------------------------------------------------------------------
    // REGISTER FILE: READ-ONLY LAYOUT/SPECIFICATION
    // ------------------------------------------------------------------------
     
    // MAGIC
    assign ro[15:0]     = 16'h2301;                     // +000: MAGIC
    // SPECIAL
    assign ro[16]       = ctx.cfg_mgmt_rd_en;           // +002: SPECIAL
    assign ro[17]       = ctx.cfg_mgmt_wr_en;           //
    assign ro[31:18]    = 0;                            //
    // SIZEOF / BYTECOUNT [little-endian]
    assign ro[63:32]    = $bits(ro) >> 3;               // +004: BYTECOUNT
    // PCIe CFG STATUS
    assign ro[71:64]    = ctx.cfg_bus_number;           // +008:
    assign ro[76:72]    = ctx.cfg_device_number;        //
    assign ro[79:77]    = ctx.cfg_function_number;      //
    // PCIe PL PHY
    assign ro[85:80]    = ctx.pl_ltssm_state;           // +00A
    assign ro[87:86]    = ctx.pl_rx_pm_state;           //
    assign ro[90:88]    = ctx.pl_tx_pm_state;           // +00B
    assign ro[93:91]    = ctx.pl_initial_link_width;    //
    assign ro[95:94]    = ctx.pl_lane_reversal_mode;    //
    assign ro[97:96]    = ctx.pl_sel_lnk_width;         // +00C
    assign ro[98]       = ctx.pl_phy_lnk_up;            //
    assign ro[99]       = ctx.pl_link_gen2_cap;         //
    assign ro[100]      = ctx.pl_link_partner_gen2_supported; //
    assign ro[101]      = ctx.pl_link_upcfg_cap;        //
    assign ro[102]      = ctx.pl_sel_lnk_rate;          //
    assign ro[103]      = ctx.pl_directed_change_done;  // +00D:
    assign ro[104]      = ctx.pl_received_hot_rst;      //
    assign ro[126:105]  = 0;                            //       SLACK
    assign ro[127]      = ctx.cfg_mgmt_rd_wr_done;      //
    // PCIe CFG MGMT
    assign ro[159:128]  = ctx.cfg_mgmt_do;              // +010:
    // PCIe CFG STATUS
    assign ro[175:160]  = ctx.cfg_command;              // +014:
    assign ro[176]      = ctx.cfg_aer_rooterr_corr_err_received;            // +016:
    assign ro[177]      = ctx.cfg_aer_rooterr_corr_err_reporting_en;        //
    assign ro[178]      = ctx.cfg_aer_rooterr_fatal_err_received;           //
    assign ro[179]      = ctx.cfg_aer_rooterr_fatal_err_reporting_en;       //
    assign ro[180]      = ctx.cfg_aer_rooterr_non_fatal_err_received;       //
    assign ro[181]      = ctx.cfg_aer_rooterr_non_fatal_err_reporting_en;   //
    assign ro[182]      = ctx.cfg_bridge_serr_en;                           //
    assign ro[183]      = ctx.cfg_received_func_lvl_rst;                    //
    assign ro[186:184]  = ctx.cfg_pcie_link_state;      // +017:
    assign ro[187]      = ctx.cfg_pmcsr_pme_en;         //
    assign ro[189:188]  = ctx.cfg_pmcsr_powerstate;     //
    assign ro[190]      = ctx.cfg_pmcsr_pme_status;     //
    assign ro[191]      = 0;                            //       SLACK
    assign ro[207:192]  = ctx.cfg_dcommand;             // +018:
    assign ro[223:208]  = ctx.cfg_dcommand2;            // +01A:
    assign ro[239:224]  = ctx.cfg_dstatus;              // +01C:
    assign ro[255:240]  = ctx.cfg_lcommand;             // +01E:
    assign ro[271:256]  = ctx.cfg_lstatus;              // +020:
    assign ro[287:272]  = ctx.cfg_status;               // +022:
    assign ro[293:288]  = ctx.tx_buf_av;                // +024:
    assign ro[294]      = ctx.tx_cfg_req;               //
    assign ro[295]      = ctx.tx_err_drop;              //
    assign ro[302:296]  = ctx.cfg_vc_tcvc_map;          // +025:
    assign ro[303]      = 0;                            //       SLACK
    assign ro[304]      = ctx.cfg_root_control_pme_int_en;              // +026:
    assign ro[305]      = ctx.cfg_root_control_syserr_corr_err_en;      //
    assign ro[306]      = ctx.cfg_root_control_syserr_fatal_err_en;     //
    assign ro[307]      = ctx.cfg_root_control_syserr_non_fatal_err_en; //
    assign ro[308]      = ctx.cfg_slot_control_electromech_il_ctl_pulse;//
    assign ro[309]      = ctx.cfg_to_turnoff;                           //
    assign ro[319:310]  = 0;                                            //       SLACK
    // PCIe INTERRUPT
    assign ro[327:320]  = ctx.cfg_interrupt_do;         // +028:
    assign ro[330:328]  = ctx.cfg_interrupt_mmenable;   // +029:
    assign ro[331]      = ctx.cfg_interrupt_msienable;  //
    assign ro[332]      = ctx.cfg_interrupt_msixenable; //
    assign ro[333]      = ctx.cfg_interrupt_msixfm;     //
    assign ro[334]      = ctx.cfg_interrupt_rdy;        //
    assign ro[335]      = 0;                            //       SLACK
    // CFG SPACE READ RESULT
    assign ro[345:336]  = rwi_cfgrd_addr;               // +02A:
    assign ro[346]      = 0;                            //       SLACK
    assign ro[347]      = rwi_cfgrd_valid;              //
    assign ro[351:348]  = rwi_cfgrd_byte_en;            //
    assign ro[383:352]  = rwi_cfgrd_data;               // +02C:
    
    
    // ------------------------------------------------------------------------
    // INITIALIZATION/RESET BLOCK _AND_
    // REGISTER FILE: READ-WRITE LAYOUT/SPECIFICATION
    // ------------------------------------------------------------------------
    
    localparam integer  RWPOS_CFG_RD_EN                 = 16;
    localparam integer  RWPOS_CFG_WR_EN                 = 17;
    localparam integer  RWPOS_CFG_WAIT_COMPLETE         = 18;
    localparam integer  RWPOS_CFG_CFGSPACE_STATUS_CL_EN = 20;
    localparam integer  RWPOS_CFG_CFGSPACE_COMMAND_EN   = 21;
    
    task pcileech_pcie_cfg_a7_initialvalues;        // task is non automatic
        begin
            out_wren <= 1'b0;
            rwi_cfg_mgmt_rd_en <= 1'b0;
            rwi_cfg_mgmt_wr_en <= 1'b0;
            base_address_register_reg <= 32'h00000000;
    
            // MAGIC
            rw[15:0]    <= 16'h6745;                // +000:
            // SPECIAL START TASK BLOCK (write 1 to start action)
            rw[16]      <= 0;                       // +002: CFG RD EN
            rw[17]      <= 0;                       //       CFG WR EN
            rw[18]      <= 0;                       //       WAIT FOR PCIe CFG SPACE RD/WR COMPLETION
            rw[20]      <= 1;                       //       CFGSPACE_STATUS_REGISTER_AUTO_CLEAR
            rw[21]      <= 0;                       //       CFGSPACE_COMMAND_REGISTER_AUTO_SET
            rw[31:22]   <= 0;                       //       RESERVED FUTURE
            
            // SIZEOF / BYTECOUNT [little-endian]
            rw[63:32]   <= $bits(rw) >> 3;          // +004: bytecount [little endian]
            // DSN
            rw[127:64]  <= 64'h001000203EF540;    // +008: cfg_dsn
            // PCIe CFG MGMT
            rw[159:128] <= 0;                       // +010: cfg_mgmt_di
            rw[169:160] <= 0;                       // +014: cfg_mgmt_dwaddr
            rw[170]     <= 0;                       //       cfg_mgmt_wr_readonly
            rw[171]     <= 0;                       //       cfg_mgmt_wr_rw1c_as_rw
            rw[175:172] <= 4'hf;                    //       cfg_mgmt_byte_en
            
            // PCIe PL PHY
            rw[176]     <= 0;                       // +016: pl_directed_link_auton
            rw[178:177] <= 0;                       //       pl_directed_link_change
            rw[179]     <= 1;                       //       pl_directed_link_speed 
            rw[181:180] <= 0;                       //       pl_directed_link_width            
            rw[182]     <= 1;                       //       pl_upstream_prefer_deemph
            rw[183]     <= 0;                       //       pl_transmit_hot_rst
            rw[184]     <= 0;                       // +017: pl_downstream_deemph_source
            rw[191:185] <= 0;                       //       SLACK  
            
            // PCIe CTRL
            rw[209:208] <= 0;                       // +01A: cfg_pm_force_state
            rw[210]     <= 0;                       //       cfg_pm_force_state_en
            rw[211]     <= 0;                       //       cfg_pm_halt_aspm_l0s
            rw[212]     <= 0;                       //       cfg_pm_halt_aspm_l1
            rw[213]     <= 0;                       //       cfg_pm_send_pme_to
            rw[214]     <= 0;                       //       cfg_pm_wake
            rw[215]     <= 0;                       //       cfg_trn_pending
            rw[216]     <= 0;                       // +01B: cfg_turnoff_ok
            rw[217]     <= 1;                       //       rx_np_ok
            rw[218]     <= 1;                       //       rx_np_req
            rw[219]     <= 1;                       //       tx_cfg_gnt
            rw[223:220] <= 0;                       //       SLACK 
            
            // PCIe STATUS register clear timer
            rw[672+:32] <= 62500;                   // +054: CFGSPACE_STATUS_CLEAR TIMER
        end
    endtask
    
    // PCIe Interface Signal Assignments
    assign ctx.cfg_mgmt_rd_en               = rwi_cfg_mgmt_rd_en & ~ctx.cfg_mgmt_rd_wr_done;
    assign ctx.cfg_mgmt_wr_en               = rwi_cfg_mgmt_wr_en & ~ctx.cfg_mgmt_rd_wr_done;
    assign ctx.cfg_dsn                      = rw[127:64];
    assign ctx.cfg_mgmt_di                  = rw[159:128];
    assign ctx.cfg_mgmt_dwaddr              = rw[169:160];
    assign ctx.cfg_mgmt_wr_readonly         = rw[170];
    assign ctx.cfg_mgmt_wr_rw1c_as_rw       = rw[171];
    assign ctx.cfg_mgmt_byte_en             = rw[175:172];
    
    // PCIe Physical Layer Assignments
    assign ctx.pl_directed_link_auton       = rw[176];
    assign ctx.pl_directed_link_change      = rw[178:177];
    assign ctx.pl_directed_link_speed       = rw[179];
    assign ctx.pl_directed_link_width       = rw[181:180];
    assign ctx.pl_upstream_prefer_deemph    = rw[182];
    assign ctx.pl_transmit_hot_rst          = rw[183];
    assign ctx.pl_downstream_deemph_source  = rw[184];
    
    // Interrupt Interface Assignments
    assign ctx.cfg_interrupt_di             = cfg_int_di;
    assign ctx.cfg_pciecap_interrupt_msgnum = cfg_msg_num;
    assign ctx.cfg_interrupt_assert         = cfg_int_assert;
    assign ctx.cfg_interrupt                = cfg_int_valid;
    assign ctx.cfg_interrupt_stat           = cfg_int_stat;
    
    // Power Management Assignments
    assign ctx.cfg_pm_force_state           = rw[209:208];
    assign ctx.cfg_pm_force_state_en        = rw[210];
    assign ctx.cfg_pm_halt_aspm_l0s         = rw[211];
    assign ctx.cfg_pm_halt_aspm_l1          = rw[212];
    assign ctx.cfg_pm_send_pme_to           = rw[213];
    assign ctx.cfg_pm_wake                  = rw[214];
    assign ctx.cfg_trn_pending              = rw[215];
    assign ctx.cfg_turnoff_ok               = rw[216];
    assign ctx.rx_np_ok                     = rw[217];
    assign ctx.rx_np_req                    = rw[218];
    assign ctx.tx_cfg_gnt                   = rw[219];
    
    assign pcie_id                          = ro[79:64];

    // TLP Interface Assignments
    assign tlps_static.tdata = 128'h0;
    assign tlps_static.tkeepdw = 4'h0;
    assign tlps_static.tlast = 1'b0;
    assign tlps_static.tuser = 1'b0;
    assign tlps_static.tvalid = 1'b0;
    assign tlps_static.has_data = 1'b0;
    // ------------------------------------------------------------------------
    // Command and Status Register Handling
    // ------------------------------------------------------------------------
    
    wire [15:0] in_cmd_address_byte = in_dout[31:16];
    wire [17:0] in_cmd_address_bit  = {in_cmd_address_byte[14:0], 3'b000};
    wire [15:0] in_cmd_value        = {in_dout[48+:8], in_dout[56+:8]};
    wire [15:0] in_cmd_mask         = {in_dout[32+:8], in_dout[40+:8]};
    wire        f_rw                = in_cmd_address_byte[15]; 
    wire [15:0] in_cmd_data_in      = (in_cmd_address_bit < (f_rw ? $bits(rw) : $bits(ro))) ? 
                                     (f_rw ? rw[in_cmd_address_bit+:16] : ro[in_cmd_address_bit+:16]) : 16'h0000;
    wire        in_cmd_read         = in_dout[12] & in_valid;
    wire        in_cmd_write        = in_dout[13] & in_cmd_address_byte[15] & in_valid;
    wire        pcie_cfg_rw_en      = rwi_cfg_mgmt_rd_en | rwi_cfg_mgmt_wr_en | 
                                     rw[RWPOS_CFG_RD_EN] | rw[RWPOS_CFG_WR_EN];
    
    // FIFO read enable control
    assign in_rden = tick_count[1] & ~pcie_cfg_rx_almost_full & 
                    (~rw[RWPOS_CFG_WAIT_COMPLETE] | ~pcie_cfg_rw_en);
    
    // ------------------------------------------------------------------------
    // Main Control Logic
    // ------------------------------------------------------------------------
    
    initial pcileech_pcie_cfg_a7_initialvalues();
    
    always @(posedge clk_pcie) begin
        if (rst) begin
            pcileech_pcie_cfg_a7_initialvalues();
        end else begin
            // READ config
            out_wren <= in_cmd_read;
            if (in_cmd_read) begin
                out_data[31:16] <= in_cmd_address_byte;
                out_data[15:0]  <= {in_cmd_data_in[7:0], in_cmd_data_in[15:8]};
            end

            // WRITE config
            if (in_cmd_write) begin
                for (integer i_write = 0; i_write < 16; i_write = i_write + 1) begin
                    if (in_cmd_mask[i_write])
                        rw[in_cmd_address_bit+i_write] <= in_cmd_value[i_write];
                end
            end

            // STATUS REGISTER CLEAR
            if ((rw[RWPOS_CFG_CFGSPACE_STATUS_CL_EN] | rw[RWPOS_CFG_CFGSPACE_COMMAND_EN]) & 
                ~in_cmd_read & ~in_cmd_write & ~rw[RWPOS_CFG_RD_EN] & ~rw[RWPOS_CFG_WR_EN] & 
                ~rwi_cfg_mgmt_rd_en & ~rwi_cfg_mgmt_wr_en) begin
                
                if (rwi_count_cfgspace_status_cl < rw[672+:32])
                    rwi_count_cfgspace_status_cl <= rwi_count_cfgspace_status_cl + 1;
                else begin
                    rwi_count_cfgspace_status_cl <= 0;
                    rw[RWPOS_CFG_WR_EN] <= 1'b1;
                    rw[143:128] <= 16'h0007;
                    rw[159:144] <= 16'hff00;
                    rw[169:160] <= 1;
                    rw[170]     <= 0;
                    rw[171]     <= 0;
                    rw[172]     <= rw[RWPOS_CFG_CFGSPACE_COMMAND_EN];
                    rw[173]     <= rw[RWPOS_CFG_CFGSPACE_COMMAND_EN];
                    rw[174]     <= 0;
                    rw[175]     <= rw[RWPOS_CFG_CFGSPACE_STATUS_CL_EN];
                end
            end

            // BAR Configuration
            if ((base_address_register_reg == 32'h00000000) || 
                (base_address_register_reg == 32'hFFFFC004) ||
                (base_address_register_reg == 32'h00000004)) begin
                
                if (~in_cmd_read & ~in_cmd_write & ~rw[RWPOS_CFG_RD_EN] & 
                    ~rw[RWPOS_CFG_WR_EN] & ~rwi_cfg_mgmt_rd_en & ~rwi_cfg_mgmt_wr_en) begin
                    rw[RWPOS_CFG_RD_EN] <= 1'b1;
                    rw[169:160] <= 6;
                    rw[175:172] <= 4'h0;
                end
            end

            // CONFIG SPACE READ/WRITE                        
            if (ctx.cfg_mgmt_rd_wr_done) begin
                if ((base_address_register_reg == 32'h00000000) || 
                    (base_address_register_reg == 32'hFFFFC004) ||
                    (base_address_register_reg == 32'h00000004)) begin
                    if ((ctx.cfg_mgmt_dwaddr == 8'h06) & rwi_cfg_mgmt_rd_en)
                        base_address_register_reg <= ctx.cfg_mgmt_do;
                end

                rwi_cfg_mgmt_rd_en  <= 1'b0;
                rwi_cfg_mgmt_wr_en  <= 1'b0;
                rwi_cfgrd_valid     <= 1'b1;
                rwi_cfgrd_addr      <= ctx.cfg_mgmt_dwaddr;
                rwi_cfgrd_data      <= ctx.cfg_mgmt_do;
                rwi_cfgrd_byte_en   <= ctx.cfg_mgmt_byte_en;
            end else if (rw[RWPOS_CFG_RD_EN]) begin
                rw[RWPOS_CFG_RD_EN] <= 1'b0;
                rwi_cfg_mgmt_rd_en  <= 1'b1;
                rwi_cfgrd_valid     <= 1'b0;
            end else if (rw[RWPOS_CFG_WR_EN]) begin
                rw[RWPOS_CFG_WR_EN] <= 1'b0;
                rwi_cfg_mgmt_wr_en  <= 1'b1;
                rwi_cfgrd_valid     <= 1'b0;
            end
        end
    end

endmodule
