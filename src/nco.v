//////////////////////////////////////////////////////////////////////
// @BRIEF: nco.v
//////////////////////////////////////////////////////////////////////

`default_nettype none

// Timescale / Precision	
`timescale 1 ns / 1 ps 

//////////////////////////////////////////////////////////////////////
// @DEFINE: Defines
//////////////////////////////////////////////////////////////////////

// Uncomment this when driving a commmon anode LED parallel chain
`define SIN_OUT_ACTIVE_LOW           /* important: no ';' here */

module nco
(
	clk_in,						// External to FPGA Clock Source that feeds internal PLL
	reset_n,
	sample_clk_ce_n,
	sin_out,
	brd_led,
	high_freq_en_n,
	from_pc, 					// uart_xcvr submodule
    to_ir,                      // uart_xcvr submodule
    ir_sd,                      // uart_xcvr submodule
    i_serial_data,              // uart_xcvr submodule
    o_serial_data,              // uart_xcvr submodule
    test1,                      // uart_xcvr submodule
    test2,                      // uart_xcvr submodule
    test3                       // uart_xcvr submodule
);

//////////////////////////////////////////////////////////////////////
// @LOCALPARAMS: Local Parameters
//////////////////////////////////////////////////////////////////////

localparam SIN_OUT_WIDTH = 8;	 	// Sine wave output bit width
localparam SIN_OUT_ZERO = {SIN_OUT_WIDTH{1'b0}};
localparam SIN_LUT_DEPTH = 256; 

localparam PHASE_WIDTH = 32;   	    // Width of phase accumulator
localparam PHASE_ZERO = {PHASE_WIDTH{1'b0}};
localparam PHASE_P_WIDTH = 8;		// How many of the MSBs of phase is used in LUT phase input	( Note: (2^PHASE_P_WIDTH) - 1 = SINE_LUT_DEPTH )

//localparam FREQ_STEP = 32'd357913941; // fout = 1 MHZ, fs = 12 MHz, N = 32
//localparam FREQ_STEP = 32'd112286727; // fout = 1 MHZ, fs = 38.25 MHz (PLL out), N = 32 => delta_f = (2^N)*fout/fs
localparam FREQ_STEP = 32'd112286727; 		// fout = 1 Hz, fs = 38.25 MHz (PLL out), N = 32 => delta_f = (2^N)*fout/fs

//localparam HIGH_FREQ_STEP = 32'd2021161080; // fout = 18 MHz, fs = 38.25 MHz (PLL out), N = 32 => delta_f = (2^N)*fout/fs
localparam HIGH_FREQ_STEP = 32'd2021161080; // fout = 1 MHz, fs = 38.25 MHz (PLL out), N = 32 => delta_f = (2^N)*fout/fs

localparam NUM_BRD_LEDS = 5;					// Number of iCEStick on-board LEDs
localparam ALL_LED_OFF = {NUM_BRD_LEDS{1'b0}};	// Active-high
localparam ALL_LED_ON  = {NUM_BRD_LEDS{1'b1}};	// Active-high

localparam UART_BYTE_WIDTH = 8;

//////////////////////////////////////////////////////////////////////
// @IN: Input Port(s)
//////////////////////////////////////////////////////////////////////

input clk_in;
input reset_n;
input sample_clk_ce_n;
input high_freq_en_n;
input from_pc;			// Connected to TX of external FTDI USB
input i_serial_data;

//////////////////////////////////////////////////////////////////////
// @OUTPUT: Output Port(s)
//////////////////////////////////////////////////////////////////////

output [SIN_OUT_WIDTH-1:0] sin_out;
output [NUM_BRD_LEDS-1:0] brd_led;
output to_ir;			// To Pin 3 (TXD) of U4 (TFDU4101-TR3, IR XCVR)
output ir_sd;
output o_serial_data;
output test1;			// Used for debug
output test2;			// Used for debug
output test3;			// Used for debug

//////////////////////////////////////////////////////////////////////
// @PORT_WIRES: Port Wires
//////////////////////////////////////////////////////////////////////

wire clk_in;
wire reset_n;
wire sample_clk_ce_n;
wire [SIN_OUT_WIDTH-1:0] sin_out;
wire [NUM_BRD_LEDS-1:0] brd_led;
wire high_freq_en_n;
wire from_pc;
wire to_ir;
wire ir_sd;
wire i_serial_data;
wire o_serial_data;
wire test1;
wire test2;
wire test3;

//////////////////////////////////////////////////////////////////////
// @LOCALVAR: Local Variables
//////////////////////////////////////////////////////////////////////

wire clk_pll_out;

reg [UART_BYTE_WIDTH-1:0]o_rx_data_reg;		// TEST
wire [UART_BYTE_WIDTH-1:0]o_rx_data;		// TEST
//assign o_rx_data = o_rx_data_reg;			// TEST
wire o_rx_data_ready;						// TEST
//reg[UART_BYTE_WIDTH-1:0] rx_data_buf_reg;

reg [SIN_OUT_WIDTH-1:0]	sin_lut[0:SIN_LUT_DEPTH-1];	// Sine wave lookup table

// Sin wave output register
reg [SIN_OUT_WIDTH-1:0] sin_out_reg;

// Phase Accumulator
reg [PHASE_WIDTH-1:0] phase;

// Board LED Output Register
reg [NUM_BRD_LEDS-1:0] brd_led_reg;

// Connect sin wave output to register
//assign sin_out = sin_out_reg;	// DEFAULT

`ifdef SIN_OUT_ACTIVE_LOW

assign sin_out = ~o_rx_data_reg;	// DEBUG UART

`else

assign sin_out = o_rx_data_reg;	// DEBUG UART

`endif // `ifdef SIN_OUT_ACTIVE_LOW

// Connect board LED output to register
assign brd_led = brd_led_reg;

//////////////////////////////////////////////////////////////////////
// @INITIAL: Inital Block
//////////////////////////////////////////////////////////////////////
initial begin
	
	// NOTE: Target file must be located in PROJECT ROOT of IceCube2 project in order
	// to be found by $readmemh
	
	$readmemh("my_sin_lut_tbl.ini", sin_lut);
	
end	// initial begin 

// PLL instantiation
// NOTE: 12 MHz oscillator output is being fed to FPGA via Global Buffer GBUF6 (Pin 21, IO Bank 3)
// NOTE: Global Buffers are high-drive, low-skew lines designed primarily for clock distribution or other
// high fan-out signals such as set/reset lines.

wire PLLOUTGLOBAL;	// TEST

ice_pll ice_pll_inst(
     .REFERENCECLK ( clk_in        ),  // input 12 MHz
     .PLLOUTCORE   ( clk_pll_out   ),  // output 38 MHz ( targeted ), 38.25 MHz ( actual )
     .PLLOUTGLOBAL ( PLLOUTGLOBAL  ),
     .RESET        ( 1'b1  )		   // Active-low RESET, tie HIGH to allow PLL operation
     );
	 
// UART Transceiver instantiation
uart_xcvr uart_xcvr_inst( 
        .clk_in			( clk_in ),			// Input 12 MHz (from external 12 MHz XOSC)
		.CLKOP 			( clk_pll_out ),	// Input 38 MHz (from output of internal PLL)
        .from_pc       	( from_pc ), 		// To Pin 38 (BDBUS0) of  U1 ( FT2232, FTDI FTDI USB )
        .to_ir			( to_ir ),			// To Pin 3 (TXD) of U4 (TFDU4101-TR3, IR XCVR)
        .sd             ( ir_sd ),			// To Pin 5 (SD) of U4 (TFDU4101-TR3, IR XCVR)
        .o_serial_data  ( o_serial_data ),	// To Pin 39 (BDBUS1) of  U1 ( FT2232, FTDI FTDI USB )	
        .test1			( test1 ),			// For debug, internaly connected to: to_ir
        .test2			( test2 ),			// For debug, internaly connected to: from_pc
        .test3          ( test3 ),			// For debug, internaly connected to: i_rst
		.o_rx_data		( o_rx_data ),		  // TEST
		.o_rx_data_ready ( o_rx_data_ready ) // TEST
		 );

// BEGIN TEST
//always @( posedge o_rx_data_ready ) begin : RxUartDataReady
always @( posedge clk_in ) begin : RxUartDataReady

	o_rx_data_reg <= o_rx_data;

end // begin : RxUartDataReady
// END TEST

// Falling Edge Reset
always @(negedge reset_n) begin : ResetAssert

	brd_led_reg ^= ALL_LED_ON;

end // begin : ResetAssert
	 
always @(posedge clk_pll_out) begin : SinoutDriver

    if( reset_n == 1'b0 )	// Active-low
		sin_out_reg <= SIN_OUT_ZERO;
		
	if (sample_clk_ce_n == 1'b0) begin	// Active-low
	
		`ifdef SIN_OUT_ACTIVE_LOW
		 
		 // Used for driving common-anode LEDs
		 sin_out_reg <= ~sin_lut[phase[PHASE_WIDTH-1:PHASE_WIDTH-PHASE_P_WIDTH]];	// e.g. sinewave <= sine_lut[phase[31:24]] 
		 
		`else
		
		 // Used for driving R2R network
		 sin_out_reg <= sin_lut[phase[PHASE_WIDTH-1:PHASE_WIDTH-PHASE_P_WIDTH]];	// e.g. sinewave <= sine_lut[phase[31:24]] 
		 
		`endif // `ifdef SIN_OUT_ACTIVE_LOW
		
    end // if (sample_clk_ce_n == 0) begin	
		
end //  always : SinoutDriver


// Phase Accumulator
always @(posedge clk_pll_out) begin : PhaseAccumulator

	if( reset_n == 1'b0 )	// Active-low
		phase <= PHASE_ZERO;
		
    // Allow for an D/A running at a lower speed from your FPGA
    // E.g. D/A controls this signal to signify when it is ready for next LUT entry
	if (sample_clk_ce_n == 1'b0) begin // Active-low
	
		if( high_freq_en_n == 1'b0 )
			phase <= phase + HIGH_FREQ_STEP;
		else
			phase <= phase + FREQ_STEP;
    end
		
end // always : PhaseAccumulator



endmodule