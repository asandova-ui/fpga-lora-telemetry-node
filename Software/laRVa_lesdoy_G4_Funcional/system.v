//--------------------------------------------------------------------
// RISC-V things 
// by Jesús Arias (2022)
//--------------------------------------------------------------------
/*
	Description:
	A LaRVA RISC-V system with 16KB of internal memory
	
	Memory map:
	
	0x00000000 to 0x00003FFF	Internal RAM (with inital contents)
	0x00004000 to 0x1FFFFFFF	the same internal RAM repeated each 16KB
	0x20000000 to 0xDFFFFFFF       xxxx
	0xE0000000 to 0xE00000FF    IO registers
	0xE0000100 to 0xFFFFFFFF    the same IO registers repeated each 256B

	IO register map (all registers accessed as 32-bit words):
	
   address  |      WRITE        |      READ 
    -----------|-------------------|---------------
    0xE0000080 | UART0 TX data     | UART0 RX data
    0xE0000084 | UART0 Baud Divider| UART0 flags
			   |                   |
	0xE0000090 | UART1 TX data     | UART1 RX data
	0xE0000094 | UART1 Baud Divider| UART1 flags
			   |                   |
	0xE00000A0 | UART2 TX data     | UART2 RX data
	0xE00000A4 | UART2 Baud Divider| UART2 flags
			   |                   |
    0xE0000070 | SPI0 TX data      | SPI0 RX data
    0xE0000074 | SPI0 Control      | SPI0 flags
    0xE0000078 | SPI0 Slave Select | xxxx
			   |                   |
    0xE0000060 | SPI1 TX data      | SPI1 RX data
    0xE0000064 | SPI1 Control      | SPI1 flags
    0xE0000068 | SPI1 Slave Select | xxxx
			   |                   |
    0xE0000040 | MAX_COUNT         | TIMER
			   |                   |
    0xE0000030 | GPOUT             | GPOUT
    0xE0000034 | GPOUT             | GPIN
			   |                   |
    0xE0000020 | Interrupt Enable  | Interrupt enable
			   |                   |
    0xE0000000 | IRQ vector 0 Trap |
    0xE0000004 | IRQ vector 1 Timer|
    0xE0000008 | IRQ vector 2 RX0  |
    0xE000000C | IRQ vector 3 TX0  |
    0xE0000010 | IRQ vector 4 RX1  |
    0xE0000014 | IRQ vector 5 TX1  |
    0xE0000018 | IRQ vector 6 RX2  |
    0xE000001C | IRQ vector 7 TX2  |

    UART Baud Divider: Baud = Fcclk / (DIVIDER+1) , with DIVIDER >=7
    
    UART FLAGS:    bits 31-5  bit 4  bit 3 bit 2 bit 1 bit 0
                     xxxx      OVE    FE    TEND  THRE   DV
        DV:   Data Valid (RX complete if 1. Cleared reading data register)
        THRE: TX Holding register empty (ready to write to data register if 1)
        TEND: TX end (holding reg and shift reg both empty if 1)
        FE:   Frame Error (Stop bit received as 0 if FE=1)
        OVE:  Overrun Error (Character received when DV was still 1)
        (DV and THRE can request interrupts when 1)
	
	------ 

    SPI Control:   bits 31-14  bits 13-8  bits 7-0 
                      xxxx        DLEN     DIVIDER 

        DLEN:    Data lenght (8 to 32 bits) 
        DIVIDER: SCK frequency = Fclk / (2*(DIVIDER+1)) 

    SPI Flags:     bits 31-1  bit 0
                      xxxx     BUSY 

        BUSY:  SPI exchanging data when 1 

    SPI Slave Select: bits 31-2  bit 1   bit 0 
                         xxxx     ss1     ss0 

        ss0 : Selects the SPI slave 0 when 0 (active low) 
        ss1 : Selects the SPI slave 1 when 0 (active low) 

    ------ 
    I2C Data/Control: bit 10  bit 9  bit 8  bits 7-0 
                      STOP   START   ACK     DATA 
					  
        STOP:  Send Stop sequence 
        START: Send Start sequence 
        ACK:   ACK bit. Must be 1 on writes and 0 on reads except last one 
        DATA:  Data to write (Must be 0xFF on reads) 
          - ACK and DATA are ignored if START or STOP are one 
          - Do not set START and STOP simultaneously 
          - Repeated START is not supported 
          - Writing to this register sets the BUSY flag until the start, 
            stop, or data, is sent 

    I2C Data/Status:          bit 9  bit 8  bits 7-0 
                               BUSY   ACK     DATA
							   
        BUSY:  Controller busy if 1.  
        ACK:   Received ACK bit (for writes) 
        DATA:  Received data (for reads) 

    I2C Divider: bits 6-0 
        SCL frequency = Fclk /(4*(DIVIDER+1)) 
		
    ------ 
	
    MAX_COUNT: Holds the maximum value of the timer counter. When the timer 
        reaches this value gets reset and request an interrupt if enabled. 
        Writes to MAX_COUNT also resets the timer and its interrupt flag. 

    TIMER: the current value of the timer (incremented each clock cycle) 
        Reads also clear the interrupt flag. 
		
    ------ 
	
    GPOUT: General purpose outputs. 

    GPIN: General purpose inputs. 
	
    ------ 
	
    Interrupt enable:
        bit 0: Not used
        bit 1: Enable TIMER interrupt if 1
        bit 2: Enable UART0 RX interrupt if 1
        bit 3: Enable UART0 TX interrupt if 1
        bit 4: Enable UART1 RX interrupt if 1
        bit 5: Enable UART1 TX interrupt if 1
        bit 6: Enable UART2 RX interrupt if 1
        bit 7: Enable UART2 TX interrupt if 1
*/

`include "laRVa.v"
`include "uart.v"
`include "spi.v"
`include "timer.v"

module SYSTEM (
	input clk,		// Main clock input 25MHz
	input reset,	// Global reset (active high)

	input	rxd,	// UART
	output 	txd,
	
	input rxd1,		// UART 1 changed
	output txd1,
	
	input rxd2,		// UART 2 changed
	output txd2,

	output sck,		// SPI0
	output mosi,
	input  miso,	
	
	output sck1,	//SPI1
	input miso1,	
	output mosi1,
	
	output [31:0] gpout,	//GPOUT
	input [31:0] gpin,
	
	output ADC_CS,
	output BME_CS,
	output LORA_CS
);

wire		cclk;	// CPU clock
assign	cclk=clk;

///////////////////////////////////////////////////////
////////////////////////// CPU ////////////////////////
///////////////////////////////////////////////////////

wire [31:0]	ca;		// CPU Address
wire [31:0]	cdo;	// CPU Data Output
wire [3:0]	mwe;	// Memory Write Enable (4 signals, one per byte lane)
wire irq;
wire [31:2]ivector;	// Where to jump on IRQ
wire trap;			// Trap irq (to IRQ vector generator)

laRVa cpu (
		.clk     (cclk ),
		.reset   (reset),
		.addr    (ca[31:2] ),
		.wdata   (cdo  ),
		.wstrb   (mwe  ), //Instanciación de LaRVa 
		.rdata   (cdi  ),
		.irq     (irq  ),
		.ivector (ivector),
		.trap    (trap)
	);

 
///////////////////////////////////////////////////////
///// Memory mapping
wire iramcs;
wire iocs;
// Internal RAM selected in lower 512MB (0-0x1FFFFFFF)
assign iramcs = (ca[31:29]==3'b000);
// IO selected in last 512MB (0xE0000000-0xFFFFFFFF)
assign iocs   = (ca[31:29]==3'b111);

// Input bus mux
reg [31:0]cdi;	// Not a register
always@*
 casex ({iocs,iramcs})
        2'b01: cdi<=mdo; 
        2'b10: cdi<=iodo;
        default: cdi<=32'hxxxxxxxx;
 endcase

///////////////////////////////////////////////////////
//////////////////// internal memory //////////////////
///////////////////////////////////////////////////////
wire [31:0]	mdo;	// Output data
ram32	 ram0 ( .clk(~cclk), .re(iramcs), .wrlanes(iramcs?mwe:4'b0000),
			.addr(ca[13:2]), .data_read(mdo), .data_write(cdo) );

//////////////////////////////////////////////////
////////////////// Peripherals ///////////////////
//////////////////////////////////////////////////
wire uart0cs;	// UART0		at offset 0x00
wire uart1cs;	// UART1		at offset 0x00
wire uart2cs;	// UART2		at offset 0x00

wire spi0cs;		// SPI0     at offset 0x20 	
wire spi1cs;		// SPI1     at offset 0x20 	

wire irqcs;		// IRQEN    at offset 0xC0 	//añadido irqcs
wire irqvectcs;
wire timercs;
				//		 ...
				// other at offset 0xE0
//Añadidos los bitsb 
assign uart0cs = iocs&(ca[7:4]==4'b1000);		//0x80
assign uart1cs = iocs&(ca[7:4]==4'b1001);		//0x90
assign uart2cs = iocs&(ca[7:4]==4'b1010);		//0xA0
assign spi0cs  = iocs&(ca[7:4]==4'b0111);		//0x70
assign spi1cs  = iocs&(ca[7:4]==4'b0110);		//0x60
assign timercs = iocs&(ca[7:4]==4'b0100);		//0x40
assign gpoutcs = iocs&(ca[7:4]==4'b0011);		//0x80
assign irqcs   = iocs&(ca[7:5]==4'b0010); 		//0xC0
assign irqvectcs = iocs&(ca[7:3]==3'b000);		//0xE0

// Peripheral output bus mux
reg [31:0]iodo;	// Not a register
always@*

		case (ca[7:2])
			6'h08: iodo <= {24'h0, irqen};
			6'h0C: iodo <= gpout;
			6'h0D: iodo <= gpin;
			6'h10: iodo <= count;
			6'h18: iodo <= spirx1;
			6'h19: iodo <= {31'h0, busy1};
			6'h1C: iodo <= spirx;
			6'h1D: iodo <= {31'h0, busy};
			6'h20: iodo <= {24'h0, uart_do};
			6'h21: iodo <= {27'h0, ove, fe, tend, thre, dv};
			6'h24: iodo <= {24'h0, uart_do1};
			6'h25: iodo <= {27'h0, ove1, fe1, tend1, thre1, dv1};
			6'h28: iodo <= {24'h0, uart_do2};
			6'h29: iodo <= {27'h0, ove2, fe2, tend2, thre2, dv2};
			default: iodo <= 32'hxxxxxxxx;
		endcase


// GPOUT register
reg [31:0] gpout;
always @(posedge clk)
	begin
		if (gpoutcs & (mwe==4'b1111))
			begin
				gpout <= cdo[31:0];
			end
	end

/////////////////////////////
// UART 0

wire tend,thre,dv,fe,ove; // Flags
wire [7:0] uart_do;	// RX output data
wire uwrtx;			// UART TX write
wire urd;			// UART RX read (for flag clearing)
wire uwrbaud;		// UART BGR write
// Register mapping
// Offset 0: write: TX Holding reg
// Offset 0: read strobe: Clear DV, OVE (also reads RX data buffer)
// Offset 1: write: BAUD divider

assign uwrtx   = 	uart0cs & (~ca[3]) & (~ca[2]) & mwe[0]; 
assign uwrbaud =  	uart0cs & (~ca[3]) & (ca[2]) & mwe[0] & mwe[1];
assign urd     =  	uart0cs & (~ca[3]) & (~ca[2]) & (mwe==4'b0000); // Clear DV, OVE flgas

UART_CORE #(.BAUDBITS(12)) uart0 ( .clk(cclk), .txd(txd), .rxd(rxd), 
	.d(cdo[15:0]), .wrtx(uwrtx), .wrbaud(uwrbaud),. rd(urd), .q(uart_do),
	.dv(dv), .fe(fe), .ove(ove), .tend(tend), .thre(thre) );
	
// UART1
wire tend1,thre1,dv1,fe1,ove1; // Flags
wire [7:0] uart_do1;	// RX output data
wire uwrtx1;			// UART TX write
wire urd1;			// UART RX read (for flag clearing)
wire uwrbaud1;		// UART BGR write
// Register mapping
// Offset 0: write: TX Holding reg
// Offset 0: read strobe: Clear DV, OVE (also reads RX data buffer)
// Offset 1: write: BAUD divider

assign uwrtx1   =	uart1cs & (~ca[3]) & (~ca[2]) & mwe[0]; 
assign uwrbaud1 =  	uart1cs & (~ca[3]) & (ca[2]) & mwe[0] & mwe[1];
assign urd1     =  	uart1cs & (~ca[3]) & (~ca[2]) & (mwe==4'b0000); // Clear DV, OVE flgas

UART_CORE #(.BAUDBITS(12)) uart1 ( .clk(cclk), .txd(txd1), .rxd(rxd1), 
	.d(cdo[15:0]), .wrtx(uwrtx1), .wrbaud(uwrbaud1),. rd(urd1), .q(uart_do1),
	.dv(dv1), .fe(fe1), .ove(ove1), .tend(tend1), .thre(thre1));	
	
// UART2
wire tend2,thre2,dv2,fe2,ove2; // Flags
wire [7:0] uart_do2;	// RX output data
wire uwrtx2;			// UART TX write
wire urd2;			// UART RX read (for flag clearing) 
wire uwrbaud2;		// UART BGR write
// Register mapping
// Offset 0: write: TX Holding reg
// Offset 0: read strobe: Clear DV, OVE (also reads RX data buffer)
// Offset 1: write: BAUD divider 

assign uwrtx2   = 	uart2cs & (~ca[3]) & (~ca[2]) & mwe[0]; 
assign uwrbaud2 =  	uart2cs & (~ca[3]) & (ca[2]) & mwe[0] & mwe[1];
assign urd2     =  	uart2cs & (~ca[3]) & (~ca[2]) & (mwe==4'b0000); // Clear DV, OVE flgas

UART_CORE #(.BAUDBITS(12)) uart2 ( .clk(cclk), .txd(txd2), .rxd(rxd2), 
	.d(cdo[15:0]), .wrtx(uwrtx2), .wrbaud(uwrbaud2),. rd(urd2), .q(uart_do2),
	.dv(dv2), .fe(fe2), .ove(ove2), .tend(tend2), .thre(thre2));
	
/////////////////////////////   
// SPI0
wire spitx;					// SPI write
wire spictrl;
wire spisswr;
wire busy;					// SPI flag
reg [13:0] spicontrol;		// SPI Control 
wire [31:0] spirx;
reg [1:0] spi_ss= 2'b11;	// SPI Slave Select
assign spitx 		= spi0cs & (~ca[3]) & (~ca[2]) & (mwe==4'b1111);
assign spictrl 		= spi0cs & (~ca[3]) & (ca[2]) & (mwe==4'b1111); 
assign spisswr 		= spi0cs & (ca[3]) & (~ca[2]) & (mwe==4'b1111); 

always @(posedge clk)
	begin
		if (spictrl)
			begin
				spicontrol <= cdo[13:0];
			end
		if(spisswr)
			begin
				spi_ss <= cdo[1:0];
			end
	end
 
assign BME_CS = spi_ss[0];
assign ADC_CS = spi_ss[1];
SPI_master spi0 (.clk(cclk), .miso(miso), .wr(spitx), .din(cdo), 
	.divider(spicontrol[7:0]), .bits(spicontrol[13:8]), .sck(sck),
	.mosi(mosi), .busy(busy), .dout(spirx) );

/////////////////////////////
// SPI1
wire spitx1;		
wire spictrl1;
wire spisswr1;
wire busy1;
reg [13:0] spicontrol1;
wire [31:0] spirx1;
reg spi_ss1 = 1'b1;
assign spitx1 		= spi1cs & (~ca[3]) & (~ca[2]) & (mwe==4'b1111);
assign spictrl1 	= spi1cs & (~ca[3]) & (ca[2]) & (mwe==4'b1111);
assign spisswr1 	= spi1cs & (ca[3]) & (~ca[2]) & (mwe==4'b1111);

always @(posedge clk)
	begin
		if (spictrl1)
			begin
				spicontrol1 <= cdo[13:0];
			end
		if(spisswr1)
			begin
				spi_ss1 <= cdo[0];
			end
	end
 
assign LORA_CS = spi_ss1;
SPI_master spi1 (.clk(cclk), .miso(miso1), .wr(spitx1), .din(cdo), 
	.divider(spicontrol1[7:0]), .bits(spicontrol1[13:8]), .sck(sck1), .mosi(mosi1),
	.busy(busy1), .dout(spirx1) );

/////////////////////////////
// TIMER
wire flagtimer; 
wire [31:0] count;
assign timerwr = timercs & (mwe==4'b1111); 
assign timerrd = timercs & (mwe==4'b0000);
TIMER timer (.clk(cclk), .maxcount(cdo[31:0]), .flagtimer(flagtimer),
 	.rd(timerrd), .wr(timerwr) , .count(count[31:0]));

//////////////////////////////////////////
// Interrupt control

// IRQ enable reg
reg [7:0]irqen=0;
always @(posedge cclk or posedge reset) 
begin
	if (reset) irqen<=0; 
	else
	  if (irqcs & (mwe==4'b1111)) irqen<=cdo[7:0];
end

// IRQ vectors
reg [31:2]irqvect[0:7];
always @(posedge cclk) 
 if (irqvectcs & (mwe==4'b1111)) 
    irqvect[ca[4:2]]<=cdo[31:2];

wire irq_timer = irqen[1] & flagtimer;
wire irq_rx0   = irqen[2] & dv;
wire irq_tx0   = irqen[3] & thre;
wire irq_rx1   = irqen[4] & dv1;
wire irq_tx1   = irqen[5] & thre1;
wire irq_rx2   = irqen[6] & dv2;
wire irq_tx2   = irqen[7] & thre2;

// Priority encoder (trap highest, then vector order)
wire [2:0]vecn = trap      ? 3'b000 :
				 irq_timer ? 3'b001 :
				 irq_rx0   ? 3'b010 :
				 irq_tx0   ? 3'b011 :
				 irq_rx1   ? 3'b100 :
				 irq_tx1   ? 3'b101 :
				 irq_rx2   ? 3'b110 :
				 irq_tx2   ? 3'b111 :
				 3'bxxx;
assign ivector = irqvect[vecn];
assign irq = trap | irq_timer | irq_rx0 | irq_tx0 | irq_rx1 | 
			 irq_tx1 | irq_rx2 | irq_tx2;

endmodule	// System



//////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------------
//-- 32-bit RAM Memory with independent byte-write lanes
//----------------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////

module ram32
 (	input	clk,
	input	re,
	input	[3:0]	wrlanes,
	input	[L2N_RAM_SIZE-1:0]	addr,
	output	[31:0]	data_read,
	input	[31:0] 	data_write
 );

parameter RAM_SIZE = 4096;
localparam L2N_RAM_SIZE = $clog2(RAM_SIZE);

reg [31:0] ram_array [0:RAM_SIZE-1];
reg [31:0] data_out;
        
assign data_read = data_out;
        
always @(posedge clk) begin
	if (wrlanes[0]) ram_array[addr][ 7: 0] <= data_write[ 7: 0];
	if (wrlanes[1]) ram_array[addr][15: 8] <= data_write[15: 8];
	if (wrlanes[2]) ram_array[addr][23:16] <= data_write[23:16];
	if (wrlanes[3]) ram_array[addr][31:24] <= data_write[31:24];
end

always @(posedge clk) begin
	if (re) data_out <= ram_array[addr];
end

initial begin
`ifdef SIMULATION
	$readmemh("rom.hex", ram_array);
`else
	$readmemh("rand.hex", ram_array);
`endif
end

endmodule

