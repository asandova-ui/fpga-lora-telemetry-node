//--------------------------------------------------------------------
// RISC-V things
// by Jesús Arias
//--------------------------------------------------------------------
`include "system.v"
`include "pll.v"

// Top module. (signals assigned to actual pins in file "pines.pcf")
module main(
	input  CLKIN, 		// Cristal de cuarzo externo (16MHz)
	
	// --- Bus SPI 0 (Sensores Ambientales) ---
	output ICE_SCK,		// Reloj SPI
    output ICE_MOSI,    // Salida de datos (Master Out Slave In)
    input  ICE_MISO,    // Entrada de datos (Master In Slave Out)
    output BME680_CS,   // Chip Select del sensor de Gas/Clima
    output ADC_CS,      // Chip Select del ADC MCP3004
	
	// --- Bus SPI 1 (Comunicación LoRa) ---	
	output LORA_SCK,
	output LORA_MOSI,
	input  LORA_MISO,
	output LORA_CS,
	output LORA_RESET,
	output L_RX,		// Señales de control adicionales para radio
	output L_TX,
	input  LORA_BUSY,	// Feedback de estado del módulo LoRa
    input  LORA_DIO1,   // Interrupción de radio
	
	// --- Control de Potencia MQ9 (Sensor de Gas) ---
    // El MQ9 requiere diferentes voltajes para ciclos de limpieza y sensado
	output EN_5V_M4,
	output EN_1V4_M4,
	output EN_5V_UP,
	output M2_ON_OFF,
	
	// --- LEDs de diagnóstico ---
	output ICE_LED1,
	output ICE_LED2,
	output ICE_LED3,
	output ICE_LED4,
	
	// --- UARTs (Puertos Serie) ---
	// UART	0: Consol de depuración
	input RXD,
	output TXD,
	
	// UART	1: Conexión con el GPS
	input RXD1,
	output TXD1,
	
	// UART	2: Ampliación
	input RXD2,
	output TXD2
);

//--------------------------------------------------------------------
// 1. GESTIÓN DE RELOJ (Clock Management)
//--------------------------------------------------------------------

wire clk,pll_lock;

// El PLL convierte los 16MHz de entrada en 25MHz para que el 
// procesador RISC-V trabaje con tiempos de ciclo constantes.
pll 
  pll1(
	.clock_in(CLKIN),
	.clock_out(clk),
	.locked(pll_lock)
	);

//--------------------------------------------------------------------
// 2. BUS DE ENTRADA/SALIDA DE PROPÓSITO GENERAL (GPIO)
//--------------------------------------------------------------------

// gpin: Señales que el procesador LEE desde la FPGA
wire [31:0] gpin;
assign gpin[1] = LORA_DIO1;
assign gpin[0] = LORA_BUSY;

// gpout: Señales que el procesador ESCRIBE para controlar el hardware
wire [31:0] gpout;
assign ICE_LED1 = gpout[0];
assign ICE_LED2 = gpout[1];
assign ICE_LED3 = gpout[2];
assign ICE_LED4 = gpout[3];
assign M2_ON_OFF = gpout[4];
assign EN_1V4_M4 = gpout[5];
assign EN_5V_M4 = gpout[6];
assign EN_5V_UP = gpout[7];
assign L_TX = gpout[8];
assign L_RX = gpout[9];
assign LORA_RESET = gpout[10];

//--------------------------------------------------------------------
// 3. INSTANCIA DEL SISTEMA (SOC RISC-V)
//--------------------------------------------------------------------
// Aquí se conecta el procesador con todos los módulos UART y SPI
SYSTEM sys1( .clk(clk), .reset(reset),
		.txd(TXD), .rxd(RXD),			// UART0
		.txd1(TXD1), .rxd1(RXD1),		// UART1
		.txd2(TXD2), .rxd2(RXD2),		// UART2
		.sck(ICE_SCK), .mosi(ICE_MOSI), .miso(ICE_MISO), 		// SPI0
		.sck1(LORA_SCK), .mosi1(LORA_MOSI), .miso1(LORA_MISO),	// SPI1
		.BME_CS(BME680_CS), .ADC_CS(ADC_CS), .LORA_CS(LORA_CS),	// Chip Select de los dos buses SPI
		.gpout(gpout),	// Bus de salida digital
    	.gpin(gpin)    // Bus de entrada digital
);

//--------------------------------------------------------------------
// 4. GENERADOR DE RESET AUTOMÁTICO
//--------------------------------------------------------------------
// Crea un pulso de reset al encender la FPGA.
// El contador 'cnt' empieza en el valor máximo y baja hasta 0.
// Mientras no sea 0, el sistema se mantiene en estado de RESET.
reg [21:0]cnt=22'h3fffff;
wire reset=(cnt!=0);

always @(posedge clk) cnt<= reset ? cnt-1: cnt;	// Cuenta regresiva tras el encendido



endmodule


