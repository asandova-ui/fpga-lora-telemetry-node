/*
Taller de Proyectos I
Curso 2025/2026
Grupo 4

Este archivo constituye el núcleo del firmware. Regula el flujo principal del programa, 
la inicialización de los periféricos del sistema y la interfaz de usuario mediante un menú 
por consola. Gestiona el despacho de tareas (scheduler básico) y el control directo de los 
reguladores de potencia de los sensores analógicos.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
 
#include "BME680.h"
#include "GPS_code.h" 
#include "SPI.h"
#include "lora_sx1262.h"
#include "MCP3004.h"
#include "GP2Y1010.h"

/*-----------------------------------------------------
---  Sección de Tipos de Datos y Macros de Tiempo   ---
-----------------------------------------------------*/

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;
  
typedef signed char  s8; 
typedef signed short s16;   
typedef signed int   s32;

// Configuración de reloj y macros para retardos precisos
#define CCLK (18000000u)	//Frecuencia del reloj del sistema: 18 MHz

/**
 * @brief Genera un retardo de tiempo por software.
 * * Utiliza un bucle de decremento de ciclos de CPU para pausar la ejecución 
 * durante un periodo determinado. La precisión depende de la frecuencia CCLK.
 * * @param val Número de iteraciones del bucle de retardo.
 * @return void
 */
void delay_loop(uint32_t val);	// (3 + 3*val) cycles

#define _delay_us(n) delay_loop((n*(CCLK/1000)-3000)/3000)
#define _delay_ms(n) delay_loop((n*(CCLK/1000)-30)/3) 

/*-----------------------------------------------------
---  Sección de Registros Mapeados en Memoria (I/O) ---
-----------------------------------------------------*/

// UART0: Consola principal de depuración y menú
#define UART0DAT  (*(volatile uint32_t*)0xE0000080)	// Registro de datos
#define UART0STA  (*(volatile uint32_t*)0xE0000084)	// Registro de estado (RX/TX ready)
#define UART0BAUD (*(volatile uint32_t*)0xE0000084)	// Registro de baudrate (comparte dir)

// UART1: Interfaz de comunicación con el módulo GPS
#define UART1DAT  (*(volatile uint32_t*)0xE0000090)
#define UART1STA  (*(volatile uint32_t*)0xE0000094)
#define UART1BAUD (*(volatile uint32_t*)0xE0000094)

// UART2: Interfaz auxiliar
#define UART2DAT  (*(volatile uint32_t*)0xE00000A0)
#define UART2STA  (*(volatile uint32_t*)0xE00000A4)
#define UART2BAUD (*(volatile uint32_t*)0xE00000A4)

// SPI0: Bus para BME680 y ADC MCP3004
#define SPI0DAT	 (*(volatile uint32_t*)0xE0000070)	// Registro de datos
#define SPI0CTRL (*(volatile uint32_t*)0xE0000074)	// Registro de estado (RX/TX ready)
#define SPI0STA	 (*(volatile uint32_t*)0xE0000074)	// Registro de baudrate (comparte dir)
#define SPI0SS	 (*(volatile uint32_t*)0xE0000078)	// Slave Select

// SPI1: Bus dedicado al transceptor LoRa SX1262
#define SPI1DAT	 (*(volatile uint32_t*)0xE0000060)
#define SPI1CTRL (*(volatile uint32_t*)0xE0000064)
#define SPI1STA	 (*(volatile uint32_t*)0xE0000064)
#define SPI1SS	 (*(volatile uint32_t*)0xE0000068)

// Temporizador del Sistema
#define MAX_COUNT (*(volatile uint32_t*)0xE0000040)	// Valor de maximo del timer
#define TIMER     (*(volatile uint32_t*)0xE0000040)	// Valor actual del contador

// Puertos de Propósito General (GPIO)
#define GPOUT    (*(volatile uint32_t*)0xE0000030)	// Registro de salida (Escritura)
#define GPIN     (*(volatile uint32_t*)0xE0000034)	// Registro de entrada (Lectura)

/*-----------------------------------------------------
---  Sección de Puertos de Propósito General (GPIO) ---
-----------------------------------------------------*/

/* --- Definición de Bits en GPOUT (Actuadores y Energía) --- */
#define LORA_RESET  (1u<<10)	// Reset físico del SX1262 (activo en bajo/alto según config)
#define L_RX        (1u<<9)  	// LED o señal de control de recepción LoRa
#define L_TX        (1u<<8)  	// LED o señal de control de transmisión LoRa
#define EN_5V_UP    (1u<<7)	  	// Habilita el conversor DC-DC para sensores (BME/ADC)
#define EN_5V_M4    (1u<<6)	  	// Activa la línea de 5V para el sensor de gases/partículas
#define EN_1V4_M4   (1u<<5)  	// Control de tensión específica para el filamento del M4
#define M2_ON_OFF   (1u<<4)  	// Control de encendido/apagado para periférico auxiliar M2
#define LED3        (1u<<3)  	// Led 3
#define LED2        (1u<<2)  	// Led 2
#define LED1        (1u<<1)  	// Led 1
#define LED0        (1u<<0)  	// Led 0

/* --- Definición de Bits en GPIN (Estado de Periféricos) --- */
#define LORA_DIO1   (1<<1)		// Entrada de interrupción del LoRa (Evento terminado)
#define LORA_BUSY   (1<<0)		// Indica si el chip LoRa está procesando un comando

/*-----------------------------------------------------
---  Sección de Control de Interrupciones (NVIC)    ---
-----------------------------------------------------*/

#define IRQEN	 (*(volatile uint32_t*)0xE0000020)	// Registro de habilitación global
#define IRQVECT0 (*(volatile uint32_t*)0xE0000000)	// Vector 0: Fallos de CPU (TRAP)
#define IRQVECT1 (*(volatile uint32_t*)0xE0000004)	// Vector 1: Timer
#define IRQVECT2 (*(volatile uint32_t*)0xE0000008)	// Vector 2: UART0 RX
#define IRQVECT3 (*(volatile uint32_t*)0xE000000C)	// Vector 3: UART0 TX
#define IRQVECT4 (*(volatile uint32_t*)0xE0000010)	// Vector 4: UART1 RX
#define IRQVECT5 (*(volatile uint32_t*)0xE0000014)	// Vector 5: UART1 TX
#define IRQVECT6 (*(volatile uint32_t*)0xE0000018)	// Vector 6: UART2 RX
#define IRQVECT7 (*(volatile uint32_t*)0xE000001C)	// Vector 7: UART2 TX

/*-----------------------------------------------------
---  Sección de Máscaras de Interrupción (IRQEN)    ---
-----------------------------------------------------*/

// Estas máscaras se usan con el registro IRQEN para habilitar/deshabilitar canales

#define IRQ_TIMER     (1u<<1)	// Bit para el temporizador del sistema
#define IRQ_UART0_RX  (1u<<2) // Bit para recepción de consola (Usuario)
#define IRQ_UART0_TX  (1u<<3) // Bit para transmisión de consola
#define IRQ_UART1_RX  (1u<<4) // Bit para recepción del GPS
#define IRQ_UART1_TX  (1u<<5) // Bit para transmisión del GPS
#define IRQ_UART2_RX  (1u<<6) // Bit para puerto auxiliar
#define IRQ_UART2_TX  (1u<<7) // Bit para puerto auxiliar

/*-----------------------------------------------------
---  Sección de Punteros y Buffers (FIFOs)          ---
-----------------------------------------------------*/

uint8_t udat[32];				// Buffer circular UART0 (Consola)
volatile uint8_t rdix,wrix;		// Punteros de lectura/escritura UART0

uint8_t udat1[32]; 				// Buffer circular UART1 (GPS)
volatile uint8_t rdix1,wrix1; 	// Punteros de lectura/escritura UART1

uint8_t udat2[32]; 				// Buffer circular UART2 (Auxiliar)
volatile uint8_t rdix2,wrix2; 	// Punteros de lectura/escritura UART2

/*-----------------------------------------------------
---  Sección de Funciones de Bajo Nivel y Drivers   ---
-----------------------------------------------------*/

#define putchar(d) _putch(d)
#include "printf.c"
#include "GPS_code.c" 
#include "BME680.c"
#include "GP2Y1010.c"
#include "SPI.c"
#include "lora_sx1262.c"
#include "MCP3004.c"  

/**
 * @brief Envía un carácter individual a través de la UART0.
 * * Monitoriza el registro de estado UART0STA para verificar la disponibilidad 
 * del buffer de transmisión antes de escribir el byte en el registro de datos.
 * * @param c Carácter ASCII a transmitir.
 * @return void
 */
void _putch(int c)
{
	while((UART0STA&2)==0);
	UART0DAT = c;
}

/**
 * @brief Transmite una cadena de caracteres terminada en nulo por UART0.
 * * Itera sobre un puntero de memoria enviando cada carácter secuencialmente 
 * mediante llamadas a la función _putch.
 * * @param p Puntero a la cadena de caracteres (string).
 * @return void
 */
void _puts(const char *p)
{
	while (*p)
		_putch(*(p++));
}
 
/**
 * @brief Lee un carácter de forma bloqueante desde la UART0.
 * * Detiene la ejecución hasta que el bit de recepción en UART0STA indica 
 * que un nuevo byte está disponible en el registro de datos.
 * * @return uint8_t Byte recibido.
 */
uint8_t _getch()
{
	while((UART0STA&1)==0);
	return UART0DAT;
}

/**
 * @brief Verifica la disponibilidad de datos en el buffer de recepción de la UART0.
 * * Consulta el registro de estado de la UART (UART0STA) para comprobar si el 
 * bit de recepción está activo. Se utiliza habitualmente como condición de 
 * salida en bucles de telemetría para permitir la interrupción del usuario.
 * * @return uint8_t Retorna un valor distinto de cero si hay un carácter 
 * esperando en el registro de datos; cero en caso contrario.
 */
uint8_t haschar() {return UART0STA&1;}

/**
 * @brief Extrae de forma segura un byte del buffer circular (FIFO) de la UART0.
 * * Implementa una lectura bloqueante que espera hasta que el puntero de escritura (wrix) 
 * sea distinto al de lectura (rdix). Tras extraer el dato, incrementa el puntero 
 * y aplica una máscara binaria para asegurar el retorno al inicio del array (wrap-around).
 * * @return uint8_t El byte más antiguo almacenado en el buffer.
 */
 uint8_t _getchUART0()
{
	uint8_t d;
	while(rdix==wrix);
	d=udat[rdix++];
	rdix&=31;
	return d;
}

/**
 * @brief Extrae un byte del buffer circular (FIFO) de la UART1, optimizado para el GPS.
 * * Funciona bajo el mismo principio que la UART0, pero utiliza un espacio de 
 * memoria independiente (udat1) con un tamaño de buffer mayor (128 posiciones), 
 * adecuado para procesar ráfagas de sentencias NMEA.
 * * @return uint8_t Byte de datos proveniente del módulo GPS.
 */
uint8_t _getchUART1()
{
	uint8_t d;
	while(rdix1==wrix1);
	d=udat1[rdix1++];
	rdix1&=127;
	return d;
}          

/**
 * @brief Recupera el valor del registro de control MEPC (Machine Exception Program Counter).
 * * Utiliza instrucciones en ensamblador de forma nativa para leer el registro CSR 0x341. 
 * El atributo 'naked' indica al compilador que no genere prólogo ni epílogo de función 
 * (gestión de pila), permitiendo un acceso directo y puro a los registros del sistema.
 * * @return uint32_t Dirección de memoria (PC) donde se produjo la última excepción o trap.
 */                  
uint32_t __attribute__((naked)) getMEPC()
{
	asm volatile(
	"	.word	0x34101573 		\n"
	"	.word   0x34151073		\n"
	"	ret						\n"
	);
}

/*-----------------------------------------------------
---  Sección de Manejadores de Interrupción (IRQs)  ---
-----------------------------------------------------*/

/**
 * @brief Manejador de excepciones por TRAP (Excepción de Hardware).
 * * Captura errores de ejecución no controlados y reporta la dirección de 
 * memoria donde ocurrió el fallo mediante el registro MEPC (Machine Exception Program Counter).
 * * @return void
 */
void __attribute__((interrupt ("machine"))) irq0_handler()
{
	_printf("\nTRAP at 0x%x\n",getMEPC());
}

/**
 * @brief Interrupción de recepción para la UART0.
 * * Gestiona la llegada de datos de forma asíncrona, almacenando los bytes 
 * en una estructura de buffer circular (FIFO) para su procesamiento posterior.
 * * @return void
 */
void __attribute__((interrupt ("machine"))) irq1_handler()
{ 
	udat[wrix++]=UART0DAT;
	wrix&=31;
} 

/**
 * @brief Manejador de interrupción para la transmisión de la UART0 (TX).
 * * Se activa automáticamente cuando el buffer de transmisión de la UART está vacío. 
 * En esta implementación, se utiliza para realizar una prueba de envío cíclico 
 * de caracteres ASCII (desde el espacio ' ' hasta '~'), incrementando el valor 
 * en cada ciclo.
 * * @return void
 */
void  __attribute__((interrupt ("machine"))) irq2_handler(){
	static uint8_t a=32;
	UART0DAT=a;
	if (++a>=128) a=32;
} 
	 
/**
 * @brief Interrupción de temporización (Timer).
 * * Se dispara cuando el contador hardware alcanza el valor definido. Se utiliza 
 * para el control de tiempos de espera y la sincronización de tareas periódicas.
 * * @return void
 */
void __attribute__((interrupt("machine"))) irq3_handler()
{
	IRQEN = 0b00000000;	//Deshabilitamos la interrupción del timer
}

/**
 * @brief Interrupción de recepción para la UART1 (GPS).
 * * Se activa al recibir datos del módulo GPS. Invoca la rutina de filtrado 
 * read_GPS() para procesar tramas NMEA en tiempo real.
 * * @return void
 */
void __attribute__((interrupt("machine"))) irq4_handler()
{
	read_GPS();
	IRQEN = 0b00000000;
}

/**
 * @brief Manejador de interrupción para la transmisión de la UART1 (TX).
 * * Gestiona la salida de datos hacia el módulo GPS o puerto auxiliar 1. 
 * Similar al manejador de la UART0, implementa un generador de caracteres 
 * ASCII cíclico para verificar la integridad del canal de transmisión.
 * * @return void
 */
void __attribute__((interrupt("machine"))) irq5_handler()
{
	static uint8_t a = 32;
	UART1DAT = a;
	if (++a >= 128)
		a = 32;
}

/**
 * @brief Manejador de interrupción para la recepción de la UART2 (RX).
 * * Captura los bytes entrantes por el tercer puerto serie del sistema. 
 * Actualmente configurado para reenviar un patrón de caracteres como eco 
 * de diagnóstico para validar la conexión del puerto.
 * * @return void
 */
void __attribute__((interrupt("machine"))) irq6_handler()
{
	static uint8_t a = 32;
	UART2DAT = a;
	if (++a >= 128)
		a = 32;
}

/**
 * @brief Manejador de interrupción para la transmisión de la UART2 (TX).
 * * Controla el flujo de salida del tercer puerto serie. Implementa la 
 * lógica de barrido de caracteres ASCII para pruebas de loopback y 
 * verificación de baudrate en el canal auxiliar.
 * * @return void
 */
void __attribute__((interrupt("machine"))) irq7_handler()
{
	static uint8_t a = 32;
	UART2DAT = a;
	if (++a >= 128)
		a = 32;
}

/*-----------------------------------------------------
---  Sección Principal: Bucle de Control y Menú     ---
-----------------------------------------------------*/

#define BAUD 115200u
#define BAUD1 9600u
#define BAUD2 115200u
#define NULL ((void *)0)

const static char *menutxt="\n"
"    :::     ::::::::::: :::    :::     :::     :::          ::::::::::: :::::::: ::::::::::: \n"
"  :+: :+:       :+:     :+:    :+:     :+:   :+: :+:            :+:    :+:    :+:    :+:     \n"
" +:+   +:+      +:+     +:+    +:+     +:+  +:+   +:+           +:+    +:+    +:+    +:+     \n"
"+#++:++#++:     +#+     +#+    +#+     +:+ +#++:++#++:          +#+    +#+    +:+    +#+     \n"
"+#+     +#+     +#+     +#+     +#+   +#+  +#+     +#+          +#+    +#+    +#+    +#+     \n"
"#+#     #+#     #+#     #+#      #+#+#+#   #+#     #+#          #+#    #+#    #+#    #+#     \n"
"###     ### ########### ########## ###     ###     ###      ########### ########     ###     \n"
"------------------------------------------------\n"
"\n"; 

const unsigned static char *menu="\n"
"x----------------------------------------------x\n"
"|  1 -> Lectura Pres, Temp y Hum (BME680)      |\n"
"|  2 -> Leer canales ADC                       |\n"
"|  3 -> EN_5V_UP on/off                        |\n"
"|  4 -> EN_5V_M4 on/off                        |\n"
"|  5 -> EN_1V4_M4 on/off                       |\n"
"|  6 -> Leer TIMER                             |\n"
"|  7 -> Leer GP2Y1010                          |\n"
"|  8 -> Secuencia Leds                         |\n"
"|  c -> Prueba de campo LoRa                   |\n"
"|  r -> Recibir LoRa                           |\n"
"|  t -> Transmitir LoRa                        |\n"
"|  g -> Lectura GPS                            |\n"
"x----------------------------------------------x\n"
"\n\n";

/**
 * @brief Función de entrada principal del sistema.
 * * Configura el hardware inicial: establece los divisores de frecuencia para 
 * SPI0/SPI1 y las UART0/1/2. Inicializa la tabla de vectores de interrupción 
 * (IRQVECT) y habilita el sistema de excepciones de la CPU. Ejecuta el bucle 
 * de ejecución de tareas que responde a los comandos del usuario.
 * * @return int (No retorna).
 */
void main()
{
	char c,buf[17];
	uint8_t *p;
	unsigned int i,j;
	int n;
	void (*pcode)();
	uint32_t *pi;
	uint16_t *ps;
	int temp_comp,press_comp, hum_comp;
	unsigned char readDATA;
	  
	// --- Inicialización de Hardware ---	
	
	SPI0CTRL = (8<<8)|8;  	// Configuración SPI0
	SPI1CTRL = (8<<8)|8;	// Configuración SPI1

	// Configuración de velocidades UART (Baudrate)
	UART0BAUD  =(CCLK+BAUD/2)/BAUD - 1; 
	UART1BAUD =(CCLK+BAUD1/2)/BAUD1 - 1;	
	UART2BAUD =(CCLK+BAUD2/2)/BAUD2 - 1;
         
	c = UART0DAT;		// Clear RX garbage
	c = UART1DAT;		// Clear RX garbage
	         
	//Inicializamos vectores de interrupcion
	IRQVECT0=(uint32_t)irq0_handler;
	IRQVECT1=(uint32_t)irq1_handler;
	IRQVECT2=(uint32_t)irq2_handler;
	IRQVECT3=(uint32_t)irq3_handler;
	IRQVECT4=(uint32_t)irq4_handler;
	IRQVECT5=(uint32_t)irq5_handler;
	IRQVECT6=(uint32_t)irq6_handler;
	IRQVECT7=(uint32_t)irq7_handler;

	// Activación de sistema de interrupciones y Timer
	IRQEN=1;			// Enable UART RX IRQ
	MAX_COUNT = 0xFFFFFFFFu;
	IRQEN |= IRQ_TIMER;
	
	uint8_t Buff[500];
	asm volatile ("ecall");
	asm volatile ("ebreak");
 
	while (1)      
	{     
			_puts(menu);				// Muestra opciones disponibles
			char cmd = _getch();		// Espera comando del usuario
			if (cmd > 32 && cmd < 127)
				_putch(cmd); 
			_puts("\n"); 
           
			switch (cmd){ 
			
				case 't':	//Transmisión LoRa	   
					if(SX1262_Init()){
						_printf("Lambda-62 iniciado\n");
						SX1262_configSetFrequency(868000000);	// f = 868 MHz
						SX1262_configSetBandwidth(4);      		// BW = 125 kHz
						SX1262_configSetSpreadingFactor(12); 	//SF12
						SX1262_transmit((uint8_t *)"Hola AILVA IoT", 14);
					}
					else {
						_printf("Error al iniciar\n");
					}
					break;

				case 'r':	//Recepción LoRa
					if(SX1262_Init()){
						_printf("Lambda-62 iniciado\n");
						SX1262_configSetFrequency(868000000);	// f = 868 MHz
						SX1262_configSetBandwidth(4);      		// BW = 125 kHz
						SX1262_configSetSpreadingFactor(12); 	// SF12
						SX1262_lora_receive_async(Buff, 500);
					} 
					else {
						_printf("Error al iniciar\n");
					}
					break;

				case 'c': 	//Prueba de campo (envio cada 25 s).
					{
						int idmsj = 0;
						char lat[16];
						char lon[16];
						char msj[200];

						if(!SX1262_Init()){
							_printf("Error al iniciar\n");
							break;
						}

						SX1262_configSetFrequency(869000000);	// f = 868 MHz
						SX1262_configSetBandwidth(4);      		// BW =125 kHz
						SX1262_configSetSpreadingFactor(12); 	//SF12

						while (!haschar()) {
							startBME680();
							temp_comp = returnTemp();
							press_comp = returnPressure();
							hum_comp = returnHumidity(temp_comp);


							GPS_LatLon(lat, (int)sizeof(lat), lon, (int)sizeof(lon));
							// GUarda en msj la cadena con los argumentos posteriores
							_sprintf(msj,
									"%3d Grupo 4, Temp %2d.%2d, Hum %2d.%3d %%, Pres %d, Lat  %10s, Lon %10s",
									idmsj,
									temp_comp/100, temp_comp%100,
									hum_comp/1000, hum_comp%1000,
									press_comp,
									lat, lon);

							_printf("%s\n", msj);
							SX1262_transmit((uint8_t *)msj, 72);
							idmsj++;
							_delay_ms(25000);
						}

						if (haschar()) _getch();
						break;
					}	
					
				case '1':	//Lectura del sensor BME680 (humedad, temperatura y presión)
					startBME680(); 
					temp_comp =returnTemp();
					press_comp = returnPressure();
					hum_comp = returnHumidity(temp_comp);
					
					_printf("==============================================\n");
					_printf("------------------- BME680 -------------------\n");
					_printf("==============================================\n");
					_printf("                                                ");
					_printf("\n La temperatura registrada es: ");
					_printf("%02d.%d%c", temp_comp/100, temp_comp%100, 167);
					_printf("\n La presion registrada es: ");
					_printf("%d Pascales",press_comp/100);
					_printf("\n La humedad registrada es: ");
					_printf("%02d.%03d%c", hum_comp/1000, hum_comp%1000, 37);
					_printf("                                              \n");
					_printf("\n\n");
					_printf("==============================================\n");    
					break;
				
				case '2':	//Lectura del ADC MCP3004. 
					{
						int adc0, adc1, adc2, adc3;

						adc0 = MCP3004_Read(MCP3004_CH0);
						adc1 = MCP3004_Read(MCP3004_CH1);
						adc2 = MCP3004_Read(MCP3004_CH2);
						adc3 = MCP3004_Read(MCP3004_CH3);

						_printf("==============================================\n");
						_printf("---------------- ADC MCP3004 -----------------\n");
						_printf("==============================================\n");
						_printf("CH0: %d\n", adc0);
						_printf("CH1: %d\n", adc1);
						_printf("CH2: %d\n", adc2);
						_printf("CH3: %d\n", adc3);
						_printf("==============================================\n");
						break;
					}

				// Controles manuales de las alimentaciones.
				case '3':
					GPOUT ^= EN_5V_UP;  
					_printf("EN_5V_UP %s\n", (GPOUT & EN_5V_UP) ? "ON" : "OFF");
					break;
					
				case '4':
					GPOUT ^= EN_5V_M4;
					_printf("EN_5V_M4 %s\n", (GPOUT & EN_5V_M4) ? "ON" : "OFF");
					break;
					
				case '5':
					GPOUT ^= EN_1V4_M4;
					_printf("EN_1V4_M4 %s\n", (GPOUT & EN_1V4_M4) ? "ON" : "OFF");
					break;

				
				case '6':	// Lectura del estado del TIMER.
					if ((IRQEN & IRQ_TIMER) == 0) {
						MAX_COUNT = 0xFFFFFFFFu;
						IRQEN |= IRQ_TIMER;
					}
					_printf("TIMER: %u\n", TIMER);
					break;
				
				case '7':	// Lectura del sensor de polvo GP2Y1010.
					{
						int raw = GP2Y1010_Raw(MCP3004_CH1);
						_printf("GP2Y1010 ADC: %d\n", raw);
						break;
					}
				
				case '8':	// Activación de los LEDs 
					// Encendido 0→1→2→3
					GPOUT |= LED0;    _delay_ms(200);
					GPOUT |= LED1;    _delay_ms(200);
					GPOUT |= LED2;    _delay_ms(200);
					GPOUT |= LED3;    _delay_ms(200);
					
					// Apagado 0→1→2→3
					GPOUT &= ~LED0;   _delay_ms(200);
					GPOUT &= ~LED1;   _delay_ms(200);
					GPOUT &= ~LED2;   _delay_ms(200);
					GPOUT &= ~LED3;   _delay_ms(200);
					break;
	
				case 'g': 	// Lectura del GPS 	
					read_GPS(); 
					break;
						
				default:
					continue;
			}
	}
}

