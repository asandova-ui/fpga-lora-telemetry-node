
/*-----------------------------------------------------
---  Módulo SPI: Implementación para buses 0 y 1    ---
-----------------------------------------------------*/

/*RESUMEN DE REGISTROS SPI:
    SPI Control:   bits 31-14   bits 13-8   bits 7-0 
                      xxxx        DLEN     DIVIDER 

        DLEN:    Longitud de datos (8 a 32 bits). En este driver se usan 8 bits.
        DIVIDER: Frecuencia SCK = Fclk / (2 * (DIVIDER + 1)) 

    SPI Flags (STA): bits 31-1   bit 0
                      xxxx     BUSY 

        BUSY: Indica que el SPI está intercambiando datos cuando vale 1.

    SPI Slave Select: bits 31-2  bit 1   bit 0 
                         xxxx     ss1     ss0 

        ss0 : Selecciona el esclavo 0 cuando es 0 (lógica activa baja).
        ss1 : Selecciona el esclavo 1 cuando es 0 (lógica activa baja).
*/

// Configuración de frecuencia base para el bus
#define FREQ_SPI 1000000
// Cálculo del divisor basado en el reloj de sistema (CCLK = 18MHz)
#define DIV_SPI 	((CCLK/(2*FREQ_SPI))-1)

/*-----------------------------------------------------
---   SPI0: Transferencia para BME680 y MCP3004     ---
-----------------------------------------------------*/

/**
 * @brief Realiza una transferencia de un byte a través del bus SPI0.
 * * Esta función es bloqueante. Escribe un byte en el registro de datos e inicia 
 * la secuencia de reloj. Espera hasta que el controlador SPI termine de rotar 
 * los bits antes de retornar el byte recibido desde el esclavo.
 * * @param d Byte de datos a transmitir (MOSI).
 * @return unsigned char Byte de datos recibido simultáneamente (MISO).
 */
unsigned char spixfer (unsigned char d)
{
	SPI0DAT=d;
	while(SPI0STA&1);
	return SPI0DAT;
}

/*-----------------------------------------------------
---    SPI1: Transferencia para el módulo LoRa      ---
-----------------------------------------------------*/

/**
 * @brief Realiza una transferencia de un byte a través del bus SPI1 (LoRa).
 * * Opera de forma idéntica a spixfer pero sobre el controlador hardware 1. 
 * Se utiliza para la comunicación de alta prioridad con el transceptor SX1262.
 * * @param d Byte de datos a transmitir hacia la radio.
 * @return unsigned char Byte de respuesta del módulo LoRa.
 */
unsigned char spixfer1 (unsigned char d)
{
	SPI1DAT=d;
	while(SPI1STA&1);
	return SPI1DAT;
}
