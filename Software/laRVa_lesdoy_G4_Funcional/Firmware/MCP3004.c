
/*-----------------------------------------------------
---   Módulo MCP3004: Convertidor A/D Externo       ---
-----------------------------------------------------*/

#include "MCP3004.h"
#include "SPI.h"

#define MCP3004_MAX_VALUE 0x3FF
#define MCP3004_CH0       0
#define MCP3004_CH1       1
#define MCP3004_CH2       2
#define MCP3004_CH3       3

/**
 * @brief Función interna para realizar la lectura física del ADC.
 * * Configura el protocolo SPI según el datasheet del MCP3004:
 * * Byte 0: Bit de inicio (0x01).
 * * Byte 1: Configuración (Single/Diff) y Selección de Canal (CH0-CH3).
 * * Byte 2: Byte de relleno (Dummy) para recibir los últimos 8 bits del ADC.
 * * @param channel Canal a leer (0 a 3).
 * @param single  1 para modo Single-Ended, 0 para Diferencial.
 * @return int    Valor digital (0-1023).
 */
static int MCP3004_ReadADC(uint8_t channel, uint8_t single)
{
	// Validación de seguridad: evita acceder a canales inexistentes
	if (channel >= 4) return 0;

	uint8_t data[3] = {0, 0, 0};

	// Byte 0: El bit de inicio es el LSB (0x01)
	data[0] = 0x01;

	// Byte 1: 
    // - Bit 7: Modo Single/Diff (1 = Single)
    // - Bits 6-4: Selección de canal (D2, D1, D0)
	data[1] = single ? 0x80 : 0x00;
	data[1] |= (channel << 4);

	// Seleccionamos el esclavo ADC
	SPI0SS = 0b01;

	// Transmisión SPI: enviamos configuración y recibimos datos
	data[0] = spixfer(data[0]);
	data[1] = spixfer(data[1]);	// Recibimos los bits más significativos (MSB)
	data[2] = spixfer(data[2]);	// Recibimos los bits menos significativos (LSB)

	// Liberación del bus SPI
	SPI0SS = 0b11;

	/**
     * Reconstrucción del dato de 10 bits:
     * El MCP3004 devuelve el dato alineado a la derecha en los últimos 
     * 10 bits recibidos durante los bytes 1 y 2.
     */
	return ((data[1] << 8) | data[2]) & MCP3004_MAX_VALUE;
}

/**
 * @brief Realiza una lectura estándar (Single-Ended).
 * @param channel Canal (0, 1, 2 o 3).
 * @return int Valor de 0 a 1023.
 */
int MCP3004_Read(uint8_t channel)
{
	return MCP3004_ReadADC(channel, 1);
}

/**
 * @brief Realiza una lectura diferencial entre dos canales.
 * * Útil para medir señales con ruido común o sensores en puente (Wheatstone).
 * @param channel Par de canales (definido por el hardware del MCP).
 * @return int Diferencia digitalizada.
 */
int MCP3004_DifferentialRead(uint8_t channel)
{
	return MCP3004_ReadADC(channel, 0);
}

/**
 * @brief Lee múltiples canales secuencialmente y guarda los resultados en un array.
 * @param channels    Puntero al array con los números de canal deseados.
 * @param numChannels Cantidad de canales a leer.
 * @param readings    Puntero al array de destino para los resultados.
 */
void MCP3004_ReadMultiple(const uint8_t *channels, uint8_t numChannels, int *readings)
{
	uint8_t i;
	for (i = 0; i < numChannels; i++) {
		readings[i] = MCP3004_Read(channels[i]);
	}
}

/**
 * @brief Función de compatibilidad para llamadas rápidas desde registros de control.
 */
int MCP3004_Read_Reg(unsigned char channel)
{
	return MCP3004_Read((uint8_t)channel);
}
