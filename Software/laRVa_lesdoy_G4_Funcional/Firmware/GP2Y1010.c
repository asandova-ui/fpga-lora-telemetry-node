
/*-----------------------------------------------------
---  Módulo GP2Y1010: Medición de Densidad de Polvo ---
-----------------------------------------------------*/

/* RESUMEN DE TIEMPOS GP2Y1010:
    Ciclo de pulso total:   320 us (GP2Y1010_LED_ON_US)
    Punto de muestreo:      280 us (GP2Y1010_SAMPLE_US)
    Señal de control:       M2_ON_OFF (GPOUT)
    Canal de lectura:       Variable (MCP3004_CH1 por defecto)

    El sensor requiere que el LED se encienda, se espere a la estabilización 
    del haz, se realice la conversión analógica y se apague el LED para 
    evitar el sobrecalentamiento del emisor IR y ahorrar energía.
*/

#include "GP2Y1010.h"
#include "MCP3004.h"

#define GP2Y1010_SAMPLE_US  280
#define GP2Y1010_LED_ON_US  320

/**
 * @brief Calcula el número de ciclos de reloj (ticks) necesarios para un tiempo en microsegundos.
 * * Convierte una magnitud temporal en unidades de contador del sistema basándose 
 * en la frecuencia CCLK configurada.
 * * @param us Tiempo deseado en microsegundos.
 * @return uint32_t Cantidad de ticks equivalentes para el TIMER.
 */
static uint32_t gp2y1010_temporizado(uint32_t us)
{
	return (CCLK / 1000000u) * us;
}

/**
 * @brief Realiza una lectura completa del sensor de polvo siguiendo el protocolo de tiempos.
 * * Activa el emisor infrarrojo a través de GPOUT, espera el tiempo de propagación 
 * definido en GP2Y1010_SAMPLE_US mediante la consulta del TIMER, y realiza la 
 * lectura analógica a través del ADC MCP3004. Finalmente, asegura que el pulso 
 * cumpla el ciclo de trabajo total antes de permitir la continuación del programa.
 * * @param channel Canal del ADC MCP3004 al que está conectado el sensor.
 * @return int Valor digital (0-1023) proporcional a la densidad de partículas.
 */
int GP2Y1010_Raw(uint8_t channel)
{
	uint32_t sample_ticks = gp2y1010_temporizado(GP2Y1010_SAMPLE_US);
	uint32_t total_ticks = gp2y1010_temporizado(GP2Y1010_LED_ON_US);
	uint32_t start = TIMER;
	uint32_t t_sample;
	uint32_t t_off;
	int adc;

	// Encendido del LED del sensor (Pulso de excitación)
	GPOUT |= M2_ON_OFF;

	// Espera bloqueante hasta alcanzar el punto óptimo de muestreo (280us)
	while ((uint32_t)(TIMER - start) < sample_ticks) {
	}
	t_sample = TIMER;

	// Captura del valor analógico mientras el LED está activo
	adc = MCP3004_Read(channel);

	// Espera hasta completar el tiempo total del pulso (320us)
	while ((uint32_t)(TIMER - start) < total_ticks) {
	}
	t_off = TIMER;

	return adc;
}
