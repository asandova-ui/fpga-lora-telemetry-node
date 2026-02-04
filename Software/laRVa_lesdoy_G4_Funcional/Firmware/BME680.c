
/*----------------------------------------------------------
---Módulo BME680: Sensor de Humedad Presión y Temperatura---
----------------------------------------------------------*/

#define STATUS 0x73 
#define RESET 0x60
#define ID_BME 0x50

#define Config_BME 0x75
#define CTRL_MEAS 0x74
#define osrs_h 0x72
#define run_gas_l 0x71

#define gas_wait 0x6D

#define res_heat 0x63


/**
 * @brief Inicializa el sensor BME680 configurando los oversamplings y el calefactor.
 * * 1. Realiza un reset completo del chip.
 * * 2. Configura oversampling: Humedad (1x), Temperatura (2x), Presión (16x).
 * * 3. Configura el tiempo de calentamiento del sensor de gas (100ms) y su consigna.
 * * 4. Activa el modo "Forced" para disparar una medición única.
 */
void startBME680(){ 
	// Reset de software: devuelve todos los registros a su valor por defecto
	writeBME680(0xB6,RESET);
    // Cambiamos a la página de registros 1 para configurar controles de medición
	writeBME680(0x10,STATUS);
	// 1.Set humidity oversampling to 1x by writing 0b001 to osrs_h
	writeBME680(0x01,osrs_h);
	// 2.Set temperature oversampling to 2x by writing 0b010 to osrs_t
	// 3.Set pressure oversampling to 16x  by writing 0b101 to osrs_p
	writeBME680(0x54,CTRL_MEAS);
	// 4.Set gas_wait_0 to 0x59 to select 100 ms heat up duration
	writeBME680(0x59,gas_wait);
	// 5.Set the corresponding heater set-point by writing the target heater resistance to res_heat
	writeBME680(0x00,res_heat);
	// 6.Set nb_conv to 0x0 to select the previously defined heater settings
	// 7.Set run_gas_l to 1 to enable gas measurements
	writeBME680(0x10,run_gas_l);
	// 8.Set mode to 0b01 to trigger a single measurement.
	writeBME680(0x55,CTRL_MEAS);
}

/**
 * @brief Escribe un dato en un registro del BME680 vía SPI.
 * @param data Valor a escribir.
 * @param dir Dirección del registro (0x00 | dir para escritura).
 */
void writeBME680(char data,char dir){
     SPI0SS = 0b10;    //Seleccionamos el chipSelect
     spixfer(0x00|dir);     //Pasamos direccion
     spixfer(data);         //Pasamos dato
    SPI0SS = 0b11;             //Devolvemos a 1 chip select
}

/**
 * @brief Lee el contenido de un registro del BME680.
 * @param dir Dirección del registro (0x80 | dir para lectura).
 * @return char Valor almacenado en el registro.
 */
char readBME680(char dir){ //Lectura del registro de una direccion de memoria concreta
    unsigned char readDATA;
    SPI0SS = 0b10;     //Seleccionamos chip select
	
    spixfer(0x80|dir);      //Lectura del SPI
    readDATA = spixfer(0);  //Enviamos 0
    SPI0SS=0b11;
    return readDATA;
}

/**
 * @brief Obtiene la temperatura compensada en centígrados (x100).
 * * Extrae los parámetros par_t1..3 de la página 0 y el valor ADC de la página 1.
 * * Aplica las fórmulas de compensación de Bosch para obtener t_fine.
 * @return int Temperatura (ej: 2550 representa 25.50°C).
 */
int returnTemp(){

    int par_t1, par_t2, par_t3, temp_adc, var1, var2, var3, t_fine, temp_comp;

    // Acceso a Coeficientes de Calibración (Página 0)
	writeBME680(0x00,STATUS); 

    par_t1 = (int32_t)((readBME680(0xEA) << 8) | readBME680(0xE9));
    par_t2 = (int32_t)((readBME680(0x8B) << 8) | readBME680(0x8A));
    par_t3 = (int32_t)readBME680(0x8C);
	
    // Acceso a datos del ADC (Página 1)
	writeBME680(0x10,STATUS); 
    // La temperatura es un valor de 20 bits repartido en 3 registros
    temp_adc = (int32_t)((readBME680(0x22) << 12) | (readBME680(0x23) << 4) | (readBME680(0x24) >> 4));
    
    // Fórmulas de compensación de punto fijo
    var1 = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
    var2 = (var1 * (int32_t)par_t2) >> 11;
    var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)par_t3 << 4)) >> 14;

    // t_fine es un valor intermedio crucial para compensar Presión y Humedad
    t_fine = var2 + var3;
    // Resultado final en centígrados
    temp_comp = ((t_fine * 5) + 128) >> 8;
	
    return temp_comp;
}

/**
 * @brief Obtiene la presión atmosférica compensada en Pascales.
 * * Requiere el valor t_fine calculado previamente (aquí se usa de forma simplificada).
 * * Extrae 10 parámetros de calibración (par_p1..10) para corregir la no linealidad.
 * @return int Presión en Pascales (Pa).
 */
int returnPressure(){

    int par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, press_adc,press_comp;
    int var1, var2, var3, t_fine;
	
    // Acceso a Coeficientes de Calibración (Página 0)
	writeBME680(0x00,STATUS); 

    par_p1 = (int32_t)((readBME680(0x8F) << 8) | readBME680(0x8E));
    par_p2 = (int32_t)((readBME680(0x91) << 8) | readBME680(0x90));
    par_p3 = (int32_t)readBME680(0x92);
    par_p4 = (int32_t)((readBME680(0x95) << 8) | readBME680(0x94));
    par_p5 = (int32_t)((readBME680(0x97) << 8) | readBME680(0x96));
    par_p6 = (int32_t)readBME680(0x99);
    par_p7 = (int32_t)readBME680(0x98);
    par_p8 = (int32_t)((readBME680(0x9D) << 8) | readBME680(0x9C));
    par_p9 = (int32_t)((readBME680(0x9F) << 8) | readBME680(0x9E));
    par_p10 = (int32_t)readBME680(0xA0);
	
    // Acceso a datos del ADC (Página 1)
	writeBME680(0x10,STATUS); 
    // Presión ADC: Valor de 20 bits (Registros 0x1F a 0x21)
    press_adc = (int32_t)((readBME680(0x1F) << 12) | (readBME680(0x20) << 4) | (readBME680(0x21) >> 4));

    // Compensación de Presión: Requiere t_fine calculado en returnTemp()
    var1 = ((int32_t)t_fine >> 1) - 64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)par_p4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)par_p3 << 5)) >> 3) + (((int32_t)par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)par_p1) >> 15;
    press_comp = 1048576 - press_adc;
    press_comp = (uint32_t)((press_comp - (var2 >> 12)) * ((uint32_t)3125));
    if (press_comp >= (1 << 30)) press_comp = ((press_comp / (uint32_t)var1) << 1);
    else press_comp = ((press_comp << 1) / (uint32_t)var1);
    var1 = ((int32_t)par_p9 * (int32_t)(((press_comp >> 3) * (press_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(press_comp >> 2) * (int32_t)par_p8) >> 13;
    var3 = ((int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) * (int32_t)par_p10) >> 17;
    press_comp = (int32_t)(press_comp) + ((var1 + var2 + var3 + ((int32_t)par_p7 << 7)) >> 4);

    // Devuelve la presión en Pascales (Pa)
    return press_comp;
}

/**
 * @brief Obtiene la humedad relativa compensada en %RH (x1000).
 * @param temp_comp Valor de temperatura compensada necesario para el cálculo.
 * @return int Humedad relativa (ej: 45000 representa 45.000% RH).
 */
int returnHumidity(int temp_comp){
    int par_h1, par_h2, par_h3, par_h4, par_h5, par_h6, par_h7, hum_adc,temp_scaled, hum_comp;
	int var1, var2, var3, var4, var5, var6;
	
    // Acceso a Coeficientes de Calibración (Página 0)    
	writeBME680(0x00,STATUS); 
	
    par_h1 = (int32_t)((readBME680(0xE3) << 4) | (readBME680(0xE2) & 0b00001111));
    par_h2 = (int32_t)((readBME680(0xE1) << 4) | (readBME680(0xE2) >> 4));
    par_h3 = (int32_t)readBME680(0xE4);
    par_h4 = (int32_t)readBME680(0xE5);
    par_h5 = (int32_t)readBME680(0xE6);
    par_h3 = (int32_t)readBME680(0xE7);
    par_h3 = (int32_t)readBME680(0xE8);
	
    // Acceso a datos del ADC (Página 1)
	writeBME680(0x10,STATUS); 
    // Humedad ADC: Valor de 16 bits (Registros 0x25 y 0x26)
    hum_adc = (int32_t)((readBME680(0x25) << 8) | readBME680(0x26));

    temp_scaled = (int32_t)temp_comp;
    var1 = (int32_t)hum_adc - (int32_t)((int32_t)par_h1 << 4) - (((temp_scaled * (int32_t)par_h3) / ((int32_t)100)) >> 1);
    var2 = ((int32_t)par_h2 * (((temp_scaled * (int32_t)par_h4) / ((int32_t)100)) + (((temp_scaled * ((temp_scaled * (int32_t)par_h5) / ((int32_t)100))) >> 6) / ((int32_t)100)) + ((int32_t)(1 << 14)))) >> 10;
    var3 = var1 * var2;
    var4 = (((int32_t)par_h6 << 7) + ((temp_scaled * (int32_t)par_h7) / ((int32_t)100))) >> 4;
    var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    var6 = (var4 * var5) >> 1;
    hum_comp = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;

    // Retorna el porcentaje de humedad relativa
    return hum_comp;
}
