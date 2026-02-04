
/*-----------------------------------------------------
---   Módulo GPS: Recepción y Filtrado NMEA         ---
-----------------------------------------------------*/

/* RELACIÓN FORMATO NMEA ($GNGGA) VS CAMPOS PROCESADOS:
    
    Formato: $GNGGA , hhmmss.ss , llll.ll , a , yyyyy.yy , a , x , xx , x.x , a.a , M ...
    ----------------------------------------------------------------------------------
    TOKEN  |  FORMATO    | DESCRIPCIÓN TÉCNICA           | ACCIÓN EN CÓDIGO
    ----------------------------------------------------------------------------------
    case 0 | $GNGGA      | Cabecera de la trama          | Validación de ID
    case 1 | hhmmss.ss   | Hora UTC (HH:MM:SS)           | Formateo con ':'
    case 2 | llll.ll     | Latitud (Grados/Minutos)      | Extracción a last_lat
    case 3 | a           | Hemisferio (N/S)              | Impresión de signo
    case 4 | yyyyy.yy    | Longitud (Grados/Minutos)     | Extracción a last_lon
    case 5 | a           | Hemisferio (E/W)              | Impresión de signo
    case 6 | x           | Calidad (0=No fix, 1=GPS)     | Retorno de estado
    case 7 | xx          | Satélites en uso              | Impresión entera
    case 9 | a.a         | Altitud (MSL)                 | Impresión en metros
    ----------------------------------------------------------------------------------
*/

uint8_t trama_GPS[1000];

/**
 * @brief Lee un byte de forma bloqueante desde la UART1 (GPS).
 * * Monitoriza el bit de recepción del registro de estado UART1STA.
 * * @return uint8_t Byte recibido desde el hardware.
 */
uint8_t _getch1() {
    while((UART1STA&1)==0);
    return UART1DAT;
}

/**
 * @brief Escanea el flujo de datos de la UART buscando una trama GNGGA válida.
 * * Utiliza un buffer local para capturar la línea completa una vez detectado el 
 * carácter '$'. Si la cabecera coincide con la sentencia GNGGA, invoca al 
 * procesador de cadena y finaliza la búsqueda.
 * * @return void
 */
void read_GPS() { 
    char cadenaUart[80];
    char buffer;
    int i;

    while(1) {
        buffer = _getch1();
        if(buffer == '$') {
            cadenaUart[0] = '$';
            for (i = 1; i < 80; i++) {
                cadenaUart[i] = _getch1();
                if(cadenaUart[i] == '\n') {
                    cadenaUart[i] = '\0';
                    break;
                }
            }
            
            if(cadenaUart[0]=='$' && cadenaUart[1]=='G' && cadenaUart[2]=='N' 
             && cadenaUart[3]=='G' && cadenaUart[4]=='G' && cadenaUart[5]=='A')
            {
                _printf("%s\n", cadenaUart);  
                procesaCadena(cadenaUart);
                return;
            }
        }
    }
}

/**
 * @brief Segmenta la trama NMEA y extrae la información de cada campo.
 * * Utiliza un autómata de estados basado en el contador 'a' para identificar 
 * los campos delimitados por comas y formatea la salida para mostrarla 
 * por la consola de depuración.
 * * @param cadena Puntero a la cadena de texto con la trama NMEA.
 * @return void
 */
void procesaCadena(const char *cadena) {
    const char separador = ',';
    char caracter[50];
    int a = 0;
    const char *p = cadena;
    
    while (*p) {
        int i = 0;
        while (*p != separador && *p != '\0') {
            caracter[i++] = *p++;
        }
        caracter[i] = '\0';
        
        switch(a) {
            case 0: _printf("Id: %s\n", caracter); break;
            case 1: _printf("UTC: %c%c:%c%c:%c%c\n", caracter[0],caracter[1],caracter[2],caracter[3],caracter[4],caracter[5]); break;
            case 2: _printf("Latitud: %c%c.%c%c%c%c%c%c%c\n", caracter[0],caracter[1],caracter[2],caracter[3],caracter[5],caracter[6],caracter[7],caracter[8],caracter[9]); break;
            case 3: _printf("Norte(N)/Sur(S): %s\n", caracter); break;
            case 4: _printf("Longitud: %c%c.%c%c%c%c%c%c%c \n", caracter[1],caracter[2],caracter[3],caracter[4],caracter[6],caracter[7],caracter[8],caracter[9],caracter[10]); break;
            case 5: _printf("Este(E)/Oeste(W): %s\n", caracter); break;
            case 6: _printf("Status: %s\n", caracter); break;
            case 7: _printf("Satelites Usados: %s\n", caracter); break;
            case 9: _printf("Altitud: %s\n", caracter); break;
        }
        a++;
        if (*p == separador) p++;
    }
}

/**
 * @brief Obtiene y reformatea las coordenadas buscando activamente una trama GNGGA.
 * * Esta función sincroniza con el flujo NMEA para extraer Latitud y Longitud. 
 * Realiza una reestructuración de caracteres para convertir el formato nativo 
 * del GPS (DDMM.MMMMM) a un formato visual directo (DD.MMMMMM).
 * * @param lat Buffer donde se almacenará la latitud.
 * @param lat_len Tamaño del buffer de latitud.
 * @param lon Buffer donde se almacenará la longitud.
 * @param lon_len Tamaño del buffer de longitud.
 * @return int Retorna 1 si hay fix (señal válida), 0 en caso contrario.
 */
int GPS_LatLon(char *lat, int lat_len, char *lon, int lon_len) {
    static char last_lat[16] = "0";
    static char last_lon[16] = "0";
    char line[80];
    char caracter[20];
    char fix = '0';
    char c;
    int i = 0, field = 0, encontrado = 0, intentos = 0;

    while (!encontrado && intentos < 10) {
        intentos++;
        do { c = _getch1(); } while (c != '$');
        line[0] = '$';
        for (i = 1; i < 79; i++) {
            c = _getch1();
            if (c == '\n' || c == '\r') break;
            line[i] = c;
        }
        line[i] = '\0';
        if (line[1] == 'G' && line[2] == 'N' && line[3] == 'G' && line[4] == 'G' && line[5] == 'A') encontrado = 1;
    }

    if (encontrado) {
        i = 0; field = 0;
        while (line[i] != '\0' && field <= 6) {
            int t = 0;
            while (line[i] != ',' && line[i] != '\0') {
                if (t < 19) caracter[t++] = line[i];
                i++;
            }
            caracter[t] = '\0';

            if (field == 2 && t >= 5) {        
                last_lat[0] = caracter[0];
                last_lat[1] = caracter[1];
                last_lat[2] = '.';
                last_lat[3] = caracter[2];
                last_lat[4] = caracter[3];
                
                for(int k = 5; caracter[k] != '\0' && k < 15; k++) last_lat[k-1] = caracter[k];
                last_lat[t] = '\0'; 
            } 
            else if (field == 4 && t >= 6) {   
                last_lon[0] = caracter[1];    
                last_lon[1] = caracter[2];
                last_lon[2] = '.';
                last_lon[3] = caracter[3];
                last_lon[4] = caracter[4];
               
                for(int k = 6; caracter[k] != '\0' && k < 15; k++) last_lon[k-2] = caracter[k];
                last_lon[t-1] = '\0'; 
            } 
            else if (field == 6) fix = caracter[0];

            if (line[i] == ',') i++;
            field++;
        }
    }

    for (i = 0; i < lat_len - 1 && last_lat[i]; i++) lat[i] = last_lat[i];
    lat[i] = '\0';
    for (i = 0; i < lon_len - 1 && last_lon[i]; i++) lon[i] = last_lon[i];
    lon[i] = '\0';
    
    return (fix != '0');
}
