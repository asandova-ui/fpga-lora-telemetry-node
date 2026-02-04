//------------------------------------------------------------------------------
//            TIMER
// Temporizador hardware de 32 bits con generación de interrupciones.
// Utiliza un registro auxiliar para almacenar el valor máximo y 
// aplica un factor de escala (x18) para la cuenta.
//------------------------------------------------------------------------------

module TIMER(
  input clk,               // Señal de reloj del sistema
  input [31:0] maxcount,   // Valor de configuración para el límite de cuenta
  input rd,                // Señal de lectura (Clear on Read del flag)
  input wr,                // Señal de escritura (Carga de configuración y Reset)
  output reg flagtimer,    // Bandera de estado/interrupción (1 = Tiempo alcanzado)
  output reg [31:0] count  // Valor actual del contador
);

  // Registro interno para retener el valor máximo configurado
  reg [31:0] aux = 0;
  
  // Inicialización de registros
  initial
    begin 
      count = 0;
      flagtimer = 0;
    end

  // Lógica secuencial del temporizador
  always @(posedge clk)
  begin
    //---------------------------------------------
    // Escritura / Configuración
    //---------------------------------------------
    if (wr) 
    begin
      aux <= maxcount;     // Carga el nuevo límite en el registro interno
      count <= 0;          // Reinicia el contador
      flagtimer <= 0;      // Limpia cualquier interrupción pendiente
    end

    //---------------------------------------------
    // Funcionamiento del Contador
    //---------------------------------------------
    // El temporizador solo corre si se ha configurado un valor distinto de 0
    if(aux > 0)
    begin
      // Gestión de la bandera de interrupción (Clear on Read)
      if(rd) 
        flagtimer <= 0;

      // Comparación del contador con el límite escalado
      // Nota: El factor *18 ajusta la base de tiempos según la frecuencia de reloj
      if(count == aux * 18)
      begin
        count <= 0;        // Reinicio cíclico del contador
        flagtimer <= 1;    // Activación de la bandera de interrupción
      end
      else
        count <= (count + 1); // Incremento normal
      end
  end

endmodule
