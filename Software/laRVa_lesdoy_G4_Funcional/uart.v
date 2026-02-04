
//---------------------------------------------
//		Core de UART
//	- Doble función de transmisor y receptor
//---------------------------------------------

module UART_CORE(
  output txd,    // Salida TX
  output tend,	 // Flag TX completa
  output thre,   // Flag Buffer TX vacío
  input [15:0]d, // Datos TX,BRG
  input wrtx,    // Escritura en TX
  input wrbaud,	 // Escritura en BRG
  output [7:0]q, // Datos RX
  output dv,     // Flag dato RX válido
  output fe,     // Flag Framing Error
  output ove,    // Flag Overrun
  input rxd,     // Entrada RX
  input rd,      // Lectura RX (borra DV)
  input clk
);

// Tamaño del registro divisor (define el rango de velocidades)
parameter BAUDBITS=9;

//////// Generación de Baud Rate (BRG) /////////
reg [BAUDBITS-1:0]divider=0;
// El divisor determina cuántos ciclos de reloj dura un solo bit
always @(posedge clk) if (wrbaud) divider<=d[BAUDBITS-1:0];

/*-----------------------------------------------------
---                    TRANSMISOR                   ---
-----------------------------------------------------*/

reg [7:0]thr;		// Buffer TX
reg thre=1;			// Estado THR 1: vacío, 0: con dato
reg [8:0]shtx=9'h1FF;	// Reg. desplazamiento de 9 bits (8 bits de datos + START)
reg [3:0]cntbit;	// Contador de bits transmitidos (0 a 9)
reg rdy=1;			// Estado reg. despl. (1==idle)

// Divisor de TX: Genera el pulso 'clko' para mover cada bit
reg [BAUDBITS-1:0] divtx=0;
wire clko;			
assign clko = (divtx==0);   // Pulso activo cuando el contador llega a cero

always @ (posedge clk) 
    // Si escribimos y estamos en IDLE, reiniciamos el tiempo; si no, decrementamos
    divtx <= (wrtx&rdy) ? 0 : (clko ? divider: divtx-1);

always @(posedge clk)
begin
    if (wrtx) begin      // El procesador pone un dato en THR
    `ifdef SIMULATION
            $write ("%c",d&255); $fflush ( );
    `endif
        thr<=d[7:0];
        thre<=1'b0;     // Marcamos que el buffer está ocupado
    end
    
    // Si el transmisor está libre y el buffer tiene un dato esperando
    if(rdy&(~thre)) begin      // Carga de reg. desp
        rdy<=1'b0;             // El transmisor pasa a estar ocupado
        thre<=1'b1;            // Liberamos el buffer para que pueda escribir el siguiente byte
        shtx<={thr[7:0],1'b0}; // Cargamos el dato y añadimos el '0' de START al principio
        cntbit<=4'b0000;
    end

    // En cada tiempo de bit
    if (clko) begin 
        if(~rdy) begin             // Desplazamos a la derecha e insertamos '1' (reposo) por la izquierda
            shtx<={1'b1,shtx[8:1]};
            cntbit<=cntbit+1;
            // Cuando llegamos a 9 (Start + 8 datos), volvemos a reposo
            if (cntbit[3]&cntbit[0]) rdy<=1'b1; 
        end
    end
end

assign txd = shtx[0];   // El bit de salida siempre es el bit menos significativo del registro
assign tend = thre&rdy; // Transmisión completa solo si buffer y registro están libres

/*-----------------------------------------------------
---                    RECEPTOR                     ---
-----------------------------------------------------*/

/// Sincronismo de reloj y detección de flancos
reg [1:0]rrxd=0;        // Registro de doble etapa para evitar metaestabilidad en señales externas
wire resinc;            // Indica cualquier cambio en la línea RXD
wire falling;           // Indica el inicio de un bit de START (flanco de bajada)
always @(posedge clk) rrxd<={rrxd[0],rxd};
assign resinc = rrxd[0]^rrxd[1];
assign falling = (~rrxd[0])&rrxd[1];

/// Divisor de Recepción
// Genera un pulso en mitad de la cuenta (centro de bit) y se reinicia con resinc
reg [BAUDBITS-1:0] divrx=0;
wire shift;		// Pulso para capturar el bit (ocurre en el CENTRO del bit)
wire clki0;     // Pulso para recargar el contador
// 'shift' se activa a la mitad de la cuenta para muestrear donde el dato es más estable
assign shift = (divrx=={1'b0,divider[BAUDBITS-1:1]});
assign clki0= (divrx==0);

// El receptor se resincroniza con cada flanco para evitar acumular errores de tiempo
always @ (posedge clk) divrx <= (resinc|clki0) ? divider: divrx-1;

reg dv=0;               // Dato válido si 1
reg ove=0;              // Overrun
reg [8:0]shrx;          // Reg. desplazamiento entrada (9 bits para stop)
reg [7:0]rbr;           // Buffer RX
reg stopb;              // Bit de stop recibido
reg [3:0]cbrx=4'b1111;  // Contador de bits / estado (1111== idle)
wire rxst;              // Guardar shr en buffer si flanco subida
assign rxst=(cbrx==4'b1111);
reg rxst0;              // Para detectar flanco

always @(posedge clk)
begin

    rxst0<=rxst;
    // Si detectamos flanco de bajada en IDLE, empezamos a recibir 9 bits
    if (rxst & falling) cbrx<=4'h9;

    // Si estamos en el centro de un bit y no estamos en IDLE
    if (shift & (~rxst)) begin       
        shrx<= #1 {rrxd[0],shrx[8:1]};  // Desplazamos e insertamos el bit actual
        cbrx<=cbrx-1;                  // Decrementamos contador de bits
    end

    // Al terminar la cuenta (flanco de subida interno de rxst)
    if (rxst & (~rxst0)) begin   // Final de cuenta
        {stopb,rbr}<=shrx;       // Guardando dato y bit STOP
        dv<=1;                   // Dato válido
        ove<=dv;                 // Overrun si ya hay dato válido
    end

    // Cuando se lee el registro se borran los flancos
    if (rd) begin   
        dv<=0;
        ove<=0;
    end
end

assign fe=~stopb;   // el Flag FE es el bit de STOP invertido
assign q = rbr;
endmodule

