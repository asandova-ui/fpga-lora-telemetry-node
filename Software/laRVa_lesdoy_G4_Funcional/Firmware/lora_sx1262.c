//--------------------------------------------------------------------------
// Taller de Proyectos 1
// Master de Ingeniería de Telecomunicaciones
// E.T.S.I.Telecomunicaciones, Universidad de Valladolid
// Jesús Arias Álvarez, Jesús M. Hernández Mangas
// Curso 2022-2023
// -------------------------------------------------------------------------
//Presets. These help make radio config easier
#define PRESET_DEFAULT    0
#define PRESET_LONGRANGE  1
#define PRESET_FAST       2

#define true (1)
#define false (0)

//These variables show signal quality, and are updated automatically whenever a packet is received
int      SX1262_rssi = 0;
int      SX1262_snr = 0;
int      SX1262_signalRssi = 0;

uint8_t  SX1262_spiBuff[32];   //Buffer for sending SPI commands to radio
uint32_t SX1262_pllFrequency;
uint8_t  SX1262_bandwidth;
uint8_t  SX1262_codingRate;
uint8_t  SX1262_spreadingFactor;
uint8_t  SX1262_lowDataRateOptimize;
uint32_t SX1262_transmitTimeout;
uint8_t  SX1262_inReceiveMode = false;

void     SX1262_transmit(uint8_t* data, int dataLen);
         
void     SX1262_setModeReceive();
int      SX1262_lora_receive_async(uint8_t* buff, int buffMaxLen);
int      SX1262_lora_receive_blocking(uint8_t* buff, int buffMaxLen, uint32_t timeout);
         
uint8_t  SX1262_configSetPreset(int preset);
uint8_t  SX1262_configSetFrequency(long frequencyInHz);
uint8_t  SX1262_configSetBandwidth(int bandwidth);
uint8_t  SX1262_configSetCodingRate(int codingRate);
uint8_t  SX1262_configSetSpreadingFactor(int spreadingFactor);
         
void     SX1262_updateModulationParameters();
uint8_t  SX1262_waitForRadioCommandCompletion(uint32_t timeout);
void     SX1262_updateRadioFrequency();

#define  SX1262_ReadRegister            (0x1D)
#define  SX1262_SetDIO2AsRfSwitchCtrl   (0x9D)
#define  SX1262_SetPacketType           (0x8A)
#define  SX1262_StopTimerOnPreamble     (0x9F)
#define  SX1262_SetPaConfig             (0x95)
#define  SX1262_SetTxParams             (0x8E)
#define  SX1262_SetLoRaSymbNumTimeout   (0xA0)
#define  SX1262_SetDioIrqParams         (0x08)
#define  SX1262_SetPacketParameters     (0x8C)
#define  SX1262_WriteBuffer             (0x0E)
#define  SX1262_ReadBuffer              (0x1E)
#define  SX1262_SetRx                   (0x82)
#define  SX1262_SetTx                   (0x83)
#define  SX1262_SetModulationParameters (0x8B)
#define  SX1262_GetRxBufferStatus       (0x13)
#define  SX1262_GetPacketStatus         (0x14)
#define  SX1262_ClearIRQStatus          (0x02)
#define  SX1262_GetStatus               (0xC0)

//int kk;
//#define delay(a) kk=get_time(); _delay_ms(a); _printf("delay %d %d\n", a, ((get_time()-kk)/(CCLK/1000));
#define delay(a) _delay_ms(a); 

// GPOUT
// #define LORA_RESET  (1<<10)
// #define L_RX        (1<< 9)
// #define L_TX        (1<< 8)

// GPIN
#define LORA_DIO1   (1<<1)
#define LORA_DIO0   (1<< 0)
 
#define LORA_BUSY   (GPIN & LORA_DIO0)

#define LORA_CS_1  SPI1SS = -1;	// LORA_CS alto
#define LORA_CS_0  while(LORA_BUSY); SPI1SS = ~1;	// LORA_CS bajo

#define LORA_RESET_1 GPOUT |=  LORA_RESET; // LORA_RESET alto
#define LORA_RESET_0 GPOUT &= ~LORA_RESET; // LORA_RESET bajo
#define LORA_RX_1    GPOUT |=  L_RX;       // LORA_RX  alto 
#define LORA_RX_0    GPOUT &= ~L_RX;       // LORA_RX  bajo
#define LORA_TX_1    GPOUT |=  L_TX;       // LORA_TX  alto
#define LORA_TX_0    GPOUT &= ~L_TX;       // LORA_TX  bajo

int iii;
#define memcpy(a,b,c) for(iii=0;iii<c;iii++) (a)[iii]=(b)[iii];
#define get_time() (TIMER) //(TIMER*1000)/CCLK  // debe  devolver ms
#define get_DIO1() (GPIN & 2) // revisar que el pin esté configurado correctamente
int debug = 0;

void SPI_TRANSFER_BUF(uint8_t *a, uint32_t b)
{ 
 if(debug)
 {
  _printf("\nSent/Response\n"); 
  for(int iii=0;iii<b;iii++){ _printf("%02x ", a[iii]); a[iii] = spi1xfer(a[iii]); }; 
  _printf("\n"); 
  for(int iii=0;iii<b;iii++){ _printf("%02x ", a[iii]); };
  _printf("\n");
 }
 else
 {
  for(int iii=0;iii<b;iii++) a[iii] = spi1xfer(a[iii]); 
 }
}

// -------------------------------------------------------------------------
uint8_t SX1262_Init()
{
 uint8_t d;
 //Set up SPI to talk to the LoRa Radio shield
 //Set I/O pins based on the configuration
 LORA_CS_1;
 
 MAX_COUNT = 0xFFFFFFFF;
 
 //Hardware reset the radio by toggling the reset pin 10 ms
 LORA_RESET_0;
 delay(10);
 LORA_RESET_1;
 delay(10);
 // Check module present and SPI working [0x0740] => 0x14 (LoRA Sync word MSB)
 LORA_CS_0;
 SX1262_spiBuff[0] = SX1262_ReadRegister;
 SX1262_spiBuff[1] = 0x07;
 SX1262_spiBuff[2] = 0x40;
 SX1262_spiBuff[3] = 0x00;
 SX1262_spiBuff[4] = 0x00;
 SPI_TRANSFER_BUF(SX1262_spiBuff,5);
 LORA_CS_1;
 if(!(SX1262_spiBuff[4] == 0x14)) return false;
 _printf("SX1262 present\n");
 
 //Tell DIO2 to control the RF switch so we don't have to do it manually **************************
 LORA_CS_0;
 SX1262_spiBuff[0] = SX1262_SetDIO2AsRfSwitchCtrl;
 SX1262_spiBuff[1] = 0x01;  //Enable
 SPI_TRANSFER_BUF(SX1262_spiBuff,2);
 LORA_CS_1;
 delay(100); //Give time for the radio to proces command
 
 SX1262_configSetFrequency(868000000);  // Default frequency
 //Set modem to LoRa (described in datasheet section 13.4.2)
 LORA_CS_0;
 SX1262_spiBuff[0] = SX1262_SetPacketType;
 SX1262_spiBuff[1] = 0x01;          //Packet Type: 0x00=GFSK, 0x01=LoRa
 SPI_TRANSFER_BUF(SX1262_spiBuff,2);
 LORA_CS_1;
 delay(100);                  //Give time for radio to process the command
 
 //Set Rx Timeout to reset on SyncWord or Header detection
 LORA_CS_0;
 SX1262_spiBuff[0] = SX1262_StopTimerOnPreamble;
 SX1262_spiBuff[1] = 0x00;    //Stop timer on:  0x00=SyncWord or header detection, 0x01=preamble detection  SPI_TRANSFER_BUF(SX1262_spiBuff,2);
 SPI_TRANSFER_BUF(SX1262_spiBuff,2);
 LORA_CS_1;
 delay(100);                  //Give time for radio to process the command
 
 //Set modulation parameters is just one more SPI command, but since it
 //is often called frequently when changing the radio config, it's broken up into its own function
 //SX1262_configSetPreset(PRESET_DEFAULT);  //Sets default modulation parameters
 SX1262_configSetPreset(PRESET_LONGRANGE);
 
 // Set PA Config
 // See datasheet 13.1.4 for descriptions and optimal settings recommendations
 LORA_CS_0;
 SX1262_spiBuff[0] = SX1262_SetPaConfig;
 SX1262_spiBuff[1] = 0x04;    //paDutyCycle. See datasheet, set in conjuntion with hpMax
 SX1262_spiBuff[2] = 0x07;    //hpMax.  Basically Tx power.  0x00-0x07 where 0x07 is max power
 SX1262_spiBuff[3] = 0x00;    //device select: 0x00 = SX1262, 0x01 = SX1261
 SX1262_spiBuff[4] = 0x01;    //paLut (reserved, always set to 1)
 SPI_TRANSFER_BUF(SX1262_spiBuff,5);
 LORA_CS_1;
 delay(100);                  //Give time for radio to process the command
 
 // Set TX Params
 // See datasheet 13.4.4 for details
 LORA_CS_0;
 SX1262_spiBuff[0] = SX1262_SetTxParams;
 SX1262_spiBuff[1] = 22;      //Power.  Can be -17(0xEF) to +14(0x0E) in Low Pow mode.  
                              //                -9(0xF7) to  22(0x16) in high power mode
 SX1262_spiBuff[2] = 0x02;    //Ramp time. Lookup table.  See table 13-41. 0x02="40uS"
 SPI_TRANSFER_BUF(SX1262_spiBuff,3);
 LORA_CS_1;
 delay(100);                  //Give time for radio to process the command
 
 //Set LoRa Symbol Number timeout
 //How many symbols are needed for a good receive.
 //Symbols are preamble symbols
 LORA_CS_0;
 SX1262_spiBuff[0] = SX1262_SetLoRaSymbNumTimeout;
 SX1262_spiBuff[1] = 0x00;    //Number of symbols.  Ping-pong example from Semtech uses 5
 SPI_TRANSFER_BUF(SX1262_spiBuff,2);
 LORA_CS_1;
 delay(100);                  //Give time for radio to process the command
 
  //Enable interrupts
 LORA_CS_0;
 SX1262_spiBuff[0] = SX1262_SetDioIrqParams;
 SX1262_spiBuff[1] = 0x00;   //IRQMask MSB.  IRQMask is "what interrupts are enabled"
 SX1262_spiBuff[2] = 0x02;   //IRQMask LSB    See datasheet table 13-29 for details
 SX1262_spiBuff[3] = 0xFF;   //DIO1 mask MSB.  Of the interrupts detected, which should be triggered on DIO1 pin
 SX1262_spiBuff[4] = 0xFF;   //DIO1 Mask LSB
 SX1262_spiBuff[5] = 0x00;   //DIO2 Mask MSB
 SX1262_spiBuff[6] = 0x00;   //DIO2 Mask LSB
 SX1262_spiBuff[7] = 0x00;   //DIO3 Mask MSB
 SX1262_spiBuff[8] = 0x00;   //DIO3 Mask LSB
 SPI_TRANSFER_BUF(SX1262_spiBuff,9);
 LORA_CS_1;
 delay(100);                  //Give time for radio to process the command
 
 return true;
}
// -------------------------------------------------------------------------
void SX1262_transmit(uint8_t *data, int dataLen)
{
  // Max lora packet size is 255 uint8_ts
  if (dataLen > 255) { dataLen = 255;}

  // Set packet parameters
  // # Tell LoRa what kind of packet we're sending (and how long)
  // # Parameters are:
  // # - Preamble Length MSB
  // # - Preamble Length LSB
  // # - Header Type (variable vs fixed len)
  // # - Payload Length
  // # - CRC on/off
  // # - IQ Inversion on/off
  LORA_RX_0;
  LORA_TX_1; 
  
  LORA_CS_0;
  SX1262_spiBuff[0] = SX1262_SetPacketParameters;
  SX1262_spiBuff[1] = 0x00;          //PacketParam1 = Preamble Len MSB
  SX1262_spiBuff[2] = 0x08;          //PacketParam2 = Preamble Len LSB
  SX1262_spiBuff[3] = 0x00;          //PacketParam3 = Header Type. 0x00 = Variable Len, 
                                     //                            0x01 = Fixed Length
  SX1262_spiBuff[4] = dataLen;       //PacketParam4 = Payload Length (Max is 255 uint8_ts)
  SX1262_spiBuff[5] = 0x00;          //PacketParam5 = CRC Type. 0x00 = Off, 0x01 = on
  SX1262_spiBuff[6] = 0x00;          //PacketParam6 = Invert IQ.  0x00 = Standard, 0x01 = Inverted
  SPI_TRANSFER_BUF(SX1262_spiBuff,7);
  LORA_CS_1;
  
  SX1262_waitForRadioCommandCompletion(100);  //Give time for radio to process the command

  // Write the payload to the buffer
  //  Reminder: PayloadLength is defined in setPacketParams
  LORA_CS_0;
  SX1262_spiBuff[0] = SX1262_WriteBuffer;
  SX1262_spiBuff[1] = 0x00;           //Dummy uint8_t before writing payload
  SPI_TRANSFER_BUF(SX1262_spiBuff,2); //Send header info

  // SPI_TRANSFER_BUF overwrites original buffer.  This could probably be confusing to the user
  // If they tried writing the same buffer twice and got different results
  uint8_t size = sizeof(SX1262_spiBuff);
  for(uint16_t i = 0; i < dataLen; i += size) 
  {
    if (i + size > dataLen) { size = dataLen - i; }
    memcpy(SX1262_spiBuff,&(data[i]),size);
    SPI_TRANSFER_BUF(SX1262_spiBuff,size); //Write the payload itself
  }
  LORA_CS_1;

  SX1262_waitForRadioCommandCompletion(1000);   //Give time for radio to process the command

  //Transmit!
  // An interrupt will be triggered if we surpass our timeout
  LORA_CS_0;
  SX1262_spiBuff[0] = SX1262_SetTx;
  SX1262_spiBuff[1] = 0xFF;          //Timeout (3-uint8_t number)
  SX1262_spiBuff[2] = 0xFF;          //Timeout (3-uint8_t number)
  SX1262_spiBuff[3] = 0xFF;          //Timeout (3-uint8_t number)
  SPI_TRANSFER_BUF(SX1262_spiBuff,4);
  LORA_CS_1;

  SX1262_waitForRadioCommandCompletion(SX1262_transmitTimeout); //Wait for tx to complete, with a timeout so we don't wait forever

  //Remember that we are in Tx mode.  If we want to receive a packet, we need to switch into receiving mode
  SX1262_inReceiveMode = false;
}
// -------------------------------------------------------------------------
// This command will wait until the radio reports that it is no longer busy.
// This is useful when waiting for commands to finish that take a while such as transmitting packets.
// Specify a timeout (in milliseconds) to avoid an infinite loop if something happens to the radio
// Returns TRUE on success, FALSE if timeout hit
// -------------------------------------------------------------------------
uint8_t SX1262_waitForRadioCommandCompletion(uint32_t timeout)
{
 uint32_t startTime = get_time();
 uint8_t dataTransmitted = false;

 //Keep checking radio status until it has completed
 while (!dataTransmitted)
 {
  //Wait some time between spamming SPI status commands, asking if the chip is ready yet
  //Some commands take a bit before the radio even changes into a busy state,
  //so if we check too fast we might pre-maturely think we're done processing the command
  //3ms delay gives inconsistent results.  4ms seems stable.  Using 5ms to be safe
  delay(5);

  //Ask the radio for a status update
  LORA_CS_0;
  SX1262_spiBuff[0] = SX1262_GetStatus;  //Opcode for "getStatus" command
  SX1262_spiBuff[1] = 0x00;              //Dummy uint8_t, status will overwrite this uint8_t
  SPI_TRANSFER_BUF(SX1262_spiBuff,2);
  LORA_CS_1;

  //Parse out the status (see datasheet for what each bit means)
  uint8_t chipMode      = (SX1262_spiBuff[1] >> 4) & 0x7;     //Chip mode is bits [6:4] (3-bits)
  uint8_t commandStatus = (SX1262_spiBuff[1] >> 1) & 0x7;//Command status is bits [3:1] (3-bits)

  if(debug) _printf("chip mode = %d, status = %d\n", chipMode, commandStatus);
  //Status 0, 1, 2 mean we're still busy.  Anything else means we're done.
  //Commands 3-6 = command timeout, command processing error, failure to execute command, and Tx Done (respoectively)
  if (commandStatus != 0 && commandStatus != 1 && commandStatus != 2) { dataTransmitted = true; }

  //If we're in standby mode, we don't need to wait at all
  //0x03 = STBY_XOSC, 0x02= STBY_RC
  if (chipMode == 0x03 || chipMode == 0x02) { dataTransmitted = true; }
  
  //Avoid infinite loop by implementing a timeout
  if (((get_time() - startTime)/(CCLK/1000)) >= timeout) 
  {
  // _printf("Timeout %6d \n",timeout); 
   return false; 
  }
 }
 
 return true;
}
// -------------------------------------------------------------------------
//Sets the radio into receive mode, allowing it to listen for incoming packets.
//If radio is already in receive mode, this does nothing.
//There's no such thing as "setModeTransmit" because it is set automatically when transmit() is called
// -------------------------------------------------------------------------
void SX1262_setModeReceive()
{
  LORA_RX_1;
  LORA_TX_0; 

  if (SX1262_inReceiveMode) { return; }  //We're already in receive mode, this would do nothing
 //_printf("Set receivemode\n");
  //Set packet parameters
  LORA_CS_0;
  SX1262_spiBuff[0] = SX1262_SetPacketParameters;
  SX1262_spiBuff[1] = 0x00;          //PacketParam1 = Preamble Len MSB
  SX1262_spiBuff[2] = 0x08;          //PacketParam2 = Preamble Len LSB 
  SX1262_spiBuff[3] = 0x00;          //PacketParam3 = Header Type. 0x00 = Variable Len, 0x01 = Fixed Length
  SX1262_spiBuff[4] = 0xFF;          //PacketParam4 = Payload Length (Max is 255 uint8_ts)
  SX1262_spiBuff[5] = 0x00;          //PacketParam5 = CRC Type. 0x00 = Off, 0x01 = on
  SX1262_spiBuff[6] = 0x00;          //PacketParam6 = Invert IQ.  0x00 = Standard, 0x01 = Inverted
  SPI_TRANSFER_BUF(SX1262_spiBuff,7);
  LORA_CS_1;
  SX1262_waitForRadioCommandCompletion(100);
  
  // Tell the chip to wait for it to receive a packet.
  // Based on our previous config, this should throw an interrupt when we get a packet
  LORA_CS_0;
  SX1262_spiBuff[0] = SX1262_SetRx;
  SX1262_spiBuff[1] = 0xFF;          //24-bit timeout, 0xFFFFFF means no timeout
  SX1262_spiBuff[2] = 0xFF;          // ^^
  SX1262_spiBuff[3] = 0xFF;          // ^^
  SPI_TRANSFER_BUF(SX1262_spiBuff,4);
  LORA_CS_1;
  SX1262_waitForRadioCommandCompletion(100);
  
  //Remember that we're in receive mode so we don't need to run this code again unnecessarily
  SX1262_inReceiveMode = true;
}

/*Receive a packet if available
If available, this will return the size of the packet and store the packet contents into the user-provided buffer.
A max length of the buffer can be provided to avoid buffer overflow.  If buffer is not large enough for entire payload, overflow is thrown out.
Recommended to pass in a buffer that is 255 uint8_ts long to make sure you can received any lora packet that comes in.

Returns -1 when no packet is available.
Returns 0 when an empty packet is received (packet with no payload)
Returns payload size (1-255) when a packet with a non-zero payload is received. If packet received is larger than the buffer provided, this will return buffMaxLen
*/
// -------------------------------------------------------------------------
int SX1262_lora_receive_async(uint8_t* buff, int buffMaxLen)
{
 SX1262_setModeReceive(); //Sets the mode to receive (if not already in receive mode)

 //Radio pin DIO1 (interrupt) goes high when we have a packet ready.  If it's low, there's no packet yet
 if (get_DIO1() == false) { return -1; } //Return -1, meanining no packet ready
 //_printf("DIO1 asserted\n");
 //Tell the radio to clear the interrupt, and set the pin back inactive.
 while (get_DIO1())
 {
  //Clear all interrupt flags.  This should result in the interrupt pin going low
  LORA_CS_0;
  SX1262_spiBuff[0] = SX1262_ClearIRQStatus ;
  SX1262_spiBuff[1] = 0xFF;          //IRQ bits to clear (MSB) (0xFFFF means clear all interrupts)
  SX1262_spiBuff[2] = 0xFF;          //IRQ bits to clear (LSB)
  SPI_TRANSFER_BUF(SX1262_spiBuff,3);
  LORA_CS_1;
 }

 // (Optional) Read the packet status info from the radio.
 // This is things like radio strength, noise, etc.
 // See datasheet 13.5.3 for more info
 // This provides debug info about the packet we received
 LORA_CS_0;
 SX1262_spiBuff[0] = SX1262_GetPacketStatus;
 SX1262_spiBuff[1] = 0xFF;          //Dummy uint8_t. Returns status
 SX1262_spiBuff[2] = 0xFF;          //Dummy uint8_t. Returns rssi
 SX1262_spiBuff[3] = 0xFF;          //Dummy uint8_t. Returns snr
 SX1262_spiBuff[4] = 0xFF;          //Dummy uint8_t. Returns signal RSSI
 SPI_TRANSFER_BUF(SX1262_spiBuff,5);
 LORA_CS_1;

 //Store these values as class variables so they can be accessed if needed
 //Documentation for what these variables mean can be found in the .h file
 SX1262_rssi       = -((int   )SX1262_spiBuff[2]) / 2; //"Average over last packet received of RSSI. Actual signal power is –RssiPkt/2 (dBm)"
 SX1262_snr        =  ((int8_t)SX1262_spiBuff[3]) / 4; //SNR is returned as a SIGNED uint8_t, so we need to do some conversion first
 SX1262_signalRssi = -((int   )SX1262_spiBuff[4]) / 2;

 _printf("rssi %4d, snr %4d :", SX1262_rssi, SX1262_snr);
 //We're almost ready to read the packet from the radio
 //But first we have to know how big the packet is, and where in the radio memory it is stored
 LORA_CS_0;
 SX1262_spiBuff[0] = SX1262_GetRxBufferStatus;
 SX1262_spiBuff[1] = 0xFF;          //Dummy.  Returns radio status
 SX1262_spiBuff[2] = 0xFF;          //Dummy.  Returns loraPacketLength
 SX1262_spiBuff[3] = 0xFF;          //Dummy.  Returns memory offset (address)
 SPI_TRANSFER_BUF(SX1262_spiBuff,4);
 LORA_CS_1;

 uint8_t payloadLen   = SX1262_spiBuff[2]; //How long the lora packet is
 uint8_t startAddress = SX1262_spiBuff[3]; //Where in 1262 memory is the packet stored
 
 //Make sure we don't overflow the buffer if the packet is larger than our buffer
 if (buffMaxLen < payloadLen) {payloadLen = buffMaxLen;}

 //Read the radio buffer from the SX1262 into the user-supplied buffer
 LORA_CS_0;
 SX1262_spiBuff[0] = SX1262_ReadBuffer;
 SX1262_spiBuff[1] = startAddress;  //SX1262 memory location to start reading from
 SX1262_spiBuff[2] = 0x00;          //Dummy uint8_t
 SPI_TRANSFER_BUF(SX1262_spiBuff,3);    //Send commands to get read started
 SPI_TRANSFER_BUF(buff,payloadLen);  //Get the contents from the radio and store it into the user provided buffer
 LORA_CS_1;

 return payloadLen;  //Return how many uint8_ts we actually read
}

/*Waits for a packet to come in.  This code will block until something is received, or the timeout is hit.

Set timeout=0 for no timeout, or set to a positive integer to specify a timeout in milliseconds
This will store the contents of the payload into the user-provided buffer.  Recommended to use a buffer with 255 uint8_ts to always receive full packet.
If a smaller buffer is used, maxBuffLen can be set to avoid buffer overflow. Any packets larger than this will have remaining uint8_ts thrown out.

Returns -1 when no packet is available and timeout was hit.
Returns 0 when an empty packet is received (packet with no payload)
Returns payload size (1-255) when a packet with a non-zero payload is received. If packet received is larger than the buffer provided, this will return buffMaxLen
*/
// -------------------------------------------------------------------------
int SX1262_lora_receive_blocking(uint8_t *buff, int buffMaxLen, uint32_t timeout)
{
 SX1262_setModeReceive(); //Sets the mode to receive (if not already in receive mode)

 uint32_t startTime = get_time();
 uint32_t elapsed = startTime;

 //Wait for radio interrupt pin to go high, indicating a packet was received, or if we hit our timeout
 while (get_DIO1() == false)
 {
  //If user specified a timeout, check if we hit it
  if (timeout > 0)
  {
   elapsed = (get_time() - startTime)/(CCLK/1000);
   if (elapsed >= timeout) { return -1; }
  }
 }
 //If our pin went high, then we got a packet!  Return it
 return SX1262_lora_receive_async(buff,buffMaxLen);
}

//Set the radio modulation parameters.
//This is things like bandwitdh, spreading factor, coding rate, etc.
//This is broken into its own function because this command might get called frequently
// -------------------------------------------------------------------------
void SX1262_updateModulationParameters()
{
  /*Set modulation parameters
  # Modulation parameters are:
  #  - SpreadingFactor
  #  - Bandwidth
  #  - CodingRate
  #  - LowDataRateOptimize
  # None of these actually matter that much.  You can set them to anything, and data will still show up
  # on a radio frequency monitor.
  # You just MUST call "setModulationParameters", otherwise the radio won't work at all*/
  LORA_CS_0;      
  SX1262_spiBuff[0] = SX1262_SetModulationParameters;
  SX1262_spiBuff[1] = SX1262_spreadingFactor;     //ModParam1 = Spreading Factor.  Can be SF5-SF12, written in hex (0x05-0x0C)
  SX1262_spiBuff[2] = SX1262_bandwidth;           //ModParam2 = Bandwidth.  See Datasheet 13.4.5.2 for details. 0x00=7.81khz (slowest)
  SX1262_spiBuff[3] = SX1262_codingRate;          //ModParam3 = CodingRate.  Semtech recommends CR_4_5 (which is 0x01).  Options are 0x01-0x04, which correspond to coding rate 5-8 respectively
  SX1262_spiBuff[4] = SX1262_lowDataRateOptimize; //LowDataRateOptimize.  0x00 = 0ff, 0x01 = On.  Required to be on for SF11 + SF12
  SPI_TRANSFER_BUF(SX1262_spiBuff,5);
  LORA_CS_1;
  delay(100);                  //Give time for radio to process the command

  //Come up with a reasonable timeout for transmissions
  //SF12 is painfully slow, so we want a nice long timeout for that,
  //but we really don't want someone using SF5 to have to wait MINUTES for a timeout
  //I came up with these timeouts by measuring how long it actually took to transmit a packet
  //at each spreading factor with a MAX 255-uint8_t payload and 7khz Bandwitdh (the slowest one)
  switch (SX1262_spreadingFactor) {
    case 12:
      SX1262_transmitTimeout = 252000; //Actual tx time 126 seconds
      break;
    case 11:
      SX1262_transmitTimeout = 160000; //Actual tx time 81 seconds
      break;
    case 10:
      SX1262_transmitTimeout = 60000; //Actual tx time 36 seconds
      break;
    case 9:
      SX1262_transmitTimeout = 40000; //Actual tx time 20 seconds
      break;
    case 8:
      SX1262_transmitTimeout = 20000; //Actual tx time 11 seconds
      break;
    case 7:
      SX1262_transmitTimeout = 12000; //Actual tx time 6.3 seconds
      break;
    case 6:
      SX1262_transmitTimeout = 7000; //Actual tx time 3.7s seconds
      break;
    default:  //SF5
      SX1262_transmitTimeout = 5000; //Actual tx time 2.2 seconds
      break;
  }
}

//--------------------------
// ADVANCED FUNCTIONS
//--------------------------
//The functions below are intended for advanced users who are more familiar with LoRa Radios at a lower level
//(Optional) Use one of the pre-made radio configurations
// This is ideal for making simple changes to the radio config
// without needing to understand how the underlying settings work
//
// Argument: pass in one of the following
//     - PRESET_DEFAULT:   Default radio config.
//                         Medium range, medium speed
//     - PRESET_FAST:      Faster speeds, but less reliable at long ranges.
//                         Use when you need fast data transfer and have radios close together
//     - PRESET_LONGRANGE: Most reliable option, but slow. Suitable when you prioritize
//                         reliability over speed, or when transmitting over long distances
//
// -------------------------------------------------------------------------
uint8_t SX1262_configSetPreset(int preset)
{
  if (preset == PRESET_DEFAULT) {
      /*
    SX1262_bandwidth = 5;            //250khz
    SX1262_codingRate = 1;           //CR_4_5
    SX1262_spreadingFactor = 7;      //SF7
    SX1262_lowDataRateOptimize = 0;  //Don't optimize (used for SF12 only)
    */
    SX1262_bandwidth = 4;            //125khz Match LilyGo LoRa
    SX1262_codingRate = 1;           //CR_4_5
    SX1262_spreadingFactor = 7;      //SF7
    SX1262_lowDataRateOptimize = 0;  //Don't optimize (used for SF12 only)
    SX1262_updateModulationParameters();
    return true;
  }

  if (preset == PRESET_LONGRANGE) {
    SX1262_bandwidth = 4;            //125khz
    SX1262_codingRate = 1;           //CR_4_5
    SX1262_spreadingFactor = 12;     //SF12
    SX1262_lowDataRateOptimize = 1;  //Optimize for low data rate (SF12 only)
    SX1262_updateModulationParameters();
    return true;
  }

  if (preset == PRESET_FAST) {
    SX1262_bandwidth = 6;            //500khz
    SX1262_codingRate = 1;           //CR_4_5
    SX1262_spreadingFactor = 5;      //SF5
    SX1262_lowDataRateOptimize = 0;  //Don't optimize (used for SF12 only)
    SX1262_updateModulationParameters();
    return true;
  }

  //Invalid preset specified
  return false;
}
// Set the operating frequency of the radio.
// The 1262 radio supports 150-960Mhz.
// -------------------------------------------------------------------------
uint8_t SX1262_configSetFrequency(long frequencyInHz)
{
 if (frequencyInHz < 150000000 || frequencyInHz > 960000000) { return false;}
 // PLL frequency (See datasheet section 13.4.1 for calculation)
 SX1262_pllFrequency = 910163968UL; // para 868 MHz //SX1262_frequencyToPLL(frequencyInHz);

 //Set PLL frequency (this is a complicated math equation.  See datasheet entry for SetRfFrequency)
 LORA_CS_0;
 SX1262_spiBuff[0] = 0x86;  //Opcode for set RF Frequencty
 SX1262_spiBuff[1] = (SX1262_pllFrequency >> 24) & 0xFF;  //MSB of pll frequency
 SX1262_spiBuff[2] = (SX1262_pllFrequency >> 16) & 0xFF;  //
 SX1262_spiBuff[3] = (SX1262_pllFrequency >>  8) & 0xFF;  //
 SX1262_spiBuff[4] = (SX1262_pllFrequency >>  0) & 0xFF;  //LSB of requency
 SPI_TRANSFER_BUF(SX1262_spiBuff,5);
 LORA_CS_1;
 delay(100);

 return true;
}

// Set the bandwith (basically, this is how big the frequency span is that we occupy)
// Bigger bandwidth allows us to transmit large amounts of data faster, but it occupies a larger span of frequencies.
// Smaller bandwith takes longer to transmit large amounts of data, but its less likely to collide with other frequencies.
// /Available bandwidth settings, pulled from datasheet 13.4.5.2
//  SETTING.   | Bandwidth
// ------------+-----------
//    0x00     |    7.81khz
//    0x08     |   10.42khz
//    0x01     |   15.63khz
//    0x09     |   20.83khz
//    0x02     |   31.25khz
//    0x0A     |   41.67khz
//    0x03     |   62.50khz
//    0x04     |  125.00khz
//    0x05     |  250.00khz (default)
//    0x06     |  500.00khz
// /Returns TRUE on success, FALSE on failure (invalid bandwidth)
//
// -------------------------------------------------------------------------
uint8_t SX1262_configSetBandwidth(int bandwidth)
{
  //Bandwidth setting must be 0-10 (excluding 7 for some reason)
  if (bandwidth < 0 || bandwidth > 0x0A || bandwidth == 7) { return false; }
  SX1262_bandwidth = bandwidth;
  SX1262_updateModulationParameters();
  return true;
}

// I honestly don't really know what coding rate means.  It's something technical to have to do with radios
// Set it here if you want.  See datasheet 13.4.5.2 for details
//  SETTING. | Coding Rate
// ----------+--------------------
//    0x01   |   CR_4_5 (default)
//    0x02   |   CR_4_6
//    0x03   |   CR_4_7
//    0x04   |   CR_4_8
// Returns TRUE on success, FALSE on failure (invalid coding rate)
//
// -------------------------------------------------------------------------
uint8_t SX1262_configSetCodingRate(int codingRate)
{
  //Coding rate must be 1-4 (inclusive)
  if (codingRate < 1 || codingRate > 4) { return false; }
  SX1262_codingRate = codingRate;
  SX1262_updateModulationParameters();
  return true;
}

// Change the spreading factor of a packet
// The higher the spreading factor, the slower and more reliable the transmission will be.
// Higher spreading factors are good for longer distances with slower transmit speeds.
// Lower spreading factors are good when the radios are close, which allows faster transmission speeds.
//
// Setting | Spreading Factor
// --------+---------------------------
//    5    | SF5 (fastest, short range)
//    6    | SF6
//    7    | SF7 (default)
//    8    | SF8
//    9    | SF9
//   10    | SF10
//   11    | SF11
//   12    | SF12 (Slowest, long range, most reliable)
//
// Returns TRUE on success, FALSE on failure (incorrect spreading factor)
//
// -------------------------------------------------------------------------
uint8_t SX1262_configSetSpreadingFactor(int spreadingFactor)
{
  if (spreadingFactor < 5 || spreadingFactor > 12) { return false; }

  //The datasheet highly recommends enabling "LowDataRateOptimize" for SF11 and SF12
  SX1262_lowDataRateOptimize = (spreadingFactor >= 11) ? 1 : 0;  //Turn on for SF11+SF12, turn off for anything else
  SX1262_spreadingFactor = spreadingFactor;
  SX1262_updateModulationParameters();
  return true;
}

// Convert a frequency in hz to the respective PLL setting.
// The radio requires that we set the PLL, which controls the multipler on
// the internal clock to achieve the desired frequency.
// Valid frequencies are 150mhz to 960mhz
// /NOTE: This assumes the radio is using a 32mhz clock, which is standard.
// See datasheet section 13.4.1 for this calculation.
//
// -------------------------------------------------------------------------
// Uses libc
// 868 MHz
// q = 868000000 / 15625 = 55552
// r = 860000000 % 15625 = 0
// q = q*16384 = 55552*16384 = 910163968
// r = r*16384 =     0*16384 = 0
// return q + (r / 15625UL) = 910163968
/*
uint32_t SX1262_frequencyToPLL(long rfFreq)
{
 // Datasheet Says:
 //		rfFreq = (pllFreq * xtalFreq) / 2^25
 // Rewrite to solve for pllFreq
 //		pllFreq = (2^25 * rfFreq)/xtalFreq
 //
 //	In our case, xtalFreq is 32mhz
 //	pllFreq = (2^25 * rfFreq) / 32000000
 //
 //Basically, we need to do "return ((1 << 25) * rfFreq) / 32000000L"
 //It's very important to perform this without losing precision or integer overflow.
 //If arduino supported 64-bit varibales (which it doesn't), we could just do this:
 //    uint64_t firstPart = (1 << 25) * (uint64_t)rfFreq;
 //    return (uint32_t)(firstPart / 32000000L);
 //
 //Instead, we need to break this up mathimatically to avoid integer overflow
 //First, we'll simplify the equation by dividing both parts by 2048 (2^11)
 //    ((1 << 25) * rfFreq) / 32000000L      -->      (16384 * rfFreq) / 15625;
 //
 // Now, we'll divide first, then multiply (multiplying first would cause integer overflow)
 // Because we're dividing, we need to keep track of the remainder to avoid losing precision
 uint32_t q = rfFreq / 15625UL;  //Gives us the result (quotient), rounded down to the nearest integer
 uint32_t r = rfFreq % 15625UL;  //Everything that isn't divisible, aka "the part that hasn't been divided yet"
  //Multiply by 16384 to satisfy the equation above
 q *= 16384UL;
 r *= 16384UL; //Don't forget, this part still needs to be divided because it was too small to divide before

 return q + (r / 15625UL);  //Finally divide the the remainder part before adding it back in with the quotient
}
*/

// -------------------------------------------------------------------------
