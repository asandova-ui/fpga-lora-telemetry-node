#pragma once

#include <stdint.h>

#define PRESET_DEFAULT    0
#define PRESET_LONGRANGE  1
#define PRESET_FAST       2

// SPI helper from SPI.c
unsigned char spixfer1(unsigned char d);
#ifndef spi1xfer
#define spi1xfer spixfer1
#endif

extern int SX1262_rssi;
extern int SX1262_snr;
extern int SX1262_signalRssi;

uint8_t SX1262_Init(void);
void SX1262_transmit(uint8_t *data, int dataLen);
int SX1262_lora_receive_async(uint8_t *buff, int buffMaxLen);
int SX1262_lora_receive_blocking(uint8_t *buff, int buffMaxLen, uint32_t timeout);

uint8_t SX1262_configSetPreset(int preset);
uint8_t SX1262_configSetFrequency(long frequencyInHz);
uint8_t SX1262_configSetBandwidth(int bandwidth);
uint8_t SX1262_configSetCodingRate(int codingRate);
uint8_t SX1262_configSetSpreadingFactor(int spreadingFactor);
