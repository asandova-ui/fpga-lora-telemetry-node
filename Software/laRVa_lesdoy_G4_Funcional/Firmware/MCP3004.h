
#include <stdint.h>

int MCP3004_Read(uint8_t channel);
int MCP3004_DifferentialRead(uint8_t channel);
void MCP3004_ReadMultiple(const uint8_t *channels, uint8_t numChannels, int *readings);

int MCP3004_Read_Reg(unsigned char channel);
