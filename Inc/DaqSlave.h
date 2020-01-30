#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_conf.h"
#include "stm32f1xx_it.h"
#include "usart.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"



#define bool	_Bool
#define true	1
#define false	0

#define CAL_ACOMMON_L 0xD0
#define CAL_ACOMMON_H 0xD1
#define CAL_ACP_L 0xD3
#define CAL_ACP_H 0xD4
#define CAL_BCP 0xD5
#define CAL_alphaCP_L 0xD6
#define CAL_alphaCP_H 0xD7
#define CAL_TGC 0xD8
#define CAL_AI_SCALE 0xD9
#define CAL_BI_SCALE 0xD9

#define VTH_L 0xDA
#define VTH_H 0xDB
#define KT1_L 0xDC
#define KT1_H 0xDD
#define KT2_L 0xDE
#define KT2_H 0xDF
#define KT_SCALE 0xD2

//Common sensitivity coefficients
#define CAL_A0_L 0xE0
#define CAL_A0_H 0xE1
#define CAL_A0_SCALE 0xE2
#define CAL_DELTA_A_SCALE 0xE3
#define CAL_EMIS_L 0xE4
#define CAL_EMIS_H 0xE5

//Config register = 0xF5-F6
#define OSC_TRIM_VALUE 0xF7


extern float read_max6675(SPI_HandleTypeDef *hspi, uint8_t cs_port, uint8_t cs_pin);
int32_t map_valor_Hx (uint32_t convert,uint32_t OFFSET);
uint32_t ReadCount_1();
uint32_t ReadCount_2();
uint32_t ReadCount_3();
bool checkConfig(void);
void lerEEPROM(void);
void writeTrimmingValue(void);
void setConfiguration(void);
uint16_t Pot_map(uint32_t leitura_Pot);
void transmit_dados(void);
int16_t readConfig(void);
void readPTAT(void);
void readCPIX(void);
void readIR(void);
