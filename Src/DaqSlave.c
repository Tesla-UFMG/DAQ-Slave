#include "DaqSlave.h"
#define DEBUG 1

CAN_HandleTypeDef hcan;
CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

const uint8_t refreshRate=8;
int16_t irData[64];
float temperatures[64];
uint8_t eepromData[260] = {0};
int16_t k_t1_scale=0, k_t2_scale=0, resolution=0,configuration=0,cpix=0;
uint16_t  ptat=0;
float Tambient=0;
int16_t IRmedia[16];
uint32_t convert_1;
uint32_t OFFSET_1=0;
uint32_t OFFSET_2=0;
uint32_t OFFSET_3=0;
uint16_t leitura_PotInt;
uint16_t leitura_BetinaInt;
uint8_t vetTx[8];
uint8_t vetTy[8];
uint8_t vetTz[8];
uint8_t vetTv[8];
uint8_t vetTn[8];
uint8_t vetTu[8];
uint8_t vetTm[8];
uint8_t vetTt[8];
int32_t Dado_1 = 1;
int32_t Dado_2 = 1;
int32_t Dado_3 = 1;
uint16_t MLXir = 0x60;
uint16_t MLXEEPROM = 0x50;
I2C_HandleTypeDef hi2c2;
uint8_t buffer[10] = {0};
uint8_t bigbuff[260] = {0};
uint8_t input;
int16_t IRCan[16] = {0};
int16_t IRcan0 = 1;
int16_t IRcan1 = 1;
int16_t IRcan2 = 1;
int16_t IRcan3 = 1;
int16_t IRcan4 = 1;
int16_t IRcan5 = 1;
int16_t IRcan6 = 1;
int16_t IRcan7 = 1;
int16_t IRcan8 = 1;
int16_t IRcan9 = 1;
int16_t IRcan10 = 1;
int16_t IRcan11 = 1;
int16_t IRcan12 = 1;
int16_t IRcan13 = 1;
int16_t IRcan14 = 1;
int16_t IRcan15 = 1;
uint16_t Pot;
uint32_t ext1 = 1;
uint16_t temp = 1;


/*extern float read_max6675(SPI_HandleTypeDef *hspi, uint8_t cs_port, uint8_t cs_pin)
{
    uint8_t data[2]; //create and initialize data container
    data[0] = 0;
    data[1] = 0;
    uint32_t timeout = 255;                   //255ms timeout on spi bus
    uint16_t n_bytes = 2;                     //data consists over 2 8-bits(byte)
    uint16_t combined_bit = 0x00;             //combination of both bytes
    HAL_StatusTypeDef status_hal = HAL_ERROR; //finding out whether spi-bus is working properly (HAL_OK)
    //select chip by pulling cs line down
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(1); // timing requirement for chip needed 100ns time before applying clock this is 1ms
    //request data over spi-bus
    status_hal = HAL_SPI_Receive(hspi, data, n_bytes, timeout);

    //deselect chip by pulling cs line up
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

    //combine msb and lsb to 16-bit
    combined_bit = (data[0] << 8) | data[1];

    //right shift 3 digits
    uint16_t temp_raw = combined_bit >> 3;

    //conversion
    temp_raw = temp_raw - 1;
    float temp_aux = (float)(temp_raw * 0.25);

    return temp_aux;
}*/

int32_t map_valor_Hx(uint32_t convert, uint32_t OFFSET)
{
    int32_t dif1 = (convert - OFFSET);
    return dif1;
}

uint32_t ReadCount(GPIO_TypeDef *HxCLOCK, uint16_t HxCLOCK_Pin, GPIO_TypeDef *DOUT, uint16_t DOUT_Pin)
 {


    uint32_t Count_1=0;
	unsigned char i;


    HAL_GPIO_WritePin(HxCLOCK,HxCLOCK_Pin, GPIO_PIN_RESET);
    while(HAL_GPIO_ReadPin(DOUT,DOUT_Pin));

    for(i=0;i<24;i++)
    {
	   HAL_GPIO_WritePin(HxCLOCK, HxCLOCK_Pin, GPIO_PIN_SET);
       Count_1 = Count_1 << 1;
       HAL_GPIO_WritePin(HxCLOCK,HxCLOCK_Pin, GPIO_PIN_RESET);
       if(HAL_GPIO_ReadPin(DOUT,DOUT_Pin)) Count_1++;

     }

     HAL_GPIO_WritePin(HxCLOCK,HxCLOCK_Pin, GPIO_PIN_SET);
     Count_1 = Count_1^0x0800000;
     HAL_GPIO_WritePin(HxCLOCK,HxCLOCK_Pin, GPIO_PIN_RESET);
     UART_print("%d\n", Count_1);
     return(Count_1);

  }



uint16_t Pot_map(uint32_t leituraPot)
{
   // Pot = ((Pot) * (270) / (4095));
    //leituraPot = (uint16_t)Pot;
    return leituraPot;
}
void CAN_Transmit(uint8_t *vet, uint32_t id){
	TxHeader.StdId = id;


#if DEBUG
	if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, vet, &TxMailbox) == HAL_OK) HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#else
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, vet, &TxMailbox);

#endif

}
void transmit_dados()
{

	/*SLAVE 1*/

    vetTx[0] = leitura_BetinaInt;
    vetTx[1] = leitura_BetinaInt >> 8;
    vetTx[2] = leitura_PotInt;
    vetTx[3] = leitura_PotInt >> 8;
    vetTx[4] = temp;
    vetTx[5] = temp >> 8;
    vetTx[6] = IRcan0;
    vetTx[7] = IRcan0 >> 8;
    CAN_Transmit(vetTx, 170);


    vetTy[0] = Dado_1;
    vetTy[1] = Dado_1 >> 8;
    vetTy[2] = Dado_1 >> 16;
    vetTy[3] = Dado_1 >> 24;
    vetTy[4] = Dado_3;
    vetTy[5] = Dado_3 >> 8;
    vetTy[6] = Dado_3 >> 16;
    vetTy[7] = Dado_3 >> 24;
    CAN_Transmit(vetTy, 171);


    vetTv[0] = IRmedia[0];
    vetTv[1] = IRmedia[0] >> 8;
    vetTv[2] = IRmedia[1];
    vetTv[3] = IRmedia[1] >> 8;
    vetTv[4] = IRmedia[2];
    vetTv[5] = IRmedia[2]>> 8;
    vetTv[6] = IRmedia[3];
    vetTv[7] = IRmedia[3]>> 8;
    CAN_Transmit(vetTv, 156);


    vetTn[0] = IRcan5;
    vetTn[1] = IRcan5 >> 8;
    vetTn[2] = IRcan6;
    vetTn[3] = IRcan6 >> 8;
    vetTn[4] = IRcan7;
    vetTn[5] = IRcan7 >> 8;
    vetTn[6] = IRcan8;
    vetTn[7] = IRcan8 >> 8;
    CAN_Transmit(vetTn, 157);


    vetTu[0] = IRcan9;
    vetTu[1] = IRcan9 >> 8;
    vetTu[2] = IRcan10;
    vetTu[3] = IRcan10 >> 8;
    vetTu[4] = IRcan11;
    vetTu[5] = IRcan11 >> 8;
    vetTu[6] = IRcan12;
    vetTu[7] = IRcan12 >> 8;
    CAN_Transmit(vetTu, 158);


    vetTm[0] = IRcan13;
    vetTm[1] = IRcan13 >> 8;
    vetTm[2] = IRcan14;
    vetTm[3] = IRcan14 >> 8;
    vetTm[4] = IRcan15;
    vetTm[5] = IRcan15 >> 8;
    vetTm[6] = 0;
    vetTm[7] = 0;
    CAN_Transmit(vetTm, 159);




	/*SLAVE 3
    vetTy[0] = Dado_1;
    vetTy[1] = Dado_1 >> 8;
    vetTy[2] = Dado_1 >> 16;
    vetTy[3] = Dado_1 >> 24;
    vetTy[4] = Dado_3;
    vetTy[5] = Dado_3 >> 8;
    vetTy[6] = Dado_3 >> 16;
    vetTy[7] = Dado_3 >> 24;
    CAN_Transmit(vetTy, 178);

*/

}


int16_t readConfig(void)
{
	uint8_t configLow = 0, configHigh = 0;
	uint32_t comando = 0x02920001;
	while(HAL_I2C_Mem_Read2(&hi2c2, (uint16_t)0xC0, comando, I2C_MEMADD_SIZE_32BIT, buffer, 2, 1000) != HAL_OK);
	configLow = buffer[0];
	configHigh = buffer[1];
	configuration = ((int16_t)(configHigh << 8) | configLow);
	return configuration;
}

bool checkConfig(void)
{
	bool check = !((readConfig() & 0x0400) >> 10);
	return check;
}


void lerEEPROM(void)
{
	int i = 0;
	int k = 0;
	int j = 0;
	buffer[0] = (uint8_t)j;
	while(HAL_I2C_Mem_Read(&hi2c2,  (uint16_t)0xA1, (uint16_t)0x00, 1, bigbuff, 260, 100) != HAL_OK);
	for(k = 0; k < 260; k++)
    {
		eepromData[i] = (uint8_t)bigbuff[k];
		i++;
	}
}

void writeTrimmingValue(void)
{
	buffer[0] = 0x04;
	buffer[1] = ((uint8_t)eepromData[214] - 0xAA);
	buffer[2] = eepromData[214];
	buffer[3] = 0x100-0xAA;
	buffer[4] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)MLXir<<1, buffer, 5, 100) != HAL_OK);
}

void setConfiguration(void)
{
	uint8_t Hz_LSB;
	switch (refreshRate){
	case 0:
		Hz_LSB = 0b00111111;
		break;
	case 1:
		Hz_LSB = 0b00111110;
		break;
	case 2:
		Hz_LSB = 0b00111101;
		break;
	case 4:
		Hz_LSB = 0b00111100;
		break;
	case 8:
		Hz_LSB = 0b00111011;
		break;
	case 16:
		Hz_LSB = 0b00111010;
		break;
	case 32:
		Hz_LSB = 0b00111001;
		break;
	default:
		Hz_LSB = 0b00111110;
	}
	uint8_t defaultConfig_H = 0b00000100;
	buffer[0] = 0x03;
	buffer[1] = ((uint8_t)Hz_LSB - 0x55);
	buffer[2] = Hz_LSB;
	buffer[3] = defaultConfig_H - 0x55;
	buffer[4] = defaultConfig_H;
	while(HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)MLXir<<1, buffer, 5, 100) != HAL_OK);
	resolution = (readConfig() & 0x30) >> 4;
}

void readPTAT(void)
{
	uint8_t ptatLow=0,ptatHigh=0;
	uint32_t comando = 0x02400001;
	while(HAL_I2C_Mem_Read2(&hi2c2, (uint16_t)0xC0, comando, I2C_MEMADD_SIZE_32BIT, buffer, 2, 1000) != HAL_OK);
	ptatLow = buffer[0];
	ptatHigh = buffer[1];
	ptat = ((uint16_t) (ptatHigh << 8) | ptatLow);
}

void readCPIX(void)
{
	uint8_t cpixLow=0,cpixHigh=0;
	buffer[0] = 0x02;
	while(HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)MLXir<<1, buffer, 1, 100) != HAL_OK);
	buffer[0] = 0x41;
	while(HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)MLXir<<1, buffer, 1, 100) != HAL_OK);
	buffer[0] = 0x00;
	while(HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)MLXir<<1, buffer, 1, 100) != HAL_OK);
	buffer[0] = 0x01;
	while(HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)MLXir<<1, buffer, 1, 100) != HAL_OK);
	while(HAL_I2C_Master_Receive(&hi2c2,  (uint16_t)MLXir<<1, bigbuff, 2, 100) != HAL_OK);
	cpixLow = bigbuff[0];
	cpixHigh = bigbuff[1];
	cpix = ((uint16_t) (cpixHigh << 8) | cpixLow);
	if (cpix >= 32768)
    {
		cpix -= 65536;
	}
}

void readIR(void)
{
	memset(IRmedia,0,16);
	volatile int i;
	uint32_t comando = 0x02000140;
	while(HAL_I2C_Mem_Read2(&hi2c2, (uint16_t)(0x60<<1), comando, I2C_MEMADD_SIZE_32BIT, buffer, 128, 1000) != HAL_OK);
	i = 0;
	for(int k = 0; k < 64; k++)
    {
		irData[k] = (int16_t) ((buffer[i+1] << 8) | buffer[i]);
		i = i + 2;
	}
 	for(int aux1 = 0; aux1 < 16; aux1++)
    {
 		IRmedia[aux1] = irData[aux1] + irData[(aux1)+16] + irData[(aux1)+32] + irData[(aux1)+48];
 		IRmedia[aux1] = IRmedia[aux1]/4;
 	//	UART_print(" %d", IRmedia[aux1]);
 	}
 	//UART_print("\n");
}

void MLX_config(void){
	  lerEEPROM();
	  writeTrimmingValue();
	  setConfiguration();
}

uint8_t contmlx = 0;
void MLX_loopread(void){
	  if(contmlx == 16){
		  if (checkConfig())
		  {
			  MLX_config();
		  }
		  contmlx = 0;
	  }
	  contmlx++;
	//readPTAT();
	  readIR();
}

uint16_t leitura;
uint16_t ADC_read(ADC_HandleTypeDef* hadc, uint32_t Timeout){

	  HAL_ADC_Start(hadc);
	  HAL_ADC_PollForConversion(hadc,Timeout);
	  leitura = HAL_ADC_GetValue(hadc);
	  UART_print("adc: %d\n", leitura);
	  return leitura;

}


