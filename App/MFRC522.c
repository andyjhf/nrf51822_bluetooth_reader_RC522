#include "MFRC522.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
//#include "simple_uart.h"
extern uint32_t *p_spi_base_address;
typedef  uint8_t u8;
typedef  uint16_t u16;
#define _MFRC_SOFT_SPI
#define Set_MFRC_CS(x)		x ? nrf_gpio_pin_set(SPI_PSELSS0):nrf_gpio_pin_clear(SPI_PSELSS0)
#define Set_MFRC_SCK(x)	x ? nrf_gpio_pin_set(SPI_PSELSCK0):nrf_gpio_pin_clear(SPI_PSELSCK0)
#define Set_MFRC_MOSI(x)	x ? nrf_gpio_pin_set(SPI_PSELMOSI0):nrf_gpio_pin_clear(SPI_PSELMOSI0)
#define MFRC_MISO_STATUS()	nrf_gpio_pin_read(SPI_PSELMISO0)
//#define Set_MFRC_RST(x)		x ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

void MFRC_Delay(u16 Delay_Time)
{
  u16 i, j;
  for (i = 40; i > 0; i--)
  {
    for (j = Delay_Time; j > 0; j--);
  }
}
#ifndef _MFRC_SOFT_SPI  
void SPI_TxByte(u8 Dat)
{
	uint8_t state;
	state = spi_master_tx(p_spi_base_address, Dat);	   
	if(state != true)
		state = true;
}

u8 SPI_RxByte(void)
{
	uint8_t state;
  state = spi_master_rx(p_spi_base_address) ;
  return state; 			    
}
#endif
/*
/////////////////////////////////////////////////////////////////////
//?    ?:?RC632???
//????:Address[IN]:?????
//?    ?:????
/////////////////////////////////////////////////////////////////////
unsigned char ReadRawRC(unsigned char Address)
{
    unsigned char ucResult=0;
		uint8_t ucAddr;
		uint8_t tx_data[2]={0x00,0x00}; 
		uint8_t rx_data[2]={0x00,0x00}; 
		ucAddr = ((Address << 1) & 0x7E) | 0x80;
		tx_data[0] = ucAddr;
		spi_master_tx_rx(p_spi_base_address, 1, tx_data, rx_data); 
		ucResult = rx_data[0];
    return ucResult;
}

/////////////////////////////////////////////////////////////////////
//?    ?:?RC632???
//????:Address[IN]:?????
//          value[IN]:????
/////////////////////////////////////////////////////////////////////
void WriteRawRC(unsigned char Address, unsigned char value)
{  
		uint8_t tx_data[2]={0x00,0x00}; 
		uint8_t rx_data[2]={0x00,0x00}; 
		uint8_t ucAddr;
		ucAddr = ((Address << 1) & 0x7E);
		tx_data[0] = ucAddr;
		tx_data[1] = value;
		spi_master_tx_rx(p_spi_base_address, 2, tx_data, rx_data); 
}*/

/////////////////////////////////////////////////////////////////////
//��    �ܣ���RC632�Ĵ���
//����˵����Address[IN]:�Ĵ�����ַ
//��    �أ�������ֵ
/////////////////////////////////////////////////////////////////////
static u8 ReadRawRC(u8 Address)
{
  u8 ucAddr;
  u8 ret = 0;
#ifdef _MFRC_SOFT_SPI 
  u8 i;
  Set_MFRC_SCK(0);
#endif  
  
  Set_MFRC_CS(0);

  ucAddr = ((Address << 1) & 0x7E) | 0x80;
#ifdef _MFRC_SOFT_SPI  
  for(i=8; i>0; i--)
  {
    Set_MFRC_MOSI((ucAddr & 0x80) == 0x80);
    Set_MFRC_SCK(1);
    ucAddr <<= 1;
    Set_MFRC_SCK(0);
  }

  for(i=8; i>0; i--)
  {
    Set_MFRC_SCK(1);
    ret <<= 1;
    ret |= MFRC_MISO_STATUS();
    Set_MFRC_SCK(0);
  }
#else
  SPI_TxByte(ucAddr);
  ret = SPI_RxByte();
	
#endif
  Set_MFRC_CS(1);
#ifdef _MFRC_SOFT_SPI 
  Set_MFRC_SCK(1);
#endif
	printf("REG ADDR:0x%x  Val: 0x%x\r\n",Address,ret);
  return ret;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ�дRC632�Ĵ���
//����˵����Address[IN]:�Ĵ�����ַ
//          value[IN]:д���ֵ
/////////////////////////////////////////////////////////////////////
void WriteRawRC(u8 Address, u8 value)
{
  u8 ucAddr;
#ifdef _MFRC_SOFT_SPI 
  u8 i;
  Set_MFRC_SCK(0);
#endif
  Set_MFRC_CS(0);

  ucAddr = ((Address << 1) & 0x7E);
#ifdef _MFRC_SOFT_SPI 
	printf("---write---REG ADDR:0x%x  Val: 0x%x\r\n",Address,value);
  for(i=8; i>0; i--)
  {
    Set_MFRC_MOSI((ucAddr & 0x80) == 0x80);
    Set_MFRC_SCK(1);
    ucAddr <<= 1;
    Set_MFRC_SCK(0);
  }

  for(i=8; i>0; i--)
  {
    Set_MFRC_MOSI((value & 0x80) == 0x80);
    Set_MFRC_SCK(1);
    value <<= 1;
    Set_MFRC_SCK(0);
  }
#else
  SPI_TxByte(ucAddr);
  SPI_TxByte(value);
	printf("---write---REG ADDR:0x%x  Val: 0x%x\r\n",Address,value);
#endif
  Set_MFRC_CS(1);
#ifdef _MFRC_SOFT_SPI
  Set_MFRC_SCK(1);
#endif
	
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ���RC522�Ĵ���λ
//����˵����reg[IN]:�Ĵ�����ַ
//          mask[IN]:��λֵ
/////////////////////////////////////////////////////////////////////
static void ClearBitMask(u8 reg, u8 mask)
{
	u8 tmp = 0x0;
	tmp = ReadRawRC(reg);
	WriteRawRC(reg, tmp & ~mask);  // clear bit mask
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ���RC522�Ĵ���λ
//����˵����reg[IN]:�Ĵ�����ַ
//          mask[IN]:��λֵ
/////////////////////////////////////////////////////////////////////
static void SetBitMask(u8 reg, u8 mask)
{
	u8 tmp = 0x0;
	tmp = ReadRawRC(reg);
	WriteRawRC(reg, tmp | mask);  // set bit mask
}


//��������  
//ÿ��������ر����շ���֮��Ӧ������1ms�ļ��
void PcdAntennaOn(void)
{
	u8 i;
	i = ReadRawRC(TxControlReg);
	if (!(i & 0x03))
	{
		SetBitMask(TxControlReg, 0x03);
	}
}

//�ر�����
void PcdAntennaOff(void)
{
	ClearBitMask(TxControlReg, 0x03);
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ�ͨ��RC522��ISO14443��ͨѶ
//����˵����Command[IN]:RC522������
//          pIn [IN]:ͨ��RC522���͵���Ƭ������
//          InLenByte[IN]:�������ݵ��ֽڳ���
//          pOut [OUT]:���յ��Ŀ�Ƭ��������
//          *pOutLenBit[OUT]:�������ݵ�λ����
/////////////////////////////////////////////////////////////////////
#define MAXRLEN                       18
static char PcdComMF522(unsigned char Command, 
                 unsigned char *pInData, 
                 unsigned char InLenByte,
                 unsigned char *pOutData, 
                 unsigned int  *pOutLenBit)
{
	char status = MI_ERR;
	unsigned char irqEn   = 0x00;
	unsigned char waitFor = 0x00;
	unsigned char lastBits;
	unsigned char n;
	unsigned int i;

	switch (Command)
	{
	case PCD_AUTHENT:
		irqEn = 0x12;
		waitFor = 0x10;
		break;
	case PCD_TRANSCEIVE:
		irqEn = 0x77;
		waitFor = 0x30;
		break;
	default:
		break;
	}

	WriteRawRC(ComIEnReg, irqEn | 0x80);
	ClearBitMask(ComIrqReg, 0x80);
	WriteRawRC(CommandReg, PCD_IDLE);
	SetBitMask(FIFOLevelReg, 0x80);

	for (i = 0; i < InLenByte; i++)
	{
		WriteRawRC(FIFODataReg, pInData[i]);
	}
	WriteRawRC(CommandReg, Command);

	if (Command == PCD_TRANSCEIVE)
	{
		SetBitMask(BitFramingReg, 0x80);
	}

	i = 800; 
	do
	{
		n = ReadRawRC(ComIrqReg);
		i--;
	} while ((i != 0) && !(n & 0x01) && !(n & waitFor));
	ClearBitMask(BitFramingReg, 0x80);

	if (i != 0)
	{
		if (!(ReadRawRC(ErrorReg) & 0x1B))
		{
			status = MI_OK;
			if (n & irqEn & 0x01)
			{
				status = MI_NOTAGERR;
			}
			if (Command == PCD_TRANSCEIVE)
			{
				n = ReadRawRC(FIFOLevelReg);
				lastBits = ReadRawRC(ControlReg) & 0x07;
				if (lastBits)
				{
					*pOutLenBit = (n - 1) * 8 + lastBits;
				}
				else
				{
					*pOutLenBit = n * 8;
				}
				if (n == 0)
				{
					n = 1;
				}
				if (n > MAXRLEN)
				{
					n = MAXRLEN;
				}
				for (i = 0; i < n; i++)
				{
					pOutData[i] = ReadRawRC(FIFODataReg);
				}
			}
		}
		else
		{
			status = MI_ERR;
		}
	}
	SetBitMask(ControlReg, 0x80);           // stop timer now
	WriteRawRC(CommandReg, PCD_IDLE);
	return status;
}


/////////////////////////////////////////////////////////////////////
//��    �ܣ���λRC522
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////
char PcdReset(void)
{
	nrf_gpio_pin_set(SPI_RST);
	MFRC_Delay(10);  
	nrf_gpio_pin_clear(SPI_RST);
	MFRC_Delay(60000);  
	nrf_gpio_pin_set(SPI_RST);
	MFRC_Delay(500);  
	WriteRawRC(CommandReg, PCD_RESETPHASE);
	MFRC_Delay(2000);  

	WriteRawRC(ModeReg, 0x3D);            
	WriteRawRC(TReloadRegL, 30);             
	WriteRawRC(TReloadRegH, 0);
	WriteRawRC(TModeReg, 0x8D);
	WriteRawRC(TPrescalerReg, 0x3E);
	WriteRawRC(TxAutoReg, 0x40);
	
	ClearBitMask(TestPinEnReg, 0x80);         //off MX and DTRQ out
	WriteRawRC(TxAutoReg, 0x40);

	return MI_OK;
}	

/////////////////////////////////////////////////////////////////////
//��MF522����CRC16����
/////////////////////////////////////////////////////////////////////
void CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData)
{
    unsigned char i,n;
	
    ClearBitMask(DivIrqReg,0x04);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<len; i++)
    {   
        WriteRawRC(FIFODataReg, *(pIndata+i));  
    }
    WriteRawRC(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do 
    {
        n = ReadRawRC(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));
    pOutData[0] = ReadRawRC(CRCResultRegL);
    pOutData[1] = ReadRawRC(CRCResultRegM);
}


//////////////////////////////////////////////////////////////////////
//����RC522�Ĺ�����ʽ 
//////////////////////////////////////////////////////////////////////
signed char M500PcdConfigISOType(unsigned char type)
{
  if('A' == type)
  {
    ClearBitMask(Status2Reg,0x08);
    WriteRawRC(ModeReg,0x3D);
    WriteRawRC(RxSelReg,0x86);
    WriteRawRC(RFCfgReg,0x7F);   
    WriteRawRC(TReloadRegL,30);
    WriteRawRC(TReloadRegH,0);
    WriteRawRC(TModeReg,0x8D);
    WriteRawRC(TPrescalerReg,0x3E);
    MFRC_Delay(10000);
    PcdAntennaOn();
  }
  else
  {
    return -1;
  }
	
  return MI_OK;
}

/***
��ʼ��RC522
*/

void MFRC522_Init(void)
{
//  MFRC_GPIOConfigure();
	
  PcdReset();
  PcdAntennaOff();
  MFRC_Delay(2000);
  PcdAntennaOn();
  M500PcdConfigISOType('A');
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ�Ѱ��
//����˵��: req_code[IN]:Ѱ����ʽ
//                0x52 = Ѱ��Ӧ�������з���14443A��׼�Ŀ�
//                0x26 = Ѱδ��������״̬�Ŀ�
//          pTagType[OUT]����Ƭ���ʹ���
//                0x4400 = Mifare_UltraLight
//                0x0400 = Mifare_One(S50)
//                0x0200 = Mifare_One(S70)
//                0x0800 = Mifare_Pro(X)
//                0x4403 = Mifare_DESFire
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////
char PcdRequest(u8 req_code, u8 *pTagType)
{
  char status;  
	unsigned int  unLen;
	unsigned char ucComMF522Buf[MAXRLEN]; 

	ClearBitMask(Status2Reg, 0x08);
	WriteRawRC(BitFramingReg, 0x07);
	SetBitMask(TxControlReg, 0x03);
//   
	ucComMF522Buf[0] = req_code;

	status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf,
			&unLen);
//   UART_send_byte(status);
	if ((status == MI_OK) && (unLen == 0x10))
	{
		*pTagType = ucComMF522Buf[0];
		*(pTagType + 1) = ucComMF522Buf[1];
	}
	else
	{
	  status = MI_ERR;
	}

	return status;
}


/////////////////////////////////////////////////////////////////////
//��    �ܣ�����ײ
//����˵��: pSnr[OUT]:��Ƭ���кţ�4�ֽ�
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////  
char PcdAnticoll(unsigned char *pSnr)
{
    char status;
    unsigned char i,snr_check=0;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    

    ClearBitMask(Status2Reg,0x08);
    WriteRawRC(BitFramingReg,0x00);
    ClearBitMask(CollReg,0x80);
 
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x20;

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

    if (status == MI_OK)
    {
    	 for (i=0; i<4; i++)
         {   
             *(pSnr+i)  = ucComMF522Buf[i];
             snr_check ^= ucComMF522Buf[i];
         }
         if (snr_check != ucComMF522Buf[i])
         {   status = MI_ERR;    }
    }
    
    SetBitMask(CollReg,0x80);
    return status;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ�ѡ����Ƭ
//����˵��: pSnr[IN]:��Ƭ���кţ�4�ֽ�
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////
char PcdSelect(unsigned char *pSnr)
{
    char status;
    unsigned char i;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i=0; i<4; i++)
    {
    	ucComMF522Buf[i+2] = *(pSnr+i);
    	ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
  
    ClearBitMask(Status2Reg,0x08);

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
    
    if ((status == MI_OK) && (unLen == 0x18))
    {   status = MI_OK;  }
    else
    {   status = MI_ERR;    }

    return status;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ����Ƭ��������״̬
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////
char PcdHalt(void)
{
//    int status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    //status =
    PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    return MI_OK;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ���֤��Ƭ����
//����˵��: auth_mode[IN]: ������֤ģʽ
//                 0x60 = ��֤A��Կ
//                 0x61 = ��֤B��Կ 
//          addr[IN]�����ַ
//          pKey[IN]������
//          pSnr[IN]����Ƭ���кţ�4�ֽ�
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////               
char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+2] = *(pKey+i);   }
    for (i=0; i<6; i++)
    {    ucComMF522Buf[i+8] = *(pSnr+i);   }
 //   memcpy(&ucComMF522Buf[2], pKey, 6); 
 //   memcpy(&ucComMF522Buf[8], pSnr, 4); 
    
    status = PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
    if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
    {   status = MI_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ���ȡM1��һ������
//����˵��: addr[IN]�����ַ
//          pData[OUT]�����������ݣ�16�ֽ�
//��    ��: �ɹ�����MI_OK
///////////////////////////////////////////////////////////////////// 
char PcdRead(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
   
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    if ((status == MI_OK) && (unLen == 0x90))
 //   {   memcpy(pData, ucComMF522Buf, 16);   }
    {
        for (i=0; i<16; i++)
        {    *(pData+i) = ucComMF522Buf[i];   }
    }
    else
    {   status = MI_ERR;   }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ�д���ݵ�M1��һ��
//����˵��: addr[IN]�����ַ
//          pData[IN]��д������ݣ�16�ֽ�
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////                  
char PcdWrite(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        //memcpy(ucComMF522Buf, pData, 16);
        for (i=0; i<16; i++)
        {    ucComMF522Buf[i] = *(pData+i);   }
        CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);

        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    
    return status;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ��ۿ�ͳ�ֵ
//����˵��: dd_mode[IN]��������
//               0xC0 = �ۿ�
//               0xC1 = ��ֵ
//          addr[IN]��Ǯ����ַ
//          pValue[IN]��4�ֽ���(��)ֵ����λ��ǰ
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////                 
char PcdValue(unsigned char dd_mode,unsigned char addr,unsigned char *pValue)
{
    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = dd_mode;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
        
    if (status == MI_OK)
    {
        memcpy(ucComMF522Buf, pValue, 4);
 //       for (i=0; i<16; i++)
 //       {    ucComMF522Buf[i] = *(pValue+i);   }
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
        unLen = 0;
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }
    
    if (status == MI_OK)
    {
        ucComMF522Buf[0] = PICC_TRANSFER;
        ucComMF522Buf[1] = addr;
        CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]); 
   
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   status = MI_ERR;   }
    }
    return status;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ�����Ǯ��
//����˵��: sourceaddr[IN]��Դ��ַ
//          goaladdr[IN]��Ŀ���ַ
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////
char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr)
{
    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_RESTORE;
    ucComMF522Buf[1] = sourceaddr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
    
    if (status == MI_OK)
    {
        ucComMF522Buf[0] = 0;
        ucComMF522Buf[1] = 0;
        ucComMF522Buf[2] = 0;
        ucComMF522Buf[3] = 0;
        CalulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
 
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
        if (status != MI_ERR)
        {    status = MI_OK;    }
    }
    
    if (status != MI_OK)
    {    return MI_ERR;   }
    
    ucComMF522Buf[0] = PICC_TRANSFER;
    ucComMF522Buf[1] = goaladdr;

    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }

    return status;
}



