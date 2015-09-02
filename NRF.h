
//==============================================================================
#define  RF24L01_CE_0        IO_Init(0x02)         
#define  RF24L01_CE_1        IO_Init(0x01)         
//=============================RF24L01_CSN??==================================
#define  RF24L01_CSN_0       IO_Init(0x04)          
#define  RF24L01_CSN_1       IO_Init(0x03)    
//=============================RF24L01_SCK======================================
#define  RF24L01_SCK_0       IO_Init(0x06)       
#define  RF24L01_SCK_1       IO_Init(0x05) 
//============================= RF24L01_MOSI??================================
#define  RF24L01_MOSI_0      IO_Init(0x08) 
#define  RF24L01_MOSI_1      IO_Init(0x07) 
//=============================MISO??=========================================//输入引脚
#define  RF24L01_MISO_0      IO_Init(0x10) 
#define  RF24L01_MISO_1      IO_Init(0x09) 
//==========================IRQ??============================================//输入引脚
#define  RF24L01_IRQ_0       IO_Init(0x12)        
#define  RF24L01_IRQ_1       IO_Init(0x11) 

//==============================================================================
#define  Busy	             0x80
//==============================================================================
#define TX_ADR_WIDTH    5   	// 5 uints TX address width
#define RX_ADR_WIDTH    5   	// 5 uints RX address width
#define TX_PLOAD_WIDTH  32  	// 32 TX payload
#define RX_PLOAD_WIDTH  32  	// 32 uints TX payload
//=========================NRF24L01?????===================================
#define READ_REG        0x00  	// ??????
#define WRITE_REG       0x20 	// ??????
#define RD_RX_PLOAD     0x61  	// ????????
#define WR_TX_PLOAD     0xA0  	// ???????
#define FLUSH_TX        0xE1 	// ???? FIFO??
#define FLUSH_RX        0xE2  	// ???? FIFO??
#define REUSE_TX_PL     0xE3  	// ??????????
#define NOP1            0xFF  	// ??
//========================SPI(nRF24L01)?????===============================
#define CONFIG          0x00  // ??????,CRC??????????????
#define EN_AA           0x01  // ????????
#define EN_RXADDR       0x02  // ??????
#define SETUP_AW        0x03  // ????????
#define SETUP_RETR      0x04  // ????????
#define RF_CH           0x05  // ??????
#define RF_SETUP        0x06  // ???????????
#define STATUS          0x07  // ?????
#define OBSERVE_TX      0x08  // ??????
#define CD              0x09  // ????           
#define RX_ADDR_P0      0x0A  // ??0??????
#define RX_ADDR_P1      0x0B  // ??1??????
#define RX_ADDR_P2      0x0C  // ??2??????
#define RX_ADDR_P3      0x0D  // ??3??????
#define RX_ADDR_P4      0x0E  // ??4??????
#define RX_ADDR_P5      0x0F  // ??5??????
#define TX_ADDR         0x10  // ???????
#define RX_PW_P0        0x11  // ????0??????
#define RX_PW_P1        0x12  // ????0??????
#define RX_PW_P2        0x13  // ????0??????
#define RX_PW_P3        0x14  // ????0??????
#define RX_PW_P4        0x15  // ????0??????
#define RX_PW_P5        0x16  // ????0??????
#define FIFO_STATUS     0x17  // FIFO???????????
//=============================RF24l01??=====================================

//==============================================================================

void RF24L01_IO_set(void);
void IO_Init(unsigned char x);
void ms_delay(void);

void Delay(int s);
char SPI_RW(char data);
char SPI_Read(char reg);
char SPI_RW_Reg(char reg, char value);
char SPI_Read_Buf(char reg, char *pBuf, char uchars);
char SPI_Write_Buf(char reg, char *pBuf, char uchars);
void SetRX_Mode(void);
char nRF24L01_RxPacket(char* rx_buf);
void nRF24L01_TxPacket(char * tx_buf);
void init_NRF24L01(void);
void inerDelay_us(int n);
//void Rec_Data_Pro(char RxBuf);
void NRF_Init(void);
//void Rec_Data_Init(void);
void IfReciveNRF(void);
void GPIO_INT_B(void);
