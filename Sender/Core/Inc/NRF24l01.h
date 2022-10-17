/*
 * NRF24l01.h
 *
 *  Created on: Sep 13, 2022
 *      Author: Lantern
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#define GPIOA_ODR_Addr    0x4001080C
#define GPIOB_ODR_Addr    0x40010C0C
#define GPIOB_IDR_Addr    0x40010C08

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)           *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)

//NRF24L01寄存器操作命令
#define NRF_READ_REG    0x00  //读配置寄存器,低5位为寄存器地址
#define NRF_WRITE_REG   0x20  //写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD     0x61  //读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX        0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     0xE3  //重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP             0xFF  //空操作,可以用来读状态寄存器
//SPI(NRF24L01)寄存器地址
#define CONFIG          0x00  //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                              //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA           0x01  //使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR       0x02  //接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR      0x04  //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH           0x05  //RF通道,bit6:0,工作通道频率;
#define RF_SETUP        0x06  //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define STATUS          0x07  //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
                              //bit5:数据发送完成中断;bit6:接收数据中断;
#define MAX_TX  		0x10  //达到最大发送次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收到数据中断

#define OBSERVE_TX      0x08  //发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define CD              0x09  //载波检测寄存器,bit0,载波检测;
#define RX_ADDR_P0      0x0A  //数据通道0接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P1      0x0B  //数据通道1接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P2      0x0C  //数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3      0x0D  //数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4      0x0E  //数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5      0x0F  //数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define TX_ADDR         0x10  //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define RX_PW_P0        0x11  //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12  //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13  //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14  //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15  //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16  //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define NRF_FIFO_STATUS 0x17  //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
                              //bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//24L01操作线
#define NRF24L01_CE     PBout(8)  //24L01片选信号
#define NRF24L01_CSN    PBout(7)  //SPI片选信号
#define NRF24L01_IRQ    PBin(6)   //IRQ主机数据输入
//24L01发送接收数据宽度定义
#define TX_ADR_WIDTH    5   	//5字节的地址宽度
#define RX_ADR_WIDTH    5   	//5字节的地址宽度
#define TX_PLOAD_WIDTH  32  	//32字节的用户数据宽度
#define RX_PLOAD_WIDTH  32  	//32字节的用户数据宽度


SPI_HandleTypeDef* hspi;

void NRF24L01_Init(SPI_HandleTypeDef* hspi);//初始化
void NRF24L01_RX_Mode(void);//配置为接收模式
void NRF24L01_TX_Mode(void);//配置为发送模式
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t u8s);//写数据区
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t u8s);//读数据区
uint8_t NRF24L01_Read_Reg(uint8_t reg);			//读寄存器
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value);//写寄存器
uint8_t NRF24L01_Check(void);//检查24L01是否存在
uint8_t NRF24L01_TxPacket(uint8_t *txbuf);//发送一个包的数据
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf);//接收一个包的数据


const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //接受地址

void blink_fast()
 {
	  int t = 50;
	   while( t-- ){
	  	  HAL_GPIO_WritePin(UserLED2_GPIO_Port, UserLED2_Pin, GPIO_PIN_RESET);
	  	  PAout(12) = 0;
	  	  HAL_Delay(40);
	  	  HAL_GPIO_WritePin(UserLED2_GPIO_Port, UserLED2_Pin, GPIO_PIN_SET);
	  	  PAout(12) = 1;
	  	  HAL_Delay(40);
	   }
 }

uint8_t SPI_ReadWriteByte(uint8_t TxData)
{
	uint8_t Rxdata;
    HAL_SPI_TransmitReceive(hspi , &TxData, &Rxdata, 1, 1000);
 	return Rxdata;          		    //返回收到的数据
}


//针对NRF24L01修改SPI驱动
void NRF24L01_SPI_Init(void)
{
    __HAL_SPI_DISABLE(hspi);                        // 先关闭 SPI
    hspi -> Init.CLKPolarity  =  SPI_POLARITY_LOW;    // 串行同步时钟的空闲状态为低电平
    hspi -> Init.CLKPhase     =  SPI_PHASE_1EDGE;     // 串行同步时钟的第1个跳变沿（上升或下降）数据被采样							//
    if (HAL_SPI_Init(hspi) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_SPI_ENABLE(hspi);                         // 使能SPI
}

//初始化24L01的IO口
void NRF24L01_Init(SPI_HandleTypeDef* thspi)
{
	hspi = thspi;

    NRF24L01_SPI_Init();                    //针对NRF的特点修改SPI的设置
	NRF24L01_CE  = 0; 			                //使能24L01
	NRF24L01_CSN = 1;			                //SPI片选取消
}
//检测24L01是否存在
//返回值:0，成功;1，失败
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;

	NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, buf, 5);//写入5个字节的地址.
	NRF24L01_Read_Buf(TX_ADDR , buf , 5); //读出写入的地址
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;
	if(i!=5)return 1;//检测24L01错误
	return 0;		 //检测到24L01
}
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
   	NRF24L01_CSN=0;                 //使能SPI传输
  	status = SPI_ReadWriteByte(reg);//发送寄存器号
  	SPI_ReadWriteByte(value);      //写入寄存器的值
  	NRF24L01_CSN=1;                 //禁止SPI传输
  	return(status);       		    //返回状态值
}
//读取SPI寄存器值
//reg:要读的寄存器
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	 uint8_t reg_val;
   	NRF24L01_CSN=0;             //使能SPI传输
  	SPI_ReadWriteByte(reg);    //发送寄存器号
  	reg_val=SPI_ReadWriteByte(0XFF);//读取寄存器内容
  	NRF24L01_CSN=1;             //禁止SPI传输
  	return(reg_val);            //返回状态值
}
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status, u8_ctr;
  	NRF24L01_CSN = 0;            //使能SPI传输
  	status = SPI_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
	for(u8_ctr = 0; u8_ctr < len; u8_ctr++)pBuf[u8_ctr] = SPI_ReadWriteByte(0XFF);//读出数据
  	NRF24L01_CSN = 1;            //关闭SPI传输
  	return status;             //返回读到的状态值
}
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status, u8_ctr;
	NRF24L01_CSN = 0;             //使能SPI传输
  	status = SPI_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
  	for(u8_ctr = 0 ; u8_ctr < len ; u8_ctr++ )SPI_ReadWriteByte(*pBuf++); //写入数据
  	NRF24L01_CSN = 1;             //关闭SPI传输
  	return status;              //返回读到的状态值
}
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
	NRF24L01_CE=0;
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 	NRF24L01_CE=1;                         //启动发送
	while(NRF24L01_IRQ!=0);                 //等待发送完成
	sta = NRF24L01_Read_Reg(STATUS);          //读取状态寄存器的值
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX)                          //达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);  //清除TX FIFO寄存器
		return MAX_TX;
	}
	if(sta&TX_OK)                           //发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
}
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;
	sta = NRF24L01_Read_Reg(STATUS);          //读取状态寄存器的值
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);  //清除RX FIFO寄存器
		return 0;
	}
	return 1;//没收到任何数据
}
//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE = 0;
  	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址

  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);       //使能通道0的自动应答
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);   //使能通道0的接收地址
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);	        //设置RF通信频率
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);    //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);     //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式
  	NRF24L01_CE = 1; //CE为高,进入接收模式
}
//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了
//CE为高大于10us,则启动发送.
void NRF24L01_TX_Mode(void)
{
	NRF24L01_CE = 0;
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK

	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //设置RF通道为40
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	NRF24L01_CE = 1;//CE为高,10us后启动发送
}



#endif /* INC_NRF24L01_H_ */
