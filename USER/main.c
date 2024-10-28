// 注:本程序基于安信可的开发板,系统晶振频率为16MHz

#include "stm32f10x.h" // Device header
#include "delay.h"
#include "UART.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "UART.h"

#include <string.h>

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
	2,				 /* Channel number. */
	DWT_PRF_64M,	 /* Pulse repetition frequency. */
	DWT_PLEN_1024,	 /* Preamble length. Used in TX only. */
	DWT_PAC32,		 /* Preamble acquisition chunk size. Used in RX only. */
	9,				 /* TX preamble code. Used in TX only. */
	9,				 /* RX preamble code. Used in RX only. */
	1,				 /* 0 to use standard SFD, 1 to use non-standard SFD. */
	DWT_BR_110K,	 /* Data rate. */
	DWT_PHRMODE_STD, /* PHY header mode. */
	(1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'B', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'C', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2//序列帧
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* 用于存储接收到的消息的缓冲区。

*其大小已调整为该示例代码预期处理的最长帧*/
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];
static uint8 rx_byte[1];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2750
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 3300
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 8

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

/* 空气中光速约为 \(299,792,458\) 米每秒。通常可简化为 \(3 \times 10^8\) 米每秒。 */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* String used to display measured distance on LCD screen (16 characters maximum). */
char dist_str[16] = {0};

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
extern uint8_t recive[2];
int main(void)
{
	/* LED Init */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_13);

	/* Preiph Init */
	delay_init();
	UART_Init();
	peripherals_init();

	UART_SendStr(USART1, "Start DW1000 Init\n");
	/* DW Init */
	reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
	port_set_dw1000_slowrate();
	if (dwt_initialise(DWT_LOADUCODE) == -1)
	{
		while (1)
		{
			UART_SendStr(USART1, "Init Fail\n");
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			deca_sleep(1000);
		}
	}
	port_set_dw1000_fastrate();
	dwt_configure(&config);
	dwt_setleds(1); // 允许dwt1000开关灯

	/* Apply default antenna delay value. See NOTE 1 below. */
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	/* Set preamble timeout for expected frames. See NOTE 6 below. */
	dwt_setpreambledetecttimeout(PRE_TIMEOUT);

	/* Loop forever responding to ranging requests. */
	while (1)
	{
		/* Clear reception timeout to start next ranging process. */
		dwt_setrxtimeout(0);

		/* Activate reception immediately. */
		dwt_rxenable(DWT_START_RX_IMMEDIATE);

		/* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
		{
		};

		if (status_reg & SYS_STATUS_RXFCG)
		{
			uint32 frame_len;
			
			/* Clear good RX frame event in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

			/* A frame has been received, read it into the local buffer. */
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
			if (frame_len <= RX_BUFFER_LEN)
			{
				dwt_readrxdata(rx_buffer, frame_len, 0);
			}

			/* Check that the frame is a poll sent by "DS TWR initiator" example.
			 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
			rx_buffer[ALL_MSG_SN_IDX] = 0;
			if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
			{
				uint32 resp_tx_time;
				int ret;

				/* Retrieve poll reception timestamp. */
				poll_rx_ts = get_rx_timestamp_u64();//获取64位poll过程时间

				/* Set send time for response. See NOTE 9 below. */
				resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;//计算延时时间加poll时间，得到resp时间，中间夹带微秒单位转化为dwt单位的过程
				dwt_setdelayedtrxtime(resp_tx_time);//设置延时发送时间，此时还未设置resp内容

				/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. *///疑似第二对话阶段
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);//此函数设置在帧传输后接收器开启的延迟时间
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);//它控制接收器在接收使能命令后保持开启的时间。如果在设定的时间内没有接收到信号，接收器将自动关闭。通过设置超时时间，可以有效管理接收器的功耗和响应时间

				/* Write and send the response message. See NOTE 10 below.*/
				tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;//帧序列号，在每次传输后递增。数组的第三位（下标是2）是序列帧号
				dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. *///第三位参数表示偏移量为0
				dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);		  /* Zero offset in TX buffer, ranging. *///此API函数在发送帧之前配置TX帧控制寄存器。控制tx帧的发送细节。第二个参数是帧的偏移量，第三个参数0/1：非测距/测距
				ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);//此调用启动传输，输入参数指示使用的TX模式（延时模式、立即发送、是否期望响应等），这里是延时发送后打开接收器等待响应

				/* 如果dwt_starttx()返回错误，请放弃此次测距交换并继续进行下一个 */
				if (ret == DWT_ERROR)
				{
					
					continue;
				}

				/* 轮询接收预期的“最终”帧或错误/超时。请参见下面的NOTE 8。. */
				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{//status_reg状态寄存器。第一位：读取状态寄存器，第二位：校验帧序列，第三位：用户定义的接收超时（帧等待超时和前导码检测超时）掩码，第四位：所有接收错误掩码
				};

				/* 在响应消息传输后递增帧序列号（模256） */
				frame_seq_nb++;

				if (status_reg & SYS_STATUS_RXFCG)//状态寄存器正常，校验帧序列正常
				{
					/* 清除DW1000状态寄存器中的良好接收帧事件和发送的传输帧. */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

					/* 已接收到一帧，将其读取到本地缓冲区中. */
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;//接收帧信息（在双缓冲区设置中）+接收帧长度（这段代码的目的是读取接收到的帧的长度，并通过掩码获取相关的信息。）
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);//读取回应
						//Serial_Printf("%s",rx_buffer);
					}

					/*检查该帧是否为“DS TWR 发起者”示例发送的最终消息。
					*由于在此示例中帧的序列号字段未使用，因此可以将其置为零，以简化帧的验证。. */
					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)//比对检查rx消息
					{
						uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
						uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
						double Ra, Rb, Da, Db;
						int64 tof_dtu;

						/* 获取响应传输和最终接收的时间戳。 */
						resp_tx_ts = get_tx_timestamp_u64();
						final_rx_ts = get_rx_timestamp_u64();

						/* 获取嵌入最终消息中的时间戳。 */
						final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
						final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
						final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

						/*计算飞行时间。32位减法即使在时钟回绕的情况下也能给出正确的结果。请参见下面的注释12。
						*时钟回绕是指计算机系统中使用的时钟计数器达到其最大值后重新从零开始计数的现象。这通常发生在计数器的位数有限时，例如一个32位的计数器在计数到 (2^{32} - 1) 后会回绕到0。
						*在涉及时间测量的应用中，时钟回绕可能会导致错误的时间计算，但通过使用适当的算法（如32位减法），可以确保即使发生回绕，计算出的时间差仍然是正确的。*/
						poll_rx_ts_32 = (uint32)poll_rx_ts;
						resp_tx_ts_32 = (uint32)resp_tx_ts;
						final_rx_ts_32 = (uint32)final_rx_ts;
						Ra = (double)(resp_rx_ts - poll_tx_ts);
						Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
						Da = (double)(final_tx_ts - resp_rx_ts);
						Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
						tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

						tof = tof_dtu * DWT_TIME_UNITS;//dwt单位时间转秒
						distance = tof * SPEED_OF_LIGHT;
						
						/* 在LCD上显示计算出的距离。. */
						//sprintf(dist_str, "DIST: %3.2f m\n", distance);
						
						
						Serial_Printf("A:%f",distance);
						Serial_Printf("          B:%d\n",recive[0]|recive[1]<<8);
					}
				}
				else
				{
					UART_SendStr(USART1, "No FNAL Error\n");
					/* 清除 DW1000 状态寄存器中的接收错误/超时事件 */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

					/* 2024/10/7 00:34:34
					重置接收(RX)，以正确重新初始化低延迟检测(LDE)操作。. */
					dwt_rxreset();
				}
			}
		}
		else
		{
			/* 清除 DW1000 状态寄存器中的接收错误/超时事件 */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

			/* 重置接收(RX)，以正确重新初始化低延迟检测(LDE)操作。. */
			dwt_rxreset();
		}
	}
}

static uint64 get_tx_timestamp_u64(void)
{
	uint8 ts_tab[5];
	uint64 ts = 0;
	int i;
	dwt_readtxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--)
	{
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

static uint64 get_rx_timestamp_u64(void)
{
	uint8 ts_tab[5];
	uint64 ts = 0;
	int i;
	dwt_readrxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--)
	{
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
	int i;
	*ts = 0;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++)
	{
		*ts += ts_field[i] << (i * 8);
	}
}
