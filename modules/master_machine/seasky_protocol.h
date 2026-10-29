#ifndef __SEASKY_PROTOCOL_H
#define __SEASKY_PROTOCOL_H

#include <stdio.h>
#include <stdint.h>

#define PROTOCOL_CMD_ID 0XA5
#define OFFSET_BYTE 8 // 出数据段外，其他部分所占字节数

/* 通用救援车帧：SOF + LEN + CRC8 + CMD + SEQ + FLAGS + PAYLOAD + CRC16 */
#define RESCUE_PROTOCOL_HEADER_SIZE 8u
#define RESCUE_PROTOCOL_TAIL_SIZE 2u
#define RESCUE_PROTOCOL_MAX_PAYLOAD 128u

typedef enum
{
    RESCUE_CMD_UP_TELEMETRY = 0x0101,
    RESCUE_CMD_DOWN_CONTROL = 0x0201,
} Rescue_Command_e;

typedef struct
{
    uint16_t cmd_id;
    uint8_t seq;
    uint8_t flags;
    const uint8_t *payload;
    uint16_t payload_len;
} Rescue_Frame_s;

typedef struct
{
	struct
	{
		uint8_t sof;
		uint16_t data_length;
		uint8_t crc_check; // 帧头CRC校验
	} header;			   // 数据帧头
	uint16_t cmd_id;	   // 数据ID
	uint16_t frame_tail;   // 帧尾CRC校验
} protocol_rm_struct;


/*更新发送数据帧，并计算发送数据帧长度*/
void get_protocol_send_data(uint16_t send_id,		 // 信号id
							uint16_t flags_register, // 16位寄存器
							float *tx_data,			 // 待发送的float数据
							uint8_t float_length,	 // float的数据长度
							uint8_t *tx_buf,		 // 待发送的数据帧
							uint16_t *tx_buf_len);	 // 待发送的数据帧长度

/*接收数据处理*/
uint16_t get_protocol_info(uint8_t *rx_buf,			 // 接收到的原始数据
						   uint16_t *flags_register, // 接收数据的16位寄存器地址
						   uint8_t *rx_data);			 // 接收的float数据存储地址

uint16_t RescueProtocolPack(uint16_t cmd_id, uint8_t seq, uint8_t flags,
                            const void *payload, uint16_t payload_len,
                            uint8_t *tx_buf, uint16_t tx_capacity);
uint16_t RescueProtocolFrameLength(const uint8_t *rx_buf);
uint8_t RescueProtocolUnpack(const uint8_t *rx_buf, uint16_t rx_len,
                             Rescue_Frame_s *frame);

#endif
