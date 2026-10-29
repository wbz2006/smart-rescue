/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include <math.h>
#include <string.h>

static USARTInstance *vision_usart_instance;

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
static Rescue_Control_s rescue_control;
static uint32_t rescue_control_tick;
static uint8_t rescue_control_received;
static uint8_t rescue_tx_seq;

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

#define RESCUE_RX_ACCUM_SIZE (RESCUE_RECV_SIZE * 2U)
static uint8_t rescue_rx_accum[RESCUE_RX_ACCUM_SIZE];
static uint16_t rescue_rx_accum_len;

static uint8_t IsRescueCommand(uint16_t cmd_id)
{
    return (uint8_t)(cmd_id == RESCUE_CMD_UP_IMU ||
                     cmd_id == RESCUE_CMD_UP_WHEEL ||
                     cmd_id == RESCUE_CMD_UP_STATUS ||
                     cmd_id == RESCUE_CMD_DOWN_CONTROL);
}

static void DecodeRescueControl(const Rescue_Frame_s *frame)
{
    Rescue_Control_s candidate;

    if (frame->cmd_id != RESCUE_CMD_DOWN_CONTROL ||
        frame->payload_len != sizeof(candidate))
        return;

    memcpy(&candidate, frame->payload, sizeof(candidate));
    if (!isfinite(candidate.vx_mm_s) || !isfinite(candidate.vy_mm_s) ||
        !isfinite(candidate.wz_rad_s) || candidate.chassis_mode > CAHSSIS_MOVE ||
        candidate.claw_mode > CLAW_RELEASE || candidate.camera_pos > CAMERA_LOAD)
        return;

    rescue_control = candidate;
    rescue_control_tick = HAL_GetTick();
    rescue_control_received = 1U;
}

static void DecodeRescueBuffer(void)
{
    uint16_t offset = 0U;

    while (offset < rescue_rx_accum_len)
    {
        while (offset < rescue_rx_accum_len && rescue_rx_accum[offset] != PROTOCOL_CMD_ID)
            ++offset;
        if (rescue_rx_accum_len - offset < 6U)
            break;

        uint16_t cmd_id = (uint16_t)rescue_rx_accum[offset + 4U] |
                          ((uint16_t)rescue_rx_accum[offset + 5U] << 8);
        uint16_t frame_len;
        if (IsRescueCommand(cmd_id))
        {
            frame_len = RescueProtocolFrameLength(rescue_rx_accum + offset);
            if (frame_len == 0U)
            {
                ++offset;
                continue;
            }
            if (rescue_rx_accum_len - offset < frame_len)
                break;

            Rescue_Frame_s frame;
            if (RescueProtocolUnpack(rescue_rx_accum + offset, frame_len, &frame))
            {
                DecodeRescueControl(&frame);
                DaemonReload(vision_daemon_instance);
                offset = (uint16_t)(offset + frame_len);
                continue;
            }
        }
        else
        {
            uint16_t data_len = (uint16_t)rescue_rx_accum[offset + 1U] |
                                ((uint16_t)rescue_rx_accum[offset + 2U] << 8);
            if (data_len < 2U || data_len > VISION_RECV_SIZE - OFFSET_BYTE)
            {
                ++offset;
                continue;
            }

            frame_len = (uint16_t)(OFFSET_BYTE + data_len);
            if (rescue_rx_accum_len - offset < frame_len)
                break;

            uint16_t flags_register;
            if (frame_len == VISION_RECV_SIZE &&
                get_protocol_info(rescue_rx_accum + offset, &flags_register,
                                  (uint8_t *)&recv_data.pitch) != 0U)
            {
                DaemonReload(vision_daemon_instance);
                offset = (uint16_t)(offset + frame_len);
                continue;
            }
        }

        ++offset;
    }

    if (offset != 0U)
    {
        rescue_rx_accum_len = (uint16_t)(rescue_rx_accum_len - offset);
        if (rescue_rx_accum_len != 0U)
            memmove(rescue_rx_accum, rescue_rx_accum + offset, rescue_rx_accum_len);
    }
}



/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    uint16_t recv_len = vision_usart_instance->recv_len;
    if (recv_len > RESCUE_RX_ACCUM_SIZE)
        return;
    if (recv_len > RESCUE_RX_ACCUM_SIZE - rescue_rx_accum_len)
        rescue_rx_accum_len = 0U;
    memcpy(rescue_rx_accum + rescue_rx_accum_len, vision_usart_instance->recv_buff, recv_len);
    rescue_rx_accum_len = (uint16_t)(rescue_rx_accum_len + recv_len);
    DecodeRescueBuffer();
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = RESCUE_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 20,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_UART

uint8_t RescueSendFrame(uint16_t cmd_id, uint8_t flags, const void *payload, uint16_t payload_len)
{
#ifdef VISION_USE_UART
    static uint8_t send_buff[RESCUE_SEND_SIZE];
    uint16_t tx_len;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (vision_usart_instance == NULL || !USARTIsReady(vision_usart_instance))
    {
        __set_PRIMASK(primask);
        return 0U;
    }

    tx_len = RescueProtocolPack(cmd_id, rescue_tx_seq, flags, payload, payload_len,
                                send_buff, sizeof(send_buff));
    if (tx_len == 0U)
    {
        __set_PRIMASK(primask);
        return 0U;
    }

    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_IT);
    ++rescue_tx_seq;
    __set_PRIMASK(primask);
    return 1U;
#else
    UNUSED(cmd_id);
    UNUSED(flags);
    UNUSED(payload);
    UNUSED(payload_len);
    return 0U;
#endif
}

uint8_t RescueGetControl(Rescue_Control_s *control)
{
    uint8_t received;
    uint32_t primask;

    if (control == NULL)
        return 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    *control = rescue_control;
    received = rescue_control_received;
    __set_PRIMASK(primask);
    return received;
}

uint8_t RescueControlIsOnline(void)
{
    uint8_t online;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    online = (uint8_t)(rescue_control_received &&
                       (uint32_t)(HAL_GetTick() - rescue_control_tick) < 200U);
    __set_PRIMASK(primask);
    return online;
}

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    uint16_t flag_register;
    get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    // TODO: code to resolve flag_register;
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USBTransmit(send_buff, tx_len);
}

#endif // VISION_USE_VCP
