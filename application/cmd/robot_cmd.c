// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/

static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
// 控制结构体
// 反馈结构体

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

void RobotCMDInit()
{
#ifdef REMOTE_CONTROL
    rc_data = RemoteControlInit(&huart5);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#endif
#if defined(VISION_USE_UART) || defined(VISION_USE_VCP)
    VisionInit(&huart9); // 视觉通信串口
#endif

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));


    robot_state = START_LOCK; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}
#ifdef REMOTE_CONTROL
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    if (!RemoteControlIsOnline())
    {
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        chassis_cmd_send.wz = 0.0f;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        gimbal_cmd_send.claw_mode = CLAW_RELEASE;
        return;
    }

    // 当前底盘为差速结构: 右摇杆竖直控制前进,水平控制旋转.
    chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_r1;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 10.0f * (float)rc_data[TEMP].rc.rocker_r_;
    chassis_cmd_send.chassis_mode = CAHSSIS_MOVE;

    // 右侧开关向上控制夹爪闭合.
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
        gimbal_cmd_send.claw_mode = CLAW_LOCK;
    else
        gimbal_cmd_send.claw_mode = CLAW_RELEASE;

}
#endif

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    // if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    // {
    //     robot_state = ROBOT_STOP;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    //     chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    //     LOGERROR("[CMD] emergency stop!");
    // }
    // // 遥控器右侧开关为[上],恢复正常运行
    // if (switch_is_up(rc_data[TEMP].rc.switch_right))
    // {
    //     robot_state = ROBOT_READY;
    //     LOGINFO("[CMD] reinstate, robot ready");
    // }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);




#ifdef REMOTE_CONTROL
    RemoteControlSet();
#endif

    // EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
