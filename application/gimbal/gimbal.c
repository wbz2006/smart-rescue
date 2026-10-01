#include "gimbal.h"
#include "robot_def.h"
#include "servo_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据
static ServoInstance *claw , *camera;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU

static void ClawSetMode(claw_mode_e mode)
{
    static claw_mode_e last_mode = CLAW_ZERO_POS;

    if (mode == last_mode)
        return;

    switch (mode)
    {
    case CLAW_LOCK:
        ServoSetAngle(claw, 0.0f);
        break;
    case CLAW_RELEASE:
        ServoSetAngle(claw, 90.0f);
        break;
    default:
        return;
    }

    last_mode = mode;
}

static void CameraSetMode(camera_pos_e mode)
{
    static camera_pos_e last_mode = CAMERA_ZERO_POS;

    if (mode == last_mode)
        return;

    switch (mode)
    {
    case CAMERA_LOAD:
        ServoSetAngle(camera, 0.0f);
        break;
    case CAMERA_ZERO_POS:
        ServoSetAngle(camera, 90.0f);
        break;
    default:
        return;
    }

    last_mode = mode;
}

void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    Servo_Init_Config_s claw_config = {
        .servo_type = PWM_Servo,
        .servo_id = 0,
        .initial_angle = 90.0f,
        .pwm_init_config = {
            .htim = &htim1,
            .channel = TIM_CHANNEL_3,
            .period = 0.02f,     // 20 ms，即 50 Hz
        },
    };
    Servo_Init_Config_s camera_config = {
        .servo_type = PWM_Servo,
        .servo_id = 1,
        .initial_angle = 90.0f,
        .pwm_init_config = {
            .htim = &htim1,
            .channel = TIM_CHANNEL_1,
            .period = 0.02f,     // 20 ms，即 50 Hz
        },
    };
    claw = ServoInit(&claw_config);
    camera = ServoInit(&camera_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    ClawSetMode(gimbal_cmd_recv.claw_mode);
    
    CameraSetMode(gimbal_cmd_recv.camera_pos);
    
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
