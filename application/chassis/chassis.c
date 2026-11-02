/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        底盘采用右手系:前方为X正方向,左侧为Y正方向,逆时针航向为正。
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "serial_print.h"
#include "dap_scope.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define MOTOR_DEGREE_PER_MM (REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL)


static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

#ifdef CHASSIS_DRIVE_MECANUM
    static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
    static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
#endif
#ifdef CHASSIS_DRIVE_DIFFERENTIAL
    static DJIMotorInstance *motor_l, *motor_r;
    static float vt_l,vt_r;
    static uint32_t last_left_ecd, last_right_ecd;
    static uint8_t odometry_ecd_initialized;
    static uint32_t odometry_dwt_count;
    static float last_imu_yaw;
    static uint8_t last_imu_ready;
#endif

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘

#ifndef CHASSIS_SCOPE_ENABLE
#define CHASSIS_SCOPE_ENABLE 1
#endif

#ifndef CHASSIS_SERIAL_DEBUG_ENABLE
#define CHASSIS_SERIAL_DEBUG_ENABLE 0
#endif

void ChassisInit()
{
#ifdef CHASSIS_DRIVE_MECANUM
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 8, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M2006,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);
#endif
#ifdef CHASSIS_DRIVE_DIFFERENTIAL
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M2006,
    };

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_l = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_r = DJIMotorInit(&chassis_motor_config);
#endif

    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#ifdef CHASSIS_DRIVE_MECANUM
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    if (chassis_cmd_recv.metric_mode)
    {
        vt_lf = (-chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER * RAD_2_DEGREE) * MOTOR_DEGREE_PER_MM;
        vt_rf = (-chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER * RAD_2_DEGREE) * MOTOR_DEGREE_PER_MM;
        vt_lb = (chassis_vx - chassis_vy - chassis_cmd_recv.wz * LB_CENTER * RAD_2_DEGREE) * MOTOR_DEGREE_PER_MM;
        vt_rb = (chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER * RAD_2_DEGREE) * MOTOR_DEGREE_PER_MM;
    }
    else
    {
        vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
        vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
        vt_lb = chassis_vx - chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
        vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
    }
}
#endif
#ifdef CHASSIS_DRIVE_DIFFERENTIAL
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void DifferentialCalculate()
{
    if (chassis_cmd_recv.metric_mode)
    {
        vt_l = (-chassis_vx + chassis_cmd_recv.wz * HALF_TRACK_WIDTH) * MOTOR_DEGREE_PER_MM;
        vt_r = (-chassis_vx - chassis_cmd_recv.wz * HALF_TRACK_WIDTH) * MOTOR_DEGREE_PER_MM;
    }
    else
    {
        vt_l = -chassis_vx - chassis_cmd_recv.wz * HALF_TRACK_WIDTH * DEGREE_2_RAD;
        vt_r = -chassis_vx + chassis_cmd_recv.wz * HALF_TRACK_WIDTH * DEGREE_2_RAD;
    }
}
#endif
/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void ChassisOutput()
{
#ifdef CHASSIS_DRIVE_MECANUM
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
#endif
#ifdef CHASSIS_DRIVE_DIFFERENTIAL
    DJIMotorSetRef(motor_l, vt_l);
    DJIMotorSetRef(motor_r, vt_r);
#endif
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
#ifdef CHASSIS_DRIVE_DIFFERENTIAL
    const float wheel_circumference_mm = PERIMETER_WHEEL;
    const float motor_angle_to_wheel_mm = wheel_circumference_mm / (360.0f * REDUCTION_RATIO_WHEEL);
    const float encoder_count_to_wheel_mm = wheel_circumference_mm / (8192.0f * REDUCTION_RATIO_WHEEL);
    DJI_Motor_Measure_s left, right;
    INS_Feedback_s imu;
    taskENTER_CRITICAL();
    left = motor_l->measure;
    right = motor_r->measure;
    INS_GetFeedback(&imu);
    const uint32_t now = HAL_GetTick();
    taskEXIT_CRITICAL();
    const float sample_dt = DWT_GetDeltaT(&odometry_dwt_count);
    chassis_feedback_data.timestamp_ms = now;
    chassis_feedback_data.imu = imu;
    const uint8_t wheels_ready = left.initialized && right.initialized &&
        (uint32_t)(now - left.timestamp_ms) < ODOM_FEEDBACK_TIMEOUT_MS &&
        (uint32_t)(now - right.timestamp_ms) < ODOM_FEEDBACK_TIMEOUT_MS;
    const uint8_t imu_ready = imu.ready &&
        (uint32_t)(now - imu.timestamp_ms) < ODOM_FEEDBACK_TIMEOUT_MS;
    const float imu_yaw = imu.attitude.YawTotalAngle * DEGREE_2_RAD * ODOM_IMU_YAW_SIGN;
    if (!wheels_ready)
    {
        INS_SetChassisStationary(0);
        odometry_ecd_initialized = 0;
        last_imu_ready = 0;
        chassis_feedback_data.left_wheel_speed_mm_s = 0;
        chassis_feedback_data.right_wheel_speed_mm_s = 0;
        chassis_feedback_data.vx_mm_s = 0;
        chassis_feedback_data.wz_rad_s = 0;
        return;
    }

    if (!odometry_ecd_initialized)
    {
        last_left_ecd = left.total_ecd;
        last_right_ecd = right.total_ecd;
        last_imu_yaw = imu_yaw;
        last_imu_ready = imu_ready;
        odometry_ecd_initialized = 1;
        return;
    }

    const int32_t left_delta_ecd = (int32_t)(left.total_ecd - last_left_ecd);
    const int32_t right_delta_ecd = (int32_t)(right.total_ecd - last_right_ecd);
    last_left_ecd = left.total_ecd;
    last_right_ecd = right.total_ecd;

    // 电机反转配置使左右轮前进时编码器方向相反,统一转换为前进为正。
    const float left_distance_mm = (float)left_delta_ecd * encoder_count_to_wheel_mm * ODOM_LEFT_DISTANCE_SCALE;
    const float right_distance_mm = -(float)right_delta_ecd * encoder_count_to_wheel_mm * ODOM_RIGHT_DISTANCE_SCALE;
    const float left_speed_mm_s = left.speed_aps * motor_angle_to_wheel_mm * ODOM_LEFT_DISTANCE_SCALE;
    const float right_speed_mm_s = -right.speed_aps * motor_angle_to_wheel_mm * ODOM_RIGHT_DISTANCE_SCALE;
    const uint8_t wheels_stationary = left_delta_ecd == 0 && right_delta_ecd == 0 &&
        fabsf(left_speed_mm_s) < ODOM_STATIONARY_SPEED_MM_S &&
        fabsf(right_speed_mm_s) < ODOM_STATIONARY_SPEED_MM_S;
    INS_SetChassisStationary(wheels_stationary);

    const float wheel_delta_yaw = (right_distance_mm - left_distance_mm) / ODOM_TRACK_WIDTH_MM;
    float delta_yaw_rad = wheel_delta_yaw;
    if (imu_ready && last_imu_ready && sample_dt > 1e-6f)
    {
        const float imu_delta_yaw = imu_yaw - last_imu_yaw;
        // 使用累计航向增量,避免直接把坡面上的机体系Gyro[Z]当作航向角速度。
        const float disagreement = fabsf(wheel_delta_yaw - imu_delta_yaw) / sample_dt;
        const float imu_weight = disagreement > ODOM_SLIP_THRESHOLD_RAD_S ? 1.0f : ODOM_IMU_WEIGHT;
        delta_yaw_rad = imu_weight * imu_delta_yaw + (1.0f - imu_weight) * wheel_delta_yaw;
        if (imu.stationary && wheels_stationary)
            delta_yaw_rad = 0;
    }
    last_imu_yaw = imu_yaw;
    last_imu_ready = imu_ready;
    const float distance_mm = (left_distance_mm + right_distance_mm) * 0.5f;
    const float midpoint_yaw_rad = chassis_feedback_data.yaw_rad + delta_yaw_rad * 0.5f;

    chassis_feedback_data.left_wheel_distance_mm += left_distance_mm;
    chassis_feedback_data.right_wheel_distance_mm += right_distance_mm;
    chassis_feedback_data.x_mm += distance_mm * arm_cos_f32(midpoint_yaw_rad);
    chassis_feedback_data.y_mm += distance_mm * arm_sin_f32(midpoint_yaw_rad);
    chassis_feedback_data.yaw_rad += delta_yaw_rad;

    chassis_feedback_data.left_wheel_speed_mm_s = left_speed_mm_s;
    chassis_feedback_data.right_wheel_speed_mm_s = right_speed_mm_s;
    chassis_feedback_data.vx_mm_s = (left_speed_mm_s + right_speed_mm_s) * 0.5f;
    chassis_feedback_data.wz_rad_s = delta_yaw_rad / sample_dt;
#endif
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
    chassis_vx = chassis_cmd_recv.vx;
    chassis_vy = chassis_cmd_recv.vy;
    
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
#ifdef CHASSIS_DRIVE_MECANUM
    DJIMotorStop(motor_lf);
    DJIMotorStop(motor_rf);
    DJIMotorStop(motor_lb);
    DJIMotorStop(motor_rb);
#endif
#ifdef CHASSIS_DRIVE_DIFFERENTIAL
    DJIMotorStop(motor_l);
    DJIMotorStop(motor_r);
#endif
    }
    else
    { // 正常工作
#ifdef CHASSIS_DRIVE_MECANUM
    DJIMotorEnable(motor_lf);
    DJIMotorEnable(motor_rf);
    DJIMotorEnable(motor_lb);
    DJIMotorEnable(motor_rb);
#endif
#ifdef CHASSIS_DRIVE_DIFFERENTIAL
    DJIMotorEnable(motor_l);
    DJIMotorEnable(motor_r);
#endif
    }

    // 根据控制模式进行正运动学解算,计算底盘输出
#ifdef CHASSIS_DRIVE_MECANUM
    MecanumCalculate();
#endif
#ifdef CHASSIS_DRIVE_DIFFERENTIAL
    DifferentialCalculate();
#endif

    ChassisOutput();
    EstimateSpeed();
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);

#ifdef CHASSIS_DRIVE_DIFFERENTIAL
#if CHASSIS_SERIAL_DEBUG_ENABLE
    // UART7 @921600, VOFA+ FireWater: 左轮目标/反馈/误差, 右轮目标/反馈/误差, 单位为度/s.
    // 使用PID实际参考值, 包含电机反转设置; 每20ms输出一次, 避免每轮控制都阻塞发送.
    static uint32_t last_print_tick;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_print_tick) >= 20U)
    {
        last_print_tick = now;
        SerialPrintf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
                     motor_l->motor_controller.speed_PID.Ref,
                     motor_l->motor_controller.speed_PID.Measure,
                     motor_l->motor_controller.speed_PID.Err,
                     motor_r->motor_controller.speed_PID.Ref,
                     motor_r->motor_controller.speed_PID.Measure,
                     motor_r->motor_controller.speed_PID.Err);
    }
#endif

#if CHASSIS_SCOPE_ENABLE
    const float left_ref = motor_l->motor_controller.speed_PID.Ref;
    const float left_measure = motor_l->motor_controller.speed_PID.Measure;
    const float right_ref = motor_r->motor_controller.speed_PID.Ref;
    const float right_measure = motor_r->motor_controller.speed_PID.Measure;
    SCOPE_PLOT(left_ref, left_measure, right_ref, right_measure);
#endif
#endif

}
