#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H


/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz运行;上下位机通信按100Hz更新
 * 
 */
void RobotCMDTask();

#endif // !ROBOT_CMD_H
