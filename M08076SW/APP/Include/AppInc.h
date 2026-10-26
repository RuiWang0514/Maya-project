/******************************************************************************
 * @file    AppInc.h
 * @brief   APP - Aggregation header for the application layer
 * @layer   APP
 * @note    Include this single header to get all APP-layer declarations.
 ******************************************************************************/
#ifndef APP_INC_H_
#define APP_INC_H_

#include "fault_detect.h"
#include "canHandle.h"
#include "MotorCtrl.h"
#include "trip_zone.h"
#include "Background_task.h"
#include "can.h"
#include "pid_gain_schedule.h"   /* 转速分段线性插值 PID 增益调度 */

#endif /* APP_INC_H_ */
