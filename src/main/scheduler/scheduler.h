/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/time.h"
#include "config/config.h"

#define TASK_PERIOD_HZ(hz) (1000000 / (hz))
#define TASK_PERIOD_MS(ms) ((ms) * 1000)
#define TASK_PERIOD_US(us) (us)

#define GYRO_TASK_GUARD_INTERVAL_US 10  // Don't run any other tasks if gyro task will be run soon

#if defined(USE_TASK_STATISTICS)
#define TASK_STATS_MOVING_SUM_COUNT 32
#endif

#define LOAD_PERCENTAGE_ONE 100

typedef enum {
    TASK_PRIORITY_REALTIME = -1, // Task will be run outside the scheduler logic
    TASK_PRIORITY_IDLE = 0,      // Disables dynamic scheduling, task is executed only if no other task is active this cycle
    TASK_PRIORITY_LOW = 1,
    TASK_PRIORITY_MEDIUM = 3,
    TASK_PRIORITY_MEDIUM_HIGH = 4,
    TASK_PRIORITY_HIGH = 5,
    TASK_PRIORITY_MAX = 255
} taskPriority_e;

typedef struct {
    timeUs_t     maxExecutionTimeUs;
    timeUs_t     totalExecutionTimeUs;
    timeUs_t     averageExecutionTimeUs;
    timeUs_t     averageDeltaTimeUs;
} cfCheckFuncInfo_t;

typedef struct {
    const char * taskName;
    const char * subTaskName;
    bool         isEnabled;
    int8_t       staticPriority;
    timeDelta_t  desiredPeriodUs;
    timeDelta_t  latestDeltaTimeUs;
    timeUs_t     maxExecutionTimeUs;
    timeUs_t     totalExecutionTimeUs;
    timeUs_t     averageExecutionTimeUs;
    timeUs_t     averageDeltaTimeUs;
    float        movingAverageCycleTimeUs;
#if defined(USE_LATE_TASK_STATISTICS)
    uint32_t     runCount;
    uint32_t     lateCount;
    timeUs_t     execTime;
#endif
} taskInfo_t;

typedef enum {
    /* Actual tasks */
    TASK_SYSTEM = 0,
    TASK_MAIN,
    TASK_GYRO,
    TASK_FILTER,
    TASK_PID,
    TASK_ACCEL,
    TASK_ATTITUDE,
    TASK_RX,
    TASK_SERIAL,
    TASK_DISPATCH,
    TASK_BATTERY_VOLTAGE,
    TASK_BATTERY_CURRENT,
    TASK_BATTERY_ALERTS,
#ifdef USE_BEEPER
    TASK_BEEPER,
#endif
#ifdef USE_GPS
    TASK_GPS,
#endif
#ifdef USE_MAG
    TASK_COMPASS,
#endif
#ifdef USE_BARO
    TASK_BARO,
#endif
#ifdef USE_RANGEFINDER
    TASK_RANGEFINDER,
#endif
#if defined(USE_BARO) || defined(USE_GPS)
    TASK_ALTITUDE,
#endif
#ifdef USE_DASHBOARD
    TASK_DASHBOARD,
#endif
#ifdef USE_TELEMETRY
    TASK_TELEMETRY,
#endif
#ifdef USE_LED_STRIP
    TASK_LEDSTRIP,
#endif
#ifdef USE_TRANSPONDER
    TASK_TRANSPONDER,
#endif
#ifdef USE_STACK_CHECK
    TASK_STACK_CHECK,
#endif
#ifdef USE_OSD
    TASK_OSD,
#endif
#ifdef USE_BST
    TASK_BST_MASTER_PROCESS,
#endif
#ifdef USE_ESC_SENSOR
    TASK_ESC_SENSOR,
#endif
#ifdef USE_CMS
    TASK_CMS,
#endif
#ifdef USE_VTX_CONTROL
    TASK_VTXCTRL,
#endif
#ifdef USE_CAMERA_CONTROL
    TASK_CAMCTRL,
#endif

#ifdef USE_RCDEVICE
    TASK_RCDEVICE,
#endif

#ifdef USE_ADC_INTERNAL
    TASK_ADC_INTERNAL,
#endif

#ifdef USE_PINIOBOX
    TASK_PINIOBOX,
#endif

#ifdef USE_CRSF_V3
    TASK_SPEED_NEGOTIATION,
#endif

    /* Count of real tasks */
    TASK_COUNT,

    /* Service task IDs */
    TASK_NONE = TASK_COUNT,
    TASK_SELF
} taskId_e;

typedef struct {
    // Configuration
#if defined(USE_TASK_STATISTICS)
    const char * taskName;
    const char * subTaskName;
#endif
    bool (*checkFunc)(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
    void (*taskFunc)(timeUs_t currentTimeUs);
    timeDelta_t desiredPeriodUs;        // target period of execution
    const int8_t staticPriority;        // dynamicPriority grows in steps of this size

    // Scheduling
    uint16_t dynamicPriority;           // measurement of how old task was last executed, used to avoid task starvation
    uint16_t taskAgeCycles;
    timeDelta_t taskLatestDeltaTimeUs;
    timeUs_t lastExecutedAtUs;          // last time of invocation
    timeUs_t lastSignaledAtUs;          // time of invocation event for event-driven tasks
    timeUs_t lastDesiredAt;             // time of last desired execution

#if defined(USE_TASK_STATISTICS)
    // Statistics
    float    movingAverageCycleTimeUs;
    timeUs_t movingSumExecutionTimeUs;  // moving sum over 32 samples
    timeUs_t movingSumDeltaTimeUs;      // moving sum over 32 samples
    timeUs_t maxExecutionTimeUs;
    timeUs_t totalExecutionTimeUs;      // total time consumed by task since boot
    timeUs_t lastStatsAtUs;             // time of last stats gathering for rate calculation
#if defined(USE_LATE_TASK_STATISTICS)
    uint32_t runCount;
    uint32_t lateCount;
    timeUs_t execTime;
#endif
#endif
} task_t;

void getCheckFuncInfo(cfCheckFuncInfo_t *checkFuncInfo);
void getTaskInfo(taskId_e taskId, taskInfo_t *taskInfo);
void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs);
void setTaskEnabled(taskId_e taskId, bool newEnabledState);
timeDelta_t getTaskDeltaTimeUs(taskId_e taskId);
void ignoreTaskTime();
void schedulerSetCalulateTaskStatistics(bool calculateTaskStatistics);
void schedulerResetTaskStatistics(taskId_e taskId);
void schedulerResetTaskMaxExecutionTime(taskId_e taskId);
void schedulerResetCheckFunctionMaxExecutionTime(void);

void schedulerInit(void);
void scheduler(void);
timeUs_t schedulerExecuteTask(task_t *selectedTask, timeUs_t currentTimeUs);
void taskSystemLoad(timeUs_t currentTimeUs);
void schedulerOptimizeRate(bool optimizeRate);
void schedulerEnableGyro(void);
uint16_t getAverageSystemLoadPercent(void);
