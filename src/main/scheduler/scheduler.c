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

#define SRC_MAIN_SCHEDULER_C_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "fc/core.h"
#include "fc/tasks.h"

#include "scheduler.h"

#define TASK_AVERAGE_EXECUTE_FALLBACK_US 30 // Default task average time if USE_TASK_STATISTICS is not defined
#define TASK_AVERAGE_EXECUTE_PADDING_US 5   // Add a little padding to the average execution time

// DEBUG_SCHEDULER, timings for:
// 0 - gyroUpdate()
// 1 - pidController()
// 2 - time spent in scheduler
// 3 - time spent executing check function

static FAST_DATA_ZERO_INIT task_t *currentTask = NULL;
static FAST_DATA_ZERO_INIT bool ignoreCurrentTaskTime;

static FAST_DATA_ZERO_INIT uint32_t totalWaitingTasks;
static FAST_DATA_ZERO_INIT uint32_t totalWaitingTasksSamples;

static FAST_DATA_ZERO_INIT bool calculateTaskStatistics;
FAST_DATA_ZERO_INIT uint16_t averageSystemLoadPercent = 0;

static FAST_DATA_ZERO_INIT int taskQueuePos = 0;
STATIC_UNIT_TESTED FAST_DATA_ZERO_INIT int taskQueueSize = 0;

static FAST_DATA int periodCalculationBasisOffset = offsetof(task_t, lastExecutedAtUs);
static FAST_DATA_ZERO_INIT bool gyroEnabled;

// No need for a linked list for the queue, since items are only inserted at startup

STATIC_UNIT_TESTED FAST_DATA_ZERO_INIT task_t* taskQueueArray[TASK_COUNT + 1]; // extra item for NULL pointer at end of queue

void queueClear(void)
{
    memset(taskQueueArray, 0, sizeof(taskQueueArray));
    taskQueuePos = 0;
    taskQueueSize = 0;
}

bool queueContains(task_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            return true;
        }
    }
    return false;
}

bool queueAdd(task_t *task)
{
    if ((taskQueueSize >= TASK_COUNT) || queueContains(task)) {
        return false;
    }
    for (int ii = 0; ii <= taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == NULL || taskQueueArray[ii]->staticPriority < task->staticPriority) {
            memmove(&taskQueueArray[ii+1], &taskQueueArray[ii], sizeof(task) * (taskQueueSize - ii));
            taskQueueArray[ii] = task;
            ++taskQueueSize;
            return true;
        }
    }
    return false;
}

bool queueRemove(task_t *task)
{
    for (int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            memmove(&taskQueueArray[ii], &taskQueueArray[ii+1], sizeof(task) * (taskQueueSize - ii));
            --taskQueueSize;
            return true;
        }
    }
    return false;
}

/*
 * Returns first item queue or NULL if queue empty
 */
FAST_CODE task_t *queueFirst(void)
{
    taskQueuePos = 0;
    return taskQueueArray[0]; // guaranteed to be NULL if queue is empty
}

/*
 * Returns next item in queue or NULL if at end of queue
 */
FAST_CODE task_t *queueNext(void)
{
    return taskQueueArray[++taskQueuePos]; // guaranteed to be NULL at end of queue
}

void taskSystemLoad(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // Calculate system load
    if (totalWaitingTasksSamples > 0) {
        averageSystemLoadPercent = 100 * totalWaitingTasks / totalWaitingTasksSamples;
        totalWaitingTasksSamples = 0;
        totalWaitingTasks = 0;
    }
#if defined(SIMULATOR_BUILD)
    averageSystemLoadPercent = 0;
#endif
}

#if defined(USE_TASK_STATISTICS)
timeUs_t checkFuncMaxExecutionTimeUs;
timeUs_t checkFuncTotalExecutionTimeUs;
timeUs_t checkFuncMovingSumExecutionTimeUs;
timeUs_t checkFuncMovingSumDeltaTimeUs;

void getCheckFuncInfo(cfCheckFuncInfo_t *checkFuncInfo)
{
    checkFuncInfo->maxExecutionTimeUs = checkFuncMaxExecutionTimeUs;
    checkFuncInfo->totalExecutionTimeUs = checkFuncTotalExecutionTimeUs;
    checkFuncInfo->averageExecutionTimeUs = checkFuncMovingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    checkFuncInfo->averageDeltaTimeUs = checkFuncMovingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
}
#endif

void getTaskInfo(taskId_e taskId, taskInfo_t * taskInfo)
{
    taskInfo->isEnabled = queueContains(getTask(taskId));
    taskInfo->desiredPeriodUs = getTask(taskId)->desiredPeriodUs;
    taskInfo->staticPriority = getTask(taskId)->staticPriority;
#if defined(USE_TASK_STATISTICS)
    taskInfo->taskName = getTask(taskId)->taskName;
    taskInfo->subTaskName = getTask(taskId)->subTaskName;
    taskInfo->maxExecutionTimeUs = getTask(taskId)->maxExecutionTimeUs;
    taskInfo->totalExecutionTimeUs = getTask(taskId)->totalExecutionTimeUs;
    taskInfo->averageExecutionTimeUs = getTask(taskId)->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    taskInfo->averageDeltaTimeUs = getTask(taskId)->movingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
    taskInfo->latestDeltaTimeUs = getTask(taskId)->taskLatestDeltaTimeUs;
    taskInfo->movingAverageCycleTimeUs = getTask(taskId)->movingAverageCycleTimeUs;
#if defined(USE_LATE_TASK_STATISTICS)
    taskInfo->lateCount = getTask(taskId)->lateCount;
    taskInfo->runCount = getTask(taskId)->runCount;
    taskInfo->execTime = getTask(taskId)->execTime;
#endif
#endif
}

void rescheduleTask(taskId_e taskId, timeDelta_t newPeriodUs)
{
    if (taskId == TASK_SELF) {
        task_t *task = currentTask;
        task->desiredPeriodUs = MAX(SCHEDULER_DELAY_LIMIT, newPeriodUs);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    } else if (taskId < TASK_COUNT) {
        task_t *task = getTask(taskId);
        task->desiredPeriodUs = MAX(SCHEDULER_DELAY_LIMIT, newPeriodUs);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    }
}

void setTaskEnabled(taskId_e taskId, bool enabled)
{
    if (taskId == TASK_SELF || taskId < TASK_COUNT) {
        task_t *task = taskId == TASK_SELF ? currentTask : getTask(taskId);
        if (enabled && task->taskFunc) {
            queueAdd(task);
        } else {
            queueRemove(task);
        }
    }
}

timeDelta_t getTaskDeltaTimeUs(taskId_e taskId)
{
    if (taskId == TASK_SELF) {
        return currentTask->taskLatestDeltaTimeUs;
    } else if (taskId < TASK_COUNT) {
        return getTask(taskId)->taskLatestDeltaTimeUs;
    } else {
        return 0;
    }
}

// Called by tasks executing what are known to be short states
void ignoreTaskTime()
{
    ignoreCurrentTaskTime = true;
}

void schedulerSetCalulateTaskStatistics(bool calculateTaskStatisticsToUse)
{
    calculateTaskStatistics = calculateTaskStatisticsToUse;
}

void schedulerResetTaskStatistics(taskId_e taskId)
{
#if defined(USE_TASK_STATISTICS)
    if (taskId == TASK_SELF) {
        currentTask->movingSumExecutionTimeUs = 0;
        currentTask->movingSumDeltaTimeUs = 0;
        currentTask->totalExecutionTimeUs = 0;
        currentTask->maxExecutionTimeUs = 0;
    } else if (taskId < TASK_COUNT) {
        getTask(taskId)->movingSumExecutionTimeUs = 0;
        getTask(taskId)->movingSumDeltaTimeUs = 0;
        getTask(taskId)->totalExecutionTimeUs = 0;
        getTask(taskId)->maxExecutionTimeUs = 0;
    }
#else
    UNUSED(taskId);
#endif
}

void schedulerResetTaskMaxExecutionTime(taskId_e taskId)
{
#if defined(USE_TASK_STATISTICS)
    if (taskId == TASK_SELF) {
        currentTask->maxExecutionTimeUs = 0;
    } else if (taskId < TASK_COUNT) {
        task_t *task = getTask(taskId);
        task->maxExecutionTimeUs = 0;
#if defined(USE_LATE_TASK_STATISTICS)
        task->lateCount = 0;
        task->runCount = 0;
#endif
    }
#else
    UNUSED(taskId);
#endif
}

#if defined(USE_TASK_STATISTICS)
void schedulerResetCheckFunctionMaxExecutionTime(void)
{
    checkFuncMaxExecutionTimeUs = 0;
}
#endif

void schedulerInit(void)
{
    calculateTaskStatistics = true;
    queueClear();
    queueAdd(getTask(TASK_SYSTEM));
}

void schedulerOptimizeRate(bool optimizeRate)
{
    periodCalculationBasisOffset = optimizeRate ? offsetof(task_t, lastDesiredAt) : offsetof(task_t, lastExecutedAtUs);
}

inline static timeUs_t getPeriodCalculationBasis(const task_t* task)
{
    if (task->staticPriority == TASK_PRIORITY_REALTIME) {
        return *(timeUs_t*)((uint8_t*)task + periodCalculationBasisOffset);
    } else {
        return task->lastExecutedAtUs;
    }
}

FAST_CODE timeUs_t schedulerExecuteTask(task_t *selectedTask, timeUs_t currentTimeUs)
{
    timeUs_t taskExecutionTimeUs = 0;

    if (selectedTask) {
        currentTask = selectedTask;
        ignoreCurrentTaskTime = false;
#if defined(USE_TASK_STATISTICS)
        float period = currentTimeUs - selectedTask->lastExecutedAtUs;
#endif
        selectedTask->lastExecutedAtUs = currentTimeUs;
        selectedTask->lastDesiredAt += (cmpTimeUs(currentTimeUs, selectedTask->lastDesiredAt) / selectedTask->desiredPeriodUs) * selectedTask->desiredPeriodUs;
        selectedTask->dynamicPriority = 0;

        // Execute task
#if defined(USE_TASK_STATISTICS)
        if (calculateTaskStatistics) {
            const timeUs_t currentTimeBeforeTaskCallUs = micros();
            selectedTask->taskFunc(currentTimeBeforeTaskCallUs);
            taskExecutionTimeUs = micros() - currentTimeBeforeTaskCallUs;
            if (!ignoreCurrentTaskTime) {
                selectedTask->taskLatestDeltaTimeUs = cmpTimeUs(currentTimeUs, selectedTask->lastStatsAtUs);
                selectedTask->lastStatsAtUs = currentTimeUs;
                selectedTask->movingSumExecutionTimeUs += taskExecutionTimeUs - selectedTask->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
                selectedTask->movingSumDeltaTimeUs += selectedTask->taskLatestDeltaTimeUs - selectedTask->movingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
                selectedTask->maxExecutionTimeUs = MAX(selectedTask->maxExecutionTimeUs, taskExecutionTimeUs);
            }
            selectedTask->totalExecutionTimeUs += taskExecutionTimeUs;   // time consumed by scheduler + task
            selectedTask->movingAverageCycleTimeUs += 0.05f * (period - selectedTask->movingAverageCycleTimeUs);
#if defined(USE_LATE_TASK_STATISTICS)
            selectedTask->runCount++;
#endif
        } else
#endif
        {
            selectedTask->taskLatestDeltaTimeUs = cmpTimeUs(currentTimeUs, selectedTask->lastExecutedAtUs);
            selectedTask->taskFunc(currentTimeUs);
        }
    }

    return taskExecutionTimeUs;
}

#if defined(UNIT_TEST)
task_t *unittest_scheduler_selectedTask;
uint8_t unittest_scheduler_selectedTaskDynamicPriority;
uint16_t unittest_scheduler_waitingTasks;

static void readSchedulerLocals(task_t *selectedTask, uint8_t selectedTaskDynamicPriority, uint16_t waitingTasks)
{
    unittest_scheduler_selectedTask = selectedTask;
    unittest_scheduler_selectedTaskDynamicPriority = selectedTaskDynamicPriority;
    unittest_scheduler_waitingTasks = waitingTasks;
}
#endif

FAST_CODE void scheduler(void)
{
#if defined(USE_LATE_TASK_STATISTICS)
    static task_t *lastTask;
#endif
    // Cache currentTime
    const timeUs_t schedulerStartTimeUs = micros();
    timeUs_t currentTimeUs = schedulerStartTimeUs;
    timeUs_t taskExecutionTimeUs = 0;
    task_t *selectedTask = NULL;
    uint16_t selectedTaskDynamicPriority = 0;
    uint16_t waitingTasks = 0;
    bool realtimeTaskRan = false;
    timeDelta_t gyroTaskDelayUs = 0;

    if (gyroEnabled) {
        // Realtime gyro/filtering/PID tasks get complete priority
        task_t *gyroTask = getTask(TASK_GYRO);
        const timeUs_t gyroExecuteTimeUs = getPeriodCalculationBasis(gyroTask) + gyroTask->desiredPeriodUs;
        gyroTaskDelayUs = cmpTimeUs(gyroExecuteTimeUs, currentTimeUs);  // time until the next expected gyro sample
        if (gyroTaskDelayUs <= 0) {
            taskExecutionTimeUs = schedulerExecuteTask(gyroTask, currentTimeUs);
            if (gyroFilterReady()) {
                taskExecutionTimeUs += schedulerExecuteTask(getTask(TASK_FILTER), currentTimeUs);
            }
            if (pidLoopReady()) {
                taskExecutionTimeUs += schedulerExecuteTask(getTask(TASK_PID), currentTimeUs);
            }
            currentTimeUs = micros();
            realtimeTaskRan = true;

#if defined(USE_LATE_TASK_STATISTICS)
           // Late, so make a note of the offending task
            if (gyroTaskDelayUs < -1) {
                if (lastTask) {
                    lastTask->lateCount++;
                }
            }
            lastTask = NULL;
#endif
        }
    }

    if (!gyroEnabled || realtimeTaskRan || (gyroTaskDelayUs > GYRO_TASK_GUARD_INTERVAL_US)) {
        // The task to be invoked

        // Update task dynamic priorities
        for (task_t *task = queueFirst(); task != NULL; task = queueNext()) {
            if (task->staticPriority != TASK_PRIORITY_REALTIME) {
                // Task has checkFunc - event driven
                if (task->checkFunc) {
#if defined(SCHEDULER_DEBUG)
                    const timeUs_t currentTimeBeforeCheckFuncCallUs = micros();
#else
                    const timeUs_t currentTimeBeforeCheckFuncCallUs = currentTimeUs;
#endif
                    // Increase priority for event driven tasks
                    if (task->dynamicPriority > 0) {
                        task->taskAgeCycles = 1 + ((currentTimeUs - task->lastSignaledAtUs) / task->desiredPeriodUs);
                        task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                        waitingTasks++;
                    } else if (task->checkFunc(currentTimeBeforeCheckFuncCallUs, cmpTimeUs(currentTimeBeforeCheckFuncCallUs, task->lastExecutedAtUs))) {
#if defined(SCHEDULER_DEBUG)
                        DEBUG_SET(DEBUG_SCHEDULER, 3, micros() - currentTimeBeforeCheckFuncCallUs);
#endif
#if defined(USE_TASK_STATISTICS)
                        if (calculateTaskStatistics) {
                            const uint32_t checkFuncExecutionTimeUs = micros() - currentTimeBeforeCheckFuncCallUs;
                            checkFuncMovingSumExecutionTimeUs += checkFuncExecutionTimeUs - checkFuncMovingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT;
                            checkFuncMovingSumDeltaTimeUs += task->taskLatestDeltaTimeUs - checkFuncMovingSumDeltaTimeUs / TASK_STATS_MOVING_SUM_COUNT;
                            checkFuncTotalExecutionTimeUs += checkFuncExecutionTimeUs;   // time consumed by scheduler + task
                            checkFuncMaxExecutionTimeUs = MAX(checkFuncMaxExecutionTimeUs, checkFuncExecutionTimeUs);
                        }
#endif
                        task->lastSignaledAtUs = currentTimeBeforeCheckFuncCallUs;
                        task->taskAgeCycles = 1;
                        task->dynamicPriority = 1 + task->staticPriority;
                        waitingTasks++;
                    } else {
                        task->taskAgeCycles = 0;
                    }
                } else {
                    // Task is time-driven, dynamicPriority is last execution age (measured in desiredPeriods)
                    // Task age is calculated from last execution
                    task->taskAgeCycles = ((currentTimeUs - getPeriodCalculationBasis(task)) / task->desiredPeriodUs);
                    if (task->taskAgeCycles > 0) {
                        task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                        waitingTasks++;
                    }
                }

                if (task->dynamicPriority > selectedTaskDynamicPriority) {
                    selectedTaskDynamicPriority = task->dynamicPriority;
                    selectedTask = task;
                }
            }
        }

        totalWaitingTasksSamples++;
        totalWaitingTasks += waitingTasks;

        if (selectedTask) {
            timeDelta_t taskRequiredTimeUs = TASK_AVERAGE_EXECUTE_FALLBACK_US;  // default average time if task statistics are not available
#if defined(USE_TASK_STATISTICS)
            if (calculateTaskStatistics) {
                taskRequiredTimeUs = selectedTask->movingSumExecutionTimeUs / TASK_STATS_MOVING_SUM_COUNT + TASK_AVERAGE_EXECUTE_PADDING_US;
#if defined(USE_LATE_TASK_STATISTICS)
                selectedTask->execTime = taskRequiredTimeUs;
#endif
            }
#endif
            // Add in the time spent so far in check functions and the scheduler logic
            taskRequiredTimeUs += cmpTimeUs(micros(), currentTimeUs);
            if (!gyroEnabled || realtimeTaskRan || (taskRequiredTimeUs < gyroTaskDelayUs)) {
                taskExecutionTimeUs += schedulerExecuteTask(selectedTask, currentTimeUs);
            } else {
                selectedTask = NULL;
            }

#if defined(USE_LATE_TASK_STATISTICS)
            lastTask = selectedTask;
#endif
        }
    }


#if defined(SCHEDULER_DEBUG)
    DEBUG_SET(DEBUG_SCHEDULER, 2, micros() - schedulerStartTimeUs - taskExecutionTimeUs); // time spent in scheduler
#else
    UNUSED(taskExecutionTimeUs);
#endif

#if defined(UNIT_TEST)
    readSchedulerLocals(selectedTask, selectedTaskDynamicPriority, waitingTasks);
#endif
}

void schedulerEnableGyro(void)
{
    gyroEnabled = true;
}

uint16_t getAverageSystemLoadPercent(void)
{
    return averageSystemLoadPercent;
}
