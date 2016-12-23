/*
 * gyro_sync.c
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "sensor.h"
#include "accgyro.h"
#include "gyro_sync.h"

static uint8_t mpuDividerDrops;

bool gyroSyncCheckUpdate(gyroDev_t *gyro)
{
    if (!gyro->intStatus)
        return false;
    return gyro->intStatus(gyro);
}

uint32_t gyroSetSampleRate(uint8_t lpf, uint8_t gyroSyncDenominator)
{
    int gyroSamplePeriod;

    if (lpf == GYRO_LPF_256HZ || lpf == GYRO_LPF_NONE) {
        gyroSamplePeriod = 125;
    } else {
        gyroSamplePeriod = 1000;
        gyroSyncDenominator = 1; // Always full Sampling 1khz
    }

    // calculate gyro divider and targetLooptime (expected cycleTime)
    mpuDividerDrops  = gyroSyncDenominator - 1;
    const uint32_t targetLooptime = gyroSyncDenominator * gyroSamplePeriod;
    return targetLooptime;
}

uint8_t gyroMPU6xxxGetDividerDrops(void)
{
    return mpuDividerDrops;
}
