#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "tmc/ic/TMC2209/TMC2209.h"
#include "pid_ctrl.h"

// |================================================================================================ |
// |                                          Functions                                              |
// |================================================================================================ |

void tmc2209_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength);
uint8_t tmc2209_CRC8(uint8_t *datagram, size_t datagramLength);
void stepper_motor_task(void *params);

#ifdef __cplusplus
}
#endif