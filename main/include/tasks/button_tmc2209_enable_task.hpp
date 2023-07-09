#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "iot_button.h"

// |================================================================================================ |
// |                                          Main Task                                              |
// |================================================================================================ |

void button_tmc2209_enable_task(void *params);

#ifdef __cplusplus
}
#endif