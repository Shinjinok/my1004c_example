/*! ----------------------------------------------------------------------------
 * @file    deca_sleep.c
 * @brief   platform dependent sleep implementation
 *
* @author     Decawave
*
* @attention  Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
*             All rights reserved.
 */

#include <deca_device_api.h>
#include "port_platform.h"

void deca_sleep(unsigned int time_ms)
{
    /* This assumes that the tick has a period of exactly one millisecond. See CLOCKS_PER_SEC define. */
    unsigned long end = portGetTickCount() + time_ms;
    while ((signed long)(portGetTickCount() - end) <= 0)
        ;
}
