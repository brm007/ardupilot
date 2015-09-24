#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>

#include "RCInput_Raspilot.h"

#include "px4io_protocol.h"

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

using namespace Linux;

void LinuxRCInput_Raspilot::init(void*)
{
    // start the timer process to read samples
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&LinuxRCInput_Raspilot::_poll_data, void));
}

void LinuxRCInput_Raspilot::_poll_data(void)
{
    // Throttle read rate to 100hz maximum.
    if (hal.scheduler->micros() - _last_timer < 10000) {
        return;
    }

    _last_timer = hal.scheduler->micros();

    uint16_t rcin_raw[LINUX_RC_INPUT_NUM_CHANNELS] = {0};

    if ( hal.iomcu->read(PX4IO_PAGE_RAW_RC_INPUT, 0, LINUX_RC_INPUT_NUM_CHANNELS, rcin_raw) ) {
        uint16_t rc_ok = rcin_raw[1] & (1 << 4);

        if (rc_ok) {
            _update_periods(&rcin_raw[6], rcin_raw[0]);
        }
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE
