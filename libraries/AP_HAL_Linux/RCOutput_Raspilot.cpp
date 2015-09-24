
#include <AP_HAL/AP_HAL.h>
#include "GPIO.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

#include "RCOutput_Raspilot.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "px4io_protocol.h"

using namespace Linux;

#define PWM_CHAN_COUNT 8

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void LinuxRCOutput_Raspilot::init(void* machtnicht)
{
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&LinuxRCOutput_Raspilot::_update, void));
}

void LinuxRCOutput_Raspilot::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    hal.scheduler->suspend_timer_procs();
    hal.iomcu->write(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_DEFAULTRATE, 1, &freq_hz);
    hal.scheduler->resume_timer_procs();

    _frequency = freq_hz;
}

uint16_t LinuxRCOutput_Raspilot::get_freq(uint8_t ch)
{
    return _frequency;
}

void LinuxRCOutput_Raspilot::enable_ch(uint8_t ch)
{

}

void LinuxRCOutput_Raspilot::disable_ch(uint8_t ch)
{
    write(ch, 0);
}

void LinuxRCOutput_Raspilot::write(uint8_t ch, uint16_t period_us)
{
    if(ch >= PWM_CHAN_COUNT){
        return;
    }

    _period_us[ch] = period_us;
}

void LinuxRCOutput_Raspilot::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++)
        write(ch + i, period_us[i]);
}

uint16_t LinuxRCOutput_Raspilot::read(uint8_t ch)
{
    if(ch >= PWM_CHAN_COUNT){
        return 0;
    }

    return _period_us[ch];
}

void LinuxRCOutput_Raspilot::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++)
        period_us[i] = read(0 + i);
}

void LinuxRCOutput_Raspilot::_update(void)
{
    if (hal.scheduler->micros() - _last_update_timestamp < 10000) {
        return;
    }

    _last_update_timestamp = hal.scheduler->micros();

    uint16_t tx_buf = 75;
    hal.iomcu->write(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 1, &tx_buf);
    tx_buf = 0x560B;
    hal.iomcu->write(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_OFF, 1, &tx_buf);
    hal.iomcu->write(PX4IO_PAGE_DIRECT_PWM, 0, PWM_CHAN_COUNT, _period_us);
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
